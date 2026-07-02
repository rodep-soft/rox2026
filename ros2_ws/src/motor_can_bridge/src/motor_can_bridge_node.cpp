#include "motor_can_bridge/motor_can_bridge_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

// 2つのPWM topicを受け取り、STMが読むCAN frame形式に変換するnode。
//
// このnodeは以下の流れで動く。
//   1. /mabuchi555/pwm_value と /mad_motor/pwm_value をsubscribeする
//   2. 各callbackで「最後に受け取ったPWM値」と「受け取った時刻」を保存する
//   3. timerで一定周期ごとにCAN frameを作り、/can_txへpublishする
//
// callbackで即publishせずtimerを使う理由は、入力が来た瞬間だけでなく
// STMへ一定周期で指令を送り続けるため。

namespace
{
// CAN data frameのpayload長。STM側の受信仕様に合わせて8byte固定で送る。
constexpr uint8_t kCanDataSize = 8;

// モータごとに許可するPWM範囲を分ける。
// Mabuchiは正逆回転するため-255~255、MADモータは正方向のみなので0~255。
constexpr int16_t kMabuchiMinPwm = -255;
constexpr int16_t kMabuchiMaxPwm = 255;
constexpr int16_t kMadMotorMinPwm = 0;
constexpr int16_t kMadMotorMaxPwm = 255;

// configや別nodeから範囲外のPWM値が来ても、そのままCANへ流さず安全な範囲に丸める。
int16_t ClampToInt16Range(int value, int16_t min_value, int16_t max_value)
{
  return static_cast<int16_t>(
    std::clamp(value, static_cast<int>(min_value), static_cast<int>(max_value)));
}
}  // namespace

MotorCanBridgeNode::MotorCanBridgeNode()
: Node("motor_can_bridge_node")
{
  // parameterはlaunch fileやconfig.yamlから変更できる設定値。
  // declareしてからgetする、という順番がROS 2 C++の基本的な使い方。
  DeclareParameters();
  GetParameters();

  // 起動直後は、まだPWM topicを受け取っていないので停止指令の0から始める。
  latest_mabuchi_pwm_ = 0;
  latest_mad_motor_pwm_ = 0;
  last_mabuchi_time_ = this->now();
  last_mad_motor_time_ = this->now();

  // Mabuchi用PWM topicを購読する。messageが届くたびにMabuchiPwmCallbackが呼ばれる。
  mabuchi_subscription_ = this->create_subscription<std_msgs::msg::Int16>(
    mabuchi_pwm_topic_, 10,
    std::bind(&MotorCanBridgeNode::MabuchiPwmCallback, this, std::placeholders::_1));

  // MADモータ用PWM topicを購読する。messageが届くたびにMadMotorPwmCallbackが呼ばれる。
  mad_motor_subscription_ = this->create_subscription<std_msgs::msg::Int16>(
    mad_motor_pwm_topic_, 10,
    std::bind(&MotorCanBridgeNode::MadMotorPwmCallback, this, std::placeholders::_1));

  // STMへ送るCAN frameをpublishするtopic。
  // 実際にCAN busへ流す処理は、このtopicをsubscribeする別nodeが担当する想定。
  can_publisher_ = this->create_publisher<motor_can_bridge::msg::CanFrame>(
    can_tx_topic_, 10);

  // timerは指定周期でcallbackを呼ぶ仕組み。
  // send_period_msごとにTimerCallbackを呼び、最新PWMをCAN frameとしてpublishする。
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(send_period_ms_),
    std::bind(&MotorCanBridgeNode::TimerCallback, this));

  RCLCPP_INFO(this->get_logger(), "MotorCanBridgeNode started.");
}

void MotorCanBridgeNode::DeclareParameters()
{
  // topic名とCAN IDをparameterにしておくと、コードを変更せずconfig.yamlで調整できる。
  this->declare_parameter<std::string>("mabuchi_pwm_topic", "/mabuchi555/pwm_value");
  this->declare_parameter<std::string>("mad_motor_pwm_topic", "/mad_motor/pwm_value");
  this->declare_parameter<std::string>("can_tx_topic", "/can_tx");
  this->declare_parameter<int>("mabuchi_can_id", 0x201);
  this->declare_parameter<int>("mad_motor_can_id", 0x202);
  this->declare_parameter<int>("send_period_ms", 20);
  this->declare_parameter<int>("timeout_ms", 500);
}

void MotorCanBridgeNode::GetParameters()
{
  // declare_parameterで用意した値を、実際にメンバ変数へ読み込む。
  // 以降の処理ではROS parameterを直接読まず、このメンバ変数を使う。
  mabuchi_pwm_topic_ = this->get_parameter("mabuchi_pwm_topic").as_string();
  mad_motor_pwm_topic_ = this->get_parameter("mad_motor_pwm_topic").as_string();
  can_tx_topic_ = this->get_parameter("can_tx_topic").as_string();
  mabuchi_can_id_ = static_cast<uint32_t>(this->get_parameter("mabuchi_can_id").as_int());
  mad_motor_can_id_ = static_cast<uint32_t>(this->get_parameter("mad_motor_can_id").as_int());
  send_period_ms_ = std::max(1, static_cast<int>(this->get_parameter("send_period_ms").as_int()));
  timeout_ms_ = std::max(1, static_cast<int>(this->get_parameter("timeout_ms").as_int()));
}

void MotorCanBridgeNode::MabuchiPwmCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  // callback内ではCAN frameを作らず、最新値だけ保存する。
  // 実際のCAN送信周期はTimerCallback側でそろえる。
  latest_mabuchi_pwm_ = ClampToInt16Range(msg->data, kMabuchiMinPwm, kMabuchiMaxPwm);
  last_mabuchi_time_ = this->now();
}

void MotorCanBridgeNode::MadMotorPwmCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  // MADモータは逆回転を使わないため、0~255に丸めて保存する。
  latest_mad_motor_pwm_ = ClampToInt16Range(msg->data, kMadMotorMinPwm, kMadMotorMaxPwm);
  last_mad_motor_time_ = this->now();
}

void MotorCanBridgeNode::TimerCallback()
{
  // TimerCallbackはsend_period_msごとに呼ばれる。
  // ここで2つのモータ分のCAN frameをまとめてpublishする。
  const rclcpp::Time now = this->now();

  const int16_t mabuchi_pwm = GetPwmOrZeroOnTimeout(
    latest_mabuchi_pwm_, last_mabuchi_time_, now);
  const int16_t mad_motor_pwm = GetPwmOrZeroOnTimeout(
    latest_mad_motor_pwm_, last_mad_motor_time_, now);

  can_publisher_->publish(CreateMotorCanFrame(mabuchi_can_id_, mabuchi_pwm, now));
  can_publisher_->publish(CreateMotorCanFrame(mad_motor_can_id_, mad_motor_pwm, now));
}

int16_t MotorCanBridgeNode::GetPwmOrZeroOnTimeout(
  int16_t latest_pwm,
  const rclcpp::Time & last_update_time,
  const rclcpp::Time & now) const
{
  const auto elapsed = now - last_update_time;
  if (elapsed > rclcpp::Duration::from_seconds(static_cast<double>(timeout_ms_) / 1000.0)) {
    // controller側が止まった時に古いPWMを送り続けないよう、timeout後は0にする。
    return 0;
  }

  return latest_pwm;
}

motor_can_bridge::msg::CanFrame MotorCanBridgeNode::CreateMotorCanFrame(
  uint32_t can_id,
  int16_t pwm,
  const rclcpp::Time & stamp) const
{
  motor_can_bridge::msg::CanFrame frame;

  // header.stampは「このframeを作った時刻」。
  // 後でログや可視化を見る時に、いつのmessageか追いやすくなる。
  frame.header.stamp = stamp;
  frame.id = can_id;
  frame.extended = false;
  frame.fd = false;
  frame.brs = false;
  frame.esi = false;
  frame.rtr = false;
  frame.size = kCanDataSize;
  frame.data.fill(0);

  // data[0] is reserved for future flags. data[1..2] is int16 little-endian PWM.
  frame.data[0] = 0x00;
  frame.data[1] = static_cast<uint8_t>(pwm & 0xFF);
  frame.data[2] = static_cast<uint8_t>((pwm >> 8) & 0xFF);

  return frame;
}

int main(int argc, char * argv[])
{
  // ROS 2の初期化。nodeを作る前に必ず呼ぶ。
  rclcpp::init(argc, argv);

  // spin中はcallback待ち状態になる。
  // PWM topicが届けばsubscription callback、timer周期ではTimerCallbackが呼ばれる。
  rclcpp::spin(std::make_shared<MotorCanBridgeNode>());

  // Ctrl+Cなどでspinが終わった後、ROS 2を終了処理する。
  rclcpp::shutdown();
  return 0;
}
