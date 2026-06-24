#include "motor_can_bridge/motor_can_command_node.hpp"

// 1つのモータPWM topicを、STM向けのCAN frameに変換するnode。
//
// ROS 2では、node同士がtopicを通してmessageをやり取りする。
// このnodeはpwm_topicをsubscribeしてPWM値を受け取り、
// can_tx_topicへcan_msgs/msg/Frameをpublishする。
//
// このnodeはSocketCANへ直接writeしない。出力はcan_msgs/msg/Frameのtopicなので、
// 別のCAN driver nodeがcan_tx_topicをsubscribeしてcan0/can1へ送信する想定。
//
// モータ種別はCAN IDで判別する。STM向けpayloadは以下の形式。
//   byte0: pwm low byte
//   byte1: pwm high byte
//   byte2-7: 予備、現状は0

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace
{
constexpr uint8_t kCanDataSize = 8;
}  // namespace

MotorCanCommandNode::MotorCanCommandNode()
: Node("motor_can_command_node")
{
  // parameterはlaunch fileやconfig.yamlから変更できる設定値。
  // declareしてからgetする、という順番がROS 2 C++の基本的な使い方。
  DeclareParameters();
  GetParameters();

  // 起動直後は、まだPWM topicを受け取っていないので停止指令の0から始める。
  latest_pwm_ = 0;
  last_pwm_time_ = this->now();

  SetupRosInterfaces();

  RCLCPP_INFO(
    this->get_logger(),
    "MotorCanCommandNode started. pwm_topic=%s, can_tx_topic=%s, can_id=0x%X",
    pwm_topic_.c_str(),
    can_tx_topic_.c_str(),
    can_id_);
}

void MotorCanCommandNode::SetupRosInterfaces()
{
  // PWM topicは入力値の更新だけに使う。CAN送信はこのcallback内では行わない。
  // messageが届くたびにPwmCallbackが呼ばれ、最新PWM値だけを保存する。
  pwm_subscription_ = this->create_subscription<std_msgs::msg::Int16>(
    pwm_topic_, 10,
    std::bind(&MotorCanCommandNode::PwmCallback, this, std::placeholders::_1));

  // このtopicをCAN driver nodeがsubscribeして、実際のCAN busへ送る。
  can_publisher_ = this->create_publisher<can_msgs::msg::Frame>(can_tx_topic_, 10);

  // STMへ一定周期でcommandを送り続けるためのtimer。
  // send_period_msごとにTimerCallbackが呼ばれる。
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(send_period_ms_),
    std::bind(&MotorCanCommandNode::TimerCallback, this));
}

void MotorCanCommandNode::DeclareParameters()
{
  // launch/configから上書きできる値をここで宣言する。
  // 同じ実行ファイルを複数起動しても、parameterを変えることで別モータ用に使える。
  this->declare_parameter<std::string>("pwm_topic", "/motor/pwm_value");
  this->declare_parameter<std::string>("can_tx_topic", "/can_tx");
  this->declare_parameter<int>("can_id", 0x201);
  this->declare_parameter<bool>("is_extended", false);
  this->declare_parameter<int>("min_pwm", -255);
  this->declare_parameter<int>("max_pwm", 255);
  this->declare_parameter<int>("send_period_ms", 20);
  this->declare_parameter<int>("timeout_ms", 500);
}

void MotorCanCommandNode::GetParameters()
{
  // configで指定されたtopic名やCAN IDを読み込む。
  pwm_topic_ = this->get_parameter("pwm_topic").as_string();
  can_tx_topic_ = this->get_parameter("can_tx_topic").as_string();
  can_id_ = static_cast<uint32_t>(this->get_parameter("can_id").as_int());
  is_extended_ = this->get_parameter("is_extended").as_bool();
  min_pwm_ = static_cast<int>(this->get_parameter("min_pwm").as_int());
  max_pwm_ = static_cast<int>(this->get_parameter("max_pwm").as_int());
  send_period_ms_ = std::max(
    1,
    static_cast<int>(this->get_parameter("send_period_ms").as_int()));
  timeout_ms_ = std::max(
    1,
    static_cast<int>(this->get_parameter("timeout_ms").as_int()));

  // 設定ミスでmin/maxが逆でも、nodeを落とさず安全側に補正する。
  if (min_pwm_ > max_pwm_) {
    RCLCPP_WARN(
      this->get_logger(),
      "min_pwm is greater than max_pwm. Swapping values.");
    std::swap(min_pwm_, max_pwm_);
  }
}

void MotorCanCommandNode::PwmCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  // ここでは最新のPWMだけ保存する。CAN送信周期はTimerCallback側で管理する。
  latest_pwm_ = ClampPwm(msg->data);
  last_pwm_time_ = this->now();
}

void MotorCanCommandNode::TimerCallback()
{
  // TimerCallbackはsend_period_msごとに呼ばれる。
  // 最新PWM値をCAN frameへ変換し、can_tx_topicへpublishする。
  const rclcpp::Time now = this->now();
  // PWM入力が来ていなくても、timeoutするまでは最後のPWMを周期送信する。
  can_publisher_->publish(CreateCanFrame(GetPwmOrZeroOnTimeout(now), now));
}

int16_t MotorCanCommandNode::ClampPwm(int value) const
{
  // PWM範囲はモータごとに違うので、parameterで指定したmin/maxに丸める。
  return static_cast<int16_t>(std::clamp(value, min_pwm_, max_pwm_));
}

int16_t MotorCanCommandNode::GetPwmOrZeroOnTimeout(const rclcpp::Time & now) const
{
  const auto elapsed = now - last_pwm_time_;
  if (elapsed > rclcpp::Duration::from_seconds(static_cast<double>(timeout_ms_) / 1000.0)) {
    // PWM入力が途切れた場合は、古いPWMを送り続けず停止指令にする。
    return 0;
  }

  return latest_pwm_;
}

can_msgs::msg::Frame MotorCanCommandNode::CreateCanFrame(
  int16_t pwm,
  const rclcpp::Time & stamp) const
{
  can_msgs::msg::Frame frame;

  // header.stampは「このframeを作った時刻」。
  // 後でログや可視化を見る時に、いつのmessageか追いやすくなる。
  frame.header.stamp = stamp;
  frame.id = can_id_;

  // 通常のdata frameを送る。remote/error frameはこのnodeでは使わない。
  frame.is_rtr = false;
  frame.is_extended = is_extended_;
  frame.is_error = false;

  // STM側の受信仕様に合わせて8byte固定で送る。
  frame.dlc = kCanDataSize;

  frame.data.fill(0);
  // STM側ではint16のPWMをlittle-endianとして読む想定。
  frame.data[0] = static_cast<uint8_t>(pwm & 0xFF);
  frame.data[1] = static_cast<uint8_t>((pwm >> 8) & 0xFF);

  return frame;
}

int main(int argc, char * argv[])
{
  // ROS 2の初期化。nodeを作る前に必ず呼ぶ。
  rclcpp::init(argc, argv);

  // spin中はcallback待ち状態になる。
  // PWM topicが届けばPwmCallback、timer周期ではTimerCallbackが呼ばれる。
  rclcpp::spin(std::make_shared<MotorCanCommandNode>());

  // Ctrl+Cなどでspinが終わった後、ROS 2を終了処理する。
  rclcpp::shutdown();
  return 0;
}
