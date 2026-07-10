#include "robstride_can_node/robstride_can_node.hpp"

// RobStride 05 モーターへ、位置指令をCAN frameとして送るnode。
//
// このnodeはSocketCANへ直接writeしない。出力はcan_msgs/msg/Frameのtopicなので、
// ros2socketcan_bridgeがcan_tx_topicをsubscribeしてcan0/can1へ送信する想定。
//
// RobStrideのCANプロトコル(通信マニュアルより):
//   29bit拡張ID = [Bit28-24: Communication Type(5bit)]
//                 [Bit23-8 : Data Area 2(16bit、用途はTypeごとに異なる)]
//                 [Bit7-0  : 宛先CAN ID(8bit)]
//
//   Type 3  (Motor enabled to run) : Data Area2=host_can_id, data全byte=0
//   Type 4  (Motor stop)           : Data Area2=host_can_id, data全byte=0
//                                    (Type3と対になる無効化コマンド。ノード終了時に送る)
//   Type 18 (Single parameter write): Data Area2=host_can_id
//     data[0-1]=パラメータindex(リトルエンディアン), data[2-3]=0,
//     data[4-7]=値(uint8の場合はdata[4]のみ、floatの場合はリトルエンディアン4byte)
//
//   使用するパラメータindex:
//     0x7005 run_mode    (uint8) : 1=位置モード(PP)
//     0x7016 loc_ref     (float): 目標角度[rad]
//     0x7024 vel_max     (float): PP位置モード速度[rad/s]
//     0x7025 acc_set     (float): PP位置モード加速度[rad/s^2]

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <functional>
#include <memory>
#include <thread>

namespace
{
// Communication Type番号 (CAN IDのBit28-24に載せる値)
constexpr uint8_t CommTypeEnable = 3;       // モーター有効化
constexpr uint8_t CommTypeStop = 4;         // モーター停止(無効化)
constexpr uint8_t CommTypeParamWrite = 18;  // 単一パラメータ書き込み

// Type18で書き込む対象パラメータのindex (data[0-1]にリトルエンディアンで載せる値)
constexpr uint16_t IndexRunMode = 0x7005;      // 動作モード (uint8)
constexpr uint16_t IndexLocRef = 0x7016;       // 目標角度[rad] (float)
constexpr uint16_t IndexVelMax = 0x7024;       // PP位置モード速度[rad/s] (float)
constexpr uint16_t IndexAccSet = 0x7025;       // PP位置モード加速度[rad/s^2] (float)

// run_mode(IndexRunMode)に書き込む値
constexpr uint8_t RunModePosition = 1;  // PP位置モード
}  // namespace

RobstrideCanNode::RobstrideCanNode()
: Node("robstride_can_node"),
  startup_completed_(false),
  command_target_(0.0)
{
  // ros param
  DeclareParameters();
  GetParameters();

  // publisher
  can_publisher_ = this->create_publisher<can_msgs::msg::Frame>(can_tx_topic_, 10);

  // subscription
  command_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
    command_topic_, 10,
    std::bind(&RobstrideCanNode::CommandCallback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(send_period_ms_),
    std::bind(&RobstrideCanNode::TimerCallback, this));

  RCLCPP_INFO(
    this->get_logger(),
    "RobstrideCanNode started. can_tx_topic=%s, motor_can_id=0x%X, command_topic=%s",
    can_tx_topic_.c_str(),
    motor_can_id_,
    command_topic_.c_str());

  // run_modeの書き込みはノード起動時に一度だけでよい（モーター側は設定を保持する）。
  // publisherが確実に有効になってから送るため、1回だけの遅延timerにまとめて送る。
  // timerのcallback内でそのtimer自身をcancelしても安全（実行中のcallbackは最後まで走る）。
  // RobStride 05マニュアルのPP位置モード手順に合わせ、
  // run_mode -> enable -> vel_max -> acc_set -> loc_refの順に送る。

  startup_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    [this]() {
      SendRunMode(RunModePosition);
      if (enable_on_startup_) {
        SendEnable();               // motorのenableを送信
      }
      SendPositionStartupParameters(); // vel_max/acc_setを送信
      SendFloatParam(IndexLocRef, static_cast<float>(command_target_));
      startup_completed_ = true;
      startup_timer_->cancel();
    });
}

void RobstrideCanNode::DeclareParameters()
{
  this->declare_parameter<std::string>("can_tx_topic", "/CAN/can0/transmit");
  this->declare_parameter<int>("motor_can_id", 1);
  this->declare_parameter<int>("host_can_id", 0xFD);

  this->declare_parameter<std::string>("command_topic", "/robstride/command");

  this->declare_parameter<int>("send_period_ms", 20);

  this->declare_parameter<double>("position_min_rad", -12.566370614);
  this->declare_parameter<double>("position_max_rad", 12.566370614);


  this->declare_parameter<bool>("enable_on_startup", true);
  this->declare_parameter<double>("position_speed", -1.0);
  this->declare_parameter<double>("position_acceleration", -1.0);
}

void RobstrideCanNode::GetParameters()
{
  can_tx_topic_ = this->get_parameter("can_tx_topic").as_string();
  motor_can_id_ = static_cast<uint8_t>(this->get_parameter("motor_can_id").as_int());
  host_can_id_ = static_cast<uint8_t>(this->get_parameter("host_can_id").as_int());

  command_topic_ = this->get_parameter("command_topic").as_string();

  send_period_ms_ = std::max(1, static_cast<int>(this->get_parameter("send_period_ms").as_int()));

  position_min_rad_ = this->get_parameter("position_min_rad").as_double();
  position_max_rad_ = this->get_parameter("position_max_rad").as_double();

  enable_on_startup_ = this->get_parameter("enable_on_startup").as_bool();

  position_speed_ = this->get_parameter("position_speed").as_double();
  position_acceleration_ = this->get_parameter("position_acceleration").as_double();

  if (position_min_rad_ > position_max_rad_) {
    RCLCPP_WARN(this->get_logger(), "position_min_rad is greater than position_max_rad. Swapping.");
    std::swap(position_min_rad_, position_max_rad_);
  }
}

void RobstrideCanNode::CommandCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  command_target_ = Clamp(msg->data, position_min_rad_, position_max_rad_);
}

void RobstrideCanNode::TimerCallback()
{
  if (!startup_completed_) {
    return;
  }

  // 指令が途切れても、最後に受け取った目標角度を送り続ける
  // (loc_refはPP modeとして目標位置に居座るだけなので、これ自体は安全)。
  SendFloatParam(IndexLocRef, static_cast<float>(command_target_));
}

void RobstrideCanNode::SendEnable()
{
  std::array<uint8_t, 8> data {};
  PublishFrame(CommTypeEnable, host_can_id_, data);
}

void RobstrideCanNode::SendDisable()
{
  std::array<uint8_t, 8> data {};
  PublishFrame(CommTypeStop, host_can_id_, data);
}

void RobstrideCanNode::SendStop()
{
  // loc_refは保持されても、無効化(Type 4)でトルクが切れるため、無効化のみでよい。
  SendDisable();
  RCLCPP_INFO(this->get_logger(), "RobstrideCanNode stopping: sent motor stop frame.");
}

void RobstrideCanNode::SendRunMode(uint8_t run_mode)
{
  std::array<uint8_t, 8> data {};
  // index(0x7005)をリトルエンディアンでdata[0-1]に配置。run_modeはuint8なのでdata[4]のみ使う。
  data[0] = static_cast<uint8_t>(IndexRunMode & 0xFF);
  data[1] = static_cast<uint8_t>((IndexRunMode >> 8) & 0xFF);
  data[4] = run_mode;
  PublishFrame(CommTypeParamWrite, host_can_id_, data);
}

void RobstrideCanNode::SendPositionStartupParameters()
{
  if (position_speed_ >= 0.0) {
    SendFloatParam(IndexVelMax, static_cast<float>(position_speed_));
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "position_speed is negative (%.3f); vel_max is not written and the motor's existing value is used.",
      position_speed_);
  }
  if (position_acceleration_ >= 0.0) {
    SendFloatParam(IndexAccSet, static_cast<float>(position_acceleration_));
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "position_acceleration is negative (%.3f); acc_set is not written and the motor's existing value is used.",
      position_acceleration_);
  }
}

void RobstrideCanNode::SendFloatParam(uint16_t index, float value)
{
  // Type18のパラメータ値はraw float(IEEE754)をそのままリトルエンディアンで4byte並べる。
  // Type1(運転制御)/Type2(フィードバック)で使われる「範囲正規化したuint16」とは別物なので注意。
  uint32_t raw = 0;
  std::memcpy(&raw, &value, sizeof(raw));

  std::array<uint8_t, 8> data {};
  data[0] = static_cast<uint8_t>(index & 0xFF);
  data[1] = static_cast<uint8_t>((index >> 8) & 0xFF);
  data[4] = static_cast<uint8_t>(raw & 0xFF);
  data[5] = static_cast<uint8_t>((raw >> 8) & 0xFF);
  data[6] = static_cast<uint8_t>((raw >> 16) & 0xFF);
  data[7] = static_cast<uint8_t>((raw >> 24) & 0xFF);

  PublishFrame(CommTypeParamWrite, host_can_id_, data);
}

void RobstrideCanNode::PublishFrame(
  uint8_t comm_type,
  uint16_t data_area2,
  const std::array<uint8_t, 8> & data)
{
  can_msgs::msg::Frame frame;
  frame.header.stamp = this->now();
  // 29bit拡張ID = [comm_type(bit28-24)] [data_area2(bit23-8)] [宛先モーターCAN_ID(bit7-0)]
  // ここで渡すdata_area2は常にhost_can_id_（bit15-8にホストCAN_IDが入る用途のみ使用）。
  frame.id =
    (static_cast<uint32_t>(comm_type & 0x1F) << 24) |
    (static_cast<uint32_t>(data_area2) << 8) |
    motor_can_id_;
  frame.is_rtr = false;
  frame.is_extended = true;
  frame.is_error = false;
  // Type3/Type18はどちらもdlc=8固定（未使用バイトは0埋め）でモーター側が受け取る仕様。
  frame.dlc = static_cast<uint8_t>(data.size());
  frame.data.fill(0);
  for (size_t i = 0; i < data.size(); ++i) {
    frame.data[i] = data[i];
  }

  can_publisher_->publish(frame);
}

double RobstrideCanNode::Clamp(double value, double min_value, double max_value)
{
  return std::clamp(value, min_value, max_value);
}

namespace
{
// SIGINT/SIGTERMを自前で捕まえてフラグだけ立てる。実際の停止処理はmainループで行う。
// (rclcpp標準のsignal handlerは受信後すぐcontextを無効化してしまい、その後のpublishが
//  失敗するため。contextが有効なうちに停止フレームを送りたい。)
std::atomic<bool> g_stop_requested{false};
void HandleSignal(int) {g_stop_requested = true;}
}  // namespace

int main(int argc, char * argv[])
{
  // shutdown_on_signal=falseでrclcpp標準のsignal-driven shutdownを止め、停止送信の
  // タイミングを自分で制御する。
  rclcpp::InitOptions init_options;
  init_options.shutdown_on_signal = false;
  rclcpp::init(argc, argv, init_options);

  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);

  auto node = std::make_shared<RobstrideCanNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  while (rclcpp::ok() && !g_stop_requested) {
    executor.spin_some(std::chrono::milliseconds(10));
  }

  // contextがまだ有効なこの時点で停止フレームを送る。DDSが実際に送出しきる猶予を少し取る。
  node->SendStop();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  rclcpp::shutdown();
  return 0;
}
