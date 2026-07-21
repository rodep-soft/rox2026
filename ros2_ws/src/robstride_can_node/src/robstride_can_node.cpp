#include "robstride_can_node/robstride_can_node.hpp"

// RobStrideのCANプロトコル(通信マニュアルより):
//   29bit拡張ID = [Bit28-24: Communication Type(5bit)]
//                 [Bit23-8 : Data Area 2(16bit、用途はTypeごとに異なる)]
//                 [Bit7-0  : 宛先CAN ID(8bit)]
//   Type 3  (Motor enabled to run) : Data Area2=host_can_id, data全byte=0
//   Type 4  (Motor stop)           : Data Area2=host_can_id, data全byte=0
//   Type 18 (Single parameter write): Data Area2=host_can_id
//     data[0-1]=パラメータindex(リトルエンディアン), data[2-3]=0,
//     data[4-7]=値(uint8の場合はdata[4]のみ、floatの場合はリトルエンディアン4byte)
//   Type 18 payloadのパラメータindex:
//     05 70: run_mode (uint8), 1=位置モード(PP)
//     16 70: loc_ref (float), 目標角度[rad]
//     24 70: vel_max (float), PP位置モードの最大速度[rad/s]
//     25 70: acc_set (float), PP位置モードの加速度[rad/s^2]

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>
#include <thread>

namespace
{
constexpr uint8_t RunModePosition = 1;  // PP位置モード
constexpr uint8_t RunModeOperation = 0;  // Operation control mode (Type 6でのゼロ校正用)
constexpr uint16_t IndexLocRef = 0x7016;
constexpr uint16_t IndexLimitCur = 0x7018;
constexpr uint16_t IndexVelMax = 0x7024;
constexpr uint16_t IndexAccSet = 0x7025;
}

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

  // run_modeの書き込みはノード起動時に一度だけ
  // ros2socketcan側のsubscriptionとのDDS discoveryが完了するまで時間が読めないため、
  // subscriber数をポーリングしてから送る。
  // Type 6によるゼロ校正はPPモードではブロックされるため、
  // stop -> motion control -> enable -> mechanical zero -> stop -> PP mode -> enable の順

  startup_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    [this]() {
      if (can_publisher_->get_subscription_count() == 0) {
        return;         // ros2socketcanとまだ繋がっていない
      }
      // ノード再起動前にモーターが有効だった場合も、モード切替前に出力を止める。
      SendDisable();
      std::this_thread::sleep_for(std::chrono::milliseconds(startup_inter_frame_ms_));
      SendRunMode(RunModeOperation);
      std::this_thread::sleep_for(std::chrono::milliseconds(startup_inter_frame_ms_));
      SendEnable();
      std::this_thread::sleep_for(std::chrono::milliseconds(startup_inter_frame_ms_));
      SendSetMechanicalZero();
      std::this_thread::sleep_for(std::chrono::milliseconds(startup_inter_frame_ms_));
      SendDisable();
      std::this_thread::sleep_for(std::chrono::milliseconds(startup_inter_frame_ms_));
      SendRunMode(RunModePosition);
      std::this_thread::sleep_for(std::chrono::milliseconds(startup_inter_frame_ms_));
      if (enable_on_startup_) {
      	RCLCPP_INFO(this->get_logger(), "IF check enable_on_setup.");
        SendEnable();               // motorのenableを送信
        std::this_thread::sleep_for(std::chrono::milliseconds(startup_inter_frame_ms_));
      }
      SendPositionMaxVelocity();    // vel_maxの送信
      SendPositionAcceleration();   // acc_setの送信
      SendPositionCurrentLimit();   // 電流制限の送信
      SendZeroPositionFlag();       // zero_sta=1 (-π〜+π)を送信
      SendPositionReference(static_cast<float>(command_target_));
      startup_completed_ = true;
      startup_timer_->cancel();
      RCLCPP_INFO(this->get_logger(), "Startup command sequence sent.");
    });
}

void RobstrideCanNode::DeclareParameters()
{
  this->declare_parameter<std::string>("can_tx_topic", "/CAN/can0/transmit");
  this->declare_parameter<int>("motor_can_id", 1);
  this->declare_parameter<int>("host_can_id", 0xFD);

  this->declare_parameter<std::string>("command_topic", "/robstride/command");
  this->declare_parameter<int>("send_period_ms", 50);
  this->declare_parameter<int>("startup_inter_frame_ms", 100);

 
  this->declare_parameter<double>("position_min_rad", -1.6);
  this->declare_parameter<double>("position_max_rad", 1.7);
  this->declare_parameter<double>("home_position_rad", 0.0);


  this->declare_parameter<bool>("enable_on_startup", true);
  this->declare_parameter<double>("position_current_limit", -1.0);
  this->declare_parameter<double>("position_velocity_rad_s", 10.0);
  this->declare_parameter<double>("position_acceleration_rad_s2", 10.0);
  this->declare_parameter<int>("shutdown_return_wait_ms", 1000);
}

void RobstrideCanNode::GetParameters()
{
  can_tx_topic_ = this->get_parameter("can_tx_topic").as_string();
  const int configured_motor_can_id = this->get_parameter("motor_can_id").as_int();
  const int configured_host_can_id = this->get_parameter("host_can_id").as_int();
  if (configured_motor_can_id < 1 || configured_motor_can_id > 0x7F) {
    throw std::invalid_argument("motor_can_id must be in the range 1..127");
  }
  if (configured_host_can_id < 0 || configured_host_can_id > 0xFF) {
    throw std::invalid_argument("host_can_id must be in the range 0..255");
  }
  motor_can_id_ = static_cast<uint8_t>(configured_motor_can_id);
  host_can_id_ = static_cast<uint8_t>(configured_host_can_id);

  command_topic_ = this->get_parameter("command_topic").as_string();
  send_period_ms_ = std::max(1, static_cast<int>(this->get_parameter("send_period_ms").as_int()));
  startup_inter_frame_ms_ = std::max(
    0, static_cast<int>(this->get_parameter("startup_inter_frame_ms").as_int()));

  position_min_rad_ = this->get_parameter("position_min_rad").as_double();
  position_max_rad_ = this->get_parameter("position_max_rad").as_double();

  enable_on_startup_ = this->get_parameter("enable_on_startup").as_bool();

  position_current_limit_ = this->get_parameter("position_current_limit").as_double();
  position_velocity_rad_s_ = this->get_parameter("position_velocity_rad_s").as_double();
  position_acceleration_rad_s2_ =
    this->get_parameter("position_acceleration_rad_s2").as_double();

  if (position_current_limit_ > 11.0) {
    throw std::invalid_argument("position_current_limit must be at most 11 A");
  }
  if (position_velocity_rad_s_ < 0.0 || position_velocity_rad_s_ > 33.0) {
    throw std::invalid_argument("position_velocity_rad_s must be in the range 0..33 rad/s");
  }
  if (position_acceleration_rad_s2_ < 0.0) {
    throw std::invalid_argument("position_acceleration_rad_s2 must be non-negative");
  }

  if (position_min_rad_ > position_max_rad_) {
    RCLCPP_WARN(this->get_logger(), "position_min_rad is greater than position_max_rad. Swapping.");
    std::swap(position_min_rad_, position_max_rad_);
  }

  const double configured_home_position = this->get_parameter("home_position_rad").as_double();
  home_position_rad_ = Clamp(configured_home_position, position_min_rad_, position_max_rad_);
  if (home_position_rad_ != configured_home_position) {
    RCLCPP_WARN(
      this->get_logger(),
      "home_position_rad %.3f is outside the position range and is clamped to %.3f.",
      configured_home_position,
      home_position_rad_);
  }
  command_target_ = home_position_rad_;

  shutdown_return_wait_ms_ = std::max(
    0,
    static_cast<int>(this->get_parameter("shutdown_return_wait_ms").as_int()));
}

void RobstrideCanNode::CommandCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  /*RCLCPP_INFO(
    this->get_logger(), "Received: %s data=%.6f rad", command_subscription_->get_topic_name(),
    msg->data); */
  command_target_ = Clamp(msg->data, position_min_rad_, position_max_rad_);
}

void RobstrideCanNode::TimerCallback()
{
  if (!startup_completed_) {
    return;
  }

  // 指令が途切れても、最後に受け取った目標角度を送り続ける
  // (loc_refはPP modeとして目標位置に居座るだけなので、これ自体は安全)。
  SendPositionReference(static_cast<float>(command_target_));
}

void RobstrideCanNode::SendEnable()
{
  can_msgs::msg::Frame frame;
  frame.header.stamp = this->now();
  // Type 3: 0x0300FD00 + motor_can_id_ (FD部分はhost_can_id_で設定可能)。
  frame.id = (0x03U << 24) | (static_cast<uint32_t>(host_can_id_) << 8) | motor_can_id_;
  frame.is_rtr = false;
  frame.is_extended = true;
  frame.is_error = false;
  frame.dlc = 8;
  frame.data.fill(0);
  can_publisher_->publish(frame);
}

void RobstrideCanNode::SendDisable()
{
  can_msgs::msg::Frame frame;
  frame.header.stamp = this->now();
  // Type 4: 0x0400FD00 + motor_can_id_。
  frame.id = (0x04U << 24) | (static_cast<uint32_t>(host_can_id_) << 8) | motor_can_id_;
  frame.is_rtr = false;
  frame.is_extended = true;
  frame.is_error = false;
  frame.dlc = 8;
  frame.data.fill(0);
  can_publisher_->publish(frame);
}

void RobstrideCanNode::SendSetMechanicalZero()
{
  can_msgs::msg::Frame frame;
  frame.header.stamp = this->now();
  // Type 6: 0x0600FD00 + motor_can_id_。
  frame.id = (0x06U << 24) | (static_cast<uint32_t>(host_can_id_) << 8) | motor_can_id_;
  frame.is_rtr = false;
  frame.is_extended = true;
  frame.is_error = false;
  frame.dlc = 8;
  frame.data.fill(0);
  // Set motor mechanical zero: Byte[0] = 1。
  frame.data[0] = 0x01;
  can_publisher_->publish(frame);
}

void RobstrideCanNode::SendStop()
{
  // shutdown後にTimerCallbackが最後の指令を再送しないよう止める。
  timer_->cancel();

  if (!startup_completed_) {
    SendDisable();
    RCLCPP_INFO(
      this->get_logger(),
      "RobstrideCanNode stopped before initialization completed; sent motor stop frame only.");
    return;
  }

  command_target_ = home_position_rad_;

  // 位置フィードバックを使っていないため、指定時間はhomeへの移動完了を待つ安全マージン。
  // 待機中もloc_refを周期送信して、途中で指令が欠落してもhome位置を維持する。
  const auto wait_duration = std::chrono::milliseconds(shutdown_return_wait_ms_);
  const auto deadline = std::chrono::steady_clock::now() + wait_duration;
  do {
    SendPositionReference(static_cast<float>(home_position_rad_));
    if (std::chrono::steady_clock::now() >= deadline) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(send_period_ms_));
  } while (std::chrono::steady_clock::now() < deadline);

  SendDisable();
  RCLCPP_INFO(
    this->get_logger(),
    "RobstrideCanNode stopping: returned to %.3f rad for %d ms, then sent motor stop frame.",
    home_position_rad_,
    shutdown_return_wait_ms_);
}

void RobstrideCanNode::SendRunMode(uint8_t run_mode)
{
  can_msgs::msg::Frame frame;
  frame.header.stamp = this->now();
  // Type 18: 0x1200FD00 + motor_can_id_。
  frame.id = (0x12U << 24) | (static_cast<uint32_t>(host_can_id_) << 8) | motor_can_id_;
  frame.is_rtr = false;
  frame.is_extended = true;
  frame.is_error = false;
  frame.dlc = 8;
  frame.data.fill(0);
  // run_mode (0x7005): 05 70 00 00 <mode> 00 00 00
  frame.data[0] = 0x05;
  frame.data[1] = 0x70;
  frame.data[4] = run_mode;
  can_publisher_->publish(frame);
}

void RobstrideCanNode::SendPositionCurrentLimit()
{
  if (position_current_limit_ >= 0.0) {
    SendPositionCurrentLimit(static_cast<float>(position_current_limit_));
    std::this_thread::sleep_for(std::chrono::milliseconds(startup_inter_frame_ms_));
  } else {
    RCLCPP_INFO(
      this->get_logger(),
      "position_current_limit is negative (%.3f); limit_cur is not written and the motor's existing value is used.",
      position_current_limit_);
  }
}

void RobstrideCanNode::SendPositionCurrentLimit(float current_limit)
{
  // limit_cur (0x7018): 18 70 00 00 <float little-endian>
  SendFloatParameter(IndexLimitCur, current_limit);
}

void RobstrideCanNode::SendPositionMaxVelocity()
{
  SendPositionMaxVelocity(static_cast<float>(position_velocity_rad_s_));
  std::this_thread::sleep_for(std::chrono::milliseconds(startup_inter_frame_ms_));
}

void RobstrideCanNode::SendPositionMaxVelocity(float velocity)
{
  // vel_max (0x7024): 24 70 00 00 <float little-endian>
  SendFloatParameter(IndexVelMax, velocity);
}

void RobstrideCanNode::SendPositionAcceleration()
{
  SendPositionAcceleration(static_cast<float>(position_acceleration_rad_s2_));
  std::this_thread::sleep_for(std::chrono::milliseconds(startup_inter_frame_ms_));
}

void RobstrideCanNode::SendPositionAcceleration(float acceleration)
{
  // acc_set (0x7025): 25 70 00 00 <float little-endian>
  SendFloatParameter(IndexAccSet, acceleration);
}

void RobstrideCanNode::SendZeroPositionFlag()
{
  can_msgs::msg::Frame frame;
  frame.header.stamp = this->now();
  frame.id = (0x12U << 24) | (static_cast<uint32_t>(host_can_id_) << 8) | motor_can_id_;
  frame.is_rtr = false;
  frame.is_extended = true;
  frame.is_error = false;
  frame.dlc = 8;
  frame.data.fill(0);
  // zero_sta (0x7029): 29 70 00 00 01 00 00 00 (-π〜+π表現)
  frame.data[0] = 0x29;
  frame.data[1] = 0x70;
  frame.data[4] = 0x01;
  can_publisher_->publish(frame);
}

void RobstrideCanNode::SendPositionReference(float position)
{
  // loc_ref (0x7016): 16 70 00 00 <float little-endian>
  SendFloatParameter(IndexLocRef, position);
}

void RobstrideCanNode::SendFloatParameter(uint16_t index, float value)
{
  uint32_t raw = 0;
  std::memcpy(&raw, &value, sizeof(raw));

  can_msgs::msg::Frame frame;
  frame.header.stamp = this->now();
  frame.id = (0x12U << 24) | (static_cast<uint32_t>(host_can_id_) << 8) | motor_can_id_;
  frame.is_rtr = false;
  frame.is_extended = true;
  frame.is_error = false;
  frame.dlc = 8;
  frame.data.fill(0);
  frame.data[0] = static_cast<uint8_t>(index & 0xFF);
  frame.data[1] = static_cast<uint8_t>((index >> 8) & 0xFF);
  frame.data[4] = static_cast<uint8_t>(raw & 0xFF);
  frame.data[5] = static_cast<uint8_t>((raw >> 8) & 0xFF);
  frame.data[6] = static_cast<uint8_t>((raw >> 16) & 0xFF);
  frame.data[7] = static_cast<uint8_t>((raw >> 24) & 0xFF);

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

  // contextがまだ有効なこの時点でhome位置へ復帰させ、停止フレームを送る。
  node->SendStop();

  rclcpp::shutdown();
  return 0;
}
