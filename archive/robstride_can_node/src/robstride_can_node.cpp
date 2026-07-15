#include "robstride_can_node/robstride_can_node.hpp"

// RobStride 05 モーターへ、位置指令 or 速度指令をCAN frameとして送るnode。
//
// control_modeは起動時に固定するパラメータで、実行中の動的切り替えはしない。
// position用/velocity用の起動構成はrobot_bringup側に置き、
// モーターごとに使う方を起動する設計。
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
//     0x7005 run_mode    (uint8) : 1=位置モード(PP), 2=速度モード
//     0x7016 loc_ref     (float): 目標角度[rad]
//     0x700A spd_ref     (float): 目標角速度[rad/s]
//     0x700B limit_torque(float): トルク制限[Nm] (0~5.5)
//     0x7017 limit_spd   (float): CSP位置モード速度制限[rad/s] (0~50)
//     0x7018 limit_cur   (float): 電流制限[A] (0~11)
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
constexpr uint8_t CommTypeSetMechanicalZero = 6;  // 現在位置を機械ゼロに設定
constexpr uint8_t CommTypeParamWrite = 18;  // 単一パラメータ書き込み

// Type18で書き込む対象パラメータのindex (data[0-1]にリトルエンディアンで載せる値)
constexpr uint16_t IndexRunMode = 0x7005;      // 動作モード (uint8)
constexpr uint16_t IndexSpdRef = 0x700A;       // 目標角速度[rad/s] (float)
constexpr uint16_t IndexLimitTorque = 0x700B;  // トルク制限[N・m] (float)
constexpr uint16_t IndexLocRef = 0x7016;       // 目標角度[rad] (float)
constexpr uint16_t IndexLimitSpd = 0x7017;     // CSP位置モード速度制限[rad/s] (float)
constexpr uint16_t IndexLimitCur = 0x7018;     // 電流制限[A] (float)
constexpr uint16_t IndexVelMax = 0x7024;       // PP位置モード速度[rad/s] (float)
constexpr uint16_t IndexAccSet = 0x7025;       // PP位置モード加速度[rad/s^2] (float)

// run_mode(IndexRunMode)に書き込む値
constexpr uint8_t RunModePosition = 1;  // PP位置モード
constexpr uint8_t RunModeVelocity = 2;  // 速度モード
}  // namespace

RobstrideCanNode::RobstrideCanNode()
: Node("robstride_can_node"),
  control_mode_(ControlMode::Velocity),
  startup_completed_(false),
  command_target_(0.0)
{
  DeclareParameters();
  GetParameters();

  last_command_time_ = this->now();

  SetupRosInterfaces();

  RCLCPP_INFO(
    this->get_logger(),
    "RobstrideCanNode started. can_tx_topic=%s, motor_can_id=0x%X, "
    "control_mode=%s, command_topic=%s",
    can_tx_topic_.c_str(),
    motor_can_id_,
    control_mode_ == ControlMode::Position ? "position" : "velocity",
    command_topic_.c_str());

  // run_modeの書き込みはノード起動時に一度だけでよい（モーター側は設定を保持する）。
  // publisherが確実に有効になってから送るため、1回だけの遅延timerにまとめて送る。
  // timerのcallback内でそのtimer自身をcancelしても安全（実行中のcallbackは最後まで走る）。
  // position modeではRobStride 05マニュアルのPP位置モード手順に合わせ、
  // run_mode -> enable -> vel_max -> acc_set -> loc_refの順に送る。
  startup_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    [this]() {
      if (control_mode_ == ControlMode::Position) {
        if (set_zero_on_startup_) {
          SendSetMechanicalZero();
        }
        SendRunMode(RunModePosition);
        if (enable_on_startup_) {
          SendEnable();
        }
        SendPositionStartupParameters();
        SendStartupLimits();
        SendFloatParam(IndexLocRef, static_cast<float>(command_target_));
      } else {
        SendRunMode(RunModeVelocity);
        SendStartupLimits();
        if (enable_on_startup_) {
          SendEnable();
        }
      }
      startup_completed_ = true;
      startup_timer_->cancel();
    });
}

void RobstrideCanNode::DeclareParameters()
{
  this->declare_parameter<std::string>("can_tx_topic", "/CAN/can0/transmit");
  this->declare_parameter<bool>("is_extended", true);

  this->declare_parameter<int>("motor_can_id", 1);
  this->declare_parameter<int>("host_can_id", 0xFD);

  this->declare_parameter<std::string>("control_mode", "velocity");
  this->declare_parameter<std::string>("command_topic", "/robstride/command");

  this->declare_parameter<int>("send_period_ms", 20);
  this->declare_parameter<int>("timeout_ms", 500);

  this->declare_parameter<double>("position_min_rad", -12.566370614);
  this->declare_parameter<double>("position_max_rad", 12.566370614);
  this->declare_parameter<double>("velocity_min_rad_s", -50.0);
  this->declare_parameter<double>("velocity_max_rad_s", 50.0);

  this->declare_parameter<bool>("enable_on_startup", true);

  // 負値(デフォルト)は「未指定」を意味し、起動時にそのパラメータの書き込みをスキップする
  // (モーター側の既存設定をそのまま使う)。
  this->declare_parameter<double>("limit_torque", -1.0);
  this->declare_parameter<double>("limit_cur", -1.0);
  this->declare_parameter<double>("limit_spd", -1.0);

  this->declare_parameter<bool>("set_zero_on_startup", false);
  this->declare_parameter<double>("position_speed", -1.0);
  this->declare_parameter<double>("position_acceleration", -1.0);
}

void RobstrideCanNode::GetParameters()
{
  can_tx_topic_ = this->get_parameter("can_tx_topic").as_string();
  is_extended_ = this->get_parameter("is_extended").as_bool();

  motor_can_id_ = static_cast<uint8_t>(this->get_parameter("motor_can_id").as_int());
  host_can_id_ = static_cast<uint8_t>(this->get_parameter("host_can_id").as_int());

  control_mode_ = ParseControlMode(this->get_parameter("control_mode").as_string());
  command_topic_ = this->get_parameter("command_topic").as_string();

  send_period_ms_ = std::max(1, static_cast<int>(this->get_parameter("send_period_ms").as_int()));
  timeout_ms_ = std::max(1, static_cast<int>(this->get_parameter("timeout_ms").as_int()));

  position_min_rad_ = this->get_parameter("position_min_rad").as_double();
  position_max_rad_ = this->get_parameter("position_max_rad").as_double();
  velocity_min_rad_s_ = this->get_parameter("velocity_min_rad_s").as_double();
  velocity_max_rad_s_ = this->get_parameter("velocity_max_rad_s").as_double();

  enable_on_startup_ = this->get_parameter("enable_on_startup").as_bool();

  limit_torque_ = this->get_parameter("limit_torque").as_double();
  limit_cur_ = this->get_parameter("limit_cur").as_double();
  limit_spd_ = this->get_parameter("limit_spd").as_double();

  set_zero_on_startup_ = this->get_parameter("set_zero_on_startup").as_bool();
  position_speed_ = this->get_parameter("position_speed").as_double();
  position_acceleration_ = this->get_parameter("position_acceleration").as_double();

  if (position_min_rad_ > position_max_rad_) {
    RCLCPP_WARN(this->get_logger(), "position_min_rad is greater than position_max_rad. Swapping.");
    std::swap(position_min_rad_, position_max_rad_);
  }
  if (velocity_min_rad_s_ > velocity_max_rad_s_) {
    RCLCPP_WARN(
      this->get_logger(), "velocity_min_rad_s is greater than velocity_max_rad_s. Swapping.");
    std::swap(velocity_min_rad_s_, velocity_max_rad_s_);
  }
}

RobstrideCanNode::ControlMode RobstrideCanNode::ParseControlMode(const std::string & value) const
{
  if (value == "position") {
    return ControlMode::Position;
  }
  if (value == "velocity") {
    return ControlMode::Velocity;
  }

  // 設定ミスで動き出すのを避けるため、不明な値はvelocity(かつ後述の通り目標値0)側に倒す。
  RCLCPP_WARN(
    this->get_logger(),
    "Unknown control_mode '%s'. Falling back to 'velocity'.",
    value.c_str());
  return ControlMode::Velocity;
}

void RobstrideCanNode::SetupRosInterfaces()
{
  can_publisher_ = this->create_publisher<can_msgs::msg::Frame>(can_tx_topic_, 10);

  command_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
    command_topic_, 10,
    std::bind(&RobstrideCanNode::CommandCallback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(send_period_ms_),
    std::bind(&RobstrideCanNode::TimerCallback, this));
}

void RobstrideCanNode::CommandCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  // control_modeに応じて、指令値を対応する範囲([rad] or [rad/s])でクランプする。
  if (control_mode_ == ControlMode::Position) {
    command_target_ = Clamp(msg->data, position_min_rad_, position_max_rad_);
  } else {
    command_target_ = Clamp(msg->data, velocity_min_rad_s_, velocity_max_rad_s_);
  }
  last_command_time_ = this->now();
}

void RobstrideCanNode::TimerCallback()
{
  if (!startup_completed_) {
    return;
  }

  if (control_mode_ == ControlMode::Position) {
    // position modeでは指令が途切れても、最後に受け取った目標角度を送り続ける
    // (loc_refはPP modeとして目標位置に居座るだけなので、これ自体は安全)。
    SendFloatParam(IndexLocRef, static_cast<float>(command_target_));
    return;
  }

  const rclcpp::Time now = this->now();
  const bool timed_out =
    (now - last_command_time_) >
    rclcpp::Duration::from_seconds(static_cast<double>(timeout_ms_) / 1000.0);

  // velocity modeは指令が途切れたまま最後の速度を送り続けると回転し続けて危険なため、
  // timeoutしたらspd_ref=0を送って安全に停止させる。
  const double effective_velocity = timed_out ? 0.0 : command_target_;
  SendFloatParam(IndexSpdRef, static_cast<float>(effective_velocity));
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

void RobstrideCanNode::SendSetMechanicalZero()
{
  std::array<uint8_t, 8> data {};
  data[0] = 1;
  PublishFrame(CommTypeSetMechanicalZero, host_can_id_, data);
}

void RobstrideCanNode::SendStop()
{
  // velocity modeは回転指令が残ると危険なので、まず速度0を明示的に送ってから無効化する。
  // position modeは無効化のみ(loc_refは保持されても無効化でトルクが切れる)。
  if (control_mode_ == ControlMode::Velocity) {
    SendFloatParam(IndexSpdRef, 0.0f);
  }
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

void RobstrideCanNode::SendStartupLimits()
{
  // 負値(デフォルト)は「未指定」を意味し、そのレジスタへの書き込みをスキップする。
  if (limit_torque_ >= 0.0) {
    SendFloatParam(IndexLimitTorque, static_cast<float>(limit_torque_));
  }
  if (limit_cur_ >= 0.0) {
    SendFloatParam(IndexLimitCur, static_cast<float>(limit_cur_));
  }
  if (limit_spd_ >= 0.0) {
    SendFloatParam(IndexLimitSpd, static_cast<float>(limit_spd_));
  }
}

void RobstrideCanNode::SendPositionStartupParameters()
{
  if (control_mode_ != ControlMode::Position) {
    return;
  }

  if (position_speed_ >= 0.0) {
    SendFloatParam(IndexVelMax, static_cast<float>(position_speed_));
  }
  if (position_acceleration_ >= 0.0) {
    SendFloatParam(IndexAccSet, static_cast<float>(position_acceleration_));
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
  frame.is_extended = is_extended_;
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
