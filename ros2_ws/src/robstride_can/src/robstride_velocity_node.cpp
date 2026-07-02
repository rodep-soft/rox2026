#include "robstride_can/robstride_velocity_node.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

namespace
{
constexpr uint8_t kCanDlc = 8;
constexpr uint8_t kRunModeVelocity = 2;
constexpr uint8_t kTypeFeedback = 2;
constexpr uint8_t kTypeEnable = 3;
constexpr uint8_t kTypeStop = 4;
constexpr uint8_t kTypeWriteParameter = 18;
constexpr uint16_t kRunModeIndex = 0x7005;
constexpr uint16_t kSpeedRefIndex = 0x700A;
constexpr uint16_t kCurrentLimitIndex = 0x7018;
}  // namespace

RobstrideVelocityNode::RobstrideVelocityNode()
: Node("robstride_velocity_node")
{
  DeclareParameters();
  LoadParameters();
  OpenCanSocket();
  SetupRosInterfaces();
  StartReceiveThread();

  if (auto_enable_) {
    SendStop();
    SendSetMode(kRunModeVelocity);
    SendEnable();
  }

  RCLCPP_INFO(
    get_logger(),
    "Robstride private protocol node started. interface=%s, motor_id=0x%02X, host_id=0x%02X",
    can_interface_.c_str(),
    motor_id_,
    host_id_);
}

RobstrideVelocityNode::~RobstrideVelocityNode()
{
  running_ = false;

  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (socket_fd_ >= 0) {
    if (stop_on_shutdown_) {
      SendStop();
    }
    close(socket_fd_);
  }
}

void RobstrideVelocityNode::DeclareParameters()
{
  declare_parameter<std::string>("can_interface", "can0");
  declare_parameter<int>("motor_id", 0x7F);
  declare_parameter<int>("host_id", 0xFD);
  declare_parameter<std::string>("joint_name", "el05_joint");
  declare_parameter<double>("current_limit_a", 1.0);
  declare_parameter<bool>("auto_enable", false);
  declare_parameter<bool>("stop_on_shutdown", true);
  declare_parameter<double>("position_min_rad", -12.57);
  declare_parameter<double>("position_max_rad", 12.57);
  declare_parameter<double>("velocity_min_rad_s", -50.0);
  declare_parameter<double>("velocity_max_rad_s", 50.0);
  declare_parameter<double>("torque_min_nm", -6.0);
  declare_parameter<double>("torque_max_nm", 6.0);
}

void RobstrideVelocityNode::LoadParameters()
{
  can_interface_ = get_parameter("can_interface").as_string();
  motor_id_ = ClampCanId(get_parameter("motor_id").as_int());
  host_id_ = ClampCanId(get_parameter("host_id").as_int());
  joint_name_ = get_parameter("joint_name").as_string();
  current_limit_a_ = std::max(0.0, get_parameter("current_limit_a").as_double());
  auto_enable_ = get_parameter("auto_enable").as_bool();
  stop_on_shutdown_ = get_parameter("stop_on_shutdown").as_bool();
  position_min_rad_ = get_parameter("position_min_rad").as_double();
  position_max_rad_ = get_parameter("position_max_rad").as_double();
  velocity_min_rad_s_ = get_parameter("velocity_min_rad_s").as_double();
  velocity_max_rad_s_ = get_parameter("velocity_max_rad_s").as_double();
  torque_min_nm_ = get_parameter("torque_min_nm").as_double();
  torque_max_nm_ = get_parameter("torque_max_nm").as_double();
}

uint8_t RobstrideVelocityNode::ClampCanId(int value) const
{
  return static_cast<uint8_t>(std::clamp(value, 0, 0xFF));
}

void RobstrideVelocityNode::OpenCanSocket()
{
  // このnodeはcan_msgsを経由せず、Linux SocketCANへ直接write/readする。
  socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    throw std::runtime_error("failed to open CAN socket");
  }

  ifreq ifr {};
  std::snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "%s", can_interface_.c_str());
  if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    close(socket_fd_);
    socket_fd_ = -1;
    throw std::runtime_error("failed to get CAN interface index");
  }

  sockaddr_can address {};
  address.can_family = AF_CAN;
  address.can_ifindex = ifr.ifr_ifindex;

  if (bind(socket_fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
    close(socket_fd_);
    socket_fd_ = -1;
    throw std::runtime_error("failed to bind CAN socket");
  }
}

void RobstrideVelocityNode::SetupRosInterfaces()
{
  enable_subscription_ = create_subscription<std_msgs::msg::Bool>(
    "~/enable",
    10,
    std::bind(&RobstrideVelocityNode::OnEnable, this, std::placeholders::_1));
  stop_subscription_ = create_subscription<std_msgs::msg::Empty>(
    "~/stop",
    10,
    std::bind(&RobstrideVelocityNode::OnStop, this, std::placeholders::_1));
  set_mode_subscription_ = create_subscription<std_msgs::msg::UInt8>(
    "~/set_mode",
    10,
    std::bind(&RobstrideVelocityNode::OnSetMode, this, std::placeholders::_1));
  velocity_subscription_ = create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/velocity_command",
    10,
    std::bind(&RobstrideVelocityNode::OnVelocityCommand, this, std::placeholders::_1));

  status_publisher_ = create_publisher<std_msgs::msg::String>("~/status", 10);
  fault_publisher_ = create_publisher<std_msgs::msg::UInt32>("~/fault", 10);
  mode_status_publisher_ = create_publisher<std_msgs::msg::UInt8>("~/mode_status", 10);
  joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
    "/joint_states",
    10);
}

void RobstrideVelocityNode::StartReceiveThread()
{
  running_ = true;
  receive_thread_ = std::thread(
    [this]() {
      ReceiveLoop();
    });
}

void RobstrideVelocityNode::ReceiveLoop()
{
  while (running_) {
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(socket_fd_, &read_fds);

    timeval timeout {};
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000;

    const int ready = select(socket_fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
    if (ready <= 0) {
      continue;
    }

    can_frame frame {};
    const ssize_t nbytes = read(socket_fd_, &frame, sizeof(frame));
    if (nbytes == static_cast<ssize_t>(sizeof(frame))) {
      HandleCanFrame(frame);
    }
  }
}

void RobstrideVelocityNode::HandleCanFrame(const can_frame & frame)
{
  if ((frame.can_id & CAN_EFF_FLAG) == 0 || frame.can_dlc < kCanDlc) {
    return;
  }

  const canid_t id = frame.can_id & CAN_EFF_MASK;
  const uint8_t frame_type = static_cast<uint8_t>((id >> 24) & 0x1F);
  const uint8_t frame_target = static_cast<uint8_t>(id & 0xFF);
  const uint8_t frame_motor = static_cast<uint8_t>((id >> 8) & 0xFF);
  if (frame_type != kTypeFeedback || frame_target != host_id_ || frame_motor != motor_id_) {
    return;
  }

  const uint16_t raw_position = (static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1];
  const uint16_t raw_speed = (static_cast<uint16_t>(frame.data[2]) << 8) | frame.data[3];
  const uint16_t raw_torque = (static_cast<uint16_t>(frame.data[4]) << 8) | frame.data[5];
  const uint16_t raw_temperature = (static_cast<uint16_t>(frame.data[6]) << 8) | frame.data[7];

  const double position = UintToFloat(raw_position, position_min_rad_, position_max_rad_);
  const double velocity = UintToFloat(raw_speed, velocity_min_rad_s_, velocity_max_rad_s_);
  const double torque = UintToFloat(raw_torque, torque_min_nm_, torque_max_nm_);
  const double temperature = static_cast<double>(raw_temperature) / 10.0;
  const uint32_t fault = static_cast<uint32_t>((id >> 16) & 0x3F);
  const uint8_t mode_status = static_cast<uint8_t>((id >> 22) & 0x03);

  std_msgs::msg::UInt32 fault_msg;
  fault_msg.data = fault;
  fault_publisher_->publish(fault_msg);

  std_msgs::msg::UInt8 mode_status_msg;
  mode_status_msg.data = mode_status;
  mode_status_publisher_->publish(mode_status_msg);

  sensor_msgs::msg::JointState joint_state;
  joint_state.header.stamp = get_clock()->now();
  joint_state.name = {joint_name_};
  joint_state.position = {position};
  joint_state.velocity = {velocity};
  joint_state.effort = {torque};
  joint_state_publisher_->publish(joint_state);

  std_msgs::msg::String status_msg;
  status_msg.data =
    "{\"protocol\":\"private\",\"position_rad\":" + std::to_string(position) +
    ",\"velocity_rad_s\":" + std::to_string(velocity) +
    ",\"torque_nm\":" + std::to_string(torque) +
    ",\"temperature_c\":" + std::to_string(temperature) +
    ",\"fault\":" + std::to_string(fault) +
    ",\"mode_status\":" + std::to_string(mode_status) + "}";
  status_publisher_->publish(status_msg);
}

canid_t RobstrideVelocityNode::CreatePrivateCanId(uint8_t frame_type, uint16_t data) const
{
  return ((static_cast<canid_t>(frame_type) & 0x1F) << 24) |
         ((static_cast<canid_t>(data) & 0xFFFF) << 8) |
         (static_cast<canid_t>(motor_id_) & 0xFF);
}

double RobstrideVelocityNode::UintToFloat(uint16_t value, double min, double max) const
{
  return static_cast<double>(value) * (max - min) / 65535.0 + min;
}

uint16_t RobstrideVelocityNode::FloatToUint(double value, double min, double max) const
{
  const double clamped = std::clamp(value, min, max);
  return static_cast<uint16_t>((clamped - min) * 65535.0 / (max - min));
}

void RobstrideVelocityNode::SendEnable()
{
  can_frame frame {};
  frame.can_id = CreatePrivateCanId(kTypeEnable, host_id_) | CAN_EFF_FLAG;
  frame.can_dlc = kCanDlc;
  std::fill(std::begin(frame.data), std::end(frame.data), 0);
  WriteCanFrame(frame, "enable");
}

void RobstrideVelocityNode::SendStop()
{
  can_frame frame {};
  frame.can_id = CreatePrivateCanId(kTypeStop, host_id_) | CAN_EFF_FLAG;
  frame.can_dlc = kCanDlc;
  std::fill(std::begin(frame.data), std::end(frame.data), 0);
  WriteCanFrame(frame, "stop");
}

void RobstrideVelocityNode::SendSetMode(uint8_t run_mode)
{
  WriteUint8Parameter(kRunModeIndex, run_mode);
}

void RobstrideVelocityNode::SendVelocityCommand(double speed_rad_s, double current_limit_a)
{
  WriteFloatParameter(kSpeedRefIndex, speed_rad_s);
  WriteFloatParameter(kCurrentLimitIndex, current_limit_a);
}

void RobstrideVelocityNode::WriteFloatParameter(uint16_t index, double value)
{
  can_frame frame {};
  frame.can_id = CreatePrivateCanId(kTypeWriteParameter, host_id_) | CAN_EFF_FLAG;
  frame.can_dlc = kCanDlc;
  frame.data[0] = index & 0xFF;
  frame.data[1] = (index >> 8) & 0xFF;
  frame.data[2] = 0;
  frame.data[3] = 0;
  WriteFloatLe(&frame.data[4], value);
  WriteCanFrame(frame, "write float parameter");
}

void RobstrideVelocityNode::WriteUint8Parameter(uint16_t index, uint8_t value)
{
  can_frame frame {};
  frame.can_id = CreatePrivateCanId(kTypeWriteParameter, host_id_) | CAN_EFF_FLAG;
  frame.can_dlc = kCanDlc;
  frame.data[0] = index & 0xFF;
  frame.data[1] = (index >> 8) & 0xFF;
  frame.data[2] = 0;
  frame.data[3] = 0;
  frame.data[4] = value;
  frame.data[5] = 0;
  frame.data[6] = 0;
  frame.data[7] = 0;
  WriteCanFrame(frame, "write uint8 parameter");
}

void RobstrideVelocityNode::WriteFloatLe(uint8_t * data, double value) const
{
  const auto float_value = static_cast<float>(value);
  std::memcpy(data, &float_value, sizeof(float));
}

void RobstrideVelocityNode::OnEnable(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    SendEnable();
  } else {
    SendStop();
  }
}

void RobstrideVelocityNode::OnStop(const std_msgs::msg::Empty::SharedPtr)
{
  SendStop();
}

void RobstrideVelocityNode::OnSetMode(const std_msgs::msg::UInt8::SharedPtr msg)
{
  SendSetMode(msg->data);
}

void RobstrideVelocityNode::OnVelocityCommand(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() != 2) {
    RCLCPP_WARN(get_logger(), "velocity_command expects [speed_rad_s, current_limit_a]");
    return;
  }

  const double speed_rad_s = msg->data[0];
  current_limit_a_ = std::max(0.0, msg->data[1]);
  SendVelocityCommand(speed_rad_s, current_limit_a_);
}

void RobstrideVelocityNode::WriteCanFrame(const can_frame & frame, const char * label) const
{
  const ssize_t nbytes = write(socket_fd_, &frame, sizeof(frame));
  if (nbytes != static_cast<ssize_t>(sizeof(frame))) {
    RCLCPP_WARN(get_logger(), "Failed to write CAN frame: %s", label);
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobstrideVelocityNode>());
  rclcpp::shutdown();
  return 0;
}
