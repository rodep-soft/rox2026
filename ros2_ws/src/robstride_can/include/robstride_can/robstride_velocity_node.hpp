#pragma once

#include <linux/can.h>

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int32.hpp>

class RobstrideVelocityNode : public rclcpp::Node
{
public:
  RobstrideVelocityNode();
  ~RobstrideVelocityNode() override;

private:
  void DeclareParameters();
  void LoadParameters();
  uint8_t ClampCanId(int value) const;

  void OpenCanSocket();
  void SetupRosInterfaces();
  void StartReceiveThread();
  void ReceiveLoop();
  void HandleCanFrame(const can_frame & frame);

  canid_t CreatePrivateCanId(uint8_t frame_type, uint16_t data) const;
  double UintToFloat(uint16_t value, double min, double max) const;
  uint16_t FloatToUint(double value, double min, double max) const;

  void SendEnable();
  void SendStop();
  void SendSetMode(uint8_t run_mode);
  void SendVelocityCommand(double speed_rad_s, double current_limit_a);
  void WriteFloatParameter(uint16_t index, double value);
  void WriteUint8Parameter(uint16_t index, uint8_t value);
  void WriteFloatLe(uint8_t * data, double value) const;
  void WriteCanFrame(const can_frame & frame, const char * label) const;

  void OnEnable(const std_msgs::msg::Bool::SharedPtr msg);
  void OnStop(const std_msgs::msg::Empty::SharedPtr msg);
  void OnSetMode(const std_msgs::msg::UInt8::SharedPtr msg);
  void OnVelocityCommand(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  std::string can_interface_;
  uint8_t motor_id_;
  uint8_t host_id_;
  std::string joint_name_;
  double current_limit_a_;
  bool auto_enable_;
  bool stop_on_shutdown_;
  double position_min_rad_;
  double position_max_rad_;
  double velocity_min_rad_s_;
  double velocity_max_rad_s_;
  double torque_min_nm_;
  double torque_max_nm_;

  int socket_fd_ = -1;
  std::atomic_bool running_ {false};
  std::thread receive_thread_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr set_mode_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr fault_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mode_status_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
};
