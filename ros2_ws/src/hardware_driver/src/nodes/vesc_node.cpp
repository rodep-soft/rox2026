#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "actuator_msgs/msg/actuator_state.hpp"
#include "actuator_msgs/msg/actuator_state_array.hpp"
#include "actuator_msgs/msg/actuator_target.hpp"
#include "actuator_msgs/msg/actuator_target_array.hpp"
#include "can_msgs/msg/frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vesc_driver/vesc_protocol.hpp"

namespace vesc_driver
{
struct MotorConfig
{
  uint16_t logical_id;
  uint8_t controller_id;
  double max_rpm;
  double rpm_slew_rate;
  std::chrono::milliseconds command_timeout;
  std::chrono::milliseconds feedback_timeout;
};

struct Motor
{
  explicit Motor(MotorConfig motor_config)
  : config(std::move(motor_config)),
    last_ramp_update_time(std::chrono::steady_clock::now())
  {}

  MotorConfig config;
  double target_rpm{0.0};
  double commanded_rpm{0.0};
  float current_rpm{std::numeric_limits<float>::quiet_NaN()};
  bool command_received{false};
  bool feedback_received{false};
  std::chrono::steady_clock::time_point last_command_time{};
  std::chrono::steady_clock::time_point last_feedback_time{};
  std::chrono::steady_clock::time_point last_ramp_update_time{};
};

class Node : public rclcpp::Node
{
public:
  explicit Node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("vesc_driver", options)
  {
    can_tx_topic_ = declare_parameter<std::string>("can_tx_topic", "/socketcan_bridge/tx");
    can_rx_topic_ = declare_parameter<std::string>("can_rx_topic", "/socketcan_bridge/rx");
    target_topic_ = declare_parameter<std::string>("target_topic", "/vesc/target");
    target_array_topic_ = declare_parameter<std::string>("target_array_topic", "/vesc/target_array");
    state_topic_ = declare_parameter<std::string>("state_topic", "/vesc/state");
    state_array_topic_ = declare_parameter<std::string>("state_array_topic", "/vesc/state_array");
    const auto update_period_ms = declare_parameter<int64_t>("update_period_ms", 20);
    const auto motor_names = declare_parameter<std::vector<std::string>>("motors", std::vector<std::string>{});

    for (const auto & name : motor_names) {
      const auto prefix = name + ".";
      const auto logical_id = declare_parameter<int64_t>(prefix + "logical_id", -1);
      const auto controller_id = declare_parameter<int64_t>(prefix + "controller_id", -1);
      const auto command_timeout_ms =
        declare_parameter<int64_t>(prefix + "command_timeout_ms", 500);
      const auto feedback_timeout_ms =
        declare_parameter<int64_t>(prefix + "feedback_timeout_ms", 500);
      const auto max_rpm = declare_parameter<double>(prefix + "max_rpm", 5600.0);
      const auto rpm_slew_rate =
        declare_parameter<double>(prefix + "rpm_slew_rate", 4000.0);

      if (logical_id < 0 || logical_id > 65535) {
        throw std::runtime_error(name + ": logical_id must be in [0, 65535]");
      }
      if (controller_id < 0 || controller_id > 255) {
        throw std::runtime_error(name + ": controller_id must be in [0, 255]");
      }
      
      const auto duplicate_logical_id = std::any_of(
        motors_.cbegin(), motors_.cend(), [logical_id](const Motor & motor) {
          return motor.config.logical_id == static_cast<uint16_t>(logical_id);
        });
      const auto duplicate_controller_id = std::any_of(
        motors_.cbegin(), motors_.cend(), [controller_id](const Motor & motor) {
          return motor.config.controller_id == static_cast<uint8_t>(controller_id);
        });
      if (duplicate_logical_id || duplicate_controller_id) {
        throw std::runtime_error(name + ": logical_id and controller_id must be unique");
      }

      motors_.emplace_back(MotorConfig{
        name,
        static_cast<uint16_t>(logical_id),
        static_cast<uint8_t>(controller_id),
        max_rpm,
        rpm_slew_rate,
        std::chrono::milliseconds(command_timeout_ms),
        std::chrono::milliseconds(feedback_timeout_ms)});
    }

    const auto can_tx_qos = rclcpp::QoS(rclcpp::KeepLast(50)).reliable().durability_volatile();
    const auto can_rx_qos = rclcpp::SensorDataQoS().keep_last(50);
    can_publisher_ = create_publisher<can_msgs::msg::Frame>(can_tx_topic_, can_tx_qos);
    can_subscription_ = create_subscription<can_msgs::msg::Frame>(
      can_rx_topic_, can_rx_qos,
      std::bind(&Node::can_callback, this, std::placeholders::_1));
    target_subscription_ = create_subscription<actuator_msgs::msg::ActuatorTarget>(
      target_topic_, 10, std::bind(&Node::target_callback, this, std::placeholders::_1));
    target_array_subscription_ =
      create_subscription<actuator_msgs::msg::ActuatorTargetArray>(
      target_array_topic_, 10,
      std::bind(&Node::target_array_callback, this, std::placeholders::_1));
    state_publisher_ = create_publisher<actuator_msgs::msg::ActuatorState>(state_topic_, 10);
    state_array_publisher_ =
      create_publisher<actuator_msgs::msg::ActuatorStateArray>(state_array_topic_, 10);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(update_period_ms), std::bind(&Node::timer_callback, this));
  }

private:
  Motor * find_by_logical_id(uint16_t logical_id)
  {
    const auto iterator = std::find_if(
      motors_.begin(), motors_.end(), [logical_id](const Motor & motor) {
        return motor.config.logical_id == logical_id;
      });
    return iterator == motors_.end() ? nullptr : &*iterator;
  }

  Motor * find_by_controller_id(uint8_t controller_id)
  {
    const auto iterator = std::find_if(
      motors_.begin(), motors_.end(), [controller_id](const Motor & motor) {
        return motor.config.controller_id == controller_id;
      });
    return iterator == motors_.end() ? nullptr : &*iterator;
  }

  bool is_connected(const Motor & motor, std::chrono::steady_clock::time_point now) const
  {
    return motor.feedback_received && now - motor.last_feedback_time <= motor.config.feedback_timeout;
  }

  actuator_msgs::msg::ActuatorState make_state(
    const Motor & motor, std::chrono::steady_clock::time_point now) const
  {
    actuator_msgs::msg::ActuatorState message;
    message.logical_id = motor.config.logical_id;
    message.connected = is_connected(motor, now);
    message.configured = true;
    message.enabled = motor.command_received;
    message.position_reference_set = false;
    message.velocity = std::isfinite(motor.current_rpm) ? motor.current_rpm : 0.0f;
    message.state = message.connected ?
      actuator_msgs::msg::ActuatorState::STATE_READY :
      actuator_msgs::msg::ActuatorState::STATE_OFFLINE;
    return message;
  }

  void set_target(Motor & motor, float target_rpm)
  {
    if (!std::isfinite(target_rpm)) {
      return;
    }
    motor.target_rpm = std::clamp(
      static_cast<double>(target_rpm), -motor.config.max_rpm, motor.config.max_rpm);
    motor.last_command_time = std::chrono::steady_clock::now();
    motor.command_received = true;
  }

  void can_callback(const can_msgs::msg::Frame::SharedPtr frame)
  {
    protocol::Status1 status{};
    if (!protocol::decode_status_1(*frame, status)) {
      return;
    }
    auto * motor = find_by_controller_id(status.controller_id);
    if (motor == nullptr) {
      return;
    }
    motor->current_rpm = static_cast<float>(
      status.erpm / (static_cast<double>(protocol::MOTOR_POLES) / 2.0));
    motor->last_feedback_time = std::chrono::steady_clock::now();
    motor->feedback_received = true;
    state_publisher_->publish(make_state(*motor, motor->last_feedback_time));
  }

  void target_callback(const actuator_msgs::msg::ActuatorTarget::SharedPtr message)
  {
    if (auto * motor = find_by_logical_id(message->logical_id)) {
      set_target(*motor, message->target);
    }
  }

  void target_array_callback(
    const actuator_msgs::msg::ActuatorTargetArray::SharedPtr message)
  {
    for (const auto & target : message->actuators) {
      if (auto * motor = find_by_logical_id(target.logical_id)) {
        set_target(*motor, target.target);
      }
    }
  }


  /// @brief 一定周期で送受信がされるように調整するコールバック関数
  void timer_callback()
  {
    const auto now = std::chrono::steady_clock::now();
    actuator_msgs::msg::ActuatorStateArray state_array;
    state_array.header.stamp = this->now();
    state_array.actuators.reserve(motors_.size());

    for (auto & motor : motors_) {
      if (motor.command_received) {
        const auto desired_rpm = now - motor.last_command_time > motor.config.command_timeout ?
          0.0 : motor.target_rpm;
        const auto elapsed_seconds =
          std::chrono::duration<double>(now - motor.last_ramp_update_time).count();
        const auto maximum_step = motor.config.rpm_slew_rate * elapsed_seconds;
        motor.commanded_rpm += std::clamp(
          desired_rpm - motor.commanded_rpm, -maximum_step, maximum_step);
        const auto erpm = motor.commanded_rpm *
          (static_cast<double>(protocol::MOTOR_POLES) / 2.0);
        can_publisher_->publish(
          protocol::make_set_rpm_frame(
            motor.config.controller_id, static_cast<int32_t>(std::lround(erpm))));
      }
      motor.last_ramp_update_time = now;
      state_array.actuators.push_back(make_state(motor, now));
    }
    state_array_publisher_->publish(state_array);
  }

  std::vector<Motor> motors_;
  std::string can_tx_topic_;
  std::string can_rx_topic_;
  std::string target_topic_;
  std::string target_array_topic_;
  std::string state_topic_;
  std::string state_array_topic_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_publisher_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_subscription_;
  rclcpp::Subscription<actuator_msgs::msg::ActuatorTarget>::SharedPtr target_subscription_;
  rclcpp::Subscription<actuator_msgs::msg::ActuatorTargetArray>::SharedPtr
    target_array_subscription_;
  rclcpp::Publisher<actuator_msgs::msg::ActuatorState>::SharedPtr state_publisher_;
  rclcpp::Publisher<actuator_msgs::msg::ActuatorStateArray>::SharedPtr state_array_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace vesc_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vesc_driver::Node>());
  rclcpp::shutdown();
  return 0;
}