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
  double startup_current_a;
  double rpm_control_threshold_rpm;
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
  double rpm_command{0.0};
  float measured_rpm{std::numeric_limits<float>::quiet_NaN()};
  float measured_current_a{std::numeric_limits<float>::quiet_NaN()};
  bool command_received{false};
  bool feedback_received{false};
  bool rpm_control_active{false};
  std::chrono::steady_clock::time_point last_feedback_time{};
  std::chrono::steady_clock::time_point last_ramp_update_time{};
};

class Node : public rclcpp::Node
{
public:
  explicit Node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("vesc_driver", options)
  {
    const auto can_tx_topic =
      declare_parameter<std::string>("can_tx_topic", "/socketcan_bridge/tx");
    const auto can_rx_topic =
      declare_parameter<std::string>("can_rx_topic", "/socketcan_bridge/rx");
    const auto target_topic = declare_parameter<std::string>("target_topic", "/vesc/target");
    const auto target_array_topic = declare_parameter<std::string>(
      "target_array_topic",
      "/vesc/target_array");
    const auto state_topic = declare_parameter<std::string>("state_topic", "/vesc/state");
    const auto state_array_topic = declare_parameter<std::string>(
      "state_array_topic",
      "/vesc/state_array");
    const auto update_period_ms = declare_parameter<int64_t>("update_period_ms", 20);
    const auto state_array_publish_period_ms = declare_parameter<int64_t>(
      "state_array_publish_period_ms", 100);
    const auto motor_names = declare_parameter<std::vector<std::string>>(
      "motors", std::vector<std::string>{});

    for (const auto & name : motor_names) {
      const auto prefix = name + ".";
      const auto logical_id = declare_parameter<int64_t>(prefix + "logical_id", -1);
      const auto controller_id =
        declare_parameter<int64_t>(prefix + "controller_id", -1);
      const auto feedback_timeout_ms = declare_parameter<int64_t>(
        prefix + "feedback_timeout_ms",
        500);
      const auto max_rpm = declare_parameter<double>(prefix + "max_rpm", 5600.0);
      const auto rpm_slew_rate = declare_parameter<double>(prefix + "rpm_slew_rate", 4000.0);
      const auto startup_current_a = declare_parameter<double>(prefix + "startup_current_a", 5.0);
      const auto rpm_control_threshold_rpm = declare_parameter<double>(
        prefix + "rpm_control_threshold_rpm", 1000.0);
      if (logical_id < 0 || logical_id > 65535) {
        throw std::runtime_error(name + ": logical_id must be in [0, 65535]");
      }
      if (controller_id < 0 || controller_id > 255) {
        throw std::runtime_error(name + ": controller_id must be in [0, 255]");
      }

      motors_.emplace_back(
        MotorConfig{
          static_cast<uint16_t>(logical_id),
          static_cast<uint8_t>(controller_id), max_rpm, rpm_slew_rate,
          startup_current_a, rpm_control_threshold_rpm,
          std::chrono::milliseconds(feedback_timeout_ms)});
    }

    const auto can_tx_qos = rclcpp::QoS(rclcpp::KeepLast(50)).reliable().durability_volatile();
    const auto can_rx_qos = rclcpp::SensorDataQoS().keep_last(50);
    rclcpp::SubscriptionOptions can_subscription_options;
    for (std::size_t index = 0; index < motors_.size(); ++index) {
      if (!can_subscription_options.content_filter_options.filter_expression.empty()) {
        can_subscription_options.content_filter_options.filter_expression += " OR ";
      }
      can_subscription_options.content_filter_options.filter_expression +=
        "id = %" + std::to_string(index);
      can_subscription_options.content_filter_options.expression_parameters.push_back(
        std::to_string(
          (protocol::STATUS_1_ID << 8) | motors_[index].config.controller_id));
    }

    can_publisher_ = create_publisher<can_msgs::msg::Frame>(can_tx_topic, can_tx_qos);
    can_subscription_ = create_subscription<can_msgs::msg::Frame>(
      can_rx_topic, can_rx_qos,
      std::bind(&Node::can_callback, this, std::placeholders::_1),
      can_subscription_options);
    target_subscription_ = create_subscription<actuator_msgs::msg::ActuatorTarget>(
      target_topic, 10, std::bind(&Node::target_callback, this, std::placeholders::_1));
    target_array_subscription_ =
      create_subscription<actuator_msgs::msg::ActuatorTargetArray>(
      target_array_topic, 10,
      std::bind(&Node::target_array_callback, this, std::placeholders::_1));
    state_publisher_ = create_publisher<actuator_msgs::msg::ActuatorState>(state_topic, 10);
    state_array_publisher_ =
      create_publisher<actuator_msgs::msg::ActuatorStateArray>(state_array_topic, 10);
    state_array_message_.actuators.reserve(motors_.size());
    command_timer_ = create_wall_timer(
      std::chrono::milliseconds(update_period_ms),
      std::bind(&Node::command_timer_callback, this));
    state_timer_ = create_wall_timer(
      std::chrono::milliseconds(state_array_publish_period_ms),
      std::bind(&Node::state_timer_callback, this));
  }

  ~Node() override
  {
    for (const auto & motor : motors_) {
      can_publisher_->publish(
        protocol::make_set_current_frame(motor.config.controller_id, 0.0));
    }
  }

private:
  /// @brief 論理IDからモーターを検索
  /// @param logical_id 論理ID
  /// @return motorへのポインタ(見つからなければnullptr)
  Motor * find_by_logical_id(uint16_t logical_id)
  {
    const auto iterator = std::find_if(
      motors_.begin(), motors_.end(), [logical_id](const Motor & motor) {
        return motor.config.logical_id == logical_id;
      });
    return iterator == motors_.end() ? nullptr : &*iterator;
  }

  /// @brief controller IDからモーターを検索
  /// @param controller_id Controller ID
  /// @return motorへのポインタ(見つからなければnullptr)
  Motor * find_by_controller_id(uint8_t controller_id)
  {
    const auto iterator = std::find_if(
      motors_.begin(), motors_.end(), [controller_id](const Motor & motor) {
        return motor.config.controller_id == controller_id;
      });
    return iterator == motors_.end() ? nullptr : &*iterator;
  }

  /// @brief モーターが接続されているかを判定
  /// @param motor モーター
  /// @param now 現在時刻
  /// @return 接続されている場合はtrue、それ以外はfalse
  bool is_connected(const Motor & motor, std::chrono::steady_clock::time_point now) const
  {
    if (!motor.feedback_received) {
      return false;
    }
    const auto feedback_age = now - motor.last_feedback_time;
    return feedback_age <= motor.config.feedback_timeout;
  }

  /// @brief モーターの状態メッセージを作成
  /// @param motor モーター
  /// @param now 現在時刻
  /// @return 状態メッセージ
  actuator_msgs::msg::ActuatorState make_state(
    const Motor & motor,
    std::chrono::steady_clock::time_point now) const
  {
    actuator_msgs::msg::ActuatorState message;
    message.logical_id = motor.config.logical_id;
    message.position_reference_set = false;
    message.velocity = motor.measured_rpm;
    message.torque_nm = std::numeric_limits<float>::quiet_NaN();
    message.current_a = motor.measured_current_a;
    message.state = is_connected(motor, now) ?
      actuator_msgs::msg::ActuatorState::STATE_READY :
      actuator_msgs::msg::ActuatorState::STATE_OFFLINE;
    return message;
  }

  /// @brief 目標回転速度を設定する
  /// @param motor　設定するモータ
  /// @param target_rpm 目標回転速度
  void set_target(Motor & motor, float target_rpm)
  {
    const auto previous_target_rpm = motor.target_rpm;
    motor.target_rpm = std::clamp(
      static_cast<double>(target_rpm), -motor.config.max_rpm, motor.config.max_rpm);

    const bool stopping = motor.target_rpm == 0.0;
    const bool starting_or_reversing = previous_target_rpm * motor.target_rpm <= 0.0;
    if (stopping || starting_or_reversing) {
      motor.rpm_control_active = false;
    }
    motor.command_received = true;
  }

  /// @brief モータへ電流もしくは速度の指令を送る関数
  /// @param motor
  /// @param now 現在時刻
  void publish_motor_command(Motor & motor, std::chrono::steady_clock::time_point now)
  {
    if (!motor.command_received) {
      return;
    }
    const auto desired_rpm = motor.target_rpm;
    if (desired_rpm == 0.0) { // 停止指令
      motor.rpm_control_active = false;
      motor.rpm_command = 0.0;
      can_publisher_->publish(protocol::make_set_current_frame(motor.config.controller_id, 0.0));
      return;
    }

    const auto motor_pole_pairs = static_cast<double>(protocol::MOTOR_POLES) / 2.0;
    const auto rpm_control_start_rpm =
      std::min(motor.config.rpm_control_threshold_rpm, std::abs(desired_rpm));
    const auto measured_rpm = static_cast<double>(motor.measured_rpm);
    const bool rotating_in_target_direction = desired_rpm * measured_rpm > 0.0;
    const bool rpm_control_start_reached = rotating_in_target_direction &&
      std::abs(measured_rpm) >= rpm_control_start_rpm;
    const bool direct_rpm_control = motor.config.rpm_control_threshold_rpm <= 0.0;

    if (!motor.rpm_control_active &&
      (direct_rpm_control || (motor.feedback_received && rpm_control_start_reached)))
    {
      motor.rpm_control_active = true;
      motor.rpm_command = direct_rpm_control ? 0.0 : measured_rpm;
    }

    // モーターが停止している場合は、まずは電流制御で回転させる
    if (!motor.rpm_control_active) {
      const auto startup_current_a = std::copysign(motor.config.startup_current_a, desired_rpm);
      can_publisher_->publish(
        protocol::make_set_current_frame(
          motor.config.controller_id,
          startup_current_a));
      return;
    }

    // RPM制御を行う場合は、目標RPMに向かってスロープ制御を行う
    // elapsed_secondsは、前回のスロープ制御からの経過時間を秒単位で表す
    const auto elapsed_seconds =
      std::chrono::duration<double>(now - motor.last_ramp_update_time).count();
    const auto maximum_step = motor.config.rpm_slew_rate * elapsed_seconds;
    motor.rpm_command += std::clamp(desired_rpm - motor.rpm_command, -maximum_step, maximum_step);
    const auto erpm_command = motor.rpm_command * motor_pole_pairs;
    can_publisher_->publish(
      protocol::make_set_rpm_frame(
        motor.config.controller_id,
        static_cast<int32_t>(std::lround(erpm_command))));
  }

  /// @brief CANフレーム受信時のコールバック関数(即時にモーターの状態を配信する)
  /// @param frame
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
    motor->measured_rpm =
      static_cast<float>(status.erpm / (static_cast<double>(protocol::MOTOR_POLES) / 2.0));
    motor->measured_current_a = status.current_a;
    motor->last_feedback_time = std::chrono::steady_clock::now();
    motor->feedback_received = true;

    state_publisher_->publish(make_state(*motor, motor->last_feedback_time));
  }

  /// @brief 目標値が送られてきた場合のコールバック関数
  /// @param message
  void target_callback(const actuator_msgs::msg::ActuatorTarget::SharedPtr message)
  {
    if (auto * motor = find_by_logical_id(message->logical_id)) {
      set_target(*motor, message->target);
    }
  }

  /// @brief 目標値が複数送られてきた場合のコールバック関数
  /// @param message
  void target_array_callback(
    const actuator_msgs::msg::ActuatorTargetArray::SharedPtr message)
  {
    for (const auto & target : message->actuators) {
      if (auto * motor = find_by_logical_id(target.logical_id)) {
        set_target(*motor, target.target);
      }
    }
  }

  /// @brief 一定周期で各VESCへ指令を送信する
  void command_timer_callback()
  {
    const auto now = std::chrono::steady_clock::now();
    for (auto & motor : motors_) {
      publish_motor_command(motor, now);
      motor.last_ramp_update_time = now;
    }
  }

  /// @brief 一定周期で全VESCの状態を配信する
  void state_timer_callback()
  {
    const auto now = std::chrono::steady_clock::now();
    state_array_message_.header.stamp = this->now();
    state_array_message_.actuators.clear();

    for (const auto & motor : motors_) {
      state_array_message_.actuators.push_back(make_state(motor, now));
    }
    state_array_publisher_->publish(state_array_message_);
  }

  std::vector<Motor> motors_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_publisher_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_subscription_;

  rclcpp::Subscription<actuator_msgs::msg::ActuatorTarget>::SharedPtr target_subscription_;
  rclcpp::Subscription<actuator_msgs::msg::ActuatorTargetArray>::SharedPtr
    target_array_subscription_;

  rclcpp::Publisher<actuator_msgs::msg::ActuatorState>::SharedPtr state_publisher_;
  rclcpp::Publisher<actuator_msgs::msg::ActuatorStateArray>::SharedPtr state_array_publisher_;
  actuator_msgs::msg::ActuatorStateArray state_array_message_;

  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::TimerBase::SharedPtr state_timer_;
};
}  // namespace vesc_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vesc_driver::Node>());
  rclcpp::shutdown();
  return 0;
}
