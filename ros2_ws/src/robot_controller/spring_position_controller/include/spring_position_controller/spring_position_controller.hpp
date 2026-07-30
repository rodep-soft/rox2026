#ifndef SPRING_POSITION_CONTROLLER__SPRING_POSITION_CONTROLLER_HPP_
#define SPRING_POSITION_CONTROLLER__SPRING_POSITION_CONTROLLER_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

class SpringPositionController : public rclcpp::Node
{
public:
  SpringPositionController();

private:
  enum class OperationMode : uint8_t
  {
    STOP,
    DRIVE,
    SHOT_CYCLE,
    BELT_ONLY,
  };

  enum class Position : uint8_t
  {
    DRIBBLE,
    INTAKE,
    SHOOT,
    OPEN,
  };

  enum class SpringState : uint8_t
  {
    READY,
    LOAD,
    FIRE,
    ERROR,
  };

  enum class ShotCycleState : uint8_t
  {
    IDLE,
    INTAKE,
    SHOOT,
    HOLD,
    RETURN,
  };

  void declare_parameters();
  void get_parameters();
  void validate_parameters();
  void create_interfaces();

  void operation_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg);
  void fire_request_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void limit_switch_callback(
    const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
  void position_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg);
  void shot_cycle_start_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void position_feedback_callback(const std_msgs::msg::Float32::SharedPtr msg);
  void spring_timer_callback();
  void position_timer_callback();

  bool spring_fire_allowed() const;
  bool manual_position_allowed() const;
  void prepare_spring_for_stop();
  void start_loading();
  void start_fire();
  double position_to_rad(Position position) const;
  void set_target_position(Position position);
  void stop_shot_cycle();
  void handle_position_timeout();
  void finish_position_move();
  void publish_shot_cycle_complete();
  void publish_shot_cycle_running(bool running);

  bool configuration_valid_{true};
  bool emergency_stop_active_{false};
  bool is_loaded_{false};
  bool previous_fire_request_{false};
  bool fire_pending_{false};
  bool position_moving_{false};
  bool shot_cycle_running_{false};
  OperationMode operation_mode_{OperationMode::STOP};
  SpringState spring_state_{SpringState::LOAD};
  Position target_position_{Position::DRIBBLE};
  ShotCycleState shot_cycle_state_{ShotCycleState::IDLE};

  int limit_switch_index_{0};
  double loading_velocity_rad_s_{0.0};
  double fire_velocity_rad_s_{0.0};
  double fire_duration_sec_{0.0};
  double load_timeout_sec_{5.0};
  int spring_command_period_ms_{10};
  double dribble_position_rad_{0.0};
  double intake_position_rad_{0.0};
  double shoot_position_rad_{0.0};
  double open_position_rad_{0.0};
  double position_tolerance_rad_{0.02};
  double shoot_to_dribble_delay_sec_{1.0};
  double move_timeout_sec_{3.0};
  double feedback_timeout_sec_{0.5};
  int position_command_period_ms_{20};
  int qos_depth_{1};
  double target_position_rad_{0.0};

  rclcpp::Time fire_start_time_;
  rclcpp::Time load_start_time_;
  rclcpp::Time position_phase_start_time_;
  rclcpp::Time last_position_feedback_time_;

  std::string operation_mode_topic_;
  std::string shot_cycle_complete_topic_;
  std::string fire_request_topic_;
  std::string emergency_stop_topic_;
  std::string limit_switch_topic_;
  std::string spring_velocity_command_topic_;
  std::string position_mode_topic_;
  std::string shot_cycle_start_topic_;
  std::string shot_cycle_running_topic_;
  std::string position_command_topic_;
  std::string position_feedback_topic_;

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr
    operation_mode_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    fire_request_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    emergency_stop_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr
    limit_switch_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr
    position_mode_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    shot_cycle_start_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
    position_feedback_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
    spring_velocity_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
    position_command_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
    shot_cycle_complete_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
    shot_cycle_running_publisher_;
  rclcpp::TimerBase::SharedPtr spring_timer_;
  rclcpp::TimerBase::SharedPtr position_timer_;
};

#endif  // SPRING_POSITION_CONTROLLER__SPRING_POSITION_CONTROLLER_HPP_
