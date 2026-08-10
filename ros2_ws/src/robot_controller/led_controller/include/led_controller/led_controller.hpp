#ifndef LED_CONTROLLER__LED_CONTROLLER_HPP_
#define LED_CONTROLLER__LED_CONTROLLER_HPP_

#include <chrono>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int16.hpp"

class LedControllerNode : public rclcpp::Node
{
public:
  LedControllerNode();

private:
  enum class DisplayMode : uint8_t
  {
    STARTUP = 0,
    READY = 1,
    EMERGENCY_STOP = 2,
    SHOT_OPENING = 3,
    LOADING = 4,
    FIRING = 5,
    RETURNING = 6,
    GAME2_SEARCHING = 7,
    GAME2_ALIGNING = 8,
    ERROR = 9,
  };

  void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void belt_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg);
  void dribble_enabled_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void drive_reversed_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void shot_cycle_state_callback(const std_msgs::msg::UInt8::SharedPtr msg);
  void game2_state_callback(const std_msgs::msg::UInt8::SharedPtr msg);
  void spring_fire_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void publish_timer_callback();

  DisplayMode select_display_mode() const;
  uint8_t make_status_flags() const;

  bool emergency_stop_received_{false};
  bool emergency_stop_active_{true};
  bool dribble_enabled_{false};
  bool drive_reversed_{false};
  bool spring_fire_request_active_{false};
  uint8_t belt_mode_{0};
  uint8_t shot_cycle_state_{0};
  uint8_t game2_state_{0};
  std::chrono::steady_clock::time_point firing_display_until_{};
  std::chrono::milliseconds firing_display_duration_{500};

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr belt_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dribble_enabled_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr drive_reversed_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr shot_cycle_state_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr game2_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr spring_fire_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr led_command_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

#endif  // LED_CONTROLLER__LED_CONTROLLER_HPP_
