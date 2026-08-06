#ifndef BELT_CONTROLLER__BELT_CONTROLLER_HPP_
#define BELT_CONTROLLER__BELT_CONTROLLER_HPP_

#include <array>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/u_int8.hpp"

class BeltControllerNode : public rclcpp::Node
{
public:
  BeltControllerNode();

private:
  static constexpr std::size_t kNumLevels = 6;  ///< ベルト速度レベル数（LEVEL_1〜6）

  enum class BeltMode : uint8_t
  {
    STOP = 0,
    LEVEL_1 = 1,
    LEVEL_2 = 2,
    LEVEL_3 = 3,
    LEVEL_4 = 4,
    LEVEL_5 = 5,
    LEVEL_6 = 6,
  };

  void declare_parameters();
  void get_parameters();

  void belt_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg);
  void belt_target_rpm_callback(const std_msgs::msg::Float32::SharedPtr msg);
  void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void timer_callback();

  /// @brief 現在のベルトモードまたは直接RPMに対応する目標RPMを返す
  int belt_target_rpm() const;

  bool emergency_stop_active_{false};
  BeltMode belt_mode_{BeltMode::STOP};
  int direct_target_rpm_{0};
  bool use_direct_target_rpm_{false};

  /// @brief LEVEL_1〜6 の目標RPMテーブル（インデックス0=LEVEL_1, ..., 5=LEVEL_6）
  std::array<int, kNumLevels> level_rpms_{3000, 3500, 4000, 4500, 5000, 5500};
  int command_period_ms_{10};
  int qos_depth_{1};

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr belt_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr belt_target_rpm_sub_;

  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr underbelt_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr upperbelt_command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // BELT_CONTROLLER__BELT_CONTROLLER_HPP_
