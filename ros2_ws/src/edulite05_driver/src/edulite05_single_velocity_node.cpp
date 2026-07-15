#include <algorithm>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include "can_msgs/msg/frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include "edulite05_driver/edulite05_velocity.hpp"

class EduLite05SingleVelocityNode : public rclcpp::Node
{
public:
  EduLite05SingleVelocityNode()
  : Node("edulite05_single_velocity_node")
  {
    const auto motor_id = declare_parameter<std::int64_t>("motor_id", 20);
    max_velocity_rad_s_ = declare_parameter<double>("max_velocity_rad_s", 30.0);
    const auto current_limit_a = declare_parameter<double>("current_limit_a", 11.0);
    const auto acceleration_rad_s2 = declare_parameter<double>("acceleration_rad_s2", 20.0);
    const auto can_tx_topic = declare_parameter<std::string>("can_tx_topic", "CAN/can0/transmit");
    const auto motor_vel_topic = declare_parameter<std::string>("motor_vel_topic", "motor_vel");

    //一応警告も
    if (motor_id < 1 || motor_id > 255) {
      throw std::invalid_argument("motor_id must be in the range 1 to 255");
    }
    if (max_velocity_rad_s_ <= 0.0 || current_limit_a <= 0.0 || acceleration_rad_s2 <= 0.0) {
      throw std::invalid_argument(
              "max_velocity_rad_s, current_limit_a and acceleration_rad_s2 must be positive");
    }

    can_pub_ = create_publisher<can_msgs::msg::Frame>(can_tx_topic, 10);

    motor_ = std::make_unique<EduLite05Velocity>(
      static_cast<std::uint8_t>(motor_id), can_pub_,
      static_cast<float>(current_limit_a), static_cast<float>(acceleration_rad_s2));

    velocity_sub_ = create_subscription<std_msgs::msg::Float32>(
      motor_vel_topic, 10,
      [this](const std_msgs::msg::Float32::SharedPtr message) {
        const auto velocity = std::clamp(
          message->data,
          static_cast<float>(-max_velocity_rad_s_),
          static_cast<float>(max_velocity_rad_s_));
        motor_->set_velocity(velocity);
      });

    motor_->initialize();
    RCLCPP_INFO(
      get_logger(), "EDULITE05 motor %ld initialized (maximum velocity: %.2f rad/s)",
      static_cast<long>(motor_id), max_velocity_rad_s_);
  }

  ~EduLite05SingleVelocityNode() override
  {
    if (motor_) {
      motor_->stop();
    }
  }

private:
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr velocity_sub_;
  std::unique_ptr<EduLite05Velocity> motor_;
  double max_velocity_rad_s_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EduLite05SingleVelocityNode>());
  rclcpp::shutdown();
  return 0;
}
