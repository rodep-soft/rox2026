#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"

class RollerPositionController : public rclcpp::Node
{
public:
  RollerPositionController()
  : Node("roller_position_controller")
  {
    DeclareParameters();
    GetParameters();

    current_target_angle_ = intake_angle_;
    publisher_ = this->create_publisher<std_msgs::msg::Float32>(
      "/robstride/position_cmd", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy_second", 10,
      std::bind(&RollerPositionController::JoyCallback, this, std::placeholders::_1));

    PublishTargetAngle();
    RCLCPP_INFO(this->get_logger(), "roller_position_controller started.");
  }

private:
  static double DegreeToRadian(double degree)
  {
    constexpr double Pi = 3.14159265358979323846;
    return degree * Pi / 180.0;
  }

  void DeclareParameters()
  {
    this->declare_parameter<double>("storage_angle_deg", 0.0);
    this->declare_parameter<double>("intake_angle_deg", 10.0);
    this->declare_parameter<double>("shoot_angle_deg", 2.0);
    this->declare_parameter<int>("enable_button", 4);
    this->declare_parameter<int>("storage_button", 1);
    this->declare_parameter<int>("intake_button", 0);
    this->declare_parameter<int>("shoot_button", 2);
  }

  void GetParameters()
  {
    storage_angle_ = DegreeToRadian(this->get_parameter("storage_angle_deg").as_double());
    intake_angle_ = DegreeToRadian(this->get_parameter("intake_angle_deg").as_double());
    shoot_angle_ = DegreeToRadian(this->get_parameter("shoot_angle_deg").as_double());
    enable_button_idx_ = this->get_parameter("enable_button").as_int();
    storage_button_idx_ = this->get_parameter("storage_button").as_int();
    intake_button_idx_ = this->get_parameter("intake_button").as_int();
    shoot_button_idx_ = this->get_parameter("shoot_button").as_int();
  }

  bool IsJoyMessageValid(const sensor_msgs::msg::Joy::SharedPtr & msg) const
  {
    if (!msg) {
      return false;
    }

    const auto is_valid_button = [&msg](int index) {
        return index >= 0 && static_cast<size_t>(index) < msg->buttons.size();
      };
    return is_valid_button(enable_button_idx_) &&
           is_valid_button(storage_button_idx_) &&
           is_valid_button(intake_button_idx_) &&
           is_valid_button(shoot_button_idx_);
  }

  void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (!IsJoyMessageValid(msg)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Configured button index is outside the received Joy message.");
      return;
    }

    if (msg->buttons[enable_button_idx_] == 1) {
      if (msg->buttons[storage_button_idx_] == 1) {
        current_target_angle_ = storage_angle_;
      } else if (msg->buttons[intake_button_idx_] == 1) {
        current_target_angle_ = intake_angle_;
      } else if (msg->buttons[shoot_button_idx_] == 1) {
        current_target_angle_ = shoot_angle_;
      }
    }
    PublishTargetAngle();
  }

  void PublishTargetAngle()
  {
    std_msgs::msg::Float32 message;
    message.data = static_cast<float>(current_target_angle_);
    publisher_->publish(message);
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
  double storage_angle_{};
  double intake_angle_{};
  double shoot_angle_{};
  double current_target_angle_{};
  int enable_button_idx_{};
  int storage_button_idx_{};
  int intake_button_idx_{};
  int shoot_button_idx_{};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RollerPositionController>());
  rclcpp::shutdown();
  return 0;
}
