#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

class BaseController : public rclcpp::Node {
public:
  BaseController() : Node("base_controller") {
    this->declare_parameter("max_linear", 2.0);
    this->declare_parameter("max_angular", 2.0);

    max_linear_velocity_ = this->get_parameter("max_linear").as_double();
    angular_velocity_ = this->get_parameter("max_angular").as_double();

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10, std::bind(&BaseController::joy_callback, this, std::placeholders::_1));

    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }
private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  double max_linear_velocity_; // m/s
  double angular_velocity_;    // rad/s
  
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
  
    auto cmd_msg = geometry_msgs::msg::Twist();
    cmd_msg.linear.x = msg->axes[0] * -1 * max_linear_velocity_;  // 前後の移動;
    cmd_msg.linear.y = msg->axes[1] * max_linear_velocity_;  // 左右の移動;
    cmd_msg.angular.z = msg->axes[3] * angular_velocity_; // yawの回転
    vel_pub_->publish(cmd_msg);
  }

};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BaseController>());
  rclcpp::shutdown();
  return 0;
}
