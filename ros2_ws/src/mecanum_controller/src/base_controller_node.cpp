#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

class BaseController : public rclcpp::Node
{
public:
  BaseController()
  : Node("base_controller")
  {
    this->declare_parameter("max_linear", 1.0);
    this->declare_parameter("max_angular", 1.0);
    this->declare_parameter("smoothing_factor", 1.0);

    max_linear_velocity_ = this->get_parameter("max_linear").as_double();
    angular_velocity_ = this->get_parameter("max_angular").as_double();
    smoothing_factor_ = this->get_parameter("smoothing_factor").as_double();

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&BaseController::joy_callback, this, std::placeholders::_1));

    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  double max_linear_velocity_; // m/s
  double angular_velocity_;    // rad/s
  double smoothing_factor_;      // 急激な速度指令に対抗する，目標速度の平滑化係数
  geometry_msgs::msg::Twist cmd_msg;

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {

    cmd_msg.linear.x = smoothing_factor_ * msg->axes[1] * max_linear_velocity_ +
      (1 - smoothing_factor_) * cmd_msg.linear.x;                                                                             // 前後の移動;
    cmd_msg.linear.y = smoothing_factor_ * msg->axes[0] * max_linear_velocity_ +
      (1 - smoothing_factor_) * cmd_msg.linear.y;                                                                             // 左右の移動;
    cmd_msg.angular.z = smoothing_factor_ * msg->axes[2] * angular_velocity_ +
      (1 - smoothing_factor_) * cmd_msg.angular.z;                                                                          // yawの回転
    vel_pub_->publish(cmd_msg);
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BaseController>());
  rclcpp::shutdown();
  return 0;
}
