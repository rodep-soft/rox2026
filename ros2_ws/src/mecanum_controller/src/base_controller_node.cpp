#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include <cstddef>

class BaseController : public rclcpp::Node
{
public:
  BaseController()
  : Node("base_controller")
  {
    this->declare_parameter("max_linear", 1.0);
    this->declare_parameter("max_angular", 1.0);
    this->declare_parameter("smoothing_factor", 1.0);
    this->declare_parameter("pub_interval", 20);
    this->declare_parameter<int>("forward_reverse_toggle_button", 5);

    max_linear_velocity_ = this->get_parameter("max_linear").as_double();
    angular_velocity_ = this->get_parameter("max_angular").as_double();
    smoothing_factor_ = this->get_parameter("smoothing_factor").as_double();
    pub_interval_ = this->get_parameter("pub_interval").as_int();
    forward_reverse_toggle_button_ =
      this->get_parameter("forward_reverse_toggle_button").as_int();

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy_second", 1, std::bind(&BaseController::joy_callback, this, std::placeholders::_1));

    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(pub_interval_),
      std::bind(&BaseController::timer_callback, this));
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double max_linear_velocity_; // m/s
  double angular_velocity_;    // rad/s
  double smoothing_factor_;      // 急激な速度指令に対抗する，目標速度の平滑化係数
  int pub_interval_;
  int forward_reverse_toggle_button_;
  bool is_received_joy = false;
  bool previous_forward_reverse_toggle_pressed_ = false;
  double forward_reverse_sign_ = 1.0;
  geometry_msgs::msg::Twist cmd_msg;
  struct JoyInput
  {
    float x;
    float y;
    float yaw;
  };

  JoyInput latest_joy_;

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    is_received_joy = true;

    latest_joy_.x = msg->axes[1];
    latest_joy_.y = msg->axes[0];
    latest_joy_.yaw = msg->axes[2];

    const bool forward_reverse_toggle_pressed =
      forward_reverse_toggle_button_ >= 0 &&
      static_cast<std::size_t>(forward_reverse_toggle_button_) < msg->buttons.size() &&
      msg->buttons[forward_reverse_toggle_button_] == 1;
    if (forward_reverse_toggle_pressed && !previous_forward_reverse_toggle_pressed_) {
      forward_reverse_sign_ *= -1.0;
    }
    previous_forward_reverse_toggle_pressed_ = forward_reverse_toggle_pressed;
  }

  void timer_callback()
  {
    if (!is_received_joy) {
      return;
    }
    const double target_linear_x =
      forward_reverse_sign_ * latest_joy_.x * max_linear_velocity_;
    cmd_msg.linear.x = smoothing_factor_ * target_linear_x +
      (1 - smoothing_factor_) * cmd_msg.linear.x;                                                                             // 前後の移動;
    cmd_msg.linear.y = smoothing_factor_ * latest_joy_.y * max_linear_velocity_ +
      (1 - smoothing_factor_) * cmd_msg.linear.y;                                                                             // 左右の移動;
    cmd_msg.angular.z = smoothing_factor_ * latest_joy_.yaw * angular_velocity_ +
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
