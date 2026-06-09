#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

class JoyController : public rclcpp::Node {
public:
  JoyController() : Node("joy_controller"), max_linear_velocity_(1.0), angular_velocity_(1.0) {
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10, std::bind(&JoyController::joy_callback, this, std::placeholders::_1));

    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }
private:
  // メンバ変数やコールバック;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  double max_linear_velocity_;
  double angular_velocity_;
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // ここにジョイスティックの入力処理を記述;
    publish_command(msg);
  }
  void publish_command(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // ここにコマンドの公開処理を記述;
    //RCLCPP_INFO(this->get_logger(), "Received Joy message with axes: [%f, %f] and buttons: [%d, %d]",
    //            msg->axes[0], msg->axes[1], msg->buttons[16], msg->buttons[17]);
    auto cmd_msg = geometry_msgs::msg::Twist();
    cmd_msg.linear.x = msg->axes[0] * max_linear_velocity_;  // 前後の移動;
    cmd_msg.linear.y = msg->axes[1] * max_linear_velocity_;  // 左右の移動;
    cmd_msg.angular.z = msg->buttons[17] * angular_velocity_ - msg->buttons[16] * angular_velocity_; // yawの回転
    vel_pub_->publish(cmd_msg);
  }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyController>());
  rclcpp::shutdown();
  return 0;
}