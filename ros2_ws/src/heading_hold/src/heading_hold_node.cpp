#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>

class HeadingHoldNode : public rclcpp::Node
{
public:
  HeadingHoldNode()
  : Node("heading_hold_node") 
  {
    raw_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_raw", 10,
      std::bind(&HeadingHoldNode::rawVelocityCallback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 10,
      std::bind(&HeadingHoldNode::imuCallback, this, std::placeholders::_1));

    corrected_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "Heading Hold Node started.");
  }

private:
  void rawVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // ここに書く
    auto output_msg = *msg;
    corrected_vel_pub_->publish(output_msg);
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // ここに書く
    (void)msg; 
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr raw_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr corrected_vel_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeadingHoldNode>());
  rclcpp::shutdown();
  return 0;
}
