#include "mecanum_controller/mecanum_controller_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"


MecanumControllerNode::MecanumControllerNode() 
: Node("mecanum_controller_node")
{   
    declare_parameters();
    get_parameters();
    vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&MecanumControllerNode::velocityCallback, this, std::placeholders::_1));
    motor_vel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/motor_vel", 10);
}


void MecanumControllerNode::velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    vx = msg->linear.x;
    vy = msg->linear.y;
    wz = msg->angular.z;

    // Mecanumホイールの速度計算
    wheel_vels[FL] = (vx - vy - (robot_length + robot_width)/2 * wz) / wheel_radius;
    wheel_vels[FR] = (vx + vy + (robot_length + robot_width)/2 * wz) / wheel_radius;
    wheel_vels[RL] = (vx + vy - (robot_length + robot_width)/2 * wz) / wheel_radius;
    wheel_vels[RR] = (vx - vy + (robot_length + robot_width)/2 * wz) / wheel_radius;

    std_msgs::msg::Float64MultiArray cmd_msg;
    for (const auto& vel : wheel_vels) {
        cmd_msg.data.push_back(vel);
    }
    motor_vel_pub_->publish(cmd_msg); 
}

void MecanumControllerNode::declare_parameters()
{
    this->declare_parameter<double>("wheel_radius", 0.1);
    this->declare_parameter<double>("robot_length", 0.5);
    this->declare_parameter<double>("robot_width", 0.3);
}

void MecanumControllerNode::get_parameters()
{
    this->get_parameter("wheel_radius", wheel_radius);
    this->get_parameter("robot_length", robot_length);
    this->get_parameter("robot_width", robot_width);
}

void MecanumControllerNode::motor_init()
{
    this->get_logger().info("モーター初期化中...");
    //モーター初期化のcan_msgs/msg/Frameを送信する処理をここに追加
}
