#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"


MecanumControllerNode::MecanumControllerNode() 
: Node("mecanum_controller_node")
{   
    declare_parameters();
    vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&MecanumControllerNode::velocityCallback, this, std::placeholders::_1));
    vel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/velocity_command", 10);
}


void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    vx = msg->linear.x;
    vy = msg->linear.y;
    wz = msg->angular.z;

    // Mecanumホイールの速度計算
    wheel_vels[FL] = (vx - vy - (robot_length + robot_width) * wz) / wheel_radius;
    wheel_vels[FR] = (vx + vy + (robot_length + robot_width) * wz) / wheel_radius;
    wheel_vels[RL] = (vx + vy - (robot_length + robot_width) * wz) / wheel_radius;
    wheel_vels[RR] = (vx - vy + (robot_length + robot_width) * wz) / wheel_radius;

    std_msgs::msg::Float64MultiArray cmd_msg;
    vel_pub_->publish(cmd_msg); 
}
void declare_parameters()
{
    enum WheelIndex
    {
        FL = 0,
        FR = 1,
        RL = 2,
        RR = 3,
    };

    this->declare_parameter<double>("wheel_radius", 0.1);
    this->declare_parameter<double>("robot_length", 0.5);
    this->declare_parameter<double>("robot_width", 0.3);
}

void motor_init()
{
    
}
