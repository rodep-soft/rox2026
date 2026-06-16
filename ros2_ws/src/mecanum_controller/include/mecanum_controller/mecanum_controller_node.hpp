#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <array>

class MecanumControllerNode : public rclcpp::Node
{
public:
    MecanumControllerNode();


private:
    void declare_parameters();
    void get_parameters();

    void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void motor_init();
    void motor_enable();
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr vel_pub_;

    enum WheelIndex
        {
            FL = 0,
            FR = 1,
            RL = 2,
            RR = 3,
        };
    std::array<double, 4> wheel_vels = {0.0, 0.0, 0.0, 0.0};

    double vx, vy, wz;

    double wheel_radius ;
    double robot_length;
    double robot_width;



}