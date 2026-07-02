#include <memory>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/int16.hpp>

class BeltShooterControllerNode : public rclcpp::Node
{
public:
    BeltShooterControllerNode() : Node("belt_shooter_controller_node"), current_pwm_(0)
    {
        this->declare_parameter<int>("enable_button", 6);
        this->declare_parameter<int>("high_button", 2);
        this->declare_parameter<int>("medium_button", 1);
        this->declare_parameter<int>("low_button", 0);
        this->declare_parameter<int>("stop_button", 3);

        this->declare_parameter<int>("high_pwm", 255);
        this->declare_parameter<int>("medium_pwm", 220);
        this->declare_parameter<int>("low_pwm", 200);
        this->declare_parameter<int>("stop_pwm", 0);

        get_parameters();

       
        current_pwm_ = stop_pwm_;


        pwm_pub_ = this->create_publisher<std_msgs/msg/Int16>("/mad_motor/pwm_value", 10);
        joy_sub_ = this->create_subscription<sensor_msgs/msg/Joy>(
            "/joy", 10, std::bind(&BeltShooterControllerNode::joy_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Belt Shooter Controller Node has been started.");
    }

private:
    void get_parameters()
    {
        enable_button_ = this->get_parameter("enable_button").as_int();
        high_button_ = this->get_parameter("high_button").as_int();
        medium_button_ = this->get_parameter("medium_button").as_int();
        low_button_ = this->get_parameter("low_button").as_int();
        stop_button_ = this->get_parameter("stop_button").as_int();


        high_pwm_ = validate_and_clamp_pwm("high_pwm");
        medium_pwm_ = validate_and_clamp_pwm("medium_pwm");
        low_pwm_ = validate_and_clamp_pwm("low_pwm");
        stop_pwm_ = validate_and_clamp_pwm("stop_pwm");
    }


    int validate_and_clamp_pwm(const std::string &param_name)
    {
        int raw_value = this->get_parameter(param_name).as_int();
        if (raw_value < 0 || raw_value > 255)
        {
            int clamped_value = std::max(0, std::min(255, raw_value));
            RCLCPP_WARN(this->get_logger(),
                        "Parameter '%s' value (%d) is out of range (0-255). Clamped to %d.",
                        param_name.c_str(), raw_value, clamped_value);
            return clamped_value;
        }
        return raw_value;
    }


    bool is_button_pressed(const sensor_msgs::msg::Joy::SharedPtr msg, int button_index)
    {

        if (button_index < 0 || button_index >= static_cast<int>(msg->buttons.size()))
        {

            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Configured button index %d is out of bounds (Joy size: %zu).",
                                 button_index, msg->buttons.size());
            return false;
        }
        return msg->buttons[button_index] == 1;
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {

        if (!is_button_pressed(msg, enable_button_))
        {
  
            return;
        }

        bool value_changed = false;
        int target_pwm = current_pwm_;


        if (is_button_pressed(msg, stop_button_))
        {
            target_pwm = stop_pwm_;
            value_changed = true;
        }
        else if (is_button_pressed(msg, low_button_))
        {
            target_pwm = low_pwm_;
            value_changed = true;
        }
        else if (is_button_pressed(msg, medium_button_))
        {
            target_pwm = medium_pwm_;
            value_changed = true;
        }
        else if (is_button_pressed(msg, high_button_))
        {
            target_pwm = high_pwm_;
            value_changed = true;
        }

        if (value_changed && target_pwm != current_pwm_)
        {
            current_pwm_ = target_pwm;

            auto pub_msg = std_msgs::msg::Int16();
            pub_msg.data = static_cast<int16_t>(current_pwm_);
            pwm_pub_->publish(pub_msg);
            
            RCLCPP_INFO(this->get_logger(), "Published new PWM value: %d", current_pwm_);
        }
    }

    rclcpp::Publisher<std_msgs/msg/Int16>::SharedPtr pwm_pub_;
    rclcpp::Subscription<sensor_msgs/msg/Joy>::SharedPtr joy_sub_;

    int enable_button_;
    int high_button_;
    int medium_button_;
    int low_button_;
    int stop_button_;

    int high_pwm_;
    int medium_pwm_;
    int low_pwm_;
    int stop_pwm_;

    int current_pwm_; 
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BeltShooterControllerNode>());
    rclcpp::shutdown();
    return 0;
}