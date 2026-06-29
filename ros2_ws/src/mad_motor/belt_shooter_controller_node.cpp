#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/int16.hpp>
#include <algorithm>
#include <functional>

class BeltShooterController : public rclcpp::Node
{
public:
  BeltShooterController() : Node("ls"),last_pwm_(0)
  { 
    //パラメーター定義
    declare_parameter<int>("high_button"  ,2);
    declare_parameter<int>("medium_button",1);
    declare_parameter<int>("low_button"   ,0);
    declare_parameter<int>("stop_button"  ,3);
    declare_parameter<int>("enable_button",6);

    
    declare_parameter<int>("high_pwm"  ,255);
    declare_parameter<int>("medium_pwm",220);
    declare_parameter<int>("low_pwm"   ,200);
    declare_parameter<int>("stop_pwm"  ,0  );

    enable_button_ = get_parameter("enable_button").as_int();
    high_button_   = get_parameter("high_button").as_int();
    medium_button_ = get_parameter("medium_button").as_int();
    low_button_    = get_parameter("low_button").as_int();
    stop_button_   = get_parameter("stop_button").as_int();

    high_pwm_   = get_parameter("high_pwm").as_int();
    medium_pwm_ = get_parameter("medium_pwm").as_int();
    low_pwm_    = get_parameter("low_pwm").as_int();
    stop_pwm_   = get_parameter("stop_pwm").as_int();

    //範囲指定
    high_pwm_   = std::clamp(high_pwm_,0,255);
    medium_pwm_ = std::clamp(medium_pwm_,0,255);
    low_pwm_    = std::clamp(low_pwm_,0,255); 
    stop_pwm_   = std::clamp(stop_pwm_,0,255);


    
    joy_sub_ =create_subscription<sensor_msgs::msg::Joy>
    ("/joy",10,std::bind(&BeltShooterController::joyCallback, this,std::placeholders::_1));
    
    pwm_pub_ = create_publisher<std_msgs::msg::Int16>("/mad_motor/pwm_value",10);
  
    RCLCPP_INFO(get_logger(), "belt_shooter_controller_node_started");

  }

private:

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {   

        int pwm = computePwm(msg);

        if(pwm != last_pwm_)
        {
            last_pwm_ = pwm;
    
            std_msgs::msg::Int16 out;
            out.data = pwm;
            pwm_pub_->publish(out);
        }    
    }   

    int computePwm(const sensor_msgs::msg::Joy::SharedPtr msg)
    {   const auto &b = msg->buttons;
       //L2が押されてるか 
        if(!isValid(b,enable_button_) || b[enable_button_] == 0)
        {
            return last_pwm_;
        }

    //pwmの計算
    if      (isValid(b, stop_button_) && b[stop_button_]){
        last_pwm_ = stop_pwm_;
    }else if(isValid(b, low_button_) && b[low_button_]){
        last_pwm_ = low_pwm_;
    }else if(isValid(b, medium_button_) && b[medium_button_]){
        last_pwm_ = medium_pwm_;
    }else if(isValid(b, high_button_) && b[high_button_]){
        last_pwm_ = high_pwm_;
    }
    return last_pwm_;
    }

    bool isValid(const std::vector<int> &buttons, int idx)
    {
        return (idx >= 0 && idx < static_cast<int>(buttons.size()));
    }
    
    //変数定義
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pwm_pub_;
    
    int enable_button_;
    int high_button_;
    int medium_button_;
    int low_button_;
    int stop_button_;

    int high_pwm_;
    int medium_pwm_;
    int low_pwm_;
    int stop_pwm_;
    int last_pwm_ = 0;
};

int main(int argc, char*argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BeltShooterController>());
    rclcpp::shutdown();
    return 0;
}
