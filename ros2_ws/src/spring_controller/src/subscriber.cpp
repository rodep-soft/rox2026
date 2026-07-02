#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/u_int8.hpp"

class JoySub : public rclcpp::Node
{
    public:
    JoySub()
    : Node("joy_sub")
    {
        subscription_ =
        this->create_subscription<sensor_msgs::msg::Joy>(
            "joy",
            10,
            std::bind(&JoySub::topic_callback,this,std::placeholders::_1));
        
        publisher_ =
        this->create_publisher<std_msgs::msg::UInt8>(
            "spring_cmd",
            10);    
    }

    private:
    void topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        std_msgs::msg::UInt8 command;
        if(msg -> buttons[6] == 1 && msg -> buttons[5] == 1)
        {
            command.data = 1;
        }
        else
        {
            command.data = 0;
        }


        publisher_->publish( command );
    }
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr publisher_;

};

int main(int argc,char * argv[]){
    rclcpp::init(argc,argv);

    rclcpp::spin(std::make_shared<JoySub>());

    rclcpp::shutdown();
    return 0;
}

