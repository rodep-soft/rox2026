#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "edulite05_protcol.hpp"

void init_motor()
{
    //インスタンス化
}

class Ed05DriverNode : public rclcpp::Node
{
    
    public:
        Ed05Driverger() {
            RCLCPP_INFO(this->get_logger(), "Edulite05Node has been started.");

            declare_parameters();
            get_parameters();

            cmd_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            sub_topic_name_, 1, std::bind(&Ed05DriverNode::cmd_callback, this, std::placeholders::_1));
            frame_pub_ = this->create_publisher<can_msgs::msg::Frame>(pub_topic_name)//-----????-----

            // Ed05のクラスからインスタンス化を6回 <- これノードの外かも 
            // このときget parameterにより値を取得？？？？

        }

        ~Ed05DriverNode()
        {
            RCLCPP_INFO(this->get_logger(), "Ed05Node is shutting down.");
            // Cleanup code can be added here
        }

    private:
        std::string sub_topic_name_;  // Subscription topic name
        std::string pub_topic_name_;
        int count_motor = 0;
        std::vector<Ed05CanframeCreater> motors;

        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_sub_;  
        rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr frame_pub_;  // Publisher for command messages
        

        cmd_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
        {
            int num = std::size(*msg);

            for (int i = 0; i < num; i++)
            {
                RCLCPP_DEBUG(this->get_logger(), "Received command[%d]: %f", i, msg->data[i]);

            }
        }
        declare_parameters()
        {
            this->declare_parameter<std::string>("sub_topic_name", "cmd");
            this->declare_parameter<uint8_t>("motor_id", 0);//要変更 だってモーター複数ある そもそもここでid変更とかやらない・できない可能性
            // Declare other parameters as needed
        }
        get_parameters()
        {
            sub_topic_name_ = this->get_parameter("sub_topic_name").as_string();
            motor_id_ = this->get_parameter("motor_id").as_uint8();
            // Get other parameters as needed
        }
};

int main (int argc, char **argv)
{
    init_motor();//(名は仮) モーターもといCanframeCreaterのインスタンス化
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Edulite05CanframeGeneratorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}