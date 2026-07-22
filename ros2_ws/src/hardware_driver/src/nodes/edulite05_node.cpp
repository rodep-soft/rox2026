#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "edulite05_protcol.hpp"

void instance_motors()
{
    //インスタンス化
    std::array<std::unique_ptr<Ed05CanframeCreater>, 6> motors;
    motors[0] = std::make_unique<Velocity>(0x1);
    motors[1] = std::make_unique<Velocity>(0x2);
    motors[2] = std::make_unique<Velocity>(0x3);
    motors[3] = std::make_unique<Velocity>(0x4);
    motors[4] = std::make_unique<Velocity>(0x0A);
    motors[5] = std::make_unique<Position>(0x38);
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
            frame_pub_ = this->create_publisher<can_msgs::msg::Frame>(pub_topic_name, 10)//-----????-----

            init_motors();
        }

        ~Ed05DriverNode()
        {
            RCLCPP_INFO(this->get_logger(), "Ed05Node is shutting down.");
            // Cleanup code can be added here
        }

    private:
        std::string sub_topic_name_;  // Subscription topic name
        std::string pub_topic_name_;
        can_msgs::msg::Frame frame_;
        //int count_motor = 0;
        //std::vector<Ed05CanframeCreater> motors;

        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_sub_;  
        rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr frame_pub_;  // Publisher for command messages

        void init_motors()
        {
            std::vector<Canframe> frames; 
                frame_.is_extended = true;
                frame_.is_rtr = false;
                frame_.is_error = false;
            
            for (int i = 0; i < motors.size(); i++) {
                frames = motors[i].create_init_frame();
                for (size_t j = 0; j < frames.size(); j++) {
                    frame_.id = frames[j].id;
                    frame_.dlc = frames[j].dlc;
                    frame_.data = frames[j].data;
                    frame_pub_->pulish(frame_);
                }
            }
        }
        cmd_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
        {
            float value;
            Canframe frame;
                frame_.is_extended = true;
                frame_.is_rtr = false;
                frame_.is_error = false;


            for (int i = 0; i < motors.size(); i++)
            {
                value = *msg[i];
                frame = motors[i].create_control_frame(value);
                frame_.id = frame.id;
                frame_.dlc = frame.dlc;
                frame_.data = frame.data;

                RCLCPP_DEBUG(this->get_logger(), "Received command[%d]: %f", i, msg->data[i]);

                frame_pub_->pulish(frame_);
            }
        }

        declare_parameters()
        {
            this->declare_parameter<std::string>("sub_topic_name", "cmd");
            //this->declare_parameter<uint8_t>("motor_id", 0);
        }
        get_parameters()
        {
            sub_topic_name_ = this->get_parameter("sub_topic_name").as_string();
        }
};

int main (int argc, char **argv)
{
    instance_motors();// instanceって名詞？ モーターもといCanframeCreaterのインスタンス化
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Edulite05CanframeGeneratorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}