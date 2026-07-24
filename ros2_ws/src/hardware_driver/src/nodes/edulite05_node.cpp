#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"
#include "std_msgs/msg/float32.hpp"
#include <array>
#include <vector>
#include "edulite05_driver/edulite05_protocol.hpp"
#include <thread>
#include <chrono>


class Ed05DriverNode : public rclcpp::Node
{

public:
  Ed05DriverNode()
  : Node("edulite05_driver_node")
  {
    RCLCPP_INFO(this->get_logger(), "Edulite05Node has been started.");

    declare_parameters();
    get_parameters();

    cmd_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      sub_cmd_topic_name_, 1,
      std::bind(&Ed05DriverNode::cmd_callback, this, std::placeholders::_1));
    frame_pub_ = this->create_publisher<can_msgs::msg::Frame>(pub_topic_name_, 10);
    can_sub_ = this->create_subscription<can_msgs::msg::Frame>(
      sub_can_topic_name_, 10,
      std::bind(&Ed05DriverNode::can_callback, this, std::placeholders::_1));

    init_motor();
  }

  ~Ed05DriverNode()
  {
    RCLCPP_INFO(this->get_logger(), "Ed05Node is shutting down.");
    // Cleanup code can be added here
  }
  void terminate_motor()
  {
    Canframe frame = motor_->terminate_motor();
    frame_.is_extended = true;
    frame_.is_rtr = false;
    frame_.is_error = false;
    frame_.id = frame.id;
    frame_.dlc = frame.dlc;
    frame_.data = frame.data;
    frame_pub_->publish(frame_);
    RCLCPP_DEBUG(this->get_logger(), "Published terminate frame for motor %d.", motor_id_);
  }

private:
  std::string sub_cmd_topic_name_;        // Subscription topic name
  std::string pub_topic_name_;
  std::string sub_can_topic_name_;        // Subscription topic name for CAN messages
  can_msgs::msg::Frame frame_;

  uint8_t motor_id_;
  std::string runmode_;  // "Velocity", "Position"

  //int count_motor = 0;
  //std::vector<Ed05CanframeCreater> motors;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cmd_sub_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr frame_pub_;        // Publisher for command messages
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;  // Subscription for CAN messages
  std::unique_ptr<Ed05CanframeCreater> motor_;

  void init_motor()
  {
    if (runmode_ == "Velocity") {
      motor_ = std::make_unique<Velocity>(motor_id_);
    } else if (runmode_ == "Position") {
      motor_ = std::make_unique<Position>(motor_id_);
    } else {
      RCLCPP_ERROR(
        this->get_logger(), "Invalid runmode: %s. Must be \"Velocity\" or \"Position\".",
        runmode_.c_str());
      return;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));   

    frame_.is_extended = true;
    frame_.is_rtr = false;
    frame_.is_error = false;

    std::vector<Canframe> frames;
    frames = motor_->create_init_frame();
    for (size_t i = 0; i < frames.size(); i++) {
      frame_.id = frames[i].id;
      frame_.dlc = frames[i].dlc;
      frame_.data = frames[i].data;
      frame_pub_->publish(frame_);

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    RCLCPP_DEBUG(this->get_logger(), "Published initialization frames for motor %d.", motor_id_);
  }

  void cmd_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    float value = msg->data;
    Canframe frame;
    frame = motor_->create_control_frame(value);
    frame_.is_extended = true;
    frame_.is_rtr = false;
    frame_.is_error = false;
    frame_.id = frame.id;
    frame_.dlc = frame.dlc;
    frame_.data = frame.data;
    RCLCPP_DEBUG(this->get_logger(), "publish motor %d: %f", motor_id_, value);

    frame_pub_->publish(frame_);
  }

  void can_callback(const can_msgs::msg::Frame::SharedPtr msg)
  {
    // Handle incoming CAN messages if needed
    RCLCPP_DEBUG(this->get_logger(), "Received CAN frame with ID: %u", msg->id);

  }

  void declare_parameters()
  {
    this->declare_parameter<std::string>("sub_cmd_topic_name", "cmd");
    this->declare_parameter<std::string>("pub_topic_name", "can_tx");
    this->declare_parameter<std::string>("sub_can_topic_name", "can");
    this->declare_parameter<uint8_t>("motor_id", 0x01);
    this->declare_parameter<std::string>("runmode", "Velocity");

  }
  void get_parameters()
  {
    sub_cmd_topic_name_ = this->get_parameter("sub_cmd_topic_name").as_string();
    pub_topic_name_ = this->get_parameter("pub_topic_name").as_string();
    sub_can_topic_name_ = this->get_parameter("sub_can_topic_name").as_string();
    motor_id_ = static_cast<uint8_t>(this->get_parameter("motor_id").as_int());
    runmode_ = this->get_parameter("runmode").as_string();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Ed05DriverNode>();
  rclcpp::spin(node);
  node->terminate_motor();
  rclcpp::shutdown();
  return 0;
}
