#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <array>
#include <vector>
#include "edulite05_driver/edulite05_protocol.hpp"


class Ed05DriverNode : public rclcpp::Node
{

public:
  Ed05DriverNode()
  : Node("edulite05_driver_node")
  {
    RCLCPP_INFO(this->get_logger(), "Edulite05Node has been started.");

    declare_parameters();
    get_parameters();

    cmd_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      sub_cmd_topic_name_, 1,
      std::bind(&Ed05DriverNode::cmd_callback, this, std::placeholders::_1));
    frame_pub_ = this->create_publisher<can_msgs::msg::Frame>(pub_topic_name_, 10);

    init_motors();
  }

  ~Ed05DriverNode()
  {
    RCLCPP_INFO(this->get_logger(), "Ed05Node is shutting down.");
    // Cleanup code can be added here
  }
  void terminate_motors()
  {
    frame_.is_extended = true;
    frame_.is_rtr = false;
    frame_.is_error = false;
    for (size_t i = 0; i < motors_.size(); i++) {
      Canframe frame = motors_[i]->terminate_motor();
      frame_.id = frame.id;
      frame_.dlc = frame.dlc;
      frame_.data = frame.data;
      frame_pub_->publish(frame_);
      RCLCPP_DEBUG(this->get_logger(), "Published terminate frame for motor[%zu]", i);
    }
  }

private:
  std::string sub_cmd_topic_name_;        // Subscription topic name
  std::string pub_topic_name_;
  can_msgs::msg::Frame frame_;
  //int count_motor = 0;
  //std::vector<Ed05CanframeCreater> motors;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_sub_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr frame_pub_;        // Publisher for command messages
  std::array<std::unique_ptr<Ed05CanframeCreater>, 6> motors_;

  void init_motors()
  {
    motors_[0] = std::make_unique<Velocity>(0x1);
    motors_[1] = std::make_unique<Velocity>(0x2);
    motors_[2] = std::make_unique<Velocity>(0x3);
    motors_[3] = std::make_unique<Velocity>(0x4);
    motors_[4] = std::make_unique<Velocity>(0x0A);
    motors_[5] = std::make_unique<Position>(0x38);
    std::vector<Canframe> frames;
    frame_.is_extended = true;
    frame_.is_rtr = false;
    frame_.is_error = false;

    for (size_t i = 0; i < motors_.size(); i++) {
      frames = motors_[i]->create_init_frame();
      for (size_t j = 0; j < frames.size(); j++) {
        frame_.id = frames[j].id;
        frame_.dlc = frames[j].dlc;
        frame_.data = frames[j].data;
        frame_pub_->publish(frame_);
        RCLCPP_DEBUG(
          this->get_logger(),
          "Published init frame for motor[%zu]: ID=0x%X, DLC=%d, Data=%d",
          i, frame_.id, frame_.dlc, frame_.data[0]);
      }
    }
  }

  void cmd_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    float value;
    Canframe frame;
    frame_.is_extended = true;
    frame_.is_rtr = false;
    frame_.is_error = false;


    for (size_t i = 0; i < motors_.size(); i++) {
      RCLCPP_DEBUG(this->get_logger(), "Received command[%zu]: %f", i, value);

      if (i < msg->data.size()) {
        value = msg->data[i];
      } else {
        value = 0.0f;
      }
      frame = motors_[i]->create_control_frame(value);
      frame_.id = frame.id;
      frame_.dlc = frame.dlc;
      frame_.data = frame.data;


      frame_pub_->publish(frame_);
    }
  }


  void declare_parameters()
  {
    this->declare_parameter<std::string>("sub_cmd_topic_name", "cmd");
    this->declare_parameter<std::string>("pub_topic_name", "can_tx");
    //this->declare_parameter<std::string>("sub_can_topic_name", "can");

  }
  void get_parameters()
  {
    sub_cmd_topic_name_ = this->get_parameter("sub_cmd_topic_name").as_string();
    pub_topic_name_ = this->get_parameter("pub_topic_name").as_string();
    //sub_can_topic_name_ = this->get_parameter("sub_can_topic_name").as_string();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Ed05DriverNode>();
  rclcpp::spin(node);
  node->terminate_motors();
  rclcpp::shutdown();
  return 0;
}
