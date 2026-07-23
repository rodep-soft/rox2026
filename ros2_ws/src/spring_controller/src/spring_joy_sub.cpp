#include <memory>
#include <map>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "spring_controller/spring_controller_state.h"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/bool.hpp"

class SpringJoySub : public rclcpp::Node
{
public:
  SpringJoySub()
  : Node("spring_joy_sub")
  {
    // デフォルト値設定
    this->declare_parameter("shoot_key.type", "button");
    this->declare_parameter("shoot_key.index", 5);
    this->declare_parameter("lock_key.type", "axis");
    this->declare_parameter("lock_key.index", 4);
    std::string subscribed_joy = this->declare_parameter<std::string>("subscribed_joy", "/joy");
    // パラメータ取得
    joy_inputs["shoot_key"].type = this->get_parameter("shoot_key.type").as_string();
    joy_inputs["shoot_key"].index = this->get_parameter("shoot_key.index").as_int();
    joy_inputs["lock_key"].type = this->get_parameter("lock_key.type").as_string();
    joy_inputs["lock_key"].index = this->get_parameter("lock_key.index").as_int();

    joy_subscription = this->create_subscription<sensor_msgs::msg::Joy>(
      subscribed_joy,
      10,
      std::bind(&SpringJoySub::topic_callback, this, std::placeholders::_1));

    emergency_subscription = this->create_subscription<std_msgs::msg::Bool>(
      "/emergency_stop",
      10,
      std::bind(&SpringJoySub::emergency_callback, this, std::placeholders::_1));

    spring_publisher = this->create_publisher<std_msgs::msg::UInt8>(
      "spring_cmd",
      10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&SpringJoySub::timer_callback, this));
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_subscription;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr spring_publisher;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::Joy joy_msg;
  std_msgs::msg::Bool emergency_msg;

  struct JoyInput
  {
    std::string type;
    std::size_t index;
  };

  std::map<std::string, JoyInput> joy_inputs = {
    {"shoot_key", {"button", 10}},
    {"lock_key", {"axis", 4}}};

  void timer_callback()
  {
    std_msgs::msg::UInt8 fire_command;
    /*
    if (!joy_msg) {
      fire_command.data = static_cast<uint8_t>(State::STOP);
      spring_publisher->publish(fire_command);
      return;
    }
    */

    if (getValue(
        joy_msg,
        joy_inputs["lock_key"]) > 0.8 &&
      getValue(joy_msg, joy_inputs["shoot_key"]) > 0.8)
    {
      fire_command.data = static_cast<uint8_t>(State::FIRE); // 発射命令
    } else {
      fire_command.data = static_cast<uint8_t>(State::LOAD); // 装填命令

    }

    // if (emergency_msg && emergency_msg->data) {
    //   fire_command.data = static_cast<uint8_t>(State::STOP); // 停止命令
    //}

    spring_publisher->publish(fire_command);
  }


  void topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    joy_msg = *msg;
  }

  void emergency_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    emergency_msg = *msg;
  }

  float getValue(const sensor_msgs::msg::Joy joy, const JoyInput & input)
  {
    // axisとbuttonの値を取得する関数
    if (input.type == "axis" && input.index < joy.axes.size()) {

      return -joy.axes[input.index];
    } else if (input.type == "button" && input.index < joy.buttons.size()) {
      return joy.buttons[input.index];
    }

    return 0.0;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpringJoySub>());
  rclcpp::shutdown();
  return 0;
}
