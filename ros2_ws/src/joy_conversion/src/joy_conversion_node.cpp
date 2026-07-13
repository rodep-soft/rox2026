#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class JoyConversion : public rclcpp::Node
{
public:
  JoyConversion();

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void checkJoyTimeout();
  void publishEmergencyStop(bool emergency_stop);

  // メンバ変数
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_second_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time last_joy_time_;
  bool prev_button_state_ = false;
  bool emergency_stop_;
  int emergency_button_index_;
  int joy_timeout_ms_;
};

JoyConversion::JoyConversion()
: Node("joy_conversion_node"), emergency_stop_(true)
{
  // パラメータ宣言
  this->declare_parameter<int>("emergency_button_index", 1);
  this->declare_parameter<int>("joy_timeout_ms", 500);

  emergency_button_index_ = this->get_parameter("emergency_button_index").as_int();
  joy_timeout_ms_ = this->get_parameter("joy_timeout_ms").as_int();

  // サブスクライバー生成
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 10,
    std::bind(&JoyConversion::joyCallback, this, std::placeholders::_1)
  );

  // パブリッシャー生成
  joy_second_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("/joy_second", 10);
  emergency_pub_ = this->create_publisher<std_msgs::msg::Bool>("/emergency_stop", 10);

  // 起動時は非常停止
  publishEmergencyStop(true);

  // タイムアウト監視
  timer_ = this->create_wall_timer(
    50ms,
    std::bind(&JoyConversion::checkJoyTimeout, this)
  );

  last_joy_time_ = this->now();
}

void JoyConversion::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  last_joy_time_ = this->now();

  bool button_pressed = msg->buttons[emergency_button_index_] == 1;

  // トグル（立ち上がり検出）
  if (button_pressed && !prev_button_state_) {
    emergency_stop_ = !emergency_stop_;
    publishEmergencyStop(emergency_stop_);
  }
  //前回の状態
  prev_button_state_ = button_pressed;

  // /joy_second に転送
  // /joy_second用のJoyメッセージを作成
  sensor_msgs::msg::Joy joy_second_msg;

  if (emergency_stop_) {
    joy_second_msg.header = msg->header;

    // axesをすべて0
    joy_second_msg.axes.resize(msg->axes.size(), 0.0f);

    // buttonsをすべて0
    joy_second_msg.buttons.resize(msg->buttons.size(), 0);
  } else {
    joy_second_msg = *msg;
  }

  // /joy_secondへ送信
  joy_second_pub_->publish(joy_second_msg);
}

//joyの入力に間があったら停止
void JoyConversion::checkJoyTimeout()
{
  auto now = this->now();

  //difference_ms
  auto diff_ms = (now - last_joy_time_).nanoseconds() / 1'000'000;

  if (diff_ms > joy_timeout_ms_) {
    if (!emergency_stop_) {
      emergency_stop_ = true;
      publishEmergencyStop(true);
      RCLCPP_WARN(this->get_logger(), "/joy timeout → emergency_stop ON");
    }
  }
}

//非常停止の状態（ON/OFF）を /emergency_stop トピックに送信する関数
void JoyConversion::publishEmergencyStop(bool emergency_stop)
{
  std_msgs::msg::Bool emergency_stop_msg;
  emergency_stop_msg.data = emergency_stop;
  emergency_pub_->publish(emergency_stop_msg);

  RCLCPP_INFO(
    this->get_logger(),
    emergency_stop ? "EMERGENCY STOP: ON" : "EMERGENCY STOP: OFF");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyConversion>());
  rclcpp::shutdown();
  return 0;
}
