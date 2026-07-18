#include <chrono>
#include <memory>
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
  void setEmergencyStop(bool emergency_stop);
  void joySecondPubTimerCallback(); /// joy_second_pub_ timer_callback

  // メンバ変数
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_second_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_pub_;
  rclcpp::TimerBase::SharedPtr joy_second_pub_timer_;

  sensor_msgs::msg::Joy::SharedPtr joy_msg;
  std_msgs::msg::Bool emergency_stop_msg;
  rclcpp::Time last_joy_time_;
  bool prev_button_state_ = false;
  int emergency_button_index_;
  int joy_timeout_ms_;
  int ready_start_joy_;
};

JoyConversion::JoyConversion()
: Node("joy_conversion_node")
{
  // パラメータ宣言
  this->declare_parameter<int>("emergency_button_index", 1);
  this->declare_parameter<int>("joy_timeout_ms", 500);
  std::string subscribed_topic = this->declare_parameter<std::string>("subscribed_topic", "/joy");
  std::string published_topic = this->declare_parameter<std::string>(
    "published_topic",
    "/joy_second");

  emergency_button_index_ = this->get_parameter("emergency_button_index").as_int();
  joy_timeout_ms_ = this->get_parameter("joy_timeout_ms").as_int();

  // サブスクライバー生成
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    subscribed_topic, 10,
    std::bind(&JoyConversion::joyCallback, this, std::placeholders::_1));

  // パブリッシャー生成
  joy_second_pub_ = this->create_publisher<sensor_msgs::msg::Joy>(published_topic, 10);
  emergency_pub_ = this->create_publisher<std_msgs::msg::Bool>("/emergency_stop", 10);

  // 起動時は非常停止
  setEmergencyStop(true);

  joy_second_pub_timer_ = this->create_wall_timer(
    10ms,
    std::bind(&JoyConversion::joySecondPubTimerCallback, this));

  ready_start_joy_ = this->now();
  
  last_joy_time_ = this->now();
}

void JoyConversion::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  last_joy_time_ = this->now();
  joy_msg = msg;
}

// joyの入力に間があったら停止
void JoyConversion::checkJoyTimeout()
{ // タイムアウト監視
  // difference_ms
  auto diff_ms = (this->now() - last_joy_time_).nanoseconds() / 1'000'000;

  if (diff_ms > joy_timeout_ms_) {
    if (!emergency_stop_msg.data) {
      setEmergencyStop(false);//デバックのため変更
      RCLCPP_WARN(this->get_logger(), "/joy timeout → emergency_stop ON");
    }
  }
}

// 非常停止の状態（ON/OFF）を /emergency_stop トピックに送信する関数
void JoyConversion::setEmergencyStop(bool emergency_stop)
{
  emergency_stop_msg.data = emergency_stop;
  RCLCPP_INFO(
    this->get_logger(),
    emergency_stop ? "EMERGENCY STOP: ON" : "EMERGENCY STOP: OFF");
}

void JoyConversion::joySecondPubTimerCallback()
{

// 100ms
  if (!joy_msg &&
    ((this->now() - ready_start_joy_).nanoseconds() / 1'000'000) > 100) {
    setEmergencyStop(true);
    emergency_pub_->publish(emergency_stop_msg);
    return;
  }
  bool button_pressed = joy_msg->buttons[emergency_button_index_] == 1;

  // トグル（立ち上がり検出）
  if (button_pressed && !prev_button_state_) {
    setEmergencyStop(!emergency_stop_msg.data);
  }
  // 前回の状態
  prev_button_state_ = button_pressed;
  checkJoyTimeout();

  // /joy_second に転送
  // /joy_second用のJoyメッセージを作成
  sensor_msgs::msg::Joy joy_second_msg;

  if (emergency_stop_msg.data) {
    joy_second_msg.header = joy_msg->header;
    // axesをすべて0
    joy_second_msg.axes.resize(joy_msg->axes.size(), 0.0f);
    // buttonsをすべて0
    joy_second_msg.buttons.resize(joy_msg->buttons.size(), 0);
// RCLCPP_INFO(this->get_logger(), "%d", emergency_stop_msg.data);
  } else {
    joy_second_msg = *joy_msg;
  }

  // /joy_secondへ送信
  joy_second_pub_->publish(joy_second_msg);
  emergency_pub_->publish(emergency_stop_msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyConversion>());
  rclcpp::shutdown();
  return 0;
}
