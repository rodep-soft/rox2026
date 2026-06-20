#include "roller_controller_node/roller_controller_node.hpp"

#include <algorithm>
#include <functional>
#include <memory>

// /joy の入力からローラーの回転方向とPWM値を決める。
// enable_button と各direction_buttonが同時に押された時だけ更新し、
// それ以外のJoy入力では前回のmode/pwm_valueを継続する。

RollerControllerNode::RollerControllerNode()
: Node("roller_controller_node")
{
  DeclareParameters();
  GetParameters();

  // 起動直後は安全側としてStopを現在コマンドにする。
  current_command_.mode = RotationMode::Stop;
  current_command_.pwm_value = GetPwmValueFromMode(RotationMode::Stop);

  joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 10,
    std::bind(&RollerControllerNode::JoyCallback, this, std::placeholders::_1));

  pwm_publisher_ = this->create_publisher<std_msgs::msg::Int16>(
    "/mabuchi555/pwm_value", 10);

  RCLCPP_INFO(this->get_logger(), "RollerControllerNode started.");
}

void RollerControllerNode::DeclareParameters()
{
  this->declare_parameter<int>("enable_button", 1);
  this->declare_parameter<int>("positive_button", 2);
  this->declare_parameter<int>("negative_button", 3);
  this->declare_parameter<int>("stop_button", 0);

  this->declare_parameter<int>("positive_pwm", 200);
  this->declare_parameter<int>("negative_pwm", -200);
  this->declare_parameter<int>("stop_pwm", 0);
}

void RollerControllerNode::GetParameters()
{
  enable_button_ = this->get_parameter("enable_button").as_int();
  positive_button_ = this->get_parameter("positive_button").as_int();
  negative_button_ = this->get_parameter("negative_button").as_int();
  stop_button_ = this->get_parameter("stop_button").as_int();

  positive_pwm_ = this->get_parameter("positive_pwm").as_int();
  negative_pwm_ = this->get_parameter("negative_pwm").as_int();
  stop_pwm_ = this->get_parameter("stop_pwm").as_int();
}

// 押されてるかどうかを判断
bool RollerControllerNode::IsButtonPressed(
  const sensor_msgs::msg::Joy::SharedPtr msg,
  int button_index) const
{
  // configのbutton_indexがJoy msgのbuttons配列外なら、押されていない扱いにする。
  if (button_index < 0 || button_index >= static_cast<int>(msg->buttons.size())) {
    RCLCPP_WARN(this->get_logger(), "button_index is not valid");
    return false;
  }

  // buttonsのbutton_indexが押されていたらtrue
  return msg->buttons[button_index] == 1;
}

// '/joy'を受け取ったときに呼ばれるcallback
// joyからのmsgでRollerのmodeを判断
// int16_tでpwmの値をpublishする
// l2 + 〇, △, ×でそれぞれのモードを判断する
void RollerControllerNode::JoyCallback(
  const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // 初期値を現在値にしておくことで、新しい更新入力がない場合は前回値を維持する。
  RotationMode next_mode = current_command_.mode;
  bool has_new_command = false;

  // l2ボタンが押されてるかどうかを判断する変数
  const bool is_enable_pressed = IsButtonPressed(msg, enable_button_);

  // enable_button単体では更新しない。enable + direction_button の時だけ新コマンドにする。
  if (is_enable_pressed) {
    // それぞれのボタンが押されていてどのモードにするのかを判断するための変数
    const bool is_negative_pressed = IsButtonPressed(msg, negative_button_);
    const bool is_stop_pressed = IsButtonPressed(msg, stop_button_);
    const bool is_positive_pressed = IsButtonPressed(msg, positive_button_);

    if (is_negative_pressed) {
      next_mode = RotationMode::NegativeRotate;
      has_new_command = true;
    } else if (is_stop_pressed) {
      next_mode = RotationMode::Stop;
      has_new_command = true;
    } else if (is_positive_pressed) {
      next_mode = RotationMode::PositiveRotate;
      has_new_command = true;
    }

  }

  // 新しいコマンドが確定した時だけ、保持しているmode/pwm_valueを書き換える。
  if (has_new_command) {
    current_command_.mode = next_mode;
    current_command_.pwm_value = GetPwmValueFromMode(next_mode);

  }

  //int16型のmsgを作成し、publish
  std_msgs::msg::Int16 pwm_msg;
  pwm_msg.data = current_command_.pwm_value;

  pwm_publisher_->publish(pwm_msg);
}

// 引数のmodeによってpwmの値を返す
int16_t RollerControllerNode::GetPwmValueFromMode(RotationMode mode) const
{
  int pwm_value = stop_pwm_;

  switch (mode) {
    case RotationMode::PositiveRotate:
      pwm_value = positive_pwm_;
      break;

    case RotationMode::NegativeRotate:
      pwm_value = negative_pwm_;
      break;

    case RotationMode::Stop:
    default:
      pwm_value = stop_pwm_;
      break;
  }

  // std_msgs::msg::Int16で送るが、モータ指令として使う範囲は-255~255に制限する。
  pwm_value = std::clamp(pwm_value, -255, 255);

  return static_cast<int16_t>(pwm_value);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RollerControllerNode>());
  rclcpp::shutdown();
  return 0;
}
