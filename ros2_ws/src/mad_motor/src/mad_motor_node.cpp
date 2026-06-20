#include "mad_motor/mad_motor_node.hpp"

#include <algorithm>
#include <functional>
#include <memory>

// /joy の入力からMADモータのPWM値を決める。
// enable_button と各mode_buttonが同時に押された時だけ更新し、
// それ以外のJoy入力では前回のmode/pwm_valueを継続する。

MadMotorNode::MadMotorNode()
: Node("mad_motor_node")
{
  DeclareParameters();
  GetParameters();

  current_mode_ = MadMotorMode::Stop;
  current_pwm_value_ = GetPwmValueFromMode(current_mode_);

  joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 10,
    std::bind(&MadMotorNode::JoyCallback, this, std::placeholders::_1));

  pwm_publisher_ = this->create_publisher<std_msgs::msg::UInt8>(
    "/mad_motor/pwm_value", 10);

  RCLCPP_INFO(this->get_logger(), "MadMotorNode started.");
}

void MadMotorNode::DeclareParameters()
{
  this->declare_parameter<int>("enable_button", 7);
  this->declare_parameter<int>("stop_button", 4);
  this->declare_parameter<int>("circle_button", 1);
  this->declare_parameter<int>("cross_button", 0);
  this->declare_parameter<int>("triangle_button", 2);
  this->declare_parameter<int>("square_button", 3);

  this->declare_parameter<int>("stop_pwm", 0);
  this->declare_parameter<int>("circle_pwm", 0);
  this->declare_parameter<int>("cross_pwm", 85);
  this->declare_parameter<int>("triangle_pwm", 170);
  this->declare_parameter<int>("square_pwm", 255);
}

void MadMotorNode::GetParameters()
{
  enable_button_ = this->get_parameter("enable_button").as_int();
  stop_button_ = this->get_parameter("stop_button").as_int();
  circle_button_ = this->get_parameter("circle_button").as_int();
  cross_button_ = this->get_parameter("cross_button").as_int();
  triangle_button_ = this->get_parameter("triangle_button").as_int();
  square_button_ = this->get_parameter("square_button").as_int();

  stop_pwm_ = this->get_parameter("stop_pwm").as_int();
  circle_pwm_ = this->get_parameter("circle_pwm").as_int();
  cross_pwm_ = this->get_parameter("cross_pwm").as_int();
  triangle_pwm_ = this->get_parameter("triangle_pwm").as_int();
  square_pwm_ = this->get_parameter("square_pwm").as_int();
}

bool MadMotorNode::IsButtonPressed(
  const sensor_msgs::msg::Joy::SharedPtr msg,
  int button_index) const
{
  // configのbutton_indexがJoy msgのbuttons配列外なら、押されていない扱いにする。
  if (button_index < 0 || button_index >= static_cast<int>(msg->buttons.size())) {
    RCLCPP_WARN(this->get_logger(), "button_index is not valid");
    return false;
  }

  return msg->buttons[button_index] == 1;
}

void MadMotorNode::JoyCallback(
  const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // 初期値を現在値にしておくことで、新しい更新入力がない場合は前回値を維持する。
  MadMotorMode next_mode = current_mode_;
  bool has_new_command = false;

  const bool is_enable_pressed = IsButtonPressed(msg, enable_button_);

  // enable_button単体では更新しない。enable + mode_button の時だけ新コマンドにする。
  if (is_enable_pressed) {
    const bool is_stop_pressed = IsButtonPressed(msg, stop_button_);
    const bool is_circle_pressed = IsButtonPressed(msg, circle_button_);
    const bool is_cross_pressed = IsButtonPressed(msg, cross_button_);
    const bool is_triangle_pressed = IsButtonPressed(msg, triangle_button_);
    const bool is_square_pressed = IsButtonPressed(msg, square_button_);

    if (is_stop_pressed) {
      next_mode = MadMotorMode::Stop;
      has_new_command = true;
    } else if (is_circle_pressed) {
      next_mode = MadMotorMode::Angle1High;
      has_new_command = true;
    } else if (is_cross_pressed) {
      next_mode = MadMotorMode::Angle1Low;
      has_new_command = true;
    } else if (is_triangle_pressed) {
      next_mode = MadMotorMode::Angle2JHigh;
      has_new_command = true;
    } else if (is_square_pressed) {
      next_mode = MadMotorMode::Angle2Low;
      has_new_command = true;
    }
  }

  // 新しいコマンドが確定した時だけ、保持しているmode/pwm_valueを書き換える。
  if (has_new_command) {
    current_mode_ = next_mode;
    current_pwm_value_ = GetPwmValueFromMode(next_mode);
  }

  std_msgs::msg::UInt8 pwm_msg;
  pwm_msg.data = current_pwm_value_;

  pwm_publisher_->publish(pwm_msg);
}

uint8_t MadMotorNode::GetPwmValueFromMode(MadMotorMode mode) const
{
  int pwm_value = stop_pwm_;

  switch (mode) {
    case MadMotorMode::Angle1High:
      pwm_value = circle_pwm_;
      break;

    case MadMotorMode::Angle1Low:
      pwm_value = cross_pwm_;
      break;

    case MadMotorMode::Angle2JHigh:
      pwm_value = triangle_pwm_;
      break;

    case MadMotorMode::Angle2Low:
      pwm_value = square_pwm_;
      break;

    case MadMotorMode::Stop:
    default:
      pwm_value = stop_pwm_;
      break;
  }

  // std_msgs::msg::UInt8で送るため、範囲外の設定値はここで丸める。
  pwm_value = std::clamp(pwm_value, 0, 255);

  return static_cast<uint8_t>(pwm_value);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MadMotorNode>());
  rclcpp::shutdown();
  return 0;
}
