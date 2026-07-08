#include "mad_motor/belt_controller_node.hpp"

#include <algorithm>
#include <functional>
#include <memory>

// /joy の入力からbelt用MADモータのPWM値を決めるnode。
//
// ROS 2では、node同士がtopicを通してmessageをやり取りする。
// このnodeは /joy をsubscribeしてコントローラ入力を受け取り、
// 決定したPWM値を /belt/rpm_value にpublishする。
//
// 誤操作防止のため、enable_buttonと各mode_buttonが同時に押された時だけ
// modeを更新する。それ以外のJoy入力では前回のmode/pwm_valueを継続する。

BeltControllerNode::BeltControllerNode()
: Node("belt_controller_node")
{
  // parameterはlaunch fileやconfig.yamlから変更できる設定値。
  // declareしてからgetする、という順番がROS 2 C++の基本的な使い方。
  DeclareParameters();
  GetParameters();

  // 起動直後は安全側として停止状態から始める。
  current_mode_ = BeltControllerMode::Stop;
  current_pwm_value_ = GetPwmValueFromMode(current_mode_);

  // /joy topicにmessageが届くたびにJoyCallbackが呼ばれる。
  // 第2引数の10はqueue sizeで、処理待ちmessageを最大10個まで保持する。
  joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 10,
    std::bind(&BeltControllerNode::JoyCallback, this, std::placeholders::_1));

  // MADモータ用のPWM値をpublishする。
  // このnodeはPWM値を出すだけで、CAN変換はmotor_can_bridge側が担当する。
  pwm_publisher_ = this->create_publisher<std_msgs::msg::Int16>(
    "/belt/rpm_value", 10);

  RCLCPP_INFO(this->get_logger(), "BeltControllerNode started.");
}

void BeltControllerNode::DeclareParameters()
{
  // ボタン番号はJoy messageのbuttons配列のindex。
  // コントローラの割り当てを変えたい時はconfig.yaml側を変更する。
  this->declare_parameter<int>("enable_button", 7);
  this->declare_parameter<int>("stop_button", 4);
  this->declare_parameter<int>("circle_button", 1);
  this->declare_parameter<int>("cross_button", 0);
  this->declare_parameter<int>("triangle_button", 2);
  this->declare_parameter<int>("square_button", 3);

  // 各modeに対応するPWM値。MADモータでは0~255の範囲に丸めて使う。
  this->declare_parameter<int>("stop_pwm", 0);
  this->declare_parameter<int>("circle_pwm", 0);
  this->declare_parameter<int>("cross_pwm", 85);
  this->declare_parameter<int>("triangle_pwm", 170);
  this->declare_parameter<int>("square_pwm", 255);
}

void BeltControllerNode::GetParameters()
{
  // declare_parameterで用意した値を、実際にメンバ変数へ読み込む。
  // 以降の処理ではROS parameterを直接読まず、このメンバ変数を使う。
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

bool BeltControllerNode::IsButtonPressed(
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

void BeltControllerNode::JoyCallback(
  const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // callbackは /joy が届くたびに1回実行される。
  // ここで「今回の入力から新しいmodeを決めるか」を判断する。

  // 初期値を現在値にしておくことで、新しい更新入力がない場合は前回値を維持する。
  BeltControllerMode next_mode = current_mode_;
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
      next_mode = BeltControllerMode::Stop;
      has_new_command = true;
    } else if (is_circle_pressed) {
      next_mode = BeltControllerMode::Angle1High;
      has_new_command = true;
    } else if (is_cross_pressed) {
      next_mode = BeltControllerMode::Angle1Low;
      has_new_command = true;
    } else if (is_triangle_pressed) {
      next_mode = BeltControllerMode::Angle2JHigh;
      has_new_command = true;
    } else if (is_square_pressed) {
      next_mode = BeltControllerMode::Angle2Low;
      has_new_command = true;
    }
  }

  // 新しいコマンドが確定した時だけ、保持しているmode/pwm_valueを書き換える。
  if (has_new_command) {
    current_mode_ = next_mode;
    current_pwm_value_ = GetPwmValueFromMode(next_mode);
  }

  // publishするmessageを作る。std_msgs::msg::Int16はdataフィールドだけを持つ単純な型。
  std_msgs::msg::Int16 pwm_msg;
  pwm_msg.data = current_pwm_value_;

  // belt_can_command_node は /belt/rpm_value をsubscribeして、このPWM値を受け取る。
  pwm_publisher_->publish(pwm_msg);
}

int16_t BeltControllerNode::GetPwmValueFromMode(BeltControllerMode mode) const
{
  int pwm_value = stop_pwm_;

  switch (mode) {
    case BeltControllerMode::Angle1High:
      pwm_value = circle_pwm_;
      break;

    case BeltControllerMode::Angle1Low:
      pwm_value = cross_pwm_;
      break;

    case BeltControllerMode::Angle2JHigh:
      pwm_value = triangle_pwm_;
      break;

    case BeltControllerMode::Angle2Low:
      pwm_value = square_pwm_;
      break;

    case BeltControllerMode::Stop:
    default:
      pwm_value = stop_pwm_;
      break;
  }

  // MADモータは逆回転を使わないため、範囲外の設定値は0~255に丸める。
  pwm_value = std::clamp(pwm_value, 0, 255);

  return static_cast<int16_t>(pwm_value);
}

int main(int argc, char * argv[])
{
  // ROS 2の初期化。nodeを作る前に必ず呼ぶ。
  rclcpp::init(argc, argv);

  // spin中はcallback待ち状態になる。/joyが届くたびにJoyCallbackが呼ばれる。
  rclcpp::spin(std::make_shared<BeltControllerNode>());

  // Ctrl+Cなどでspinが終わった後、ROS 2を終了処理する。
  rclcpp::shutdown();
  return 0;
}
