#include "roller_controller_node/roller_controller_node.hpp"

#include <algorithm>
#include <functional>
#include <memory>

// /joy の入力からローラーの回転方向とPWM値を決めるnode。
//
// ROS 2では、node同士がtopicを通してmessageをやり取りする。
// このnodeは /joy をsubscribeしてコントローラ入力を受け取り、
// 決定したPWM値を /mabuchi555/pwm_value にpublishする。
//
// 誤操作防止のため、enable_buttonとdirection_buttonが同時に押された時だけ
// 回転方向を更新する。それ以外のJoy入力では前回のmode/pwm_valueを継続する。

RollerControllerNode::RollerControllerNode()
: Node("roller_controller_node")
{
  // parameterはlaunch fileやconfig.yamlから変更できる設定値。
  // declareしてからgetする、という順番がROS 2 C++の基本的な使い方。
  DeclareParameters();
  GetParameters();

  // 起動直後は安全側としてStopを現在コマンドにする。
  current_command_.mode = RotationMode::Stop;
  current_command_.pwm_value = GetPwmValueFromMode(RotationMode::Stop);

  // /joy topicにmessageが届くたびにJoyCallbackが呼ばれる。
  // 第2引数の10はqueue sizeで、処理待ちmessageを最大10個まで保持する。
  joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 10,
    std::bind(&RollerControllerNode::JoyCallback, this, std::placeholders::_1));

  // ローラー用MabuchiモータのPWM値をpublishする。
  // このnodeはPWM値を出すだけで、CAN変換はmotor_can_bridge側が担当する。
  pwm_publisher_ = this->create_publisher<std_msgs::msg::Int16>(
    "/mabuchi555/pwm_value", 10);

  RCLCPP_INFO(this->get_logger(), "RollerControllerNode started.");
}

void RollerControllerNode::DeclareParameters()
{
  // ボタン番号はJoy messageのbuttons配列のindex。
  // コントローラの割り当てを変えたい時はconfig.yaml側を変更する。
  this->declare_parameter<int>("enable_button", 1);
  this->declare_parameter<int>("positive_button", 2);
  this->declare_parameter<int>("negative_button", 3);
  this->declare_parameter<int>("stop_button", 0);

  // 各modeに対応するPWM値。ローラーは正逆回転があるため負の値も使う。
  this->declare_parameter<int>("positive_pwm", 200);
  this->declare_parameter<int>("negative_pwm", -200);
  this->declare_parameter<int>("stop_pwm", 0);
}

void RollerControllerNode::GetParameters()
{
  // declare_parameterで用意した値を、実際にメンバ変数へ読み込む。
  // 以降の処理ではROS parameterを直接読まず、このメンバ変数を使う。
  enable_button_ = this->get_parameter("enable_button").as_int();
  positive_button_ = this->get_parameter("positive_button").as_int();
  negative_button_ = this->get_parameter("negative_button").as_int();
  stop_button_ = this->get_parameter("stop_button").as_int();

  positive_pwm_ = this->get_parameter("positive_pwm").as_int();
  negative_pwm_ = this->get_parameter("negative_pwm").as_int();
  stop_pwm_ = this->get_parameter("stop_pwm").as_int();
}

// Joy messageのbuttons配列を見て、指定したボタンが押されているかを判断する。
bool RollerControllerNode::IsButtonPressed(
  const sensor_msgs::msg::Joy::SharedPtr msg,
  int button_index) const
{
  // configのbutton_indexがJoy msgのbuttons配列外なら、押されていない扱いにする。
  if (button_index < 0 || button_index >= static_cast<int>(msg->buttons.size())) {
    RCLCPP_WARN(this->get_logger(), "button_index is not valid");
    return false;
  }

  // buttons[button_index]は、押されていれば1、押されていなければ0になる。
  return msg->buttons[button_index] == 1;
}

// /joyを受け取ったときに呼ばれるcallback。
// Joy messageからRollerのmodeを判断し、Int16のPWM値をpublishする。
void RollerControllerNode::JoyCallback(
  const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // callbackは /joy が届くたびに1回実行される。
  // ここで「今回の入力から新しいmodeを決めるか」を判断する。

  // 初期値を現在値にしておくことで、新しい更新入力がない場合は前回値を維持する。
  RotationMode next_mode = current_command_.mode;
  bool has_new_command = false;

  // enableボタンが押されているかどうか。押されていない時はmodeを変更しない。
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

  // publishするmessageを作る。std_msgs::msg::Int16はdataフィールドだけを持つ単純な型。
  std_msgs::msg::Int16 pwm_msg;
  pwm_msg.data = current_command_.pwm_value;

  // 他のnodeは /mabuchi555/pwm_value をsubscribeすることで、このPWM値を受け取れる。
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
  // ROS 2の初期化。nodeを作る前に必ず呼ぶ。
  rclcpp::init(argc, argv);

  // spin中はcallback待ち状態になる。/joyが届くたびにJoyCallbackが呼ばれる。
  rclcpp::spin(std::make_shared<RollerControllerNode>());

  // Ctrl+Cなどでspinが終わった後、ROS 2を終了処理する。
  rclcpp::shutdown();
  return 0;
}
