#include "belt_controller/belt_controller_node.hpp"

#include <functional>
#include <memory>

// /joy の入力からbelt用モータのRPM値を決めるnode。
//
// ROS 2では、node同士がtopicを通してmessageをやり取りする。
// このnodeは /joy_second をsubscribeしてコントローラ入力を受け取り、
// 決定したRPM値を /belt/rpm にpublishする。
//
// 誤操作防止のため、enable_buttonと各mode_buttonが同時に押された時だけ
// modeを更新する。それ以外のJoy入力では前回のmode/rpm_valueを継続する。

BeltControllerNode::BeltControllerNode()
: Node("belt_controller_node")
{
  // parameterはlaunch fileやconfig.yamlから変更できる設定値。
  // declareしてからgetする、という順番がROS 2 C++の基本的な使い方。
  DeclareParameters();
  GetParameters();

  // 起動直後は安全側として停止状態から始める。
  current_mode_ = BeltControllerMode::Stop;
  current_rpm_value_ = GetRpmValueFromMode(current_mode_);

  // /joy_second topicにmessageが届くたびにJoyCallbackが呼ばれる。
  // 第2引数の10はqueue sizeで、処理待ちmessageを最大10個まで保持する。
  joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy_second", 10,
    std::bind(&BeltControllerNode::JoyCallback, this, std::placeholders::_1));

  // belt用のRPM値をpublishする。CAN変換はCAN packer側が担当する。
  rpm_publisher_ = this->create_publisher<std_msgs::msg::Int16>(
    "/belt/rpm", 10);

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

  // 各modeに対応するRPM値。
  this->declare_parameter<int>("stop_rpm", 0);
  this->declare_parameter<int>("circle_rpm", 0);
  this->declare_parameter<int>("cross_rpm", 3000);
  this->declare_parameter<int>("triangle_rpm", 4000);
  this->declare_parameter<int>("square_rpm", 5000);
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

  stop_rpm_ = this->get_parameter("stop_rpm").as_int();
  circle_rpm_ = this->get_parameter("circle_rpm").as_int();
  cross_rpm_ = this->get_parameter("cross_rpm").as_int();
  triangle_rpm_ = this->get_parameter("triangle_rpm").as_int();
  square_rpm_ = this->get_parameter("square_rpm").as_int();
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
  // callbackは /joy_second が届くたびに1回実行される。
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

  // 新しいコマンドが確定した時だけ、保持しているmode/rpm_valueを書き換える。
  if (has_new_command) {
    current_mode_ = next_mode;
    current_rpm_value_ = GetRpmValueFromMode(next_mode);
  }

  // publishするmessageを作る。std_msgs::msg::Int16はdataフィールドだけを持つ単純な型。
  std_msgs::msg::Int16 rpm_msg;
  rpm_msg.data = current_rpm_value_;

  // roller_belt_can_packer_node は /belt/rpm をsubscribeして、このRPM値を受け取る。
  rpm_publisher_->publish(rpm_msg);
}

int16_t BeltControllerNode::GetRpmValueFromMode(BeltControllerMode mode) const
{
  int rpm_value = stop_rpm_;

  switch (mode) {
    case BeltControllerMode::Angle1High:
      rpm_value = circle_rpm_;
      break;

    case BeltControllerMode::Angle1Low:
      rpm_value = cross_rpm_;
      break;

    case BeltControllerMode::Angle2JHigh:
      rpm_value = triangle_rpm_;
      break;

    case BeltControllerMode::Angle2Low:
      rpm_value = square_rpm_;
      break;

    case BeltControllerMode::Stop:
    default:
      rpm_value = stop_rpm_;
      break;
  }

  return static_cast<int16_t>(rpm_value);
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
