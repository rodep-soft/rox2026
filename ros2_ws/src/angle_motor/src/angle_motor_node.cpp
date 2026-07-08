#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int16.hpp"

#include "angle_motor/angle_motor_node.hpp"

RollerControllerNode::RollerControllerNode()
: Node("roller_controller_node")
{
  declareParameters();
  getParameters();

  current_command_.mode = RotationMode::Stop;
  current_command_.pwm_value = stop_pwm_;

  pwm_publisher_ = this->create_publisher<std_msgs::msg::Int16>("/roller/rpm_value", 10);
  joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 10, std::bind(&RollerControllerNode::joyCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "RollerControllerNode has been initialized with Mabuchi specs.");
}

void RollerControllerNode::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
{
  // ヌルポインタチェック（防御的プログラミング）
  if (!joy_msg) {
    return;
  }

  // 1. 判定ロジックの呼び出し (SharedPtrではなく実体の参照を渡す。引数から前回状態を削除)
  current_command_.mode = determineRotationMode(*joy_msg);

  // 2. モードからPWM値を計算
  current_command_.pwm_value = getPwmValueFromMode(current_command_.mode);

  // 3. PWM値をパブリッシュ
  auto pwm_msg = std_msgs::msg::Int16();
  pwm_msg.data = current_command_.pwm_value;
  pwm_publisher_->publish(pwm_msg);
}

// ボタン入力から次のモードを判定する関数（優先順位: Stop > Negative > Positive）
// enableが押されていない場合は安全のため一律Stopを返す
RotationMode RollerControllerNode::determineRotationMode(const sensor_msgs::msg::Joy & joy_msg)
{

  // 【指摘対応】早期リターンにより、Stop > Negative > Positive の優先順位をロジカルに保証
  if (isButtonPressed(joy_msg, stop_button_)) {
    return RotationMode::Stop;
  }

  if (isButtonPressed(joy_msg, negative_button_)) {
    return RotationMode::NegativeRotate;
  }

  if (isButtonPressed(joy_msg, positive_button_)) {
    return RotationMode::PositiveRotate;
  }

  // どのボタンも押されていない場合は現在のモードを維持（クラスメンバから直接参照）
  return current_command_.mode;
}

// モードに対応するPWM値を返す関数
int16_t RollerControllerNode::getPwmValueFromMode(RotationMode mode) const
{
  switch (mode) {
    case RotationMode::PositiveRotate: return positive_pwm_;
    case RotationMode::NegativeRotate: return negative_pwm_;
    default:                           return stop_pwm_;
  }
}

// 安全にボタン状態をチェックする関数 (SharedPtrではなく参照を受けるように変更)
bool RollerControllerNode::isButtonPressed(
  const sensor_msgs::msg::Joy & joy_msg,
  int button_index)
{
  // 【指摘対応】範囲チェックはここだけで完全に保証する
  if (button_index < 0 || static_cast<size_t>(button_index) >= joy_msg.buttons.size()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(), 1000, "Button index %d out of bounds!", button_index);
    return false;
  }
  return joy_msg.buttons[button_index] == 1;
}

// パラメータ宣言
void RollerControllerNode::declareParameters()
{
  this->declare_parameter<int>("enable_button", 6);
  this->declare_parameter<int>("positive_button", 2);
  this->declare_parameter<int>("negative_button", 0);
  this->declare_parameter<int>("stop_button", 1);
  this->declare_parameter<int>("positive_pwm", 200);
  this->declare_parameter<int>("negative_pwm", -200);
  this->declare_parameter<int>("stop_pwm", 0);
}

// パラメータ取得
void RollerControllerNode::getParameters()
{
  this->get_parameter("enable_button", enable_button_);
  this->get_parameter("positive_button", positive_button_);
  this->get_parameter("negative_button", negative_button_);
  this->get_parameter("stop_button", stop_button_);
  this->get_parameter("positive_pwm", positive_pwm_);
  this->get_parameter("negative_pwm", negative_pwm_);
  this->get_parameter("stop_pwm", stop_pwm_);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RollerControllerNode>());
  rclcpp::shutdown();
  return 0;
}
