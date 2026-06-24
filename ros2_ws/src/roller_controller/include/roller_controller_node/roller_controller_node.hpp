#pragma once

#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/int16.hpp>

// RollerControllerNode
//
// /joy のボタン入力を受け取り、ローラー用のPWM値を
// /mabuchi555/pwm_value に publish するノード。
//
// enable_button + direction_button の同時押しで回転方向を更新する。
// 新しい更新ボタンの組み合わせが押されていない間は、
// 前回決定した mode と pwm_value をそのまま publish し続ける。

// Rollerの回転状態を管理するためのmode。
enum class RotationMode {Stop, PositiveRotate, NegativeRotate};

// 現在有効なmodeと、そのmodeから計算したpwm値をまとめて保持する。
struct RollerCommand
{
  RotationMode mode;
  int16_t pwm_value;
};

class RollerControllerNode : public rclcpp::Node
{
public:
  RollerControllerNode();

private:
  // /joyを受け取るためのsubscriberと、PWM値を出すためのpublisher。
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pwm_publisher_;

  // /joy topicにmessageが届いた時に呼ばれる処理。
  void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg);

  // DeclareParametersでparameter名と初期値を登録し、GetParametersで実際の値を読む。
  void DeclareParameters();
  void GetParameters();

  // Joy msgのbuttons配列を範囲チェックしてから押下状態を返す。
  bool IsButtonPressed(const sensor_msgs::msg::Joy::SharedPtr joy_msg, int button_index) const;

  // modeに対応するpwmパラメータを返す。ローラーは正逆回転があるため-255~255に丸める。
  int16_t GetPwmValueFromMode(RotationMode mode) const;

  // Joy messageのbuttons配列で使うindex。
  // enable_button_ + 各buttonの同時押しでPositive, Negative, Stopのmodeを判断する。
  int enable_button_;
  int positive_button_;
  int negative_button_;
  int stop_button_;

  // 各modeに対応するPWM値。config.yamlで変更できる。
  int positive_pwm_;
  int negative_pwm_;
  int stop_pwm_;

  // 最後に有効だったコマンド。新しい更新入力がない時はこの値を継続する。
  RollerCommand current_command_;
};
