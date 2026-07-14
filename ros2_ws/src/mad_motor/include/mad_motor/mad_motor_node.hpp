#pragma once

#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/int16.hpp>

// MadMotorNode
//
// /joy のボタン入力を受け取り、MADモータ用のPWM値を
// /mad_motor/rpm に publish するノード。
//
// 操作は誤入力を防ぐために enable_button + mode_button の同時押しで更新する。
// 新しい更新ボタンの組み合わせが押されていない間は、
// 前回決定した mode と pwm_value をそのまま publish し続ける。

// Stop, それ以外のベルトの速度をmodeとして管理する。
enum class MadMotorMode {Stop, Angle1High, Angle1Low, Angle2JHigh, Angle2Low};

class MadMotorNode : public rclcpp::Node
{
public:
  MadMotorNode();

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

  // modeに対応するpwmパラメータを返す。MADモータは0~255に丸める。
  int16_t GetPwmValueFromMode(MadMotorMode mode) const;

  // Joy messageのbuttons配列で使うindex。
  int enable_button_;
  int stop_button_;
  int circle_button_;
  int cross_button_;
  int triangle_button_;
  int square_button_;

  // 各modeに対応するPWM値。config.yamlで変更できる。
  int stop_pwm_;
  int circle_pwm_;
  int cross_pwm_;
  int triangle_pwm_;
  int square_pwm_;

  // 最後に有効だったコマンド。新しい更新入力がない時はこの値を継続する。
  MadMotorMode current_mode_;
  int16_t current_pwm_value_;
};
