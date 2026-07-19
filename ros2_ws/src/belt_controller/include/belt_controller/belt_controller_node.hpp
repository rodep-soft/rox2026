#pragma once

#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/int16.hpp>

// BeltControllerNode
//
// /joy_second のボタン入力を受け取り、belt用モータのRPM指令を
// /belt/rpm に publish するノード。
//
// 操作は誤入力を防ぐために enable_button + mode_button の同時押しで更新する。
// 新しい更新ボタンの組み合わせが押されていない間は、
// 前回決定した mode と rpm_value をそのまま publish し続ける。

// Stop, それ以外のベルトの速度をmodeとして管理する。
enum class BeltControllerMode {Stop, Angle1High, Angle1Low, Angle2JHigh, Angle2Low};

class BeltControllerNode : public rclcpp::Node
{
public:
  BeltControllerNode();

private:
  // /joyを受け取るためのsubscriberと、RPM値を出すためのpublisher。
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr rpm_publisher_;

  // /joy_second topicにmessageが届いた時に呼ばれる処理。
  void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg);

  // DeclareParametersでparameter名と初期値を登録し、GetParametersで実際の値を読む。
  void DeclareParameters();
  void GetParameters();

  // Joy msgのbuttons配列を範囲チェックしてから押下状態を返す。
  bool IsButtonPressed(const sensor_msgs::msg::Joy::SharedPtr joy_msg, int button_index) const;

  // modeに対応するRPMパラメータを返す。
  int16_t GetRpmValueFromMode(BeltControllerMode mode) const;

  // Joy messageのbuttons配列で使うindex。
  int enable_button_;
  int stop_button_;
  int circle_button_;
  int cross_button_;
  int triangle_button_;
  int square_button_;

  // 各modeに対応するRPM値。config.yamlで変更できる。
  int stop_rpm_;
  int circle_rpm_;
  int cross_rpm_;
  int triangle_rpm_;
  int square_rpm_;

  // 最後に有効だったコマンド。新しい更新入力がない時はこの値を継続する。
  BeltControllerMode current_mode_;
  int16_t current_rpm_value_;
};
