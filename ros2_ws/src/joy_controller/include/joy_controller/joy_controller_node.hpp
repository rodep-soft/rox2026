#ifndef JOY_CONTROLLER__JOY_CONTROLLER_NODE_HPP_
#define JOY_CONTROLLER__JOY_CONTROLLER_NODE_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"

class JoyControllerNode : public rclcpp::Node
{
 public:
  JoyControllerNode();

 private:
  enum class BeltRpmMode : uint8_t
  {
    STOP,
    LEVEL_1,
    LEVEL_2,
    LEVEL_3,
    LEVEL_4,
    LEVEL_5,
    LEVEL_6,
  };

  enum class OperationMode : uint8_t
  {
    STOP,
    DRIVE,
    SHOT_CYCLE,
    BELT_ONLY,
  };

  enum class Position : uint8_t
  {
    DRIBBLE,
    INTAKE,
    SHOT,
    OPEN,
  };

  void declare_parameters();
  void get_parameters();

  // /joy受信ごとに呼ばれる。ボタンの立上りでモード・ベルト・ドリブル状態を更新し、
  // SHOT_CYCLE中かつ未実行時だけcycle要求を出す。常に/mecanum/cmd_velをpublishする。
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  // /shot_cycle/running受信時に呼ばれる。現在がSHOT_CYCLEのときだけ実行中フラグを更新し、
  // 他モードで届いた通知は無視する。
  void shot_cycle_running_callback(const std_msgs::msg::Bool::SharedPtr msg);
  // /shot_cycle/completeのtrue受信時に呼ばれる。SHOT_CYCLE中だけ実行中フラグを解除し、
  // auto_drive_on_shot_cycle_completeがtrueならDRIVEへ遷移して状態topicをpublishする。
  void shot_cycle_complete_callback(const std_msgs::msg::Bool::SharedPtr msg);

  // 10 ms周期で呼ばれる。/joyを一度でも受信後、joy_timeout_msを超えて新規入力がなければ
  // 一度だけ停止状態へ遷移し、走行・発射・ベルト・ドリブルの停止指令をpublishする。
  void joy_timeout_timer_callback();
  // 設定周期で呼ばれる。入力断中は何も出さず、通常時は状態topicを再送する。
  // 発射要求はDRIVEかつ発射コード押下中のときだけtrueをpublishする。
  void state_publish_timer_callback();

  // PositionをUInt8へ変換して/dribble/position_modeへpublishする。手動位置・停止時の復帰に使う。
  void publish_position(Position position);
  // /shot_cycle/requestへtrueを一度publishし、belt_dribble_controllerに開始可否の判定を依頼する。
  void publish_shot_cycle_request();
  // /spring/fire_requestへrequestedをpublishする。timerからはDRIVE中のコード押下状態を送る。
  void publish_spring_fire_request(bool requested);
  // operation_mode_がSTOPのときだけ/emergency_stopへtrueをpublishする。
  void publish_emergency_stop();
  // operation_mode_をUInt8へ変換して/operation_modeへpublishする。
  void publish_operation_mode();
  // 選択済みベルトレベルを/belt/modeへpublishする。
  void publish_belt_mode();
  // ドリブルON/OFF状態を/dribble/enabledへpublishする。
  void publish_dribble_enabled();
  // STOPへ遷移し、走行速度・発射・ベルト・ドリブルを停止値でpublishする。
  void publish_stop_commands();
  // operation_mode_を変更し、operation_modeとemergency_stopの状態topicを直ちにpublishする。
  void set_operation_mode(OperationMode mode);
  // 現在のJoy配列から複数ボタンのコード状態を作る。立上り判定は前回値との比較で行う。
  void update_chord_inputs(const sensor_msgs::msg::Joy& msg);
  // HOMEはSTOP/DRIVEを切替、CREATEはSHOT_CYCLE、OPTIONSはBELT_ONLYを切替える。
  // shot cycle実行中はHOME以外のモード切替を受け付けない。
  void handle_operation_mode();
  // 非実行中かつDRIVEまたはSHOT_CYCLEのときだけ、手動位置指令を許可する。
  bool is_manual_position_allowed() const;
  // 今回のコード状態を保存し、次の/joy受信時に立上りを判定できるようにする。
  void update_previous_chord_inputs();

  // Joy配列の安全な参照、値変換、モードの増減を行う小さな補助関数。
  static bool button_pressed(const sensor_msgs::msg::Joy& msg, int index);
  static double axis_value(const sensor_msgs::msg::Joy& msg, int index);
  double apply_axis_deadzone(double value) const;
  double apply_lateral_axis_direction(double value) const;
  static double apply_axis_limit(double value, double limit);
  static uint8_t increment_mode(uint8_t mode, uint8_t maximum_mode);
  static uint8_t decrement_mode(uint8_t mode);

  int joy_qos_depth_{1};
  int command_qos_depth_{1};
  int joy_timeout_ms_{200};
  int state_publish_period_ms_{20};
  double linear_x_scale_{1.0};
  double linear_y_scale_{1.0};
  double angular_z_scale_{1.0};
  double linear_x_limit_{2.0};
  double linear_y_limit_{2.0};
  double angular_z_limit_{2.0};
  double axis_deadzone_{0.05};
  double lateral_axis_threshold_{0.7};
  double axis_on_threshold_{0.7};

  int spring_fire_enable_button_{4};
  int spring_fire_button_{2};
  int create_button_{8};
  int ps_button_{12};
  int options_button_{9};
  int home_button_{13};
  int circle_button_{2};
  int dribble_enable_button_{5};
  int left_trigger_axis_{3};
  int right_trigger_axis_{4};
  int left_stick_x_axis_{0};
  int left_stick_y_axis_{1};
  int right_stick_x_axis_{2};
  int dpad_horizontal_axis_{6};
  int dpad_vertical_axis_{7};

  geometry_msgs::msg::Twist cmd_vel_;
  OperationMode operation_mode_{OperationMode::STOP};
  bool shot_cycle_running_{false};
  bool auto_drive_on_shot_cycle_complete_{true};
  uint8_t belt_rpm_mode_{static_cast<uint8_t>(BeltRpmMode::STOP)};
  bool dribble_enabled_{false};
  bool forward_reverse_{false};
  bool joy_received_{false};
  bool joy_timeout_active_{false};
  std::chrono::steady_clock::time_point last_joy_received_time_{};

  bool spring_fire_chord_on_{false};
  bool belt_mode_up_chord_on_{false};
  bool belt_mode_down_chord_on_{false};
  bool dribble_enable_button_on_{false};
  bool home_button_on_{false};
  bool create_button_on_{false};
  bool options_button_on_{false};
  bool shot_cycle_chord_on_{false};
  bool manual_dribble_chord_on_{false};
  bool manual_open_chord_on_{false};
  bool forward_reverse_button_on_{false};

  bool pre_belt_mode_up_chord_on_{false};
  bool pre_belt_mode_down_chord_on_{false};
  bool pre_dribble_enable_button_on_{false};
  bool pre_home_button_on_{false};
  bool pre_create_button_on_{false};
  bool pre_options_button_on_{false};
  bool pre_shot_cycle_chord_on_{false};
  bool pre_manual_dribble_chord_on_{false};
  bool pre_manual_open_chord_on_{false};
  bool pre_forward_reverse_button_on_{false};

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      shot_cycle_running_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      shot_cycle_complete_subscription_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr operation_mode_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
      mecanum_cmd_vel_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr spring_fire_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr belt_mode_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr dribble_enabled_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
      shot_cycle_request_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr
      dribble_position_mode_publisher_;
  rclcpp::TimerBase::SharedPtr joy_timeout_timer_;
  rclcpp::TimerBase::SharedPtr state_publish_timer_;
};

#endif  // JOY_CONTROLLER__JOY_CONTROLLER_NODE_HPP_
