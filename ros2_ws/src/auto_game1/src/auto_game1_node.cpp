#include "auto_game1/auto_game1_node.hpp"

#include <cmath>
#include <functional>
#include <limits>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace auto_game1
{

AutoGame1Node::AutoGame1Node(const rclcpp::NodeOptions & options)
: Node("auto_game1_node", options)
{
  declare_parameters();
  get_parameters();

  // TF2 BufferとListenerの初期化（Map系からBase系への座標変換用）
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // /mecanum/cmd_vel (または /cmd_vel):
  // ロボットの走行用速度指令を出力する。
  // 送信先: robot_controller または hardware_driver / mecanum_controller
  // メッセージ: geometry_msgs/msg/Twist (linear.x, linear.y, angular.z)
  cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
    cmd_vel_topic_, rclcpp::QoS(10));

  // /dribble/enabled:
  // ドリブルモータの ON (true) / OFF (false) を制御するトピック Publisher。
  dribble_publisher_ = create_publisher<std_msgs::msg::Bool>(
    dribble_enabled_topic_, 10);

  // 障害物長方形 (PolygonStamped) の Publisher 3つを作成（TRANSIENT_LOCAL QoS）
  for (size_t i = 0; i < 3; ++i) {
    const std::string topic_name = "/obstacle_polygon_" + std::to_string(i + 1);
    obstacle_polygon_publishers_[i] = create_publisher<geometry_msgs::msg::PolygonStamped>(
      topic_name, rclcpp::QoS(1).transient_local());
  }
  // 起動時に固定障害物ポリゴンを1回だけラッチ送信
  publish_obstacle_polygons();

  // /joy:
  // ジョイスティックからの操作入力を受信する。
  // 送信元: joy_node (joystick_driver)
  // メッセージ: sensor_msgs/msg/Joy
  joy_subscription_ = create_subscription<sensor_msgs::msg::Joy>(
    joy_topic_, rclcpp::SensorDataQoS(),
    std::bind(&AutoGame1Node::joy_callback, this, std::placeholders::_1));

  // /operation_mode:
  // joy_controller からのモード切替を受信し、手動切替時に AUTO_STOP へ遷移させる。
  operation_mode_subscription_ = create_subscription<std_msgs::msg::UInt8>(
    operation_mode_topic_, rclcpp::QoS(10),
    std::bind(&AutoGame1Node::operation_mode_callback, this, std::placeholders::_1));

  // nav2 NavigateThroughPoses Action Client:
  // 複数通過点および単一目的地のナビゲーションを共通処理するNav2アクションクライアント。
  nav_client_ = rclcpp_action::create_client<NavigateThroughPoses>(
    this, nav_action_name_);

  // キック機構 Action Client:
  // ゲート通過中のボール射出（キック）を実行・完了検出するためのアクションクライアント。
  kick_client_ = rclcpp_action::create_client<Kick>(
    this, kick_action_name_);

  // ドリブル機構 Action Client:
  // ドリブルモータの回転開始(start=true)・停止(start=false)を実行・完了検出するためのアクションクライアント。
  dribble_client_ = rclcpp_action::create_client<Dribble>(
    this, dribble_action_name_);

  // パス機構 Action Client:
  // パスエリア到達時のボールリリース・パスクライアント。
  pass_client_ = rclcpp_action::create_client<Pass>(
    this, pass_action_name_);

  // 主制御ループタイマー
  control_timer_ = create_wall_timer(
    std::chrono::duration<double>(control_period_sec_),
    std::bind(&AutoGame1Node::control_timer_callback, this));
}

void AutoGame1Node::declare_parameters()
{
  // トピック・アクション名
  declare_parameter<std::string>("cmd_vel_topic", "/auto_game1/cmd_vel");
  declare_parameter<std::string>("joy_topic", "/joy");
  declare_parameter<std::string>("operation_mode_topic", "/operation_mode");
  declare_parameter<std::string>("dribble_enabled_topic", "/dribble/enabled");
  declare_parameter<std::string>("nav_action_name", "navigate_through_poses");
  declare_parameter<std::string>("kick_action_name", "kick");
  declare_parameter<std::string>("dribble_action_name", "dribble");
  declare_parameter<std::string>("pass_action_name", "pass");
  declare_parameter<std::string>("global_frame_id", "map");
  declare_parameter<std::string>("robot_base_frame_id", "base_link");

  // ボタン設定
  declare_parameter<int>("auto_stop_toggle_button", default_auto_stop_toggle_button);
  declare_parameter<int>("return_to_start_button", default_return_to_start_button);
  declare_parameter<int>("side_toggle_button", default_side_toggle_button);
  declare_parameter<int>("dribble_on_button", default_dribble_on_button);
  declare_parameter<int>("start_autodrive_button", default_start_autodrive_button);

  // 制御周期・判定パラメータ
  declare_parameter<double>("control_period_sec", 0.05);
  declare_parameter<double>("kick_start_reach_threshold", 0.2);

  // PREPARE_KICK用制御パラメータ
  declare_parameter<double>("kick_target_velocity_x", 0.5);
  declare_parameter<double>("kick_target_x", 1.5);
  declare_parameter<double>("kick_tolerance_x", 0.1);
  declare_parameter<double>("kick_target_y", 0.0);
  declare_parameter<double>("kick_target_yaw", 0.0);
  declare_parameter<double>("kp_y", 1.0);
  declare_parameter<double>("kd_y", 0.1);
  declare_parameter<double>("kp_yaw", 1.0);
  declare_parameter<double>("kd_yaw", 0.1);

  // 通過点・目標点の座標パラメータ（デフォルト値例）
  declare_parameter<double>("kick_start.x", 1.0);
  declare_parameter<double>("kick_start.y", 0.0);
  declare_parameter<double>("kick_end.x", 2.0);
  declare_parameter<double>("kick_end.y", 0.5);
  declare_parameter<double>("waypoint3.x", 3.0);
  declare_parameter<double>("waypoint3.y", 0.0);

  declare_parameter<double>("gate_far_side.x", 3.5);
  declare_parameter<double>("gate_far_side.y", 0.0);
  declare_parameter<double>("pass_area.x", 2.0);
  declare_parameter<double>("pass_area.y", -1.5);
  declare_parameter<double>("start_pose.x", 0.0);
  declare_parameter<double>("start_pose.y", 0.0);
}

void AutoGame1Node::get_parameters()
{
  get_parameter("cmd_vel_topic", cmd_vel_topic_);
  get_parameter("joy_topic", joy_topic_);
  get_parameter("operation_mode_topic", operation_mode_topic_);
  get_parameter("dribble_enabled_topic", dribble_enabled_topic_);
  get_parameter("nav_action_name", nav_action_name_);
  get_parameter("kick_action_name", kick_action_name_);
  get_parameter("dribble_action_name", dribble_action_name_);
  get_parameter("pass_action_name", pass_action_name_);
  get_parameter("global_frame_id", global_frame_id_);
  get_parameter("robot_base_frame_id", robot_base_frame_id_);

  get_parameter("auto_stop_toggle_button", auto_stop_toggle_button_);
  get_parameter("return_to_start_button", return_to_start_button_);
  get_parameter("side_toggle_button", side_toggle_button_);
  get_parameter("dribble_on_button", dribble_on_button_);
  get_parameter("start_autodrive_button", start_autodrive_button_);

  get_parameter("control_period_sec", control_period_sec_);
  get_parameter("kick_start_reach_threshold", kick_start_reach_threshold_);

  get_parameter("kick_target_velocity_x", kick_target_velocity_x_);
  get_parameter("kick_target_x", kick_target_x_);
  get_parameter("kick_tolerance_x", kick_tolerance_x_);
  get_parameter("kick_target_y", kick_target_y_);
  get_parameter("kick_target_yaw", kick_target_yaw_);
  get_parameter("kp_y", kp_y_);
  get_parameter("kd_y", kd_y_);
  get_parameter("kp_yaw", kp_yaw_);
  get_parameter("kd_yaw", kd_yaw_);

  // Poses の取得と組み立て
  const auto get_pose_param = [this](const std::string & prefix, geometry_msgs::msg::PoseStamped & pose) {
    double x = 0.0;
    double y = 0.0;
    get_parameter(prefix + ".x", x);
    get_parameter(prefix + ".y", y);
    pose.header.frame_id = global_frame_id_;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
  };

  get_pose_param("kick_start", kick_start_pose_);
  get_pose_param("kick_end", kick_end_pose_);
  get_pose_param("waypoint3", waypoint3_pose_);
  get_pose_param("gate_far_side", gate_far_side_pose_);
  get_pose_param("pass_area", pass_area_pose_);
  get_pose_param("start_pose", start_pose_);
}

geometry_msgs::msg::PoseStamped AutoGame1Node::apply_side_transform(
  const geometry_msgs::msg::PoseStamped & pose) const
{
  geometry_msgs::msg::PoseStamped transformed_pose = pose;
  double side_sign = get_side_sign();

  // X軸座標の左右反転
  transformed_pose.pose.position.x *= side_sign;

  // Yaw角の左右反転
  double yaw = tf2::getYaw(pose.pose.orientation);
  double transformed_yaw = yaw * side_sign;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, transformed_yaw);
  transformed_pose.pose.orientation = tf2::toMsg(q);

  return transformed_pose;
}

// Subscription Callback
// 受信トピック: /joy
// 役割: 手動による自動停止切り替え（toggle）、ドリブルON、発進許可、スタート復帰命令、およびコートサイド切替を検出する。
void AutoGame1Node::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  auto_stop_toggle_button_on_ = button_pressed(*msg, auto_stop_toggle_button_);
  return_to_start_button_on_ = button_pressed(*msg, return_to_start_button_);
  side_toggle_button_on_ = button_pressed(*msg, side_toggle_button_);
  dribble_on_button_on_ = button_pressed(*msg, dribble_on_button_);
  start_autodrive_button_on_ = button_pressed(*msg, start_autodrive_button_);

  // 自動停止切替ボタンの立ち上がり判定
  if (auto_stop_toggle_button_on_ && !pre_auto_stop_toggle_button_on_) {
    if (current_state_ == State::AUTO_STOP) {
      RCLCPP_INFO(get_logger(), "Joy input: Resuming from AUTO_STOP. Transitioning to DRIBBLE_ON (Standby).");
      reset_all_nav_goals();
      publish_dribble_enabled(false);
      current_state_ = State::DRIBBLE_ON;
    } else {
      RCLCPP_INFO(get_logger(), "Joy input: Auto stop requested. Canceling all active Nav2 goals.");
      previous_state_ = current_state_;
      current_state_ = State::AUTO_STOP;
      publish_dribble_enabled(false);
      reset_all_nav_goals();
    }
  }

  // ドリブルON状態 (DRIBBLE_ON) での操作判定
  if (current_state_ == State::DRIBBLE_ON) {
    // ドリブルONボタン (○/Circleボタン) の立ち上がり判定
    if (dribble_on_button_on_ && !pre_dribble_on_button_on_) {
      RCLCPP_INFO(get_logger(), "Joy input: Dribble ON button pressed. Starting Dribbler motor (/dribble/enabled = true).");
      publish_dribble_enabled(true);
    }

    // 自律移動発進ボタン (×/Crossボタン等) の立ち上がり判定
    if (start_autodrive_button_on_ && !pre_start_autodrive_button_on_) {
      RCLCPP_INFO(get_logger(), "Joy input: Start AutoDrive button pressed. Transitioning to GO_TO_KICK_START.");
      publish_dribble_enabled(true);
      current_state_ = State::GO_TO_KICK_START;
    }
  }

  // スタート地点復帰ボタンの立ち上がり判定
  if (return_to_start_button_on_ && !pre_return_to_start_button_on_) {
    RCLCPP_INFO(get_logger(), "Joy input: Return to start requested. Switching to RETURN_TO_START.");
    reset_all_nav_goals();
    current_state_ = State::RETURN_TO_START;
  }

  // コートサイド切替ボタン（△ / Yボタン）の立ち上がり判定
  if (side_toggle_button_on_ && !pre_side_toggle_button_on_) {
    if (current_side_ == Side::SIDE_A) {
      current_side_ = Side::SIDE_B;
      RCLCPP_INFO(
        get_logger(),
        "Court Side switched to: SIDE_B (Left course, inverted X-axis)");
    } else {
      current_side_ = Side::SIDE_A;
      RCLCPP_INFO(
        get_logger(),
        "Court Side switched to: SIDE_A (Right course, standard X-axis)");
    }
    publish_obstacle_polygons();
  }

  pre_auto_stop_toggle_button_on_ = auto_stop_toggle_button_on_;
  pre_return_to_start_button_on_ = return_to_start_button_on_;
  pre_side_toggle_button_on_ = side_toggle_button_on_;
  pre_dribble_on_button_on_ = dribble_on_button_on_;
  pre_start_autodrive_button_on_ = start_autodrive_button_on_;
}

void AutoGame1Node::operation_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  // operation_mode: 0=STOP, 1=DRIVE (手動), 2=SHOT_CYCLE (自動), 3=BELT_ONLY
  // 2 (SHOT_CYCLE 自動モード) 以外に変更された場合（＝手動操作やSTOP切り替え時）は即座に AUTO_STOP へ落とす。
  constexpr uint8_t mode_shot_cycle = 2;
  if (msg->data != mode_shot_cycle) {
    if (current_state_ != State::AUTO_STOP) {
      RCLCPP_INFO(
        get_logger(),
        "Operation mode changed to Manual/Stop (%d). Switching Auto Drive to AUTO_STOP.",
        msg->data);
      previous_state_ = current_state_;
      current_state_ = State::AUTO_STOP;
      publish_dribble_enabled(false);
      reset_all_nav_goals();
    }
  }
}

// Timer Callback
// 周期: control_period_sec_ (デフォルト 20Hz / 50ms)
// 役割: 現在のステートに応じた処理・遷移判定を実行し、必要に応じて cmd_vel を publish する。
void AutoGame1Node::control_timer_callback()
{
  process_state_machine();
}

// 主処理: 状態遷移マシーン (State Machine)
void AutoGame1Node::process_state_machine()
{
  switch (current_state_) {
    case State::AUTO_STOP:
      process_auto_stop();
      break;
    case State::DRIBBLE_ON:
      process_dribble_on();
      break;
    case State::GO_TO_KICK_START:
      process_go_to_kick_start();
      break;
    case State::PREPARE_KICK:
      process_prepare_kick();
      break;
    case State::GO_TO_GATE_FAR_SIDE:
      process_go_to_gate_far_side();
      break;
    case State::FOLLOW_BALL:
      process_follow_ball();
      break;
    case State::CARRY_BALL_TO_PASS_AREA:
      process_carry_ball_to_pass_area();
      break;
    case State::RETURN_TO_START:
      process_return_to_start();
      break;
  }
}

void AutoGame1Node::process_auto_stop()
{
  // 自動停止状態: アクティブなNav2のアクションを確実にキャンセルし、走行速度0を出力する
  reset_all_nav_goals();
  publish_dribble_enabled(false);
  geometry_msgs::msg::Twist stop_cmd;
  cmd_vel_publisher_->publish(stop_cmd);
}

void AutoGame1Node::process_dribble_on()
{
  // 0. ドリブルON状態 (Standby)
  // 概要: 安全な待機状態。走行速度 0 を維持し、Joyボタン入力 (dribble_on_button / start_autodrive_button) を待つ。
  geometry_msgs::msg::Twist stop_cmd;
  cmd_vel_publisher_->publish(stop_cmd);
}

void AutoGame1Node::process_go_to_kick_start()
{
  // 1a. キック開始点へ向かう状態
  // 入力: kick_start_pose_, kick_end_pose_
  // 前提: State::GO_TO_KICK_START に遷移した直後であること
  // 状態変更: Nav2 Goal未送信の場合はキック開始点・終了点への NavigateThroughPoses を送信。
  // 出力: 距離 < kick_start_reach_threshold_ に達したら Nav2をキャンセルし PREPARE_KICK へ移行する。

  auto kick_start_transformed = apply_side_transform(kick_start_pose_);
  auto kick_end_transformed = apply_side_transform(kick_end_pose_);

  if (!nav_goal_handle_ && !nav_completed_) {
    RCLCPP_INFO(
      get_logger(),
      "State: GO_TO_KICK_START. Sending goals to Kick Start and Kick End (preventing deceleration).");
    std::vector<geometry_msgs::msg::PoseStamped> poses = {kick_start_transformed, kick_end_transformed};
    send_nav_goal(poses);
    return;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (get_robot_pose_map(current_pose)) {
    double dist = compute_distance_2d(current_pose.pose.position, kick_start_transformed.pose.position);
    if (dist <= kick_start_reach_threshold_) {
      RCLCPP_INFO(
        get_logger(),
        "Reached Kick Start threshold (dist: %.3f m <= %.3f m). Transitioning to PREPARE_KICK.",
        dist, kick_start_reach_threshold_);
      cancel_nav_goal();
      current_state_ = State::PREPARE_KICK;
      kick_action_active_ = false;
      kick_action_completed_ = false;
    }
  }
}

void AutoGame1Node::process_prepare_kick()
{
  // 2. キック準備・一定速度走行状態
  // 入力: Map座標系での現在ロボット位置・姿勢 (tf2)
  // 前提: 通過点1に達し、Nav2がキャンセルされた状態
  // 状態変更: 独自Twist出力 (Map系目標速度 -> Base系へtf2変換)。キックActionを送信し、完了したら GO_TO_GATE_FAR_SIDE へ遷移する。
  // 出力: cmd_vel を publish。

  geometry_msgs::msg::PoseStamped current_pose;
  if (!get_robot_pose_map(current_pose)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "PREPARE_KICK: Failed to get robot pose.");
    return;
  }

  double side_sign = get_side_sign();
  double target_x = kick_target_x_ * side_sign;
  double target_y = kick_target_y_ * side_sign;
  double target_yaw = kick_target_yaw_ * side_sign;

  // キック射出判定: 前進位置(X座標)が目標キック射出ライン (kick_target_x +- kick_tolerance_x) に到達した瞬間に送信
  if (!kick_action_active_ && !kick_action_completed_) {
    double diff_x = std::abs(current_pose.pose.position.x - target_x);
    bool line_crossed = (side_sign > 0) ? (current_pose.pose.position.x >= target_x) :
                                          (current_pose.pose.position.x <= target_x);
    if (diff_x <= kick_tolerance_x_ || line_crossed) {
      RCLCPP_INFO(
        get_logger(),
        "Reached Kick Line (x: %.3f m, target: %.3f +- %.3f m). Firing Kick Action!",
        current_pose.pose.position.x, target_x, kick_tolerance_x_);
      send_kick_goal();
      kick_action_active_ = true;
    }
  }

  // Yaw角の計算
  double current_yaw = tf2::getYaw(current_pose.pose.orientation);

  double error_y = target_y - current_pose.pose.position.y;
  double error_yaw = target_yaw - current_yaw;

  // 角度エラーの [-PI, PI] 正規化
  while (error_yaw > M_PI) error_yaw -= 2.0 * M_PI;
  while (error_yaw < -M_PI) error_yaw += 2.0 * M_PI;

  double d_error_y = (error_y - prev_error_y_) / control_period_sec_;
  double d_error_yaw = (error_yaw - prev_error_yaw_) / control_period_sec_;

  prev_error_y_ = error_y;
  prev_error_yaw_ = error_yaw;

  double v_y_map = kp_y_ * error_y + kd_y_ * d_error_y;
  double omega_z = kp_yaw_ * error_yaw + kd_yaw_ * d_error_yaw;
  double v_x_map = kick_target_velocity_x_;

  // Map座標系速度から Base座標系速度への変換
  geometry_msgs::msg::Twist cmd_vel = transform_map_velocity_to_base(
    v_x_map, v_y_map, omega_z, current_yaw);

  cmd_vel_publisher_->publish(cmd_vel);

  if (kick_action_completed_) {
    RCLCPP_INFO(get_logger(), "Kick action completed. Transitioning to GO_TO_GATE_FAR_SIDE.");
    current_state_ = State::GO_TO_GATE_FAR_SIDE;
    nav_completed_ = false;
    nav_goal_handle_ = nullptr;
  }
}

void AutoGame1Node::process_go_to_gate_far_side()
{
  // 1b. ゲート向こう側へ向かう状態
  if (!nav_goal_handle_ && !nav_completed_) {
    RCLCPP_INFO(get_logger(), "State: GO_TO_GATE_FAR_SIDE. Sending goals for remaining waypoints and far side.");
    std::vector<geometry_msgs::msg::PoseStamped> poses = {
      apply_side_transform(kick_end_pose_),
      apply_side_transform(waypoint3_pose_),
      apply_side_transform(gate_far_side_pose_)};
    send_nav_goal(poses);
    return;
  }

  if (nav_completed_) {
    RCLCPP_INFO(get_logger(), "Reached gate far side. Transitioning to FOLLOW_BALL.");
    current_state_ = State::FOLLOW_BALL;
    nav_completed_ = false;
    nav_goal_handle_ = nullptr;
  }
}

void AutoGame1Node::process_follow_ball()
{
  // 3. ボール追従状態 (拡張用プレースホルダー)
  RCLCPP_INFO(get_logger(), "State: FOLLOW_BALL (Placeholder). Transitioning to CARRY_BALL_TO_PASS_AREA.");
  current_state_ = State::CARRY_BALL_TO_PASS_AREA;
  nav_completed_ = false;
  nav_goal_handle_ = nullptr;
}

void AutoGame1Node::process_carry_ball_to_pass_area()
{
  // 4. ボールをパスエリアに運ぶ状態
  // 入力: pass_area_pose_
  // 状態変更:
  //   Step 1. pass_area_pose_ への NavigateThroughPoses Goal を送信。
  //   Step 2. Nav2 到着完了後、ボールリリース Action (Pass) を送信＆ドリブルトピックをOFFに設定。
  //   Step 3. Pass Action 完了後、RETURN_TO_START へ遷移。

  if (!nav_goal_handle_ && !nav_completed_) {
    RCLCPP_INFO(get_logger(), "State: CARRY_BALL_TO_PASS_AREA. Sending Nav goal to Pass Area.");
    send_nav_goal({apply_side_transform(pass_area_pose_)});
    pass_action_active_ = false;
    pass_action_completed_ = false;
    return;
  }

  if (nav_completed_) {
    if (!pass_action_active_ && !pass_action_completed_) {
      RCLCPP_INFO(get_logger(), "Reached Pass Area. Sending Pass Action goal & stopping Dribbler topic.");
      send_pass_goal();
      publish_dribble_enabled(false);
      pass_action_active_ = true;
    }

    if (pass_action_completed_) {
      RCLCPP_INFO(get_logger(), "Pass Action completed. Transitioning to RETURN_TO_START.");
      current_state_ = State::RETURN_TO_START;
      nav_completed_ = false;
      nav_goal_handle_ = nullptr;
      pass_action_active_ = false;
      pass_action_completed_ = false;
    }
  }
}

void AutoGame1Node::process_return_to_start()
{
  // 5. スタート位置に戻る状態
  // 入力: start_pose_
  // 状態変更: start_pose_ への NavigateThroughPoses Goal を送信。
  // 出力: Nav2 到着完了 (Succeeded) で DRIBBLE_ON（ドリブルON状態）へ移行する。

  if (!nav_goal_handle_ && !nav_completed_) {
    RCLCPP_INFO(get_logger(), "State: RETURN_TO_START. Sending Nav goal to Start Pose.");
    send_nav_goal({apply_side_transform(start_pose_)});
    return;
  }

  if (nav_completed_) {
    RCLCPP_INFO(get_logger(), "Returned to Start Pose. Mission loop complete. Transitioning to DRIBBLE_ON.");
    publish_dribble_enabled(false);
    current_state_ = State::DRIBBLE_ON;
    nav_completed_ = false;
    nav_goal_handle_ = nullptr;
  }
}

// Action送信ヘルパー関数群 (NavigateThroughPoses 1本に統一)

void AutoGame1Node::send_nav_goal(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses)
{
  if (!nav_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(get_logger(), "Nav action server not available.");
    return;
  }

  NavigateThroughPoses::Goal goal;
  goal.poses = poses;

  auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
  send_goal_options.result_callback = [this](const GoalHandleNavigateThroughPoses::WrappedResult & result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(get_logger(), "Nav goal succeeded.");
      nav_completed_ = true;
    } else {
      RCLCPP_WARN(get_logger(), "Nav goal failed or was canceled.");
      nav_failed_ = true;
    }
  };

  nav_completed_ = false;
  nav_failed_ = false;
  nav_client_->async_send_goal(goal, send_goal_options);
}

void AutoGame1Node::cancel_nav_goal()
{
  if (nav_client_) {
    nav_client_->async_cancel_all_goals();
    nav_goal_handle_ = nullptr;
  }
}

void AutoGame1Node::send_kick_goal()
{
  if (!kick_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_WARN(get_logger(), "Kick action server not available. Simulating kick completion.");
    kick_action_completed_ = true;
    return;
  }

  Kick::Goal goal;
  goal.start = true;

  auto send_goal_options = rclcpp_action::Client<Kick>::SendGoalOptions();
  send_goal_options.result_callback = [this](const GoalHandleKick::WrappedResult & result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(get_logger(), "Kick action succeeded.");
    } else {
      RCLCPP_WARN(get_logger(), "Kick action failed or was canceled.");
    }
    kick_action_completed_ = true;
  };

  kick_action_completed_ = false;
  kick_client_->async_send_goal(goal, send_goal_options);
}

void AutoGame1Node::send_dribble_goal(bool start)
{
  if (!dribble_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_WARN(
      get_logger(),
      "Dribble action server not available. Simulating dribble %s completion.",
      start ? "start" : "stop");
    dribble_action_completed_ = true;
    return;
  }

  Dribble::Goal goal;
  goal.start = start;

  auto send_goal_options = rclcpp_action::Client<Dribble>::SendGoalOptions();
  send_goal_options.result_callback = [this, start](const GoalHandleDribble::WrappedResult & result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(get_logger(), "Dribble %s action succeeded.", start ? "start" : "stop");
    } else {
      RCLCPP_WARN(get_logger(), "Dribble %s action failed or was canceled.", start ? "start" : "stop");
    }
    dribble_action_completed_ = true;
  };

  dribble_action_completed_ = false;
  dribble_client_->async_send_goal(goal, send_goal_options);
}

void AutoGame1Node::send_pass_goal()
{
  if (!pass_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_WARN(get_logger(), "Pass action server not available. Simulating pass/release completion.");
    pass_action_completed_ = true;
    return;
  }

  Pass::Goal goal;
  goal.start = true;

  auto send_goal_options = rclcpp_action::Client<Pass>::SendGoalOptions();
  send_goal_options.result_callback = [this](const GoalHandlePass::WrappedResult & result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(get_logger(), "Pass Action succeeded.");
    } else {
      RCLCPP_WARN(get_logger(), "Pass Action failed or was canceled.");
    }
    pass_action_completed_ = true;
  };

  pass_action_completed_ = false;
  pass_client_->async_send_goal(goal, send_goal_options);
}

void AutoGame1Node::publish_dribble_enabled(bool enabled)
{
  std_msgs::msg::Bool msg;
  msg.data = enabled;
  dribble_publisher_->publish(msg);
  RCLCPP_INFO(get_logger(), "Published /dribble/enabled: %s", enabled ? "true" : "false");
}

// 5. 小さな補助関数 (値取得・変換・判定)

/*
 * 入力: global_frame_id_, robot_base_frame_id_
 * 前提: TFツリー上で map から base_link の Transform が存在すること
 * 状態の変更: なし
 * 出力: current_pose (Map座標系でのロボットの現在位置・姿勢), 取得成功可否 (bool)
 */
bool AutoGame1Node::get_robot_pose_map(geometry_msgs::msg::PoseStamped & current_pose)
{
  try {
    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
      global_frame_id_, robot_base_frame_id_, tf2::TimePointZero);

    current_pose.header = transform.header;
    current_pose.pose.position.x = transform.transform.translation.x;
    current_pose.pose.position.y = transform.transform.translation.y;
    current_pose.pose.position.z = transform.transform.translation.z;
    current_pose.pose.orientation = transform.transform.rotation;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Could not transform %s to %s: %s",
      global_frame_id_.c_str(), robot_base_frame_id_.c_str(), ex.what());
    return false;
  }
}

double AutoGame1Node::compute_distance_2d(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2)
{
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  return std::sqrt(dx * dx + dy * dy);
}

/*
 * 入力: Map座標系での速度 (v_x_map, v_y_map), 角速度 (omega_z), ロボットYaw角 (robot_yaw)
 * 前提: Base座標系はROS標準 (x: 前, y: 左)
 * 状態の変更: なし
 * 出力: Base座標系での Twist (cmd_vel)
 */
geometry_msgs::msg::Twist AutoGame1Node::transform_map_velocity_to_base(
  double v_x_map, double v_y_map, double omega_z, double robot_yaw)
{
  geometry_msgs::msg::Twist cmd_vel;

  // Map系から Base系への回転行列による速度変換
  // v_x_base =  cos(yaw) * v_x_map + sin(yaw) * v_y_map
  // v_y_base = -sin(yaw) * v_x_map + cos(yaw) * v_y_map
  double cos_yaw = std::cos(robot_yaw);
  double sin_yaw = std::sin(robot_yaw);

  cmd_vel.linear.x = cos_yaw * v_x_map + sin_yaw * v_y_map;
  cmd_vel.linear.y = -sin_yaw * v_x_map + cos_yaw * v_y_map;
  cmd_vel.angular.z = omega_z;

  return cmd_vel;
}

bool AutoGame1Node::button_pressed(const sensor_msgs::msg::Joy & msg, int index)
{
  if (index < 0 || static_cast<std::size_t>(index) >= msg.buttons.size()) {
    return false;
  }
  return msg.buttons[static_cast<std::size_t>(index)] != 0;
}

/*
 * 役割: 定数 RECTANGLE_OBSTACLES[3] に定義された3つの長方形座標から PolygonStamped を生成し、
 *       それぞれ /obstacle_polygon_1, /obstacle_polygon_2, /obstacle_polygon_3 トピックへ publish する。
 *       選択されているコートサイド (SIDE_A / SIDE_B) に応じて X 座標を反転する。
 */
void AutoGame1Node::publish_obstacle_polygons()
{
  float side_sign = static_cast<float>(get_side_sign());

  for (size_t i = 0; i < 3; ++i) {
    if (!obstacle_polygon_publishers_[i]) {
      continue;
    }

    geometry_msgs::msg::PolygonStamped poly_msg;
    poly_msg.header.stamp = now();
    poly_msg.header.frame_id = global_frame_id_;

    const auto & rect = RECTANGLE_OBSTACLES[i];

    geometry_msgs::msg::Point32 p1, p2, p3, p4;
    p1.x = rect.p1.x * side_sign; p1.y = rect.p1.y; p1.z = 0.0f;
    p2.x = rect.p2.x * side_sign; p2.y = rect.p2.y; p2.z = 0.0f;
    p3.x = rect.p3.x * side_sign; p3.y = rect.p3.y; p3.z = 0.0f;
    p4.x = rect.p4.x * side_sign; p4.y = rect.p4.y; p4.z = 0.0f;

    poly_msg.polygon.points = {p1, p2, p3, p4};

    obstacle_polygon_publishers_[i]->publish(poly_msg);
  }
}

void AutoGame1Node::reset_all_nav_goals()
{
  cancel_nav_goal();
  nav_completed_ = false;
  nav_goal_handle_ = nullptr;
}

}  // namespace auto_game1
