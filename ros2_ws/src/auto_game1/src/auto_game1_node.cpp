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

  // /joy:
  // ジョイスティックからの操作入力を受信する。
  // 送信元: joy_node (joystick_driver)
  // メッセージ: sensor_msgs/msg/Joy
  joy_subscription_ = create_subscription<sensor_msgs::msg::Joy>(
    joy_topic_, rclcpp::SensorDataQoS(),
    std::bind(&AutoGame1Node::joy_callback, this, std::placeholders::_1));

  // nav2 NavigateThroughPoses Action Client:
  // 複数通過点をスムーズに走行するためのNav2アクションクライアント。
  nav_through_poses_client_ = rclcpp_action::create_client<NavigateThroughPoses>(
    this, nav_through_poses_action_name_);

  // nav2 NavigateToPose Action Client:
  // 単一の目標位置（パスエリア、スタート位置）へ移動するためのNav2アクションクライアント。
  nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(
    this, nav_to_pose_action_name_);

  // キック機構 Action Client:
  // ゲート通過中のボール射出（キック）を実行・完了検出するためのアクションクライアント。
  kick_client_ = rclcpp_action::create_client<Kick>(
    this, kick_action_name_);

  // 主制御ループタイマー
  control_timer_ = create_wall_timer(
    std::chrono::duration<double>(control_period_sec_),
    std::bind(&AutoGame1Node::control_timer_callback, this));
}

void AutoGame1Node::declare_parameters()
{
  // トピック・アクション名
  declare_parameter<std::string>("cmd_vel_topic", "/mecanum/cmd_vel");
  declare_parameter<std::string>("joy_topic", "/joy");
  declare_parameter<std::string>("nav_through_poses_action_name", "navigate_through_poses");
  declare_parameter<std::string>("nav_to_pose_action_name", "navigate_to_pose");
  declare_parameter<std::string>("kick_action_name", "kick");
  declare_parameter<std::string>("global_frame_id", "map");
  declare_parameter<std::string>("robot_base_frame_id", "base_link");

  // ボタン設定
  declare_parameter<int>("auto_stop_toggle_button", default_auto_stop_toggle_button);
  declare_parameter<int>("return_to_start_button", default_return_to_start_button);

  // 制御周期・判定パラメータ
  declare_parameter<double>("control_period_sec", 0.05);
  declare_parameter<double>("waypoint1_reach_threshold", 0.2);

  // PREPARE_KICK用制御パラメータ
  declare_parameter<double>("kick_target_velocity_x", 0.5);
  declare_parameter<double>("kick_target_y", 0.0);
  declare_parameter<double>("kick_target_yaw", 0.0);
  declare_parameter<double>("kp_y", 1.0);
  declare_parameter<double>("kd_y", 0.1);
  declare_parameter<double>("kp_yaw", 1.0);
  declare_parameter<double>("kd_yaw", 0.1);

  // 通過点・目標点の座標パラメータ（デフォルト値例）
  declare_parameter<double>("waypoint1.x", 1.0);
  declare_parameter<double>("waypoint1.y", 0.0);
  declare_parameter<double>("waypoint2.x", 2.0);
  declare_parameter<double>("waypoint2.y", 0.5);
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
  get_parameter("nav_through_poses_action_name", nav_through_poses_action_name_);
  get_parameter("nav_to_pose_action_name", nav_to_pose_action_name_);
  get_parameter("kick_action_name", kick_action_name_);
  get_parameter("global_frame_id", global_frame_id_);
  get_parameter("robot_base_frame_id", robot_base_frame_id_);

  get_parameter("auto_stop_toggle_button", auto_stop_toggle_button_);
  get_parameter("return_to_start_button", return_to_start_button_);

  get_parameter("control_period_sec", control_period_sec_);
  get_parameter("waypoint1_reach_threshold", waypoint1_reach_threshold_);

  get_parameter("kick_target_velocity_x", kick_target_velocity_x_);
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

  get_pose_param("waypoint1", waypoint1_pose_);
  get_pose_param("waypoint2", waypoint2_pose_);
  get_pose_param("waypoint3", waypoint3_pose_);
  get_pose_param("gate_far_side", gate_far_side_pose_);
  get_pose_param("pass_area", pass_area_pose_);
  get_pose_param("start_pose", start_pose_);
}

// Subscription Callback
// 受信トピック: /joy
// 役割: 手動による自動停止切り替え（toggle）およびスタート復帰命令を検出する。
// 動作:
//  - auto_stop_toggle_button 立ち上がり: AUTO_STOP と自律制御状態の相互切り替え。
//  - return_to_start_button 立ち上がり: 強制的に RETURN_TO_START 状態へ遷移。
void AutoGame1Node::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  auto_stop_toggle_button_on_ = button_pressed(*msg, auto_stop_toggle_button_);
  return_to_start_button_on_ = button_pressed(*msg, return_to_start_button_);

  // 自動停止切替ボタンの立ち上がり判定
  if (auto_stop_toggle_button_on_ && !pre_auto_stop_toggle_button_on_) {
    if (current_state_ == State::AUTO_STOP) {
      RCLCPP_INFO(get_logger(), "Joy input: Resuming auto drive. Switching to GO_TO_WAYPOINT1.");
      current_state_ = State::GO_TO_WAYPOINT1;
    } else {
      RCLCPP_INFO(get_logger(), "Joy input: Auto stop requested.");
      previous_state_ = current_state_;
      current_state_ = State::AUTO_STOP;
    }
  }

  // スタート地点復帰ボタンの立ち上がり判定
  if (return_to_start_button_on_ && !pre_return_to_start_button_on_) {
    RCLCPP_INFO(get_logger(), "Joy input: Return to start requested. Switching to RETURN_TO_START.");
    cancel_nav_through_poses_goal();
    cancel_nav_to_pose_goal();
    current_state_ = State::RETURN_TO_START;
  }

  pre_auto_stop_toggle_button_on_ = auto_stop_toggle_button_on_;
  pre_return_to_start_button_on_ = return_to_start_button_on_;
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
    case State::GO_TO_WAYPOINT1:
      process_go_to_waypoint1();
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
  // 自動停止状態: 走行速度0を出力し、Nav2をキャンセルする
  geometry_msgs::msg::Twist stop_cmd;
  cmd_vel_publisher_->publish(stop_cmd);
}

void AutoGame1Node::process_go_to_waypoint1()
{
  // 1a. 通過点1へ向かう状態
  // 入力: waypoint1_pose_
  // 前提: State::GO_TO_WAYPOINT1 に遷移した直後であること
  // 状態変更: Nav2 Goal未送信の場合は通過点1への NavigateThroughPoses を送信する。
  // 出力: 距離 < waypoint1_reach_threshold_ に達したら Nav2をキャンセルし PREPARE_KICK へ移行する。

  if (!nav_through_poses_goal_handle_ && !nav_through_poses_completed_) {
    RCLCPP_INFO(get_logger(), "State: GO_TO_WAYPOINT1. Sending goal to Waypoint 1.");
    std::vector<geometry_msgs::msg::PoseStamped> poses = {waypoint1_pose_};
    send_nav_through_poses_goal(poses);
    return;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (get_robot_pose_map(current_pose)) {
    double dist = compute_distance_2d(current_pose.pose.position, waypoint1_pose_.pose.position);
    if (dist <= waypoint1_reach_threshold_) {
      RCLCPP_INFO(
        get_logger(),
        "Reached Waypoint 1 threshold (dist: %.3f m <= %.3f m). Transitioning to PREPARE_KICK.",
        dist, waypoint1_reach_threshold_);
      cancel_nav_through_poses_goal();
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

  if (!kick_action_active_ && !kick_action_completed_) {
    RCLCPP_INFO(get_logger(), "State: PREPARE_KICK. Sending Kick Action goal.");
    send_kick_goal();
    kick_action_active_ = true;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!get_robot_pose_map(current_pose)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "PREPARE_KICK: Failed to get robot pose.");
    return;
  }

  // Yaw角の計算
  double current_yaw = tf2::getYaw(current_pose.pose.orientation);

  // PD制御による Map座標系での速度計算
  double error_y = kick_target_y_ - current_pose.pose.position.y;
  double error_yaw = kick_target_yaw_ - current_yaw;

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
    nav_through_poses_completed_ = false;
    nav_through_poses_goal_handle_ = nullptr;
  }
}

void AutoGame1Node::process_go_to_gate_far_side()
{
  // 1b. ゲート向こう側へ向かう状態
  // 入力: waypoint2_pose_, waypoint3_pose_, gate_far_side_pose_
  // 前提: キックが完了し、残り通過点を安全に通過して目標点へ向かう状態
  // 状態変更: Nav2 Goal未送信の場合は残り通過点を NavigateThroughPoses で送信する。
  // 出力: Nav2 到着完了 (Succeeded) で FOLLOW_BALL へ遷移する。

  if (!nav_through_poses_goal_handle_ && !nav_through_poses_completed_) {
    RCLCPP_INFO(get_logger(), "State: GO_TO_GATE_FAR_SIDE. Sending goals for remaining waypoints and far side.");
    std::vector<geometry_msgs::msg::PoseStamped> poses = {
      waypoint2_pose_, waypoint3_pose_, gate_far_side_pose_};
    send_nav_through_poses_goal(poses);
    return;
  }

  if (nav_through_poses_completed_) {
    RCLCPP_INFO(get_logger(), "Reached gate far side. Transitioning to FOLLOW_BALL.");
    current_state_ = State::FOLLOW_BALL;
    nav_through_poses_completed_ = false;
    nav_through_poses_goal_handle_ = nullptr;
  }
}

void AutoGame1Node::process_follow_ball()
{
  // 3. ボール追従状態
  // 概要: 自前Twist追従ロジック用の拡張用プレースホルダー。現状は自動的に次状態へ移行する。
  RCLCPP_INFO(get_logger(), "State: FOLLOW_BALL (Placeholder). Transitioning to CARRY_BALL_TO_PASS_AREA.");
  current_state_ = State::CARRY_BALL_TO_PASS_AREA;
  nav_to_pose_completed_ = false;
  nav_to_pose_goal_handle_ = nullptr;
}

void AutoGame1Node::process_carry_ball_to_pass_area()
{
  // 4. ボールをパスエリアに運ぶ状態
  // 入力: pass_area_pose_
  // 前提: ボールを保持しパスエリアへ搬送する
  // 状態変更: pass_area_pose_ への NavigateToPose Goal を送信する。
  // 出力: Nav2 到着完了 (Succeeded) で RETURN_TO_START へ遷移する。

  if (!nav_to_pose_goal_handle_ && !nav_to_pose_completed_) {
    RCLCPP_INFO(get_logger(), "State: CARRY_BALL_TO_PASS_AREA. Sending NavigateToPose goal to Pass Area.");
    send_nav_to_pose_goal(pass_area_pose_);
    return;
  }

  if (nav_to_pose_completed_) {
    RCLCPP_INFO(get_logger(), "Reached Pass Area. Transitioning to RETURN_TO_START.");
    current_state_ = State::RETURN_TO_START;
    nav_to_pose_completed_ = false;
    nav_to_pose_goal_handle_ = nullptr;
  }
}

void AutoGame1Node::process_return_to_start()
{
  // 5. スタート位置に戻る状態
  // 入力: start_pose_
  // 前提: パスエリアでの作業完了
  // 状態変更: start_pose_ への NavigateToPose Goal を送信する。
  // 出力: Nav2 到着完了 (Succeeded) で GO_TO_WAYPOINT1 へ戻りループする。

  if (!nav_to_pose_goal_handle_ && !nav_to_pose_completed_) {
    RCLCPP_INFO(get_logger(), "State: RETURN_TO_START. Sending NavigateToPose goal to Start Pose.");
    send_nav_to_pose_goal(start_pose_);
    return;
  }

  if (nav_to_pose_completed_) {
    RCLCPP_INFO(get_logger(), "Returned to Start Pose. Mission loop complete. Restarting at GO_TO_WAYPOINT1.");
    current_state_ = State::GO_TO_WAYPOINT1;
    nav_to_pose_completed_ = false;
    nav_to_pose_goal_handle_ = nullptr;
    nav_through_poses_completed_ = false;
    nav_through_poses_goal_handle_ = nullptr;
  }
}

// Action送信ヘルパー関数群

void AutoGame1Node::send_nav_through_poses_goal(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses)
{
  if (!nav_through_poses_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(get_logger(), "NavigateThroughPoses action server not available.");
    return;
  }

  NavigateThroughPoses::Goal goal;
  goal.poses = poses;

  auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
  send_goal_options.result_callback = [this](const GoalHandleNavigateThroughPoses::WrappedResult & result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(get_logger(), "NavigateThroughPoses succeeded.");
      nav_through_poses_completed_ = true;
    } else {
      RCLCPP_WARN(get_logger(), "NavigateThroughPoses failed or was canceled.");
      nav_through_poses_failed_ = true;
    }
  };

  nav_through_poses_completed_ = false;
  nav_through_poses_failed_ = false;
  nav_through_poses_client_->async_send_goal(goal, send_goal_options);
}

void AutoGame1Node::cancel_nav_through_poses_goal()
{
  if (nav_through_poses_client_) {
    nav_through_poses_client_->async_cancel_all_goals();
    nav_through_poses_goal_handle_ = nullptr;
  }
}

void AutoGame1Node::send_nav_to_pose_goal(const geometry_msgs::msg::PoseStamped & pose)
{
  if (!nav_to_pose_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(get_logger(), "NavigateToPose action server not available.");
    return;
  }

  NavigateToPose::Goal goal;
  goal.pose = pose;

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback = [this](const GoalHandleNavigateToPose::WrappedResult & result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(get_logger(), "NavigateToPose succeeded.");
      nav_to_pose_completed_ = true;
    } else {
      RCLCPP_WARN(get_logger(), "NavigateToPose failed or was canceled.");
      nav_to_pose_failed_ = true;
    }
  };

  nav_to_pose_completed_ = false;
  nav_to_pose_failed_ = false;
  nav_to_pose_client_->async_send_goal(goal, send_goal_options);
}

void AutoGame1Node::cancel_nav_to_pose_goal()
{
  if (nav_to_pose_client_) {
    nav_to_pose_client_->async_cancel_all_goals();
    nav_to_pose_goal_handle_ = nullptr;
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

}  // namespace auto_game1
