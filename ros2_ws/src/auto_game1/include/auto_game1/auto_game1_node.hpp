#ifndef AUTO_GAME1__AUTO_GAME1_NODE_HPP_
#define AUTO_GAME1__AUTO_GAME1_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "auto_game1/action/kick.hpp"

namespace auto_game1
{

struct Point2D
{
  float x;
  float y;
};

struct RectObstacle
{
  Point2D p1;
  Point2D p2;
  Point2D p3;
  Point2D p4;
};

// ナビゲーション回避用の障害物となる3つの長方形の定数座標定義
// （後から数値書き換え可能な定数定義。C++ルールに従いkプレフィックスは使用しない）
const RectObstacle RECTANGLE_OBSTACLES[3] = {
  // 長方形 1 (4頂点: p1, p2, p3, p4)
  { {0.5f, 0.5f}, {1.5f, 0.5f}, {1.5f, 1.5f}, {0.5f, 1.5f} },
  // 長方形 2
  { {2.0f, 1.0f}, {3.0f, 1.0f}, {3.0f, 2.0f}, {2.0f, 2.0f} },
  // 長方形 3
  { {1.0f, -1.5f}, {2.0f, -1.5f}, {2.0f, -0.5f}, {1.0f, -0.5f} }
};

enum class State
{
  AUTO_STOP,               // 自動停止状態（Joyボタン等で一時停止 / 再開）
  GO_TO_KICK_START,        // 1a. キック開始点へ向かう状態 (NavigateThroughPoses)
  PREPARE_KICK,            // 2. キック準備・一定速度走行状態 (独自Twist制御)
  GO_TO_GATE_FAR_SIDE,     // 1b. ゲート向こう側へ向かう状態 (NavigateThroughPoses)
  FOLLOW_BALL,             // 3. ボール追従状態 (拡張用プレースホルダー)
  CARRY_BALL_TO_PASS_AREA, // 4. ボールをパスエリアに運ぶ状態 (NavigateThroughPoses)
  RETURN_TO_START          // 5. スタート位置に戻る状態 (NavigateThroughPoses -> GO_TO_KICK_STARTへループ)
};

class AutoGame1Node : public rclcpp::Node
{
public:
  using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
  using GoalHandleNavigateThroughPoses = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

  using Kick = auto_game1::action::Kick;
  using GoalHandleKick = rclcpp_action::ClientGoalHandle<Kick>;

  explicit AutoGame1Node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~AutoGame1Node() = default;

private:
  // 1. Parameter宣言・取得
  void declare_parameters();
  void get_parameters();

  // 2. Subscription Callback
  // /joy を受信したときのコールバック
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

  // 3. Timer Callback
  // メイン制御ループタイマー（状態遷移処理および制御コマンド出力）
  void control_timer_callback();

  // 4. 主処理 (State Machine)
  void process_state_machine();
  void process_auto_stop();
  void process_go_to_kick_start();
  void process_prepare_kick();
  void process_go_to_gate_far_side();
  void process_follow_ball();
  void process_carry_ball_to_pass_area();
  void process_return_to_start();

  // Action送信・制御の補助関数（NavigateThroughPoses 1本に統一）
  void send_nav_goal(const std::vector<geometry_msgs::msg::PoseStamped> & poses);
  void cancel_nav_goal();
  void send_kick_goal();
  void reset_all_nav_goals();

  // 5. 小さな補助関数 (値取得・変換・判定)
  bool get_robot_pose_map(geometry_msgs::msg::PoseStamped & current_pose);
  double compute_distance_2d(
    const geometry_msgs::msg::Point & p1,
    const geometry_msgs::msg::Point & p2);
  geometry_msgs::msg::Twist transform_map_velocity_to_base(
    double v_x_map, double v_y_map, double omega_z, double robot_yaw);
  bool button_pressed(const sensor_msgs::msg::Joy & msg, int index);
  void publish_obstacle_polygons();

  // ボタン割り当て定数（C++ルール: kプレフィックスなし）
  static constexpr int default_auto_stop_toggle_button = 8;  // Create ボタン
  static constexpr int default_return_to_start_button = 9;   // Options ボタン

  // パラメータ
  std::string cmd_vel_topic_;
  std::string joy_topic_;
  std::string nav_action_name_;
  std::string kick_action_name_;
  std::string global_frame_id_;
  std::string robot_base_frame_id_;

  int auto_stop_toggle_button_{default_auto_stop_toggle_button};
  int return_to_start_button_{default_return_to_start_button};

  double control_period_sec_{0.05};        // 20Hz 制御周期
  double kick_start_reach_threshold_{0.2}; // キック開始点到達判定の距離閾値 [m]

  // PREPARE_KICK 状態用制御パラメータ
  double kick_target_velocity_x_{0.5}; // Map座標系での目標x速度 [m/s]
  double kick_target_y_{0.0};          // Map座標系での目標y位置 [m]
  double kick_target_yaw_{0.0};        // Map座標系での目標yaw角 [rad]
  double kp_y_{1.0};                   // y位置Pゲイン
  double kd_y_{0.1};                   // y位置Dゲイン
  double kp_yaw_{1.0};                 // yaw角Pゲイン
  double kd_yaw_{0.1};                 // yaw角Dゲイン

  // 目標座標（Waypoints & Goals）
  geometry_msgs::msg::PoseStamped kick_start_pose_;
  geometry_msgs::msg::PoseStamped kick_end_pose_;
  geometry_msgs::msg::PoseStamped waypoint3_pose_;
  geometry_msgs::msg::PoseStamped gate_far_side_pose_;
  geometry_msgs::msg::PoseStamped pass_area_pose_;
  geometry_msgs::msg::PoseStamped start_pose_;

  // 内部状態
  State current_state_{State::AUTO_STOP};
  State previous_state_{State::AUTO_STOP};

  // Joy入力状態（ボタン立ち上がり検出用）
  bool auto_stop_toggle_button_on_{false};
  bool pre_auto_stop_toggle_button_on_{false};
  bool return_to_start_button_on_{false};
  bool pre_return_to_start_button_on_{false};

  // キックAction状態
  bool kick_action_active_{false};
  bool kick_action_completed_{false};

  // Nav2 Action Goalハンドラ / 完了状態（共通化した単一のハンドラ）
  GoalHandleNavigateThroughPoses::SharedPtr nav_goal_handle_;
  bool nav_completed_{false};
  bool nav_failed_{false};

  // PD制御用前回の誤差
  double prev_error_y_{0.0};
  double prev_error_yaw_{0.0};

  // ROS 2 通信・TF インターフェース
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr obstacle_polygon_publishers_[3];
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr nav_client_;
  rclcpp_action::Client<Kick>::SharedPtr kick_client_;

  rclcpp::TimerBase::SharedPtr control_timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace auto_game1

#endif  // AUTO_GAME1__AUTO_GAME1_NODE_HPP_
