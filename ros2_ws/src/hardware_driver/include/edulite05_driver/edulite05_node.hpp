#ifndef EDULITE05_DRIVER__EDULITE05_NODE_HPP_
#define EDULITE05_DRIVER__EDULITE05_NODE_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "can_msgs/msg/frame.hpp"
#include "edulite05_driver/edulite05_protocol.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

/// @brief EduLite 05 (Robstride) モータドライバノード
///
/// 起動時に初期化フレームを送信し、その後はコマンドを受け付ける。
/// フィードバック未確認の場合は定期的に初期化フレームを再送する。
/// 初期化フレームの送信は非同期タイマーで行い、スピンスレッドをブロックしない。
class Ed05DriverNode : public rclcpp::Node
{
public:
  Ed05DriverNode();
  ~Ed05DriverNode();

  /// @brief モータを停止させる終了フレームを送信する
  void terminate_motor();

private:
  /// @brief 初期化・有効化の進行状態
  enum class InitState
  {
    IDLE,        ///< 初期化未開始
    SENDING,     ///< 初期化フレームを順番に送信中
    ENABLED,     ///< 初期化完了・トルクON確定
  };

  // ── パラメータ ──────────────────────────────────────
  void declare_parameters();
  void get_parameters();

  // ── 初期化フレーム送信 ──────────────────────────────
  /// @brief 初期化を開始する（非同期フレーム送信を予約）
  void start_init();
  /// @brief 初期化フレームを1枚ずつ送信するタイマーコールバック
  void init_frame_timer_callback();
  /// @brief イネーブル未確定時の定期リトライタイマーコールバック
  void retry_timer_callback();

  // ── ROS コールバック ────────────────────────────────
  void cmd_callback(const std_msgs::msg::Float32::SharedPtr msg);
  void can_callback(const can_msgs::msg::Frame::SharedPtr msg);

  // ── フレーム送信ヘルパー ────────────────────────────
  void publish_frame(const Canframe & frame);

  // ── パラメータ変数 ──────────────────────────────────
  std::string sub_cmd_topic_name_;
  std::string pub_can_topic_name_;
  std::string sub_can_topic_name_;
  std::string pub_fb_topic_name_;
  uint8_t motor_id_{0x01};
  std::string runmode_{"Velocity"};
  bool is_requested_fb_pub_{false};

  // ── 状態変数 ────────────────────────────────────────
  InitState init_state_{InitState::IDLE};
  std::vector<Canframe> pending_frames_;  ///< 送信待ち初期化フレームキュー
  std::size_t pending_frame_index_{0};    ///< 次に送信するフレームのインデックス

  // ── ROS インタフェース ──────────────────────────────
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cmd_sub_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr fb_pub_;

  /// @brief 初期化フレームを50ms間隔で1枚ずつ送信するタイマー
  rclcpp::TimerBase::SharedPtr init_frame_timer_;
  /// @brief イネーブル未確定時に3秒ごとに再送を試みるタイマー
  rclcpp::TimerBase::SharedPtr retry_timer_;

  std::unique_ptr<Ed05CanframeCreater> motor_;

  can_msgs::msg::Frame frame_template_;  ///< 共通ヘッダを設定済みのフレームテンプレート
  std_msgs::msg::Float32 fb_msg_;
};

#endif  // EDULITE05_DRIVER__EDULITE05_NODE_HPP_
