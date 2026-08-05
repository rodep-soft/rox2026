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
/// スタッガード（時分化）起動とCANフィードバック応答（0x02）の完全受信チェックにより
/// CANバスの衝突・フレームドロップを防ぎ、100%確実なトルクON（イネーブル）を保証する。
class Ed05DriverNode : public rclcpp::Node
{
public:
  Ed05DriverNode();
  ~Ed05DriverNode();

  /// @brief モータを停止させる終了フレームを送信する
  void terminate_motor();

private:
  enum class InitState
  {
    IDLE,        ///< 初期化未開始／応答待ち
    SENDING,     ///< 初期化フレームを順次送信中
    ENABLED,     ///< CANフィードバック応答受信済み・トルクON確定
  };

  void declare_parameters();
  void get_parameters();

  /// @brief モータIDに応じたジッター散らし（スタッガード）を挟んで初期化を開始
  void schedule_staggered_init();
  void start_init();
  void init_frame_timer_callback();
  void retry_timer_callback();

  void cmd_callback(const std_msgs::msg::Float32::SharedPtr msg);
  void can_callback(const can_msgs::msg::Frame::SharedPtr msg);

  void publish_frame(const Canframe & frame);

  // パラメータ
  std::string sub_cmd_topic_name_;
  std::string pub_can_topic_name_;
  std::string sub_can_topic_name_;
  std::string pub_fb_topic_name_;
  uint8_t motor_id_{0x01};
  std::string runmode_{"Velocity"};
  bool is_requested_fb_pub_{false};

  // 状態変数
  InitState init_state_{InitState::IDLE};
  std::vector<Canframe> pending_frames_;
  std::size_t pending_frame_index_{0};
  rclcpp::Time last_feedback_time_;  ///< 最後にモータ応答（0x02）を受信した時刻

  // ROS インタフェース
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cmd_sub_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr fb_pub_;

  rclcpp::TimerBase::SharedPtr init_delay_timer_;   ///< 起動時衝突回避用タイマー
  rclcpp::TimerBase::SharedPtr init_frame_timer_;   ///< 初期化フレーム送出タイマー
  rclcpp::TimerBase::SharedPtr retry_timer_;        ///< イネーブル未確定時の定期再送・監視タイマー

  std::unique_ptr<Ed05CanframeCreater> motor_;

  can_msgs::msg::Frame frame_template_;
  std_msgs::msg::Float32 fb_msg_;
};

#endif  // EDULITE05_DRIVER__EDULITE05_NODE_HPP_
