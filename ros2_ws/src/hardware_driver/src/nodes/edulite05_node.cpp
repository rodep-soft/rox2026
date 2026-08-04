#include "edulite05_driver/edulite05_node.hpp"

#include <chrono>

// ────────────────────────────────────────────────────────────────────────────
// コンストラクタ・デストラクタ
// ────────────────────────────────────────────────────────────────────────────

Ed05DriverNode::Ed05DriverNode()
: Node("edulite05_driver_node")
{
  RCLCPP_INFO(this->get_logger(), "Edulite05Node has been started.");

  declare_parameters();
  get_parameters();

  // フレームテンプレートの共通ヘッダを設定（全送信フレームで共有）
  frame_template_.is_extended = true;
  frame_template_.is_rtr = false;
  frame_template_.is_error = false;

  const auto can_qos_pub = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  const auto can_qos_sub = rclcpp::SensorDataQoS();

  cmd_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    sub_cmd_topic_name_, 1,
    std::bind(&Ed05DriverNode::cmd_callback, this, std::placeholders::_1));

  can_pub_ = this->create_publisher<can_msgs::msg::Frame>(pub_can_topic_name_, can_qos_pub);

  can_sub_ = this->create_subscription<can_msgs::msg::Frame>(
    sub_can_topic_name_, can_qos_sub,
    std::bind(&Ed05DriverNode::can_callback, this, std::placeholders::_1));

  fb_pub_ = this->create_publisher<std_msgs::msg::Float32>(pub_fb_topic_name_, 10);

  // モータクラスを生成して初期化フレーム送信を開始する
  if (runmode_ == "Velocity") {
    motor_ = std::make_unique<Velocity>(motor_id_);
  } else if (runmode_ == "Position") {
    motor_ = std::make_unique<Position>(motor_id_);
  } else {
    RCLCPP_ERROR(
      this->get_logger(), "Invalid runmode: %s. Must be \"Velocity\" or \"Position\".",
      runmode_.c_str());
    return;
  }

  // 3秒ごとにイネーブル未確定を検知してリトライするタイマー
  retry_timer_ = this->create_wall_timer(
    std::chrono::seconds(3),
    std::bind(&Ed05DriverNode::retry_timer_callback, this));

  start_init();
}

Ed05DriverNode::~Ed05DriverNode()
{
  RCLCPP_INFO(this->get_logger(), "Ed05Node is shutting down.");
}

// ────────────────────────────────────────────────────────────────────────────
// 公開メソッド
// ────────────────────────────────────────────────────────────────────────────

void Ed05DriverNode::terminate_motor()
{
  init_state_ = InitState::IDLE;
  publish_frame(motor_->terminate_motor());
  RCLCPP_DEBUG(this->get_logger(), "Published terminate frame for motor %d.", motor_id_);
}

// ────────────────────────────────────────────────────────────────────────────
// パラメータ
// ────────────────────────────────────────────────────────────────────────────

void Ed05DriverNode::declare_parameters()
{
  this->declare_parameter<std::string>("sub_cmd_topic_name", "cmd");
  this->declare_parameter<std::string>("pub_can_topic_name", "can_tx");
  this->declare_parameter<std::string>("sub_can_topic_name", "can_rx");
  this->declare_parameter<std::string>("pub_fb_topic_name", "fb");
  this->declare_parameter<uint8_t>("motor_id", 0x01);
  this->declare_parameter<std::string>("runmode", "Velocity");
  this->declare_parameter<bool>("is_requested_fb_pub", false);
}

void Ed05DriverNode::get_parameters()
{
  sub_cmd_topic_name_ = this->get_parameter("sub_cmd_topic_name").as_string();
  pub_can_topic_name_ = this->get_parameter("pub_can_topic_name").as_string();
  sub_can_topic_name_ = this->get_parameter("sub_can_topic_name").as_string();
  pub_fb_topic_name_ = this->get_parameter("pub_fb_topic_name").as_string();
  motor_id_ = static_cast<uint8_t>(this->get_parameter("motor_id").as_int());
  runmode_ = this->get_parameter("runmode").as_string();
  is_requested_fb_pub_ = this->get_parameter("is_requested_fb_pub").as_bool();
}

// ────────────────────────────────────────────────────────────────────────────
// 初期化フレーム送信（非同期）
// ────────────────────────────────────────────────────────────────────────────

void Ed05DriverNode::start_init()
{
  if (!motor_) {
    return;
  }

  // 送信すべき初期化フレームリストを構築してキューに詰める
  pending_frames_ = motor_->create_init_frame();
  pending_frame_index_ = 0;
  init_state_ = InitState::SENDING;

  // 50ms 間隔で1枚ずつ送信する一発タイマーを起動（キャンセル済みなら新規作成）
  init_frame_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&Ed05DriverNode::init_frame_timer_callback, this));
}

void Ed05DriverNode::init_frame_timer_callback()
{
  if (pending_frame_index_ >= pending_frames_.size()) {
    // 全フレームを送信完了 → ENABLED に遷移してタイマーを停止
    init_frame_timer_->cancel();
    init_state_ = InitState::ENABLED;
    RCLCPP_INFO(
      this->get_logger(), "Motor %d: init frames sent. Torque ON assumed.", motor_id_);
    return;
  }

  publish_frame(pending_frames_[pending_frame_index_]);
  ++pending_frame_index_;
}

void Ed05DriverNode::retry_timer_callback()
{
  if (init_state_ == InitState::ENABLED) {
    // イネーブル確定済み → リトライ不要
    retry_timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "Motor %d enabled. Retry timer cancelled.", motor_id_);
    return;
  }
  if (init_state_ == InitState::SENDING) {
    // まだ初期化フレーム送信中 → 次回まで待つ
    return;
  }
  // IDLE（初期化未完 or 再起動検知後）→ 再送
  RCLCPP_WARN(
    this->get_logger(), "Motor %d: not enabled yet. Retrying init frames...", motor_id_);
  start_init();
}

// ────────────────────────────────────────────────────────────────────────────
// ROS コールバック
// ────────────────────────────────────────────────────────────────────────────

void Ed05DriverNode::cmd_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  publish_frame(motor_->create_control_frame(msg->data));
  RCLCPP_DEBUG(this->get_logger(), "publish motor %d: %f", motor_id_, msg->data);
}

void Ed05DriverNode::can_callback(const can_msgs::msg::Frame::SharedPtr msg)
{
  const CanIdInfo id_info = decode_can_id(msg->id);

  if (id_info.motor_id != motor_id_) {
    return;
  }

  if (id_info.comm_type == 0x02) {
    // フィードバック受信 → CAN通信で正式にイネーブル確認
    if (init_state_ != InitState::ENABLED) {
      init_state_ = InitState::ENABLED;
      RCLCPP_INFO(
        this->get_logger(), "Motor %d: feedback confirmed. Torque ON.", motor_id_);
    }

    std::array<uint8_t, 8> data_array{};
    std::copy(msg->data.begin(), msg->data.end(), data_array.begin());
    const MotorFeedbackData fb_data = decode_feedback_data(data_array);

    if (runmode_ == "Velocity") {
      fb_msg_.data = fb_data.velocity;
    } else if (runmode_ == "Position") {
      fb_msg_.data = fb_data.angle;
    }

    if (is_requested_fb_pub_) {
      fb_pub_->publish(fb_msg_);
    }
  } else if (id_info.comm_type == 0x00) {
    // モータ再起動通知 → 再初期化
    RCLCPP_WARN(
      this->get_logger(), "Motor %d: reboot detected. Re-initializing...", motor_id_);
    init_state_ = InitState::IDLE;
    if (init_frame_timer_) {
      init_frame_timer_->cancel();
    }
    retry_timer_->reset();
    start_init();
  }
}

// ────────────────────────────────────────────────────────────────────────────
// フレーム送信ヘルパー
// ────────────────────────────────────────────────────────────────────────────

void Ed05DriverNode::publish_frame(const Canframe & frame)
{
  frame_template_.id = frame.id;
  frame_template_.dlc = frame.dlc;
  frame_template_.data = frame.data;
  can_pub_->publish(frame_template_);
}

// ────────────────────────────────────────────────────────────────────────────
// エントリポイント
// ────────────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Ed05DriverNode>();
  rclcpp::spin(node);
  node->terminate_motor();
  rclcpp::shutdown();
  return 0;
}
