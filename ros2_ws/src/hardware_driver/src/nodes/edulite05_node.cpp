#include "edulite05_driver/edulite05_node.hpp"

#include <chrono>

Ed05DriverNode::Ed05DriverNode()
: Node("edulite05_driver_node")
{
  RCLCPP_INFO(this->get_logger(), "Edulite05Node has been started.");

  declare_parameters();
  get_parameters();

  frame_template_.is_extended = true;
  frame_template_.is_rtr = false;
  frame_template_.is_error = false;

  // CAN通信のQoS Queue Depthを50に拡大して message was lost やドロップを強力に防止
  const auto can_qos_pub = rclcpp::QoS(rclcpp::KeepLast(50)).reliable().durability_volatile();
  const auto can_qos_sub = rclcpp::QoS(rclcpp::KeepLast(50)).reliable().durability_volatile();

  cmd_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    sub_cmd_topic_name_, 10,
    std::bind(&Ed05DriverNode::cmd_callback, this, std::placeholders::_1));

  can_pub_ = this->create_publisher<can_msgs::msg::Frame>(pub_can_topic_name_, can_qos_pub);

  can_sub_ = this->create_subscription<can_msgs::msg::Frame>(
    sub_can_topic_name_, can_qos_sub,
    std::bind(&Ed05DriverNode::can_callback, this, std::placeholders::_1));

  fb_pub_ = this->create_publisher<std_msgs::msg::Float32>(pub_fb_topic_name_, 10);

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

  // 1.5秒ごとにトルク確定状態（0x02受信）を監視し、未イネーブルなら自動再送するタイマー
  retry_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1500),
    std::bind(&Ed05DriverNode::retry_timer_callback, this));

  // 起動時のCANフレーム衝突を回避するため、モータIDごとに分散して初期化を開始する
  schedule_staggered_init();
}

Ed05DriverNode::~Ed05DriverNode()
{
  RCLCPP_INFO(this->get_logger(), "Ed05Node is shutting down.");
}

void Ed05DriverNode::terminate_motor()
{
  init_state_ = InitState::IDLE;
  publish_frame(motor_->terminate_motor());
  RCLCPP_DEBUG(this->get_logger(), "Published terminate frame for motor %d.", motor_id_);
}

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

void Ed05DriverNode::schedule_staggered_init()
{
  // モータIDの下位4ビットに基づき 50ms〜450ms の起動遅延を設けてバースト衝突を完全に防止
  const int delay_ms = 50 + (motor_id_ % 10) * 60;
  init_delay_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(delay_ms),
    [this]() {
      init_delay_timer_->cancel();
      start_init();
    });
}

void Ed05DriverNode::start_init()
{
  if (!motor_) {
    return;
  }

  pending_frames_ = motor_->create_init_frame();
  pending_frame_index_ = 0;
  init_state_ = InitState::SENDING;

  // 60ms 間隔でフレームを1枚ずつ送信（CANバス負荷を軽減）
  init_frame_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(60),
    std::bind(&Ed05DriverNode::init_frame_timer_callback, this));
}

void Ed05DriverNode::init_frame_timer_callback()
{
  if (pending_frame_index_ >= pending_frames_.size()) {
    init_frame_timer_->cancel();
    RCLCPP_INFO(
      this->get_logger(), "Motor %d: all init frames sent. Awaiting CAN feedback (0x02)...",
      motor_id_);
    return;
  }

  publish_frame(pending_frames_[pending_frame_index_]);
  ++pending_frame_index_;
}

void Ed05DriverNode::retry_timer_callback()
{
  if (init_state_ == InitState::ENABLED) {
    // 最後にフィードバックを受信してから 3.0 秒以上経過していたら通信途絶と判定して再初期化
    if ((now() - last_feedback_time_).seconds() > 3.0) {
      RCLCPP_WARN(
        this->get_logger(), "Motor %d: CAN feedback lost for >3.0s. Re-initializing...",
        motor_id_);
      init_state_ = InitState::IDLE;
      start_init();
    }
    return;
  }

  if (init_state_ == InitState::SENDING) {
    // まだ初期化フレーム送信中のためスキップ
    return;
  }

  // 初期化完了（0x02受信）に至っていないため再送
  RCLCPP_WARN(
    this->get_logger(),
    "Motor %d: Torque ON not confirmed yet (no 0x02 FB). Retrying init frames...",
    motor_id_);
  start_init();
}

void Ed05DriverNode::cmd_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  if (init_state_ != InitState::ENABLED) {
    return;  // モータからフィードバック応答がありトルクONが確定するまで制御指令を送らない
  }
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
    last_feedback_time_ = now();

    // CAN通信によりモータから実フィードバックを受信！トルクONを100%確定
    if (init_state_ != InitState::ENABLED) {
      init_state_ = InitState::ENABLED;
      RCLCPP_INFO(
        this->get_logger(), "Motor %d: CAN feedback (0x02) confirmed! Torque ON LOCKED.",
        motor_id_);
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
    // comm_type 0x00 はモータからの自動ステータス通知。
    // フォルト検知時のみ警告ログを出力し、誤ったトルクオフ（set_disable）を伴う再初期化は行わない。
    if (id_info.fault_info != 0) {
      RCLCPP_WARN(
        this->get_logger(), "Motor %d: status frame (0x00) fault_info=0x%02X",
        motor_id_, id_info.fault_info);
    }
  }
}

void Ed05DriverNode::publish_frame(const Canframe & frame)
{
  frame_template_.id = frame.id;
  frame_template_.dlc = frame.dlc;
  frame_template_.data = frame.data;
  can_pub_->publish(frame_template_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Ed05DriverNode>();
  rclcpp::spin(node);
  node->terminate_motor();
  rclcpp::shutdown();
  return 0;
}
