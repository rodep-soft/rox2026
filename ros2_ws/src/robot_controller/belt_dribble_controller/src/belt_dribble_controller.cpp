#include "belt_dribble_controller/belt_dribble_controller.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>

BeltDribbleController::BeltDribbleController()
    : Node("belt_dribble_controller_node") {
  declare_parameters();
  get_parameters();
  validate_parameters();
  create_interfaces();

  timer_ = create_wall_timer(
      std::chrono::milliseconds(command_period_ms_),
      std::bind(&BeltDribbleController::timer_callback, this));
}

void BeltDribbleController::declare_parameters() {
  // Topic名
  declare_parameter<std::string>("operation_mode_topic", "/operation_mode");
  declare_parameter<std::string>("belt_mode_topic", "/belt/mode");
  declare_parameter<std::string>("dribble_enabled_topic", "/dribble/enabled");
  declare_parameter<std::string>("game2_command_topic", "/game2_command");
  declare_parameter<std::string>("emergency_stop_topic", "/emergency_stop");
  declare_parameter<std::string>("underbelt_target_rpm_topic",
                                 "/underbelt/target/rpm");
  declare_parameter<std::string>("upperbelt_target_rpm_topic",
                                 "/upperbelt/target/rpm");
  declare_parameter<std::string>("dribble_target_rpm_topic",
                                 "/dribble/target/rpm");
  declare_parameter<std::string>("underbelt_current_rpm_topic",
                                 "/underbelt/current/rpm");
  declare_parameter<std::string>("upperbelt_current_rpm_topic",
                                 "/upperbelt/current/rpm");
  declare_parameter<std::string>("dribble_current_rpm_topic",
                                 "/dribble/current/rpm");
  declare_parameter<std::string>("intake_and_shoot_topic", "/intake_and_shoot");
  declare_parameter<std::string>("shoot_ready_topic", "/shoot_ready");

  // RPMと到達判定
  declare_parameter<int>("stop_rpm", 0);
  declare_parameter<int>("level_1_rpm", 3000);
  declare_parameter<int>("level_2_rpm", 4000);
  declare_parameter<int>("level_3_rpm", 5000);
  declare_parameter<int>("dribble_on_rpm", 2000);
  declare_parameter<int>("belt_rpm_tolerance", 100);
  declare_parameter<int>("dribble_rpm_tolerance", 100);
  declare_parameter<double>("ready_hold_sec", 0.1);
  declare_parameter<int>("command_period_ms", 10);
  declare_parameter<int>("qos_depth", 1);
}

void BeltDribbleController::get_parameters() {
  // Topic名
  get_parameter("operation_mode_topic", operation_mode_topic_);
  get_parameter("belt_mode_topic", belt_mode_topic_);
  get_parameter("dribble_enabled_topic", dribble_enabled_topic_);
  get_parameter("game2_command_topic", game2_command_topic_);
  get_parameter("emergency_stop_topic", emergency_stop_topic_);
  get_parameter("underbelt_target_rpm_topic", underbelt_target_rpm_topic_);
  get_parameter("upperbelt_target_rpm_topic", upperbelt_target_rpm_topic_);
  get_parameter("dribble_target_rpm_topic", dribble_target_rpm_topic_);
  get_parameter("underbelt_current_rpm_topic", underbelt_current_rpm_topic_);
  get_parameter("upperbelt_current_rpm_topic", upperbelt_current_rpm_topic_);
  get_parameter("dribble_current_rpm_topic", dribble_current_rpm_topic_);
  get_parameter("intake_and_shoot_topic", intake_and_shoot_topic_);
  get_parameter("shoot_ready_topic", shoot_ready_topic_);

  // RPMと到達判定
  get_parameter("stop_rpm", stop_rpm_);
  get_parameter("level_1_rpm", level_1_rpm_);
  get_parameter("level_2_rpm", level_2_rpm_);
  get_parameter("level_3_rpm", level_3_rpm_);
  get_parameter("dribble_on_rpm", dribble_on_rpm_);
  get_parameter("belt_rpm_tolerance", belt_rpm_tolerance_);
  get_parameter("dribble_rpm_tolerance", dribble_rpm_tolerance_);
  get_parameter("ready_hold_sec", ready_hold_sec_);
  get_parameter("command_period_ms", command_period_ms_);
  get_parameter("qos_depth", qos_depth_);
}

void BeltDribbleController::validate_parameters() {
  if (stop_rpm_ != 0) {
    RCLCPP_ERROR(get_logger(), "stop_rpm must be zero");
    configuration_valid_ = false;
  }

  if (!is_rpm_valid(level_1_rpm_) || !is_rpm_valid(level_2_rpm_) ||
      !is_rpm_valid(level_3_rpm_) || !is_rpm_valid(dribble_on_rpm_)) {
    RCLCPP_ERROR(get_logger(), "Configured RPM must fit in std_msgs/msg/Int16");
    configuration_valid_ = false;
  }
  if (belt_rpm_tolerance_ < 0 || dribble_rpm_tolerance_ < 0) {
    RCLCPP_ERROR(get_logger(), "RPM tolerances must be zero or greater");
    configuration_valid_ = false;
  }
  if (ready_hold_sec_ < 0.0) {
    RCLCPP_ERROR(get_logger(), "ready_hold_sec must be zero or greater");
    configuration_valid_ = false;
  }
  if (command_period_ms_ <= 0) {
    RCLCPP_ERROR(get_logger(), "command_period_ms must be greater than zero");
    configuration_valid_ = false;
    command_period_ms_ = 10;
  }
  if (qos_depth_ <= 0) {
    RCLCPP_WARN(get_logger(), "qos_depth must be positive. Using 1.");
    qos_depth_ = 1;
  }
}

void BeltDribbleController::create_interfaces() {
  const auto command_qos = rclcpp::QoS(qos_depth_);
  const auto state_qos = rclcpp::QoS(1).reliable().transient_local();

  operation_mode_subscription_ = create_subscription<std_msgs::msg::UInt8>(
      operation_mode_topic_, state_qos,
      std::bind(&BeltDribbleController::operation_mode_callback, this,
                std::placeholders::_1));
  belt_mode_subscription_ = create_subscription<std_msgs::msg::UInt8>(
      belt_mode_topic_, command_qos,
      std::bind(&BeltDribbleController::belt_mode_callback, this,
                std::placeholders::_1));
  dribble_enabled_subscription_ = create_subscription<std_msgs::msg::Bool>(
      dribble_enabled_topic_, command_qos,
      std::bind(&BeltDribbleController::dribble_enabled_callback, this,
                std::placeholders::_1));
  game2_command_subscription_ = create_subscription<std_msgs::msg::Bool>(
      game2_command_topic_, command_qos,
      std::bind(&BeltDribbleController::game2_command_callback, this,
                std::placeholders::_1));
  emergency_stop_subscription_ = create_subscription<std_msgs::msg::Bool>(
      emergency_stop_topic_, state_qos,
      std::bind(&BeltDribbleController::emergency_stop_callback, this,
                std::placeholders::_1));
  underbelt_feedback_subscription_ = create_subscription<std_msgs::msg::Int16>(
      underbelt_current_rpm_topic_, command_qos,
      std::bind(&BeltDribbleController::underbelt_feedback_callback, this,
                std::placeholders::_1));
  upperbelt_feedback_subscription_ = create_subscription<std_msgs::msg::Int16>(
      upperbelt_current_rpm_topic_, command_qos,
      std::bind(&BeltDribbleController::upperbelt_feedback_callback, this,
                std::placeholders::_1));
  dribble_feedback_subscription_ = create_subscription<std_msgs::msg::Int16>(
      dribble_current_rpm_topic_, command_qos,
      std::bind(&BeltDribbleController::dribble_feedback_callback, this,
                std::placeholders::_1));

  underbelt_target_publisher_ = create_publisher<std_msgs::msg::Int16>(
      underbelt_target_rpm_topic_, command_qos);
  upperbelt_target_publisher_ = create_publisher<std_msgs::msg::Int16>(
      upperbelt_target_rpm_topic_, command_qos);
  dribble_target_publisher_ = create_publisher<std_msgs::msg::Int16>(
      dribble_target_rpm_topic_, command_qos);
  intake_and_shoot_publisher_ = create_publisher<std_msgs::msg::Bool>(
      intake_and_shoot_topic_, command_qos);
  shoot_ready_publisher_ =
      create_publisher<std_msgs::msg::Bool>(shoot_ready_topic_, state_qos);
}

void BeltDribbleController::operation_mode_callback(
    const std_msgs::msg::UInt8::SharedPtr msg) {
  OperationMode new_mode = OperationMode::STOP;
  if (msg->data > static_cast<uint8_t>(OperationMode::GAME2_MODE)) {
    new_mode = OperationMode::STOP;
  } else {
    new_mode = static_cast<OperationMode>(msg->data);
  }
  if (new_mode == operation_mode_) {
    return;
  }

  operation_mode_ = new_mode;
  reset_shoot_ready();
}

void BeltDribbleController::belt_mode_callback(
    const std_msgs::msg::UInt8::SharedPtr msg) {
  if (msg->data < static_cast<uint8_t>(BeltMode::STOP) ||
      msg->data > static_cast<uint8_t>(BeltMode::LEVEL_3)) {
    belt_mode_ = BeltMode::STOP;
    return;
  }
  const BeltMode new_mode = static_cast<BeltMode>(msg->data);
  if (new_mode != belt_mode_) {
    belt_mode_ = new_mode;
    reset_shoot_ready();
  }
}

void BeltDribbleController::dribble_enabled_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg->data != dribble_enabled_) {
    dribble_enabled_ = msg->data;
    reset_shoot_ready();
  }
}

void BeltDribbleController::game2_command_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  if (!msg->data || emergency_stop_active_ ||
      operation_mode_ != OperationMode::INTAKE_AND_SHOOT || !shoot_ready_) {
    return;
  }

  std_msgs::msg::Bool command;
  command.data = true;
  intake_and_shoot_publisher_->publish(command);
}

void BeltDribbleController::emergency_stop_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  emergency_stop_active_ = msg->data;
  if (emergency_stop_active_) {
    reset_shoot_ready();
  }
}

void BeltDribbleController::underbelt_feedback_callback(
    const std_msgs::msg::Int16::SharedPtr msg) {
  underbelt_current_rpm_ = msg->data;
  underbelt_feedback_received_ = true;
}

void BeltDribbleController::upperbelt_feedback_callback(
    const std_msgs::msg::Int16::SharedPtr msg) {
  upperbelt_current_rpm_ = msg->data;
  upperbelt_feedback_received_ = true;
}

void BeltDribbleController::dribble_feedback_callback(
    const std_msgs::msg::Int16::SharedPtr msg) {
  dribble_current_rpm_ = msg->data;
  dribble_feedback_received_ = true;
}

void BeltDribbleController::timer_callback() {
  int current_belt_target = stop_rpm_;
  int current_dribble_target = stop_rpm_;

  if (configuration_valid_ && !emergency_stop_active_ &&
      operation_mode_ != OperationMode::STOP) {
    current_belt_target = belt_target_rpm();
    current_dribble_target = dribble_target_rpm();
  }

  std_msgs::msg::Int16 belt_command;
  belt_command.data = static_cast<int16_t>(current_belt_target);
  underbelt_target_publisher_->publish(belt_command);
  upperbelt_target_publisher_->publish(belt_command);

  std_msgs::msg::Int16 dribble_command;
  dribble_command.data = static_cast<int16_t>(current_dribble_target);
  dribble_target_publisher_->publish(dribble_command);

  shoot_ready_ =
      update_shoot_ready(current_belt_target, current_dribble_target, now());
  std_msgs::msg::Bool ready_message;
  ready_message.data = shoot_ready_;
  shoot_ready_publisher_->publish(ready_message);
}

int BeltDribbleController::belt_target_rpm() const {
  switch (belt_mode_) {
    case BeltMode::STOP:
      return stop_rpm_;
    case BeltMode::LEVEL_1:
      return level_1_rpm_;
    case BeltMode::LEVEL_2:
      return level_2_rpm_;
    case BeltMode::LEVEL_3:
      return level_3_rpm_;
  }
  return stop_rpm_;
}

int BeltDribbleController::dribble_target_rpm() const {
  if (operation_mode_ == OperationMode::GAME2_MODE || !dribble_enabled_) {
    return stop_rpm_;
  }
  return dribble_on_rpm_;
}

bool BeltDribbleController::update_shoot_ready(
    int current_belt_target, int current_dribble_target,
    const rclcpp::Time& current_time) {
  if (operation_mode_ != OperationMode::INTAKE_AND_SHOOT ||
      emergency_stop_active_ || !underbelt_feedback_received_ ||
      !upperbelt_feedback_received_ || !dribble_feedback_received_) {
    reset_shoot_ready();
    return false;
  }

  const bool belt_ready =
      std::abs(underbelt_current_rpm_ - current_belt_target) <=
          belt_rpm_tolerance_ &&
      std::abs(upperbelt_current_rpm_ - current_belt_target) <=
          belt_rpm_tolerance_;
  const bool dribble_ready =
      std::abs(dribble_current_rpm_ - current_dribble_target) <=
      dribble_rpm_tolerance_;

  if (!belt_ready || !dribble_ready) {
    reset_shoot_ready();
    return false;
  }
  if (ready_since_.nanoseconds() == 0) {
    ready_since_ = current_time;
  }
  return (current_time - ready_since_).seconds() >= ready_hold_sec_;
}

bool BeltDribbleController::is_rpm_valid(int rpm) const {
  return rpm >= std::numeric_limits<int16_t>::min() &&
         rpm <= std::numeric_limits<int16_t>::max();
}

void BeltDribbleController::reset_shoot_ready() {
  shoot_ready_ = false;
  ready_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BeltDribbleController>());
  rclcpp::shutdown();
  return 0;
}
