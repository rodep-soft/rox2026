#include "spring_controller/spring_edulite_controller.hpp"

#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>

SpringEduliteController::SpringEduliteController()
: Node("spring_controller_node")
{
  declare_parameters();
  get_parameters();

  if (limit_switch_index_ < 0) {
    RCLCPP_ERROR(get_logger(), "limit_switch_index must be zero or greater");
    is_configuration_valid_ = false;
  }
  if (fire_duration_sec_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "fire_duration_sec must be greater than zero");
    is_configuration_valid_ = false;
  }
  if (command_period_ms_ <= 0) {
    RCLCPP_ERROR(get_logger(), "command_period_ms must be greater than zero");
    is_configuration_valid_ = false;
    command_period_ms_ = 10;
  }
  if (qos_depth_ <= 0) {
    RCLCPP_WARN(get_logger(), "qos_depth must be positive. Using the default value of 1.");
    qos_depth_ = 1;
  }

  fire_request_sub_ = create_subscription<std_msgs::msg::Bool>(
    fire_request_topic_, rclcpp::QoS(qos_depth_),
    std::bind(&SpringEduliteController::fire_request_callback, this, std::placeholders::_1));
  emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
    emergency_stop_topic_, rclcpp::QoS(qos_depth_),
    std::bind(&SpringEduliteController::emergency_stop_callback, this, std::placeholders::_1));
  limit_switch_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
    limit_switch_topic_, rclcpp::QoS(qos_depth_),
    std::bind(&SpringEduliteController::limit_switch_callback, this, std::placeholders::_1));
  spring_velocity_pub_ = create_publisher<std_msgs::msg::Float32>(
    spring_velocity_command_topic_, rclcpp::QoS(qos_depth_));

  timer_ = create_wall_timer(
    std::chrono::milliseconds(command_period_ms_),
    std::bind(&SpringEduliteController::timer_callback, this));
}

void SpringEduliteController::declare_parameters()
{
  declare_parameter<std::string>("fire_request_topic", "/spring/fire_request");
  declare_parameter<std::string>("emergency_stop_topic", "/emergency_stop");
  declare_parameter<std::string>("limit_switch_topic", "/limit_switches");
  declare_parameter<std::string>("spring_velocity_command_topic", "/spring/vel_command");
  declare_parameter<int>("limit_switch_index", 0);
  declare_parameter<double>("loading_velocity_rad_s", -5.0);
  declare_parameter<double>("fire_velocity_rad_s", -20.0);
  declare_parameter<double>("fire_duration_sec", 5.0);
  declare_parameter<int>("command_period_ms", 10);
  declare_parameter<int>("qos_depth", 1);
}

void SpringEduliteController::get_parameters()
{
  get_parameter("fire_request_topic", fire_request_topic_);
  get_parameter("emergency_stop_topic", emergency_stop_topic_);
  get_parameter("limit_switch_topic", limit_switch_topic_);
  get_parameter("spring_velocity_command_topic", spring_velocity_command_topic_);
  get_parameter("limit_switch_index", limit_switch_index_);
  get_parameter("loading_velocity_rad_s", loading_velocity_rad_s_);
  get_parameter("fire_velocity_rad_s", fire_velocity_rad_s_);
  get_parameter("fire_duration_sec", fire_duration_sec_);
  get_parameter("command_period_ms", command_period_ms_);
  get_parameter("qos_depth", qos_depth_);
}

void SpringEduliteController::fire_request_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (emergency_stop_active_) {
    previous_fire_request_ = msg->data;
    return;
  }

  // READYかつ装填済みのときだけ、発射要求の立ち上がりを受け付ける。
  if (
    now_state_ == State::READY && is_loaded_ &&
    msg->data && !previous_fire_request_)
  {
    fire_pending_ = true;
  }
  previous_fire_request_ = msg->data;
}

void SpringEduliteController::emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  emergency_stop_active_ = msg->data;
  if (!emergency_stop_active_) {
    return;
  }

  // LOAD中は再装填を再開しないようREADYへ遷移して停止する。
  // FIRE中の射出は中断し、LOADへ戻して停止する。
  if (now_state_ == State::LOAD) {
    now_state_ = State::READY;
  } else if (now_state_ == State::FIRE) {
    now_state_ = State::LOAD;
  }
  fire_pending_ = false;
}

void SpringEduliteController::limit_switch_callback(
  const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
  const auto index = static_cast<std::size_t>(limit_switch_index_);
  if (limit_switch_index_ < 0 || index >= msg->data.size()) {
    RCLCPP_ERROR(
      get_logger(), "limit_switch_index %d is outside the received array of %zu elements",
      limit_switch_index_, msg->data.size());
    return;
  }
  is_loaded_ = msg->data[index] != 0;
}

void SpringEduliteController::start_fire()
{
  now_state_ = State::FIRE;
  fire_start_time_ = now();
  fire_pending_ = false;
}

void SpringEduliteController::timer_callback()
{
  std_msgs::msg::Float32 velocity_command;

  if (!is_configuration_valid_ || emergency_stop_active_) {
    // 非常停止中は、状態遷移を止めて0 rad/sを維持する。
    if (emergency_stop_active_) {
      fire_pending_ = false;
    }
    velocity_command.data = 0.0F;
    spring_velocity_pub_->publish(velocity_command);
    return;
  }

  switch (now_state_) {
    case State::LOAD:
      // 装填完了までは巻き取り、リミットスイッチ検出後にREADYへ移る。
      if (is_loaded_) {
        now_state_ = State::READY;
        velocity_command.data = 0.0F;
      } else {
        velocity_command.data = static_cast<float>(loading_velocity_rad_s_);
      }
      break;

    case State::READY:
      // 発射要求を受けたらFIREへ移る。
      if (fire_pending_ && is_loaded_) {
        start_fire();
        velocity_command.data = static_cast<float>(fire_velocity_rad_s_);
      } else if (!is_loaded_) {
        now_state_ = State::LOAD;
        velocity_command.data = static_cast<float>(loading_velocity_rad_s_);
      } else {
        velocity_command.data = 0.0F;
      }
      break;

    case State::FIRE:
      // 一定時間だけ発射速度を出し、その後は再装填のためLOADへ戻る。
      velocity_command.data = static_cast<float>(fire_velocity_rad_s_);
      if ((now() - fire_start_time_).seconds() >= fire_duration_sec_) {
        now_state_ = State::LOAD;
      }
      break;
  }

  spring_velocity_pub_->publish(velocity_command);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpringEduliteController>());
  rclcpp::shutdown();
  return 0;
}
