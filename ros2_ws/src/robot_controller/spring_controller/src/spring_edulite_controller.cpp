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
  limit_switch_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
    limit_switch_topic_, rclcpp::QoS(qos_depth_),
    std::bind(&SpringEduliteController::limit_switch_callback, this, std::placeholders::_1));
  dribble_is_stopped_sub_ = create_subscription<std_msgs::msg::Bool>(
    dribble_is_stopped_topic_, rclcpp::QoS(qos_depth_),
    std::bind(
      &SpringEduliteController::dribble_is_stopped_callback, this, std::placeholders::_1));
  spring_velocity_pub_ = create_publisher<std_msgs::msg::Float32>(
    spring_velocity_command_topic_, rclcpp::QoS(qos_depth_));
  dribble_stop_request_pub_ = create_publisher<std_msgs::msg::Bool>(
    dribble_stop_request_topic_, rclcpp::QoS(qos_depth_));

  timer_ = create_wall_timer(
    std::chrono::milliseconds(command_period_ms_),
    std::bind(&SpringEduliteController::timer_callback, this));
}

void SpringEduliteController::declare_parameters()
{
  declare_parameter<std::string>("fire_request_topic", "/spring/fire_request");
  declare_parameter<std::string>("limit_switch_topic", "/limit_switches");
  declare_parameter<std::string>("spring_velocity_command_topic", "/spring/vel_command");
  declare_parameter<std::string>("dribble_stop_request_topic", "/dribble_stop_request");
  declare_parameter<std::string>("dribble_is_stopped_topic", "/dribble_is_stopped");
  declare_parameter<int>("limit_switch_index", 0);
  declare_parameter<bool>("stop_dribble_on_fire", true);
  declare_parameter<double>("loading_velocity_rad_s", -5.0);
  declare_parameter<double>("fire_velocity_rad_s", -20.0);
  declare_parameter<double>("fire_duration_sec", 5.0);
  declare_parameter<int>("command_period_ms", 10);
  declare_parameter<int>("qos_depth", 1);
}

void SpringEduliteController::get_parameters()
{
  get_parameter("fire_request_topic", fire_request_topic_);
  get_parameter("limit_switch_topic", limit_switch_topic_);
  get_parameter("spring_velocity_command_topic", spring_velocity_command_topic_);
  get_parameter("dribble_stop_request_topic", dribble_stop_request_topic_);
  get_parameter("dribble_is_stopped_topic", dribble_is_stopped_topic_);
  get_parameter("limit_switch_index", limit_switch_index_);
  get_parameter("stop_dribble_on_fire", stop_dribble_on_fire_);
  get_parameter("loading_velocity_rad_s", loading_velocity_rad_s_);
  get_parameter("fire_velocity_rad_s", fire_velocity_rad_s_);
  get_parameter("fire_duration_sec", fire_duration_sec_);
  get_parameter("command_period_ms", command_period_ms_);
  get_parameter("qos_depth", qos_depth_);
}

void SpringEduliteController::fire_request_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  // READYかつ装填済みのときだけ、発射要求の立ち上がりを受け付ける。
  if (
    now_state_ == State::READY && is_loaded_ &&
    msg->data && !previous_fire_request_)
  {
    fire_pending_ = true;
    dribble_is_stopped_ = false;
  }
  previous_fire_request_ = msg->data;
}

void SpringEduliteController::dribble_is_stopped_callback(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  dribble_is_stopped_ = msg->data;
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

void SpringEduliteController::publish_dribble_stop_request()
{
  std_msgs::msg::Bool stop_request;
  // 発射待ちまたは発射中だけ、dribble_controllerへ停止要求を出す。
  stop_request.data = stop_dribble_on_fire_ &&
    (fire_pending_ || now_state_ == State::FIRE);
  dribble_stop_request_pub_->publish(stop_request);
}

void SpringEduliteController::timer_callback()
{
  std_msgs::msg::Float32 velocity_command;

  if (!is_configuration_valid_) {
    velocity_command.data = 0.0F;
    spring_velocity_pub_->publish(velocity_command);
    publish_dribble_stop_request();
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
      // 発射要求を受けたら、必要に応じてdribble停止完了を待ってからFIREへ移る。
      if (fire_pending_ && is_loaded_ &&
        (!stop_dribble_on_fire_ || dribble_is_stopped_))
      {
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
  publish_dribble_stop_request();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpringEduliteController>());
  rclcpp::shutdown();
  return 0;
}
