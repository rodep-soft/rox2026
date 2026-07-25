#include "dribble_position_controller/dribble_position_controller.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

// dribble機構の位置制御。joy_controllerのボタンで/dribble/position_modeを受け取り、
// hardware_driverへ目標位置(rad)をpublishする。
// 実位置feedbackは見ず、時間で位置を進める簡易方式。
//  - DRIBBLE指令: dribble_position_radへ移動する。
//  - SHOOT指令  : intake_position_radへ移動し、intake_to_shoot_delay_sec経過後に
//                 shoot_position_radへ移動する。

DribblePositionController::DribblePositionController()
: Node("dribble_position_controller")
{
  declare_parameters();
  get_parameters();

  if (command_period_ms_ <= 0) {
    RCLCPP_WARN(
      get_logger(), "command_period_ms must be positive. Using the default value of 20 ms.");
    command_period_ms_ = 20;
  }
  if (intake_to_shoot_delay_sec_ < 0.0) {
    RCLCPP_WARN(
      get_logger(),
      "intake_to_shoot_delay_sec must be zero or greater. Using the default value of 1.0 s.");
    intake_to_shoot_delay_sec_ = 1.0;
  }
  if (qos_depth_ <= 0) {
    RCLCPP_WARN(get_logger(), "qos_depth must be positive. Using the default value of 1.");
    qos_depth_ = 1;
  }

  position_command_pub_ = create_publisher<std_msgs::msg::Float32>(
    dribble_position_command_topic_, rclcpp::QoS(qos_depth_));
  position_mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
    dribble_position_mode_topic_, rclcpp::QoS(qos_depth_),
    std::bind(&DribblePositionController::position_mode_callback, this, std::placeholders::_1));
  timer_ = create_wall_timer(
    std::chrono::milliseconds(command_period_ms_),
    std::bind(&DribblePositionController::timer_callback, this));

  // 起動時はドリブル位置で保持する。
  publish_target_position(dribble_position_rad_);
}

void DribblePositionController::declare_parameters()
{
  declare_parameter<std::string>("dribble_position_command_topic", "/dribble/position_command");
  declare_parameter<std::string>("dribble_position_mode_topic", "/dribble/position_mode");
  declare_parameter<double>("dribble_position_rad", 0.0);
  declare_parameter<double>("intake_position_rad", 1.5);
  declare_parameter<double>("shoot_position_rad", 2.0);
  declare_parameter<double>("intake_to_shoot_delay_sec", 1.0);
  declare_parameter<int>("command_period_ms", 20);
  declare_parameter<int>("qos_depth", 1);
}

void DribblePositionController::get_parameters()
{
  get_parameter("dribble_position_command_topic", dribble_position_command_topic_);
  get_parameter("dribble_position_mode_topic", dribble_position_mode_topic_);
  get_parameter("dribble_position_rad", dribble_position_rad_);
  get_parameter("intake_position_rad", intake_position_rad_);
  get_parameter("shoot_position_rad", shoot_position_rad_);
  get_parameter("intake_to_shoot_delay_sec", intake_to_shoot_delay_sec_);
  get_parameter("command_period_ms", command_period_ms_);
  get_parameter("qos_depth", qos_depth_);
}

void DribblePositionController::position_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg)
{
  switch (msg->data) {
    case dribble_mode_:
      // ドリブル位置へ戻す。SHOOT進行中なら中断する。
      shoot_pending_ = false;
      publish_target_position(dribble_position_rad_);
      break;

    case shoot_mode_:
      // まずINTAKE位置へ動かし、遅延後にSHOOT位置へ進める。
      publish_target_position(intake_position_rad_);
      intake_start_time_ = now();
      shoot_pending_ = true;
      break;

    default:
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Unsupported dribble position mode %u. Ignoring.", msg->data);
      break;
  }
}

void DribblePositionController::timer_callback()
{
  // SHOOT指令後、指定時間が経過したらSHOOT位置へ進める。
  if (shoot_pending_ &&
    (now() - intake_start_time_).seconds() >= intake_to_shoot_delay_sec_)
  {
    publish_target_position(shoot_position_rad_);
    shoot_pending_ = false;
  }
}

void DribblePositionController::publish_target_position(double position_rad)
{
  std_msgs::msg::Float32 command;
  command.data = static_cast<float>(position_rad);
  position_command_pub_->publish(command);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DribblePositionController>());
  rclcpp::shutdown();
  return 0;
}
