#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "can_msgs/msg/frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "vesc_driver/vesc_protocol.hpp"

namespace vesc_driver
{

using namespace std::chrono_literals;

class Node : public rclcpp::Node
{
public:
  Node()
  : rclcpp::Node("vesc_driver_node")
  {
    const auto can_pub_topic =
      declare_parameter<std::string>("can_pub_topic", "/socketcan_bridge/tx");
    const auto can_sub_topic =
      declare_parameter<std::string>("can_sub_topic", "/socketcan_bridge/rx");
    const auto target_rpm_topic =
      declare_parameter<std::string>("target_rpm_topic", "/vesc/target/rpm");
    const auto current_rpm_topic = declare_parameter<std::string>(
      "current_rpm_topic", "/vesc/current/rpm");
    const auto controller_id = declare_parameter<int64_t>("controller_id", 1);
    const auto command_timeout_ms =
      declare_parameter<int64_t>("command_timeout_ms", 500);
    const auto feedback_timeout_ms =
      declare_parameter<int64_t>("feedback_timeout_ms", 500);
    max_rpm_ = declare_parameter<int>("max_rpm", 5600);
    rpm_slew_rate_ = declare_parameter<double>("rpm_slew_rate", 4000.0);

    if (controller_id < 0 ||
      controller_id > std::numeric_limits<uint8_t>::max())
    {
      throw std::invalid_argument("controller_id must be in the range 0 to 255");
    }
    if (command_timeout_ms <= 0 || feedback_timeout_ms <= 0) {
      throw std::invalid_argument(
              "command_timeout_ms and feedback_timeout_ms must be greater than zero");
    }
    if (max_rpm_ <= 0 ||
      max_rpm_ > std::numeric_limits<int16_t>::max())
    {
      throw std::invalid_argument(
              "max_rpm must be in the range 1 to 32767");
    }

    controller_id_ = static_cast<uint8_t>(controller_id);
    pole_pairs_ = static_cast<double>(protocol::MOTOR_POLES) / 2.0;

    command_timeout_ = std::chrono::milliseconds(command_timeout_ms);
    feedback_timeout_ = std::chrono::milliseconds(feedback_timeout_ms);
    started_at_ = std::chrono::steady_clock::now();

    auto can_qos_pub = rclcpp::QoS(rclcpp::KeepLast(10))
      .reliable()
      .durability_volatile();
    auto can_qos_sub = rclcpp::SensorDataQoS();

    can_pub_ = create_publisher<can_msgs::msg::Frame>(can_pub_topic, can_qos_pub);
    rpm_pub_ = create_publisher<std_msgs::msg::Int16>(current_rpm_topic, 10);

    rclcpp::SubscriptionOptions can_sub_options;
    can_sub_options.content_filter_options.filter_expression = "id = %0";
    can_sub_options.content_filter_options.expression_parameters = {
      std::to_string(
        (protocol::STATUS_1_ID << 8) |
        static_cast<uint32_t>(controller_id_))};
    can_sub_ = create_subscription<can_msgs::msg::Frame>(
      can_sub_topic, can_qos_sub,
      std::bind(&Node::can_callback, this, std::placeholders::_1),
      can_sub_options);
    target_rpm_sub_ = create_subscription<std_msgs::msg::Int16>(
      target_rpm_topic, 10,
      std::bind(&Node::target_rpm_callback, this, std::placeholders::_1));
    last_ramp_update_time_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(20ms, std::bind(&Node::timer_callback, this));

    if (!can_sub_->is_cft_enabled()) {
      RCLCPP_WARN(
        get_logger(),
        "CAN content filter is not supported by the current RMW; "
        "controller_id=%u will be filtered in the callback",
        static_cast<unsigned int>(controller_id_));
    }

    RCLCPP_INFO(
      get_logger(), "VESC driver started: controller_id=%u",
      static_cast<unsigned int>(controller_id_));
  }

private:
  void can_callback(const can_msgs::msg::Frame::SharedPtr frame)
  {
    protocol::Status1 status{};
    if (!protocol::decode_status_1(*frame, status) ||
      status.controller_id != controller_id_)
    {
      return;
    }

    current_rpm_ = rpm_to_int16(status.erpm / pole_pairs_);
    last_feedback_time_ = std::chrono::steady_clock::now();
    feedback_received_ = true;
    if (feedback_timed_out_) {
      RCLCPP_INFO(
        get_logger(), "VESC feedback resumed: current=%d RPM",
        current_rpm_);
      feedback_timed_out_ = false;
    }
  }

  void target_rpm_callback(const std_msgs::msg::Int16::SharedPtr msg)
  {
    if (std::abs(static_cast<int>(msg->data)) > max_rpm_) {
      RCLCPP_WARN(get_logger(), "Rejected invalid target RPM: %d", msg->data);
      return;
    }

    target_rpm_ = msg->data;
    last_command_time_ = std::chrono::steady_clock::now();
    command_received_ = true;
  }

  void timer_callback()
  {
    const auto now = std::chrono::steady_clock::now();

    if (command_received_) {
      const bool command_timed_out =
        now - last_command_time_ > command_timeout_;
      const double desired_rpm = command_timed_out ? 0.0 : target_rpm_;
      const double elapsed_seconds =
        std::chrono::duration<double>(now - last_ramp_update_time_).count();
      const double max_step = rpm_slew_rate_ * elapsed_seconds;
      commanded_rpm_ += std::clamp(
        desired_rpm - commanded_rpm_, -max_step, max_step);
      const double erpm = commanded_rpm_ * pole_pairs_;
      can_pub_->publish(
        protocol::make_set_rpm_frame(controller_id_, std::lround(erpm)));
    }
    last_ramp_update_time_ = now;

    const auto feedback_reference_time =
      feedback_received_ ? last_feedback_time_ : started_at_;
    const bool feedback_is_stale =
      now - feedback_reference_time > feedback_timeout_;
    if (feedback_is_stale) {
      if (!feedback_timed_out_) {
        RCLCPP_WARN(
          get_logger(),
          "VESC feedback timed out. RPM feedback publication is suspended.");
        feedback_timed_out_ = true;
      }
      return;
    }

    if (!feedback_received_) {
      return;
    }

    std_msgs::msg::Int16 feedback;
    feedback.data = current_rpm_;
    rpm_pub_->publish(feedback);
  }

  static int16_t rpm_to_int16(double rpm)
  {
    const int rounded_rpm = static_cast<int>(std::lround(rpm));
    const int minimum_rpm = std::numeric_limits<int16_t>::min();
    const int maximum_rpm = std::numeric_limits<int16_t>::max();
    const int limited_rpm = std::clamp(rounded_rpm, minimum_rpm, maximum_rpm);
    return static_cast<int16_t>(limited_rpm);
  }

  uint8_t controller_id_{1};
  double pole_pairs_{7.0};
  int max_rpm_{10000};
  double rpm_slew_rate_{4000.0};
  int16_t target_rpm_{0};
  double commanded_rpm_{0.0};
  int16_t current_rpm_{0};
  bool command_received_{false};
  bool feedback_received_{false};
  bool feedback_timed_out_{false};

  std::chrono::steady_clock::time_point started_at_;
  std::chrono::steady_clock::time_point last_command_time_;
  std::chrono::steady_clock::time_point last_feedback_time_;
  std::chrono::steady_clock::time_point last_ramp_update_time_;

  std::chrono::milliseconds command_timeout_{500};
  std::chrono::milliseconds feedback_timeout_{500};

  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr rpm_pub_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr target_rpm_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace vesc_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vesc_driver::Node>());
  rclcpp::shutdown();
  return 0;
}
