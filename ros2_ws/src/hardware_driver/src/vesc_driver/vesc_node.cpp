#include <chrono>
#include <cmath>
#include <limits>
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "vesc_driver/vesc_protocol.hpp"
using namespace std::chrono_literals;
namespace vesc_driver
{class Node : public rclcpp::Node
{
public:
  Node()
  : rclcpp::Node("vesc_driver_node")
  {
    auto tx = declare_parameter<std::string>("can_pub_topic", "/socketcan_bridge/tx"),
      rx = declare_parameter<std::string>("can_sub_topic", "/socketcan_bridge/rx");
    auto cmd = declare_parameter<std::string>("target_rpm_topic", "/vesc/target/rpm"),
      out = declare_parameter<std::string>("current_rpm_topic", "/vesc/current/rpm");
    auto ids = declare_parameter<std::vector<int64_t>>("controller_ids", {1}),
      pp = declare_parameter<std::vector<int64_t>>("pole_pairs", {7});
    timeout_ = std::chrono::milliseconds(declare_parameter<int>("command_timeout_ms", 250));
    if (ids.empty() || ids.size() != pp.size()) {
      throw std::invalid_argument("controller_ids/pole_pairs size mismatch");
    }
    for (size_t i = 0; i < ids.size(); i++) {
      if (ids[i] < 0 || ids[i] > 255 || pp[i] <= 0) {
        throw std::invalid_argument("invalid controller ID or pole pairs");
      }
      ids_.push_back(ids[i]);pairs_.push_back(pp[i]);map_[ids[i]] = i;
    }
    target_.assign(ids.size(), 0);rpm_.assign(
      ids.size(),
      NAN);tx_ = create_publisher<can_msgs::msg::Frame>(
      tx,
      10);pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(out, 10);
    rx_ = create_subscription<can_msgs::msg::Frame>(
      rx, 10, [this](
        can_msgs::msg::Frame::SharedPtr f) {
        protocol::Status1 s{};if (protocol::decode_status_1(*f, s) && map_.count(s.controller_id)) {
          rpm_[map_[s.controller_id]] = s.erpm / pairs_[map_[s.controller_id]];
        }
      });
    cmd_ =
      create_subscription<std_msgs::msg::Float32MultiArray>(
      cmd, 10,
      [this](std_msgs::msg::Float32MultiArray::SharedPtr m) {
        if (m->data.size() != ids_.size()) {
          RCLCPP_WARN(get_logger(), "wrong RPM array size");return;
        }
        target_ = m->data;last_ = std::chrono::steady_clock::now();active_ = true;
      });
    timer_ = create_wall_timer(
      20ms, [this] {
        if (!active_) {
          return;
        }
        bool stale = std::chrono::steady_clock::now() - last_ > timeout_;for (size_t i = 0;
        i < ids_.size(); i++)
        {
          tx_->publish(
            protocol::make_set_rpm_frame(
              ids_[i],
              std::lround((stale ? 0 : target_[i]) * pairs_[i])));
        }
        std_msgs::msg::Float32MultiArray m;m.data = rpm_;pub_->publish(m);
      });
  }

private:
  std::vector<uint8_t> ids_;std::vector<float> target_, rpm_;std::vector<double> pairs_;
  std::unordered_map<uint8_t, size_t> map_;bool active_ = false;
  std::chrono::steady_clock::time_point last_;std::chrono::milliseconds timeout_{250};
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr tx_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr rx_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_;
  rclcpp::TimerBase::SharedPtr timer_;
};}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);rclcpp::spin(std::make_shared<vesc_driver::Node>());rclcpp::shutdown();
}
