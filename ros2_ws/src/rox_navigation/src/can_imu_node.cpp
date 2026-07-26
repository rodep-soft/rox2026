#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <string>

#include "can_msgs/msg/frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class CanImuNode : public rclcpp::Node
{
public:
  CanImuNode()
  : Node("can_imu_node")
  {
    can_topic_ = declare_parameter("can_topic", "/socketcan_bridge/rx");
    frame_id_ = declare_parameter("frame_id", "imu_link");
    accel_id_ = declare_parameter("accel_can_id", 0x500);
    gyro_id_ = declare_parameter("gyro_can_id", 0x501);
    quaternion_id_ = declare_parameter("quaternion_can_id", 0x502);
    accel_scale_ = declare_parameter("accel_scale", 0.001 * 9.80665);
    gyro_scale_ = declare_parameter("gyro_scale", 0.001 * M_PI / 180.0);
    quaternion_scale_ = declare_parameter("quaternion_scale", 1.0 / 16384.0);
    max_component_age_ = declare_parameter("max_component_age", 0.1);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", 20);
    can_sub_ = create_subscription<can_msgs::msg::Frame>(
      can_topic_, 100, std::bind(&CanImuNode::on_frame, this, std::placeholders::_1));
  }

private:
  static int16_t read_i16(const std::array<uint8_t, 8> & data, std::size_t offset)
  {
    const uint16_t raw = static_cast<uint16_t>(data[offset]) |
      (static_cast<uint16_t>(data[offset + 1]) << 8U);
    return static_cast<int16_t>(raw);
  }

  void on_frame(const can_msgs::msg::Frame::SharedPtr frame)
  {
    if (frame->is_error || frame->is_rtr || frame->dlc < 6) {return;}
    const auto stamp = now();
    if (frame->id == static_cast<uint32_t>(accel_id_)) {
      for (std::size_t i = 0; i < 3; ++i) {accel_[i] = read_i16(frame->data, 2 * i) * accel_scale_;}
      accel_stamp_ = stamp;
      have_accel_ = true;
    } else if (frame->id == static_cast<uint32_t>(gyro_id_)) {
      for (std::size_t i = 0; i < 3; ++i) {gyro_[i] = read_i16(frame->data, 2 * i) * gyro_scale_;}
      gyro_stamp_ = stamp;
      have_gyro_ = true;
    } else if (frame->id == static_cast<uint32_t>(quaternion_id_) && frame->dlc >= 8) {
      for (std::size_t i = 0; i < 4; ++i) {
        quaternion_[i] = read_i16(frame->data, 2 * i) * quaternion_scale_;
      }
      quaternion_stamp_ = stamp;
      have_quaternion_ = true;
      publish_if_complete();
    }
  }

  void publish_if_complete()
  {
    if (!have_accel_ || !have_gyro_ || !have_quaternion_) {return;}
    const auto stamp = now();
    if ((stamp - accel_stamp_).seconds() > max_component_age_ ||
      (stamp - gyro_stamp_).seconds() > max_component_age_) {return;}
    const double norm = std::sqrt(
      quaternion_[0] * quaternion_[0] + quaternion_[1] * quaternion_[1] +
      quaternion_[2] * quaternion_[2] + quaternion_[3] * quaternion_[3]);
    if (norm < 0.5) {return;}
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    msg.orientation.x = quaternion_[0] / norm;
    msg.orientation.y = quaternion_[1] / norm;
    msg.orientation.z = quaternion_[2] / norm;
    msg.orientation.w = quaternion_[3] / norm;
    msg.angular_velocity.x = gyro_[0];
    msg.angular_velocity.y = gyro_[1];
    msg.angular_velocity.z = gyro_[2];
    msg.linear_acceleration.x = accel_[0];
    msg.linear_acceleration.y = accel_[1];
    msg.linear_acceleration.z = accel_[2];
    msg.orientation_covariance[0] = msg.orientation_covariance[4] = 0.02;
    msg.orientation_covariance[8] = 0.01;
    msg.angular_velocity_covariance[0] = msg.angular_velocity_covariance[4] = 0.01;
    msg.angular_velocity_covariance[8] = 0.005;
    msg.linear_acceleration_covariance[0] = msg.linear_acceleration_covariance[4] = 0.1;
    msg.linear_acceleration_covariance[8] = 0.1;
    imu_pub_->publish(msg);
  }

  std::string can_topic_, frame_id_;
  int64_t accel_id_, gyro_id_, quaternion_id_;
  double accel_scale_, gyro_scale_, quaternion_scale_, max_component_age_;
  std::array<double, 3> accel_{}, gyro_{};
  std::array<double, 4> quaternion_{};
  rclcpp::Time accel_stamp_{}, gyro_stamp_{}, quaternion_stamp_{};
  bool have_accel_{false}, have_gyro_{false}, have_quaternion_{false};
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanImuNode>());
  rclcpp::shutdown();
  return 0;
}
