#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace robot_controller {

class ImuMuxNode : public rclcpp::Node {
public:
    explicit ImuMuxNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("imu_mux_node", options) {
        primary_topic_ = this->declare_parameter<std::string>("primary_imu_topic", "/bno055/imu");
        secondary_topic_ = this->declare_parameter<std::string>("secondary_imu_topic", "/stm32/imu");
        output_topic_ = this->declare_parameter<std::string>("output_imu_topic", "/imu/data");
        timeout_ms_ = this->declare_parameter<int>("timeout_ms", 50);

        out_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(output_topic_, rclcpp::SensorDataQoS());

        primary_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            primary_topic_, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                if (!is_valid_imu(*msg)) return;
                last_primary_time_ = this->now();
                if (active_source_ != Source::PRIMARY) {
                    RCLCPP_INFO(this->get_logger(), "[IMU MUX] Switched to PRIMARY source: %s", primary_topic_.c_str());
                    active_source_ = Source::PRIMARY;
                }
                out_pub_->publish(*msg);
            });

        secondary_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            secondary_topic_, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                if (!is_valid_imu(*msg)) return;
                const auto now_time = this->now();
                const bool primary_healthy = (last_primary_time_.nanoseconds() != 0) &&
                    ((now_time - last_primary_time_).nanoseconds() <= timeout_ms_ * 1000000LL);

                if (!primary_healthy) {
                    if (active_source_ != Source::SECONDARY) {
                        RCLCPP_WARN(this->get_logger(), "[IMU MUX] Primary timed out! Fallback to SECONDARY source: %s", secondary_topic_.c_str());
                        active_source_ = Source::SECONDARY;
                    }
                    out_pub_->publish(*msg);
                }
            });

        RCLCPP_INFO(this->get_logger(),
                    "IMU Mux online: Primary=%s Secondary=%s -> Output=%s (Timeout=%d ms)",
                    primary_topic_.c_str(), secondary_topic_.c_str(), output_topic_.c_str(), timeout_ms_);
    }

private:
    static bool is_valid_imu(const sensor_msgs::msg::Imu& msg) {
        const auto& q = msg.orientation;
        if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w) ||
            !std::isfinite(msg.angular_velocity.z)) {
            return false;
        }
        const double norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
        return (norm_sq > 1e-6);
    }

    enum class Source { NONE, PRIMARY, SECONDARY };

    std::string primary_topic_;
    std::string secondary_topic_;
    std::string output_topic_;
    int timeout_ms_{50};

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr out_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr primary_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr secondary_sub_;

    rclcpp::Time last_primary_time_{0, 0, RCL_ROS_TIME};
    Source active_source_{Source::NONE};
};

}  // namespace robot_controller

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<robot_controller::ImuMuxNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
