#include <pthread.h>
#include <sched.h>

#include <chrono>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <memory>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#ifdef BNO055_ROS2_BUILDING_COMPONENT
#include <rclcpp_components/register_node_macro.hpp>
#endif
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <vector>

#include "bno055_ros2_common.hpp"
#include "libbno055-linux/bno055.hpp"

namespace bno055_ros2 {

/**
 * @brief Elegantly attempts to set Linux SCHED_FIFO real-time thread priority without hard crashing.
 */
inline void trySetRealtimePriority(rclcpp::Logger logger, int priority = 85) noexcept {
#if defined(__linux__)
    struct sched_param param;
    param.sched_priority = priority;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) == 0) {
        RCLCPP_INFO(logger, "Successfully elevated thread priority to SCHED_FIFO (Priority: %d)", priority);
    } else {
        RCLCPP_DEBUG(logger, "Running with default OS scheduling (SCHED_FIFO requires CAP_SYS_NICE privileges)");
    }
#endif
}

/**
 * @brief ROS 2 Driver Publisher Node for BNO055 with Isolated Callback Groups & Real-time Scheduling.
 */
class BNO055PublisherNode : public rclcpp::Node {
public:
    explicit BNO055PublisherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("bno055_publisher_node", options), initialized_(false) {
        this->declare_parameter<std::string>("device", "/dev/i2c-1");
        this->declare_parameter<int>("address", 0x28);
        this->declare_parameter<int>("publish_rate_hz", 100);
        this->declare_parameter<std::string>("frame_id", "imu_link");
        this->declare_parameter<std::string>("operation_mode", "imu_plus");
        this->declare_parameter<std::string>("calibration_file", "");
        this->declare_parameter<bool>("enable_auto_calibration", false);
        this->declare_parameter<bool>("use_external_crystal", true);
        this->declare_parameter<std::string>("axis_map_config", "p1");
        this->declare_parameter<std::string>("axis_map_sign", "p1");
        this->declare_parameter<int>("thread_priority", 0);
        this->declare_parameter<bool>("publish_tf", false);
        this->declare_parameter<std::string>("parent_frame_id", "odom");
        this->declare_parameter<std::string>("child_frame_id", "base_link");
        this->declare_parameter<double>("imu_offset_x", 0.0);
        this->declare_parameter<double>("imu_offset_y", 0.0);
        this->declare_parameter<double>("imu_offset_z", 0.0);

        // 2. Callback Groups Isolation
        sensor_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        admin_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // 3. Initialize BNO055 Hardware
        const std::string device = this->get_parameter("device").as_string();
        const uint8_t address = static_cast<uint8_t>(this->get_parameter("address").as_int());
        const std::string op_mode_str = this->get_parameter("operation_mode").as_string();
        const bno055lib::OpMode op_mode = bno055_ros2::parse_op_mode(op_mode_str);

        imu_driver_ = std::make_unique<bno055lib::BNO055>(address, device);
        if (imu_driver_->begin(op_mode)) {
            initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "BNO055 hardware initialized on %s (0x%02X) in mode %s", device.c_str(),
                        address, op_mode_str.c_str());

            // Redirect internal library logs to rclcpp logger
            bno055_ros2::setup_logger_redirection(this, *imu_driver_);

            // Apply advanced features (external crystal, axis remap, priority, etc.)
            bno055_ros2::apply_advanced_features(this, *imu_driver_);

            // Load pre-existing calibration offsets if provided
            const std::string calib_file = this->get_parameter("calibration_file").as_string();
            if (!calib_file.empty()) {
                if (imu_driver_->loadCalibrationFile(calib_file)) {
                    RCLCPP_INFO(this->get_logger(), "Successfully loaded calibration offsets from %s",
                                calib_file.c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(), "Failed to load calibration file %s", calib_file.c_str());
                }
            }

            // Enable auto-calibration file saving if enabled
            const bool enable_auto_calib = this->get_parameter("enable_auto_calibration").as_bool();
            if (enable_auto_calib && !calib_file.empty()) {
                imu_driver_->enableAutoCalibration(calib_file);
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize BNO055 hardware on %s", device.c_str());
        }

        // 4. Publishers
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::SensorDataQoS());
        euler_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("imu/euler", rclcpp::SensorDataQoS());
        mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", rclcpp::SensorDataQoS());
        temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("imu/temp", rclcpp::SensorDataQoS());
        linear_accel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("imu/linear_acceleration",
                                                                                       rclcpp::SensorDataQoS());
        gravity_pub_ =
            this->create_publisher<geometry_msgs::msg::Vector3Stamped>("imu/gravity", rclcpp::SensorDataQoS());
        diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", rclcpp::QoS(1));

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 5. Sensor Polling Timer (High-Frequency Sensor Callback Group)
        const int rate_hz = this->get_parameter("publish_rate_hz").as_int();
        const auto period = std::chrono::milliseconds(1000 / std::max(1, rate_hz));

        sensor_timer_ =
            this->create_wall_timer(period, std::bind(&BNO055PublisherNode::publishSensorData, this), sensor_cb_group_);

        // 6. Diagnostics Timer (1Hz - Admin Callback Group)
        diag_timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&BNO055PublisherNode::publishDiagnostics, this), admin_cb_group_);

        RCLCPP_INFO(this->get_logger(), "BNO055 Publisher Node online (Isolated CallbackGroups & Real-time Ready).");
    }

private:
    void publishSensorData() {
        if (!initialized_) return;

        // Fetch raw sequential burst read to inspect for hardware data update
        auto raw_opt = imu_driver_->getRawSensorDataNoexcept();
        if (raw_opt) {
            // Deduplication check: if raw sensor bytes haven't changed from last read, skip publishing
            if (has_last_raw_ && std::memcmp(&last_raw_, &(*raw_opt), sizeof(bno055lib::BNO055::RawSensorData)) == 0) {
                return;
            }
            last_raw_ = *raw_opt;
            has_last_raw_ = true;
        }

        const std::string frame_id = this->get_parameter("frame_id").as_string();
        const auto now = this->now();

        auto quat = imu_driver_->getQuaternionNoexcept();
        auto euler = imu_driver_->getEulerAnglesNoexcept();
        auto gyro = imu_driver_->getGyroscopeNoexcept();
        auto accel = imu_driver_->getAccelerometerNoexcept();
        auto mag = imu_driver_->getMagnetometerNoexcept();
        auto linear_accel = imu_driver_->getLinearAccelerationNoexcept();
        auto gravity = imu_driver_->getGravityNoexcept();
        auto temp = imu_driver_->getTemperatureNoexcept();

        if (quat && gyro && accel) {
            // Outlier check for NaN/Inf
            if (BNO055_UNLIKELY(std::isnan(quat->w) || std::isnan(quat->x) || std::isnan(quat->y) ||
                                std::isnan(quat->z))) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "Corrupted IMU data from I2C/UART dropped.");
                return;
            }

            auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
            imu_msg->header.stamp = now;
            imu_msg->header.frame_id = frame_id;

            imu_msg->orientation.w = quat->w;
            imu_msg->orientation.x = quat->x;
            imu_msg->orientation.y = quat->y;
            imu_msg->orientation.z = quat->z;

            imu_msg->angular_velocity.x = gyro->x * (M_PI / 180.0);
            imu_msg->angular_velocity.y = gyro->y * (M_PI / 180.0);
            imu_msg->angular_velocity.z = gyro->z * (M_PI / 180.0);

            imu_msg->linear_acceleration.x = accel->x;
            imu_msg->linear_acceleration.y = accel->y;
            imu_msg->linear_acceleration.z = accel->z;

            imu_pub_->publish(std::move(imu_msg));

            if (this->get_parameter("publish_tf").as_bool()) {
                geometry_msgs::msg::TransformStamped tf_msg;
                tf_msg.header.stamp = now;
                tf_msg.header.frame_id = this->get_parameter("parent_frame_id").as_string();
                tf_msg.child_frame_id = this->get_parameter("child_frame_id").as_string();
                tf_msg.transform.translation.x = this->get_parameter("imu_offset_x").as_double();
                tf_msg.transform.translation.y = this->get_parameter("imu_offset_y").as_double();
                tf_msg.transform.translation.z = this->get_parameter("imu_offset_z").as_double();
                tf_msg.transform.rotation.w = quat->w;
                tf_msg.transform.rotation.x = quat->x;
                tf_msg.transform.rotation.y = quat->y;
                tf_msg.transform.rotation.z = quat->z;
                tf_broadcaster_->sendTransform(tf_msg);
            }
        }

        if (euler) {
            auto euler_msg = std::make_unique<geometry_msgs::msg::Vector3Stamped>();
            euler_msg->header.stamp = now;
            euler_msg->header.frame_id = frame_id;
            euler_msg->vector.x = euler->x;  // Roll (rad)
            euler_msg->vector.y = euler->y;  // Pitch (rad)
            euler_msg->vector.z = euler->z;  // Yaw / Heading (rad)

            euler_pub_->publish(std::move(euler_msg));
        }

        if (mag) {
            auto mag_msg = std::make_unique<sensor_msgs::msg::MagneticField>();
            mag_msg->header.stamp = now;
            mag_msg->header.frame_id = frame_id;
            mag_msg->magnetic_field.x = mag->x * 1e-6;
            mag_msg->magnetic_field.y = mag->y * 1e-6;
            mag_msg->magnetic_field.z = mag->z * 1e-6;

            mag_pub_->publish(std::move(mag_msg));
        }

        if (linear_accel) {
            auto linear_accel_msg = std::make_unique<geometry_msgs::msg::Vector3Stamped>();
            linear_accel_msg->header.stamp = now;
            linear_accel_msg->header.frame_id = frame_id;
            linear_accel_msg->vector.x = linear_accel->x;
            linear_accel_msg->vector.y = linear_accel->y;
            linear_accel_msg->vector.z = linear_accel->z;
            linear_accel_pub_->publish(std::move(linear_accel_msg));
        }

        if (gravity) {
            auto gravity_msg = std::make_unique<geometry_msgs::msg::Vector3Stamped>();
            gravity_msg->header.stamp = now;
            gravity_msg->header.frame_id = frame_id;
            gravity_msg->vector.x = gravity->x;
            gravity_msg->vector.y = gravity->y;
            gravity_msg->vector.z = gravity->z;
            gravity_pub_->publish(std::move(gravity_msg));
        }

        if (temp) {
            auto temp_msg = std::make_unique<sensor_msgs::msg::Temperature>();
            temp_msg->header.stamp = now;
            temp_msg->header.frame_id = frame_id;
            temp_msg->temperature = static_cast<double>(*temp);
            temp_msg->variance = 0.1;
            temp_pub_->publish(std::move(temp_msg));
        }
    }

    void publishDiagnostics() {
        auto diag_arr = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();
        diag_arr->header.stamp = this->now();

        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = "libbno055_linux: IMU Driver";
        status.hardware_id = "BNO055_Hardware";

        if (initialized_) {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            status.message = "BNO055 IMU Driver Operational";
        } else {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            status.message = "BNO055 Hardware Failed to Initialize";
        }

        diag_arr->status.push_back(status);
        diag_pub_->publish(std::move(diag_arr));
    }

    std::unique_ptr<bno055lib::BNO055> imu_driver_;
    bool initialized_;

    bno055lib::BNO055::RawSensorData last_raw_{};
    bool has_last_raw_{false};

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr linear_accel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr gravity_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::TimerBase::SharedPtr sensor_timer_;
    rclcpp::TimerBase::SharedPtr diag_timer_;

    rclcpp::CallbackGroup::SharedPtr sensor_cb_group_;
    rclcpp::CallbackGroup::SharedPtr admin_cb_group_;
};

}  // namespace bno055_ros2

#ifdef BNO055_ROS2_BUILDING_COMPONENT
RCLCPP_COMPONENTS_REGISTER_NODE(bno055_ros2::BNO055PublisherNode)
#else
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bno055_ros2::BNO055PublisherNode>();

    bno055_ros2::trySetRealtimePriority(node->get_logger(), 85);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
#endif
