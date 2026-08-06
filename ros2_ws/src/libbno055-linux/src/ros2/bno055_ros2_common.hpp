#ifndef BNO055_ROS2_COMMON_HPP
#define BNO055_ROS2_COMMON_HPP

#include <algorithm>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <vector>

#include "libbno055-linux/bno055.hpp"

namespace bno055_ros2 {

// Declare standard parameters
template <typename T>
inline void declare_common_parameters(T* node) {
    node->template declare_parameter<std::string>("device", "/dev/i2c-1");
    node->template declare_parameter<int>("address", 0x28);
    node->template declare_parameter<std::string>("connection_type", "i2c");
    node->template declare_parameter<std::string>("uart_port", "/dev/ttyUSB0");
    node->template declare_parameter<int>("uart_baudrate", 115200);
    node->template declare_parameter<double>("uart_timeout", 0.1);
    node->template declare_parameter<bool>("uart_low_latency", false);
    node->template declare_parameter<std::string>("frame_id", "imu_link");
    node->template declare_parameter<double>("publish_rate", 50.0);
    node->template declare_parameter<std::string>("qos_reliability", "best_effort");
    node->template declare_parameter<int>("qos_history_depth", 10);
    node->template declare_parameter<std::string>("calibration_file", "");
    node->template declare_parameter<bool>("enable_auto_calibration", false);
    node->template declare_parameter<std::vector<double>>("orientation_covariance", std::vector<double>(9, 0.0));
    node->template declare_parameter<std::vector<double>>("angular_velocity_covariance", std::vector<double>(9, 0.0));
    node->template declare_parameter<std::vector<double>>("linear_acceleration_covariance",
                                                          std::vector<double>(9, 0.0));
    node->template declare_parameter<std::string>("operation_mode", "imu_plus");
    node->template declare_parameter<std::string>("axis_map_config", "p1");
    node->template declare_parameter<std::string>("axis_map_sign", "p1");
    node->template declare_parameter<bool>("use_external_crystal", true);
    node->template declare_parameter<std::vector<double>>("magnetic_field_covariance", std::vector<double>(9, 0.0));
    node->template declare_parameter<double>("temperature_variance", 0.0);
    // Advanced driver modes
    node->template declare_parameter<std::string>("read_mode", "standard");  // "standard", "raw_async", "interrupt"
    node->template declare_parameter<int>("interrupt_gpio_pin", 24);
    node->template declare_parameter<int>("thread_priority", 0);
}

// Redirect logger callback
template <typename T>
inline void setup_logger_redirection(T* node, bno055lib::BNO055& imu) {
    imu.setLogger([node](bno055lib::LogLevel level, std::string_view message) {
        switch (level) {
            case bno055lib::LogLevel::Debug:
                RCLCPP_DEBUG(node->get_logger(), "%s", message.data());
                break;
            case bno055lib::LogLevel::Info:
                RCLCPP_INFO(node->get_logger(), "%s", message.data());
                break;
            case bno055lib::LogLevel::Warning:
                RCLCPP_WARN(node->get_logger(), "%s", message.data());
                break;
            case bno055lib::LogLevel::Error:
                RCLCPP_ERROR(node->get_logger(), "%s", message.data());
                break;
        }
    });
}

// Set covariances from parameters
template <typename T>
inline void fill_imu_covariances(T* node, sensor_msgs::msg::Imu& message) {
    auto ori_cov = node->template get_parameter("orientation_covariance").as_double_array();
    auto gyro_cov = node->template get_parameter("angular_velocity_covariance").as_double_array();
    auto accel_cov = node->template get_parameter("linear_acceleration_covariance").as_double_array();

    if (ori_cov.size() == 9) std::copy(ori_cov.begin(), ori_cov.end(), message.orientation_covariance.begin());
    if (gyro_cov.size() == 9) std::copy(gyro_cov.begin(), gyro_cov.end(), message.angular_velocity_covariance.begin());
    if (accel_cov.size() == 9)
        std::copy(accel_cov.begin(), accel_cov.end(), message.linear_acceleration_covariance.begin());
}

// Build standard diagnostic array
template <typename T>
inline diagnostic_msgs::msg::DiagnosticArray::UniquePtr build_diagnostics(T* node, bno055lib::BNO055& imu,
                                                                          const std::string& node_name) {
    auto diag_arr = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();
    diag_arr->header.stamp = node->now();

    auto status = diagnostic_msgs::msg::DiagnosticStatus();
    status.name = std::string("libbno055_linux: ") + node_name;
    status.hardware_id = node->template get_parameter("device").as_string() + ":" +
                         std::to_string(node->template get_parameter("address").as_int());

    auto diag = imu.getDiagnostics();

    auto add_key_value = [](diagnostic_msgs::msg::DiagnosticStatus& stat, const std::string& key,
                            const std::string& val) {
        auto kv = diagnostic_msgs::msg::KeyValue();
        kv.key = key;
        kv.value = val;
        stat.values.push_back(kv);
    };

    add_key_value(status, "I2C Read Failures", std::to_string(diag.read_failures));
    add_key_value(status, "I2C Write Failures", std::to_string(diag.write_failures));
    add_key_value(status, "I2C Reconnect Attempts", std::to_string(diag.reconnect_attempts));

    bno055lib::CalibrationStatus calib;
    bool calib_ok = false;
    try {
        calib = imu.getCalibrationStatus();
        calib_ok = true;
    } catch (const std::exception& e) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = std::string("Failed to query calibration status: ") + e.what();
    }

    if (calib_ok) {
        add_key_value(status, "Calibration Status: Sys", std::to_string(calib.sys));
        add_key_value(status, "Calibration Status: Gyro", std::to_string(calib.gyro));
        add_key_value(status, "Calibration Status: Accel", std::to_string(calib.accel));
        add_key_value(status, "Calibration Status: Mag", std::to_string(calib.mag));

        if (calib.isFullyCalibrated()) {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            status.message = "Fully Calibrated & Streaming";
        } else {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            status.message = "Calibration incomplete (Sys:" + std::to_string(calib.sys) +
                             " G:" + std::to_string(calib.gyro) + " A:" + std::to_string(calib.accel) +
                             " M:" + std::to_string(calib.mag) + ")";
        }
    }

    if (diag.reconnect_attempts > 5) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = "Unstable I2C connection. Reconnected " + std::to_string(diag.reconnect_attempts) + " times.";
    }

    diag_arr->status.push_back(status);
    return diag_arr;
}

inline bno055lib::OpMode parse_op_mode(const std::string& mode_str) {
    if (mode_str == "config") return bno055lib::OpMode::Config;
    if (mode_str == "acc_only") return bno055lib::OpMode::AccOnly;
    if (mode_str == "mag_only") return bno055lib::OpMode::MagOnly;
    if (mode_str == "gyro_only") return bno055lib::OpMode::GyroOnly;
    if (mode_str == "acc_mag") return bno055lib::OpMode::AccMag;
    if (mode_str == "acc_gyro") return bno055lib::OpMode::AccGyro;
    if (mode_str == "mag_gyro") return bno055lib::OpMode::MagGyro;
    if (mode_str == "amg") return bno055lib::OpMode::AMG;
    if (mode_str == "imu_plus") return bno055lib::OpMode::IMUPlus;
    if (mode_str == "compass") return bno055lib::OpMode::Compass;
    if (mode_str == "m4g") return bno055lib::OpMode::M4G;
    if (mode_str == "ndof_fmc_off") return bno055lib::OpMode::NDOF_FMC_Off;
    if (mode_str == "ndof") return bno055lib::OpMode::NDOF;
    return bno055lib::OpMode::IMUPlus;
}

inline bno055lib::AxisMapConfig parse_axis_map_config(const std::string& str) {
    if (str == "p0") return bno055lib::AxisMapConfig::P0;
    if (str == "p1") return bno055lib::AxisMapConfig::P1;
    if (str == "p2") return bno055lib::AxisMapConfig::P2;
    if (str == "p3") return bno055lib::AxisMapConfig::P3;
    if (str == "p4") return bno055lib::AxisMapConfig::P4;
    if (str == "p5") return bno055lib::AxisMapConfig::P5;
    if (str == "p6") return bno055lib::AxisMapConfig::P6;
    if (str == "p7") return bno055lib::AxisMapConfig::P7;
    return bno055lib::AxisMapConfig::P1;
}

inline bno055lib::AxisMapSign parse_axis_map_sign(const std::string& str) {
    if (str == "p0") return bno055lib::AxisMapSign::P0;
    if (str == "p1") return bno055lib::AxisMapSign::P1;
    if (str == "p2") return bno055lib::AxisMapSign::P2;
    if (str == "p3") return bno055lib::AxisMapSign::P3;
    if (str == "p4") return bno055lib::AxisMapSign::P4;
    if (str == "p5") return bno055lib::AxisMapSign::P5;
    if (str == "p6") return bno055lib::AxisMapSign::P6;
    if (str == "p7") return bno055lib::AxisMapSign::P7;
    return bno055lib::AxisMapSign::P1;
}

template <typename T>
inline void apply_advanced_features(T* node, bno055lib::BNO055& imu) {
    imu.setAxisRemap(parse_axis_map_config(node->template get_parameter("axis_map_config").as_string()));
    imu.setAxisSign(parse_axis_map_sign(node->template get_parameter("axis_map_sign").as_string()));
    imu.setExtCrystalUse(node->template get_parameter("use_external_crystal").as_bool());

    int thread_prio = node->template get_parameter("thread_priority").as_int();
    if (thread_prio > 0) {
        sched_param sch_params;
        sch_params.sched_priority = thread_prio;
        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch_params) != 0) {
            RCLCPP_WARN(node->get_logger(),
                        "Failed to set SCHED_FIFO thread priority to %d. (Requires root or limits.conf rtprio)",
                        thread_prio);
        } else {
            RCLCPP_INFO(node->get_logger(), "Elevated thread priority to SCHED_FIFO %d.", thread_prio);
        }
    }
}

template <typename T>
inline void fill_mag_covariance(T* node, sensor_msgs::msg::MagneticField& message) {
    auto mag_cov = node->template get_parameter("magnetic_field_covariance").as_double_array();
    if (mag_cov.size() == 9) std::copy(mag_cov.begin(), mag_cov.end(), message.magnetic_field_covariance.begin());
}

}  // namespace bno055_ros2

#endif  // BNO055_ROS2_COMMON_HPP
