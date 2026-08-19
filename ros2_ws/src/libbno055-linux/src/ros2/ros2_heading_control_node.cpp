#include <pthread.h>
#include <rclcpp/version.h>
#include <sched.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#ifdef BNO055_ROS2_BUILDING_COMPONENT
#include <rclcpp_components/register_node_macro.hpp>
#endif
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <vector>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace bno055_ros2 {

namespace {
inline double normalize_angle(const double angle_rad) {
    return std::remainder(angle_rad, 2.0 * M_PI);
}

inline bool is_finite_twist(const geometry_msgs::msg::Twist& twist) {
    return std::isfinite(twist.linear.x) && std::isfinite(twist.linear.y) && std::isfinite(twist.linear.z) &&
           std::isfinite(twist.angular.x) && std::isfinite(twist.angular.y) && std::isfinite(twist.angular.z);
}

inline void trySetRealtimePriority(rclcpp::Logger logger, int priority = 80) noexcept {
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
}  // namespace

class BNO055HeadingControlNode : public rclcpp::Node {
public:
    explicit BNO055HeadingControlNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("bno055_heading_control_node", options),
          last_command_time_{0, 0, RCL_ROS_TIME},
          last_imu_time_{0, 0, RCL_ROS_TIME},
          last_control_time_{0, 0, RCL_ROS_TIME},
          current_yaw_rad_{0.0},
          current_angular_velocity_z_rad_s_{0.0},
          target_yaw_rad_{0.0},
          integral_error_rad_s_{0.0},
          target_yaw_initialized_{false},
          cmd_vel_timeout_logged_{false},
          last_correction_{0.0},
          last_error_deg_{0.0} {
        trySetRealtimePriority(this->get_logger(), 80);
        configure_parameters();

        // 1. Callback Groups Isolation (High-Frequency Control vs Low-Priority Admin)
        control_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        admin_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        auto control_sub_options = rclcpp::SubscriptionOptions();
        control_sub_options.callback_group = control_cb_group_;

        // 2. Subscriptions & Publishers
        command_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            raw_cmd_vel_topic_, rclcpp::QoS(command_qos_depth_),
            [this](const geometry_msgs::msg::Twist::SharedPtr message) { receive_command(*message); },
            control_sub_options);

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Imu::SharedPtr message) { receive_imu(*message); }, control_sub_options);

        auto admin_sub_options = rclcpp::SubscriptionOptions();
        admin_sub_options.callback_group = admin_cb_group_;

        enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/heading_control/enable", rclcpp::QoS(1).reliable().transient_local(),
            [this](const std_msgs::msg::Bool::SharedPtr message) {
                if (heading_hold_enabled_ != message->data) {
                    heading_hold_enabled_ = message->data;
                    reset_heading_hold();
                    if (heading_hold_enabled_) {
                        RCLCPP_INFO(this->get_logger(), "[HeadingControl] ENABLED via topic /heading_control/enable");
                    } else {
                        RCLCPP_WARN(this->get_logger(), "[HeadingControl] DISABLED (Pure Passthrough Manual Mode)");
                    }
                }
            },
            admin_sub_options);

        corrected_command_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(corrected_cmd_vel_topic_,
                                                                                   rclcpp::QoS(command_qos_depth_));

        diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", rclcpp::QoS(1));

        // 3. Trigger Service (Admin Callback Group)
        reset_heading_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "~/reset_heading",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
                target_yaw_rad_ = current_yaw_rad_;
                target_yaw_initialized_ = true;
                integral_error_rad_s_ = 0.0;
                res->success = true;
                res->message = "Heading hold target reset to current heading";
            },
            rmw_qos_profile_services_default, admin_cb_group_);

        // 4. Timers
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(control_period_ms_), [this]() { control(); }, control_cb_group_);

        if (this->get_parameter("enable_diagnostics").as_bool()) {
            diag_timer_ =
                this->create_wall_timer(std::chrono::seconds(1), [this]() { publish_diagnostics(); }, admin_cb_group_);
        }

        RCLCPP_INFO(this->get_logger(), "BNO055 Heading Control online: input=%s output=%s imu=%s (Kp=%.2f, Kd=%.3f)",
                    raw_cmd_vel_topic_.c_str(), corrected_cmd_vel_topic_.c_str(), imu_topic_.c_str(), kp_, kd_);
    }

private:
    void configure_parameters() {
        kp_ = this->declare_parameter("kp", 4.0);
        ki_ = this->declare_parameter("ki", 0.0);
        kd_ = this->declare_parameter("kd", 0.05);
        integral_limit_rad_s_ = this->declare_parameter("max_i_term", 0.5);

        double deadband_deg = this->declare_parameter("deadband_deg", 0.5);
        heading_deadband_rad_ = deadband_deg * M_PI / 180.0;

        rotation_input_deadband_rad_s_ = this->declare_parameter("angular_deadband", 0.08);
        turn_relock_delay_ms_ = static_cast<int>(this->declare_parameter("turn_relock_delay", 0.2) * 1000.0);
        max_correction_rad_s_ = this->declare_parameter("max_output", 1.5);
        control_period_ms_ = 10;
        command_timeout_ms_ = static_cast<int>(this->declare_parameter("cmd_vel_timeout", 0.5) * 1000.0);
        imu_timeout_ms_ = static_cast<int>(this->declare_parameter("imu_timeout", 0.25) * 1000.0);
        command_qos_depth_ = 10;
        raw_cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_in_topic", "/drive/cmd_vel");
        imu_topic_ = this->declare_parameter<std::string>("imu_topic", "/imu/data");
        corrected_cmd_vel_topic_ =
            this->declare_parameter<std::string>("cmd_vel_out_topic", "/mecanum/cmd_vel_heading");
        this->declare_parameter<bool>("enable_diagnostics", true);
        yaw_axis_ = this->declare_parameter<std::string>("yaw_axis", "z");
    }

    rcl_interfaces::msg::SetParametersResult update_parameters(const std::vector<rclcpp::Parameter>& parameters) {
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = false;
        result.reason = "Only PID and heading-hold limits can be changed while running";

        double next_kp = kp_;
        double next_ki = ki_;
        double next_kd = kd_;
        double next_integral_limit = integral_limit_rad_s_;
        double next_heading_deadband = heading_deadband_rad_;
        double next_rotation_deadband = rotation_input_deadband_rad_s_;
        int next_turn_relock_delay_ms = turn_relock_delay_ms_;
        double next_max_correction = max_correction_rad_s_;

        for (const auto& parameter : parameters) {
            const auto& name = parameter.get_name();
            if (name == "kp") {
                next_kp = parameter.as_double();
            } else if (name == "ki") {
                next_ki = parameter.as_double();
            } else if (name == "kd") {
                next_kd = parameter.as_double();
            } else if (name == "max_i_term") {
                next_integral_limit = parameter.as_double();
            } else if (name == "deadband_deg") {
                next_heading_deadband = parameter.as_double() * M_PI / 180.0;
            } else if (name == "angular_deadband") {
                next_rotation_deadband = parameter.as_double();
            } else if (name == "turn_relock_delay") {
                next_turn_relock_delay_ms = static_cast<int>(parameter.as_double() * 1000.0);
            } else if (name == "max_output") {
                next_max_correction = parameter.as_double();
            }
        }

        if (!std::isfinite(next_kp) || !std::isfinite(next_ki) || !std::isfinite(next_kd) ||
            !std::isfinite(next_integral_limit) || !std::isfinite(next_heading_deadband) ||
            !std::isfinite(next_rotation_deadband) || !std::isfinite(next_max_correction) || next_kp < 0.0 ||
            next_ki < 0.0 || next_kd < 0.0 || next_integral_limit < 0.0 || next_heading_deadband < 0.0 ||
            next_rotation_deadband < 0.0 || next_turn_relock_delay_ms < 0 || next_max_correction <= 0.0) {
            result.reason = "PID gains and limits must be finite and non-negative";
            return result;
        }

        kp_ = next_kp;
        ki_ = next_ki;
        kd_ = next_kd;
        integral_limit_rad_s_ = next_integral_limit;
        heading_deadband_rad_ = next_heading_deadband;
        rotation_input_deadband_rad_s_ = next_rotation_deadband;
        turn_relock_delay_ms_ = next_turn_relock_delay_ms;
        max_correction_rad_s_ = next_max_correction;
        result.successful = true;
        result.reason = "success";
        RCLCPP_INFO(this->get_logger(), "Heading-hold parameters dynamically updated");
        return result;
    }

    void receive_command(const geometry_msgs::msg::Twist& message) {
        if (!is_finite_twist(message)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Ignored a non-finite cmd_vel message");
            return;
        }
        latest_command_ = message;
        last_command_time_ = this->now();
        cmd_vel_timeout_logged_ = false;
    }

    void receive_imu(const sensor_msgs::msg::Imu& message) {
        const auto& orientation = message.orientation;
        if (!std::isfinite(orientation.x) || !std::isfinite(orientation.y) || !std::isfinite(orientation.z) ||
            !std::isfinite(orientation.w) || !std::isfinite(message.angular_velocity.z)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Ignored IMU data containing non-finite values");
            return;
        }

        tf2::Quaternion quaternion;
        tf2::fromMsg(orientation, quaternion);
        const double norm_squared = quaternion.length2();
        if (norm_squared < 1e-12) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Ignored an invalid IMU quaternion");
            return;
        }
        quaternion.normalize();

        double roll_rad = 0.0;
        double pitch_rad = 0.0;
        double yaw_rad = 0.0;
        tf2::Matrix3x3(quaternion).getRPY(roll_rad, pitch_rad, yaw_rad);

        // Proven Coordinate Alignment (matching heading_hold_node)
        const double new_yaw_rad = -yaw_rad;
        const auto now_time = this->now();

        // Discontinuity / Source-switch detection (e.g. step jump > 15 deg within 50ms)
        if (target_yaw_initialized_ && (now_time - last_imu_time_).nanoseconds() <= 50000000LL) {
            const double yaw_step = std::abs(normalize_angle(new_yaw_rad - current_yaw_rad_));
            if (yaw_step > (15.0 * M_PI / 180.0)) {
                RCLCPP_INFO(this->get_logger(),
                            "[HeadingControl] Detected IMU source transition / angle jump (%.1f°). Shockless "
                            "re-locking target.",
                            yaw_step * 180.0 / M_PI);
                reset_heading_hold();
            }
        }

        current_yaw_rad_ = new_yaw_rad;
        current_angular_velocity_z_rad_s_ = -message.angular_velocity.z;
        last_imu_time_ = now_time;
    }

    void reset_heading_hold() {
        target_yaw_initialized_ = false;
        integral_error_rad_s_ = 0.0;
        last_manual_turn_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }

    void control() {
        const auto current_time = this->now();
        const double dt_s = (current_time - last_control_time_).seconds();
        last_control_time_ = current_time;

        if (last_command_time_.nanoseconds() == 0 ||
            (current_time - last_command_time_).nanoseconds() > command_timeout_ms_ * 1000000LL) {
            corrected_command_pub_->publish(geometry_msgs::msg::Twist());
            reset_heading_hold();
            if (!cmd_vel_timeout_logged_) {
                RCLCPP_INFO(this->get_logger(), "cmd_vel idle / timed out; publishing zero velocity");
                cmd_vel_timeout_logged_ = true;
            }
            return;
        }

        if (!heading_hold_enabled_) {
            corrected_command_pub_->publish(latest_command_);
            reset_heading_hold();
            return;
        }

        const bool imu_is_fresh = last_imu_time_.nanoseconds() != 0 &&
                                  (current_time - last_imu_time_).nanoseconds() <= imu_timeout_ms_ * 1000000LL;
        if (!imu_is_fresh) {
            // Immediate failsafe: stop spinning, pass through raw joystick input
            corrected_command_pub_->publish(latest_command_);
            reset_heading_hold();
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "IMU data is unavailable; passing through upstream cmd_vel without correction");
            return;
        }

        if (std::abs(latest_command_.angular.z) > rotation_input_deadband_rad_s_) {
            target_yaw_rad_ = current_yaw_rad_;
            target_yaw_initialized_ = false;
            integral_error_rad_s_ = 0.0;
            last_manual_turn_time_ = current_time;
            corrected_command_pub_->publish(latest_command_);
            last_correction_ = latest_command_.angular.z;
            last_error_deg_ = 0.0;

            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                  "[HeadingControl] MANUAL_TURN | In(Wz=%+.2f rad/s) | New Target Yaw: %+.1f°",
                                  latest_command_.angular.z, current_yaw_rad_ * 180.0 / M_PI);
            return;
        }

        // Settling delay after manual turning: allow inertia to settle before re-locking heading
        if (last_manual_turn_time_.nanoseconds() != 0 &&
            (current_time - last_manual_turn_time_).nanoseconds() < turn_relock_delay_ms_ * 1000000LL) {
            target_yaw_rad_ = current_yaw_rad_;
            target_yaw_initialized_ = false;
            integral_error_rad_s_ = 0.0;
            corrected_command_pub_->publish(latest_command_);
            last_correction_ = latest_command_.angular.z;
            last_error_deg_ = 0.0;
            return;
        }

        if (!target_yaw_initialized_) {
            target_yaw_rad_ = current_yaw_rad_;
            target_yaw_initialized_ = true;
            integral_error_rad_s_ = 0.0;
            RCLCPP_DEBUG(this->get_logger(), "[HeadingControl] Re-locked heading target to %+.1f°",
                         target_yaw_rad_ * 180.0 / M_PI);
        }

        const double safe_dt_s = dt_s > 0.0 && dt_s < 0.5 ? dt_s : static_cast<double>(control_period_ms_) / 1000.0;
        double heading_error_rad = normalize_angle(target_yaw_rad_ - current_yaw_rad_);
        if (std::abs(heading_error_rad) < heading_deadband_rad_) {
            heading_error_rad = 0.0;
        }

        integral_error_rad_s_ = std::clamp(integral_error_rad_s_ + heading_error_rad * safe_dt_s,
                                           -integral_limit_rad_s_, integral_limit_rad_s_);

        // 100% exactly matching heading_hold_node.cpp line 274
        const double correction_rad_s =
            std::clamp(kp_ * heading_error_rad + ki_ * integral_error_rad_s_ - kd_ * current_angular_velocity_z_rad_s_,
                       -max_correction_rad_s_, max_correction_rad_s_);

        auto corrected_command = latest_command_;
        corrected_command.angular.z = correction_rad_s;
        corrected_command_pub_->publish(corrected_command);

        last_correction_ = correction_rad_s;
        last_error_deg_ = heading_error_rad * 180.0 / M_PI;

        const double curr_yaw_deg = current_yaw_rad_ * 180.0 / M_PI;
        const double target_yaw_deg = target_yaw_rad_ * 180.0 / M_PI;

        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                              "[HeadingControl] LOCKED | In(Vx=%.2f, Vy=%.2f) | Yaw: curr=%+.1f° -> target=%+.1f° "
                              "(err=%+.2f°) | GyroZ=%+.2f | Out: Wz=%+.3f rad/s",
                              latest_command_.linear.x, latest_command_.linear.y, curr_yaw_deg, target_yaw_deg,
                              last_error_deg_, current_angular_velocity_z_rad_s_, correction_rad_s);
    }

    void publish_diagnostics() {
        if (diag_pub_->get_subscription_count() == 0) {
            return;
        }

        auto diag_arr = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();
        diag_arr->header.stamp = this->now();

        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = "libbno055_linux: Heading Controller";
        status.hardware_id = "BNO055_PID_Controller";

        const bool imu_ok = (last_imu_time_.nanoseconds() != 0 &&
                             (this->now() - last_imu_time_).nanoseconds() <= imu_timeout_ms_ * 1000000LL);

        if (!imu_ok) {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            status.message = "IMU Offline/Timed out: Operating in Fail-Safe Passthrough Mode";
        } else if (target_yaw_initialized_) {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            status.message = "Active Straight Heading Correction";
        } else {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            status.message = "Passthrough Mode (Manual Turning)";
        }

        auto add_kv = [&status](const std::string& k, const std::string& v) {
            diagnostic_msgs::msg::KeyValue kv;
            kv.key = k;
            kv.value = v;
            status.values.push_back(kv);
        };

        add_kv("Target Heading (deg)", std::to_string(target_yaw_rad_ * 180.0 / M_PI));
        add_kv("Current Heading (deg)", std::to_string(current_yaw_rad_ * 180.0 / M_PI));
        add_kv("Heading Error (deg)", std::to_string(last_error_deg_));
        add_kv("PID Correction (rad/s)", std::to_string(last_correction_));
        add_kv("Target Locked", target_yaw_initialized_ ? "True" : "False");
        add_kv("IMU Healthy", imu_ok ? "True" : "False");

        diag_arr->status.push_back(status);
        diag_pub_->publish(std::move(diag_arr));
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr corrected_command_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_heading_srv_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr diag_timer_;
    rclcpp::CallbackGroup::SharedPtr control_cb_group_;
    rclcpp::CallbackGroup::SharedPtr admin_cb_group_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;

    geometry_msgs::msg::Twist latest_command_;
    rclcpp::Time last_command_time_;
    rclcpp::Time last_imu_time_;
    rclcpp::Time last_control_time_;
    rclcpp::Time last_manual_turn_time_{0, 0, RCL_ROS_TIME};
    double current_yaw_rad_;
    double current_angular_velocity_z_rad_s_;
    double target_yaw_rad_;
    double integral_error_rad_s_;
    bool target_yaw_initialized_;
    bool cmd_vel_timeout_logged_;
    bool heading_hold_enabled_{true};
    double last_correction_;
    double last_error_deg_;

    double kp_{4.0};
    double ki_{0.0};
    double kd_{0.05};
    double integral_limit_rad_s_{0.5};
    double heading_deadband_rad_{0.02};
    double rotation_input_deadband_rad_s_{0.02};
    int turn_relock_delay_ms_{200};
    double max_correction_rad_s_{1.5};
    int control_period_ms_{10};
    int command_timeout_ms_{500};
    int imu_timeout_ms_{250};
    int command_qos_depth_{10};
    std::string raw_cmd_vel_topic_;
    std::string imu_topic_;
    std::string corrected_cmd_vel_topic_;
    std::string yaw_axis_;
};

}  // namespace bno055_ros2

#ifdef BNO055_ROS2_BUILDING_COMPONENT
RCLCPP_COMPONENTS_REGISTER_NODE(bno055_ros2::BNO055HeadingControlNode)
#else
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bno055_ros2::BNO055HeadingControlNode>();

    bno055_ros2::trySetRealtimePriority(node->get_logger(), 80);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
#endif
