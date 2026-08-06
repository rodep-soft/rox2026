#include <rclcpp/version.h>

#include <algorithm>
#include <chrono>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#ifdef BNO055_ROS2_BUILDING_COMPONENT
#include <rclcpp_components/register_node_macro.hpp>
#endif
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <vector>

#include "libbno055-linux/controllers/heading_controller.hpp"

namespace bno055_ros2 {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @brief Managed Lifecycle Heading Corrector Node with Multi-threaded Callback Isolation.
 * Integrates with ROS 2 Lifecycle State Machines (unconfigured -> inactive -> active -> finalized).
 * Ideal for Nav2 Lifecycle Manager integration.
 */
class BNO055LifecycleHeadingControlNode : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit BNO055LifecycleHeadingControlNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : LifecycleNode("bno055_lifecycle_heading_control_node", options),
          target_heading_locked_(false),
          current_heading_deg_(0.0),
          gyro_z_deg_(0.0),
          has_imu_data_(false),
          has_cmd_vel_in_(false),
          is_watchdog_triggered_(false),
          is_imu_timeout_(false),
          last_correction_(0.0),
          last_error_deg_(0.0),
          yaw_axis_("z"),
          max_translation_speed_(1.0) {
        // Declare Parameters
        this->declare_parameter<double>("kp", 0.05);
        this->declare_parameter<double>("ki", 0.001);
        this->declare_parameter<double>("kd", 0.01);
        this->declare_parameter<double>("kff", 0.0);
        this->declare_parameter<double>("max_i_term", 0.2);
        this->declare_parameter<double>("max_output", 1.0);
        this->declare_parameter<double>("deadband_deg", 0.02);
        this->declare_parameter<double>("cutoff_freq_hz", 20.0);
        this->declare_parameter<double>("angular_deadband", 0.01);
        this->declare_parameter<double>("cmd_vel_timeout", 0.5);
        this->declare_parameter<double>("imu_timeout", 1.0);
        this->declare_parameter<std::string>("imu_topic", "imu/data");
        this->declare_parameter<std::string>("cmd_vel_in_topic", "cmd_vel_in");
        this->declare_parameter<std::string>("cmd_vel_out_topic", "cmd_vel");
        this->declare_parameter<bool>("enable_diagnostics", true);
        this->declare_parameter<std::string>("yaw_axis", "z");
        this->declare_parameter<double>("max_translation_speed", 1.0);

        RCLCPP_INFO(this->get_logger(), "[Lifecycle Node] BNO055 Lifecycle Heading Control Node created.");
    }

    // --- Lifecycle State Transitions ---

    CallbackReturn on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override {
        RCLCPP_INFO(this->get_logger(), "Configuring BNO055 Lifecycle Heading Control Node...");
        updateControllerConfigFromParams();

        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&BNO055LifecycleHeadingControlNode::onParameterChange, this, std::placeholders::_1));

        control_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        admin_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        auto imu_sub_options = rclcpp::SubscriptionOptions();
        imu_sub_options.callback_group = control_cb_group_;

        auto cmd_vel_sub_options = rclcpp::SubscriptionOptions();
        cmd_vel_sub_options.callback_group = control_cb_group_;

        const std::string imu_topic = this->get_parameter("imu_topic").as_string();
        const std::string cmd_vel_in_topic = this->get_parameter("cmd_vel_in_topic").as_string();
        const std::string cmd_vel_out_topic = this->get_parameter("cmd_vel_out_topic").as_string();

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_out_topic, rclcpp::QoS(10));
        diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", rclcpp::QoS(1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, rclcpp::SensorDataQoS(),
            std::bind(&BNO055LifecycleHeadingControlNode::imuCallback, this, std::placeholders::_1), imu_sub_options);

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_in_topic, 10,
            std::bind(&BNO055LifecycleHeadingControlNode::cmdVelInCallback, this, std::placeholders::_1),
            cmd_vel_sub_options);

        reset_heading_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "~/reset_heading",
            std::bind(&BNO055LifecycleHeadingControlNode::handleResetHeadingService, this, std::placeholders::_1,
                      std::placeholders::_2),
#if RCLCPP_VERSION_MAJOR >= 28
            rclcpp::ServicesQoS(),
#else
            rmw_qos_profile_services_default,
#endif
            admin_cb_group_);

        RCLCPP_INFO(this->get_logger(), "Node configured successfully with isolated CallbackGroups.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        RCLCPP_INFO(this->get_logger(), "Activating BNO055 Lifecycle Heading Control Node...");

        cmd_vel_pub_->on_activate();
        diag_pub_->on_activate();

        const auto now = this->now();
        last_time_ = now;
        last_cmd_vel_in_time_ = now;
        last_imu_time_ = now;

        watchdog_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&BNO055LifecycleHeadingControlNode::checkSystemHealth, this),
            control_cb_group_);

        if (this->get_parameter("enable_diagnostics").as_bool()) {
            diag_timer_ = this->create_wall_timer(
                std::chrono::seconds(1), std::bind(&BNO055LifecycleHeadingControlNode::publishDiagnostics, this),
                admin_cb_group_);
        }

        RCLCPP_INFO(this->get_logger(), "Node activated.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override {
        RCLCPP_INFO(this->get_logger(), "Deactivating BNO055 Lifecycle Heading Control Node...");

        watchdog_timer_.reset();
        diag_timer_.reset();

        if (cmd_vel_pub_->is_activated()) {
            auto stop_twist = std::make_unique<geometry_msgs::msg::Twist>();
            cmd_vel_pub_->publish(std::move(stop_twist));
            cmd_vel_pub_->on_deactivate();
        }
        diag_pub_->on_deactivate();

        controller_.reset();
        target_heading_locked_ = false;

        RCLCPP_INFO(this->get_logger(), "Node deactivated.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/) override {
        RCLCPP_INFO(this->get_logger(), "Cleaning up BNO055 Lifecycle Heading Control Node...");

        cmd_vel_pub_.reset();
        diag_pub_.reset();
        imu_sub_.reset();
        cmd_vel_sub_.reset();
        reset_heading_srv_.reset();
        param_callback_handle_.reset();
        control_cb_group_.reset();
        admin_cb_group_.reset();

        has_imu_data_ = false;
        has_cmd_vel_in_ = false;

        RCLCPP_INFO(this->get_logger(), "Node cleanup completed.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/) override {
        RCLCPP_INFO(this->get_logger(), "Shutting down BNO055 Lifecycle Heading Control Node...");
        return CallbackReturn::SUCCESS;
    }

private:
    void updateControllerConfigFromParams() noexcept {
        bno055lib::HeadingController::Config cfg;
        cfg.kp = this->get_parameter("kp").as_double();
        cfg.ki = this->get_parameter("ki").as_double();
        cfg.kd = this->get_parameter("kd").as_double();
        cfg.kff = this->get_parameter("kff").as_double();
        cfg.max_i_term = this->get_parameter("max_i_term").as_double();
        cfg.max_output = this->get_parameter("max_output").as_double();
        cfg.min_output = -cfg.max_output;
        cfg.deadband_deg = this->get_parameter("deadband_deg").as_double();
        cfg.cutoff_freq_hz = this->get_parameter("cutoff_freq_hz").as_double();
        cfg.max_slew_rate = this->get_parameter("max_slew_rate").as_double();
        yaw_axis_ = this->get_parameter("yaw_axis").as_string();
        max_translation_speed_ = this->get_parameter("max_translation_speed").as_double();
        controller_.setConfig(cfg);
    }

    rcl_interfaces::msg::SetParametersResult onParameterChange(const std::vector<rclcpp::Parameter>& parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        bno055lib::HeadingController::Config cfg = controller_.getConfig();

        for (const auto& param : parameters) {
            const std::string& name = param.get_name();
            if (name == "kp") {
                cfg.kp = param.as_double();
            } else if (name == "ki") {
                cfg.ki = param.as_double();
            } else if (name == "kd") {
                cfg.kd = param.as_double();
            } else if (name == "kff") {
                cfg.kff = param.as_double();
            } else if (name == "max_i_term") {
                cfg.max_i_term = param.as_double();
            } else if (name == "max_output") {
                cfg.max_output = param.as_double();
                cfg.min_output = -cfg.max_output;
            } else if (name == "deadband_deg") {
                cfg.deadband_deg = param.as_double();
            } else if (name == "cutoff_freq_hz") {
                cfg.cutoff_freq_hz = param.as_double();
            } else if (name == "max_slew_rate") {
                cfg.max_slew_rate = param.as_double();
            } else if (name == "yaw_axis") {
                yaw_axis_ = param.as_string();
            } else if (name == "max_translation_speed") {
                max_translation_speed_ = param.as_double();
            }

            if (name == "kp" || name == "ki" || name == "kd" || name == "kff" || name == "max_i_term" ||
                name == "max_output" || name == "deadband_deg" || name == "cutoff_freq_hz" || name == "max_slew_rate" ||
                name == "cmd_vel_timeout" || name == "imu_timeout" || name == "yaw_axis" ||
                name == "max_translation_speed") {
                RCLCPP_INFO(this->get_logger(), "Dynamic parameter updated: %s = %f", name.c_str(), param.as_double());
            }
        }
        controller_.setConfig(cfg);
        return result;
    }

    void handleResetHeadingService(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        if (has_imu_data_ && !is_imu_timeout_) {
            target_quat_ = current_quat_;
            target_heading_deg_ = current_heading_deg_;
            target_heading_locked_ = true;
            controller_.reset();
            res->success = true;
            res->message = "Heading target reset to: " + std::to_string(target_heading_deg_) + " deg";
        } else {
            res->success = false;
            res->message = "Cannot reset heading: IMU data unavailable.";
        }
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        const rclcpp::Time now = this->now();
        last_imu_time_ = now;
        has_imu_data_ = true;
        is_imu_timeout_ = false;

        current_quat_ = bno055lib::Quat{msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z};

        double yaw_rad = 0.0;
        double gyro_rate_rad = 0.0;

        if (yaw_axis_ == "x") {
            // Roll as Yaw
            const double sinr_cosp = 2.0 * (current_quat_.w * current_quat_.x + current_quat_.y * current_quat_.z);
            const double cosr_cosp =
                1.0 - 2.0 * (current_quat_.x * current_quat_.x + current_quat_.y * current_quat_.y);
            yaw_rad = std::atan2(sinr_cosp, cosr_cosp);
            gyro_rate_rad = msg->angular_velocity.x;
        } else if (yaw_axis_ == "y") {
            // Pitch as Yaw
            const double sinp = 2.0 * (current_quat_.w * current_quat_.y - current_quat_.z * current_quat_.x);
            if (std::abs(sinp) >= 1.0)
                yaw_rad = std::copysign(M_PI / 2.0, sinp);
            else
                yaw_rad = std::asin(sinp);
            gyro_rate_rad = msg->angular_velocity.y;
        } else {
            // Z as Yaw (default)
            const double siny_cosp = 2.0 * (current_quat_.w * current_quat_.z + current_quat_.x * current_quat_.y);
            const double cosy_cosp =
                1.0 - 2.0 * (current_quat_.y * current_quat_.y + current_quat_.z * current_quat_.z);
            yaw_rad = std::atan2(siny_cosp, cosy_cosp);
            gyro_rate_rad = msg->angular_velocity.z;
        }

        current_heading_deg_ = yaw_rad * bno055lib::RAD_TO_DEG;
        gyro_z_deg_ = gyro_rate_rad * bno055lib::RAD_TO_DEG;
    }

    void cmdVelInCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (!cmd_vel_pub_->is_activated()) return;

        const rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;
        last_cmd_vel_in_time_ = now;
        has_cmd_vel_in_ = true;

        if (is_watchdog_triggered_) {
            RCLCPP_INFO(this->get_logger(), "Watchdog disengaged: Input command resumed.");
            is_watchdog_triggered_ = false;
        }

        if (BNO055_UNLIKELY(dt <= 0.0 || dt > 1.0)) dt = 0.02;

        auto out_twist = std::make_unique<geometry_msgs::msg::Twist>();
        out_twist->linear = msg->linear;

        const double deadband = this->get_parameter("angular_deadband").as_double();
        const bool is_commanded_to_turn = std::abs(msg->angular.z) > deadband;
        const bool is_translating =
            (std::abs(msg->linear.x) > 0.01 || std::abs(msg->linear.y) > 0.01 || std::abs(msg->linear.z) > 0.01);

        if (is_commanded_to_turn || !has_imu_data_ || is_imu_timeout_) {
            target_heading_locked_ = false;
            target_quat_ = current_quat_;
            target_heading_deg_ = current_heading_deg_;
            controller_.reset();
            out_twist->angular = msg->angular;  // Fail-Safe Passthrough
            last_correction_ = 0.0;
            last_error_deg_ = 0.0;
        } else if (!is_translating) {
            // When not translating, do not apply correction to avoid creeping due to sensor drift
            controller_.reset();
            out_twist->angular.z = 0.0;
            last_correction_ = 0.0;
            last_error_deg_ = 0.0;
        } else {
            if (!target_heading_locked_) {
                // Wait until the physical rotation speed (from gyro) drops below a threshold
                // to prevent overshoot/snap-back caused by robot inertia and IMU latency.
                const double stop_threshold_deg = 5.0;  // deg/s
                if (std::abs(gyro_z_deg_) < stop_threshold_deg || !has_imu_data_ || is_imu_timeout_) {
                    target_quat_ = current_quat_;
                    target_heading_deg_ = current_heading_deg_;
                    target_heading_locked_ = true;
                } else {
                    target_quat_ = current_quat_;
                    target_heading_deg_ = current_heading_deg_;
                }
            }

            if (target_heading_locked_) {
                auto out = controller_.update(target_quat_, current_quat_, dt, gyro_z_deg_, msg->linear.x);

                // Scale the PID correction output by the velocity factor to match JoyDriverNode logic
                const double velocity_magnitude =
                    std::sqrt(msg->linear.x * msg->linear.x + msg->linear.y * msg->linear.y);
                const double velocity_factor = std::clamp(velocity_magnitude / max_translation_speed_, 0.3, 1.0);

                out_twist->angular.z = out.correction * velocity_factor;
                last_correction_ = out_twist->angular.z;
                last_error_deg_ = out.error_deg;
            } else {
                out_twist->angular.z = 0.0;
                last_correction_ = 0.0;
                last_error_deg_ = 0.0;
            }
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                             "[HeadingControl] InVel: x=%.2f, z=%.2f | CommandTurn: %s | Locked: %s | Target yaw: "
                             "%.1f, Curr yaw: %.1f | Gyro Z: %.2f | Correct Out: %.3f",
                             msg->linear.x, msg->angular.z, is_commanded_to_turn ? "YES" : "NO",
                             target_heading_locked_ ? "YES" : "NO", target_heading_deg_, current_heading_deg_,
                             gyro_z_deg_, out_twist->angular.z);

        cmd_vel_pub_->publish(std::move(out_twist));
    }

    void checkSystemHealth() {
        if (!cmd_vel_pub_->is_activated()) return;

        const rclcpp::Time now = this->now();

        if (has_imu_data_) {
            const double imu_timeout = this->get_parameter("imu_timeout").as_double();
            if ((now - last_imu_time_).seconds() > imu_timeout) {
                if (!is_imu_timeout_) {
                    RCLCPP_WARN(this->get_logger(), "IMU Timeout! Fail-Safe Passthrough engaged.");
                    is_imu_timeout_ = true;
                    target_heading_locked_ = false;
                    controller_.reset();
                }
            }
        }

        if (has_cmd_vel_in_) {
            const double cmd_timeout = this->get_parameter("cmd_vel_timeout").as_double();
            const double elapsed = (now - last_cmd_vel_in_time_).seconds();

            if (elapsed > cmd_timeout) {
                if (!is_watchdog_triggered_) {
                    RCLCPP_WARN(this->get_logger(), "Watchdog Timeout! Publishing ZERO VELOCITY.");
                    is_watchdog_triggered_ = true;
                    // Keep target_heading_locked_ to preserve target heading target across stops
                    controller_.reset();
                }

                auto stop_twist = std::make_unique<geometry_msgs::msg::Twist>();
                cmd_vel_pub_->publish(std::move(stop_twist));
            }
        }
    }

    void publishDiagnostics() {
        if (!diag_pub_->is_activated()) return;

        auto diag_arr = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();
        diag_arr->header.stamp = this->now();

        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = "libbno055_linux: Lifecycle Heading Controller";
        status.hardware_id = "BNO055_Lifecycle_PID_Controller";

        if (is_watchdog_triggered_) {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            status.message = "SAFETY WATCHDOG: Input cmd_vel_in Timed Out";
        } else if (!has_imu_data_ || is_imu_timeout_) {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            status.message = "IMU Offline/Timed out: Operating in Fail-Safe Passthrough Mode";
        } else if (target_heading_locked_) {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            status.message = "Active Straight Heading Correction";
        } else {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            status.message = "Passthrough Mode (Active Turning Command)";
        }

        auto add_kv = [&status](const std::string& k, const std::string& v) {
            diagnostic_msgs::msg::KeyValue kv;
            kv.key = k;
            kv.value = v;
            status.values.push_back(kv);
        };

        add_kv("Target Heading (deg)", std::to_string(target_heading_deg_));
        add_kv("Current Heading (deg)", std::to_string(current_heading_deg_));
        add_kv("Heading Error (deg)", std::to_string(last_error_deg_));
        add_kv("PID Correction (rad/s)", std::to_string(last_correction_));
        add_kv("Target Locked", target_heading_locked_ ? "True" : "False");
        add_kv("IMU Healthy", (has_imu_data_ && !is_imu_timeout_) ? "True" : "False");
        add_kv("Watchdog Triggered", is_watchdog_triggered_ ? "True" : "False");

        diag_arr->status.push_back(status);
        diag_pub_->publish(std::move(diag_arr));
    }

    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>> cmd_vel_pub_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>> diag_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_heading_srv_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::TimerBase::SharedPtr diag_timer_;

    rclcpp::CallbackGroup::SharedPtr control_cb_group_;
    rclcpp::CallbackGroup::SharedPtr admin_cb_group_;

    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    bno055lib::HeadingController controller_;

    rclcpp::Time last_time_;
    rclcpp::Time last_cmd_vel_in_time_;
    rclcpp::Time last_imu_time_;
    bno055lib::Quat current_quat_;
    bno055lib::Quat target_quat_;
    double current_heading_deg_;
    double gyro_z_deg_;
    double target_heading_deg_;
    bool target_heading_locked_;
    bool has_imu_data_;
    bool has_cmd_vel_in_;
    bool is_watchdog_triggered_;
    bool is_imu_timeout_;
    double last_correction_;
    double last_error_deg_;
    std::string yaw_axis_;
    double max_translation_speed_;
};

}  // namespace bno055_ros2

#ifdef BNO055_ROS2_BUILDING_COMPONENT
RCLCPP_COMPONENTS_REGISTER_NODE(bno055_ros2::BNO055LifecycleHeadingControlNode)
#else
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bno055_ros2::BNO055LifecycleHeadingControlNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
#endif
