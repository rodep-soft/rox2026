#ifndef BNO055_HEADING_CONTROLLER_HPP
#define BNO055_HEADING_CONTROLLER_HPP

#include <algorithm>
#include <cmath>

#ifndef BNO055_LIKELY
#if defined(__GNUC__) || defined(__clang__)
#define BNO055_LIKELY(x)   __builtin_expect(!!(x), 1)
#define BNO055_UNLIKELY(x) __builtin_expect(!!(x), 0)
#else
#define BNO055_LIKELY(x)   (x)
#define BNO055_UNLIKELY(x) (x)
#endif
#endif

namespace bno055lib {

constexpr double RAD_TO_DEG = 180.0 / M_PI;
constexpr double DEG_TO_RAD = M_PI / 180.0;

/**
 * @brief Simple 3D Quaternion representation.
 */
struct Quat {
    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

/**
 * @brief Sanitizes and validates Quaternion data to guard against NaN, Inf, and norm corruption.
 */
[[nodiscard]] inline bool isValidQuat(const Quat& q) noexcept {
    if (BNO055_UNLIKELY(std::isnan(q.w) || std::isnan(q.x) || std::isnan(q.y) || std::isnan(q.z) || std::isinf(q.w) ||
                        std::isinf(q.x) || std::isinf(q.y) || std::isinf(q.z))) {
        return false;
    }
    const double norm_sq = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    return (norm_sq > 0.8 && norm_sq < 1.2);
}

/**
 * @brief Normalizes angle into [-180.0, +180.0] degrees range using std::remainder.
 */
[[nodiscard]] constexpr inline double normalizeAngleDeg(double angle_deg) noexcept {
    return std::remainder(angle_deg, 360.0);
}

/**
 * @brief Fast Yaw extraction directly from Quaternion (W, X, Y, Z) in degrees.
 */
[[nodiscard]] inline double fastExtractYawDeg(double qw, double qx, double qy, double qz) noexcept {
    const double siny_cosp = 2.0 * (qw * qz + qx * qy);
    const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    return std::atan2(siny_cosp, cosy_cosp) * RAD_TO_DEG;
}

[[nodiscard]] inline double fastExtractYawDeg(const Quat& q) noexcept {
    return fastExtractYawDeg(q.w, q.x, q.y, q.z);
}

/**
 * @brief Clean, Zero-Allocation C++17 Heading PID & Feedforward Controller.
 * Features:
 *  - Slew-Rate Limiter (Max angular acceleration constraint to protect motors & prevent wheel slip)
 *  - Kinematic Feedforward (FF) + Feedback (PID) Control Architecture
 *  - Trapezoidal (Tustin) Rule Integration for Jitter-Free Precision
 *  - 1st-Order Low-Pass Filtered Gyro Derivative to Suppress Mechanical Vibration Noise
 *  - Anti-Windup Saturation Clamping & Micro-Deadband Smoothing
 */
class HeadingController {
public:
    struct Config {
        double kp{0.05};              ///< Proportional Gain
        double ki{0.001};             ///< Integral Gain (Trapezoidal Rule)
        double kd{0.01};              ///< Derivative Gain (Filtered Gyro-based)
        double kff{0.0};              ///< Feedforward Gain
        double max_output{1.0};       ///< Max angular output limit
        double min_output{-1.0};      ///< Min angular output limit
        double max_i_term{0.2};       ///< Anti-windup saturation limit
        double deadband_deg{0.02};    ///< Micro-deadband (deg) to eliminate hunting
        double cutoff_freq_hz{20.0};  ///< Low-pass filter cutoff frequency (Hz)
        double max_slew_rate{0.0};    ///< Max change rate of output per sec (rad/s^2, 0.0 = disabled)
    };

    struct Output {
        double correction{0.0};     ///< Control output u = u_FF + u_PID (Slew-rate limited)
        double left_motor{0.0};     ///< Left wheel speed normalized [0.0, 1.0]
        double right_motor{0.0};    ///< Right wheel speed normalized [0.0, 1.0]
        double error_deg{0.0};      ///< Shortest heading error in degrees
        double gyro_filtered{0.0};  ///< Low-pass filtered gyro rate
    };

    HeadingController() noexcept
        : config_(), i_term_(0.0), prev_error_(0.0), prev_output_(0.0), filtered_gyro_(0.0), initialized_(false) {}

    explicit HeadingController(const Config& config) noexcept
        : config_(config),
          i_term_(0.0),
          prev_error_(0.0),
          prev_output_(0.0),
          filtered_gyro_(0.0),
          initialized_(false) {}

    inline void setGains(double kp, double ki, double kd, double kff = 0.0) noexcept {
        config_.kp = kp;
        config_.ki = ki;
        config_.kd = kd;
        config_.kff = kff;
    }

    inline void setConfig(const Config& config) noexcept { config_ = config; }

    [[nodiscard]] inline const Config& getConfig() const noexcept { return config_; }

    inline void reset() noexcept {
        i_term_ = 0.0;
        prev_error_ = 0.0;
        prev_output_ = 0.0;
        filtered_gyro_ = 0.0;
        initialized_ = false;
    }

    /**
     * @brief Computes control output given target heading and gyro feedback.
     */
    [[nodiscard]] inline Output update(double target_heading_deg, double current_heading_deg, double dt,
                                       double gyro_z_deg = 0.0, double base_velocity = 0.5,
                                       double target_yaw_rate_deg = 0.0) noexcept {
        Output out{};
        if (BNO055_UNLIKELY(dt <= 0.0)) {
            out.left_motor = std::clamp(base_velocity, 0.0, 1.0);
            out.right_motor = std::clamp(base_velocity, 0.0, 1.0);
            return out;
        }

        // 1. Shortest path angle difference (-180 to +180 deg)
        double raw_error = normalizeAngleDeg(target_heading_deg - current_heading_deg);

        // 2. Micro-deadband smoothing
        if (std::abs(raw_error) < config_.deadband_deg) {
            raw_error = 0.0;
        }
        out.error_deg = raw_error;

        // 3. Proportional Term
        const double p_term = config_.kp * out.error_deg;

        // 4. Trapezoidal Rule Integration & Anti-Windup
        if (BNO055_LIKELY(initialized_)) {
            if (config_.ki == 0.0) {
                i_term_ = 0.0;
            } else {
                const double trapezoidal_error = (out.error_deg + prev_error_) * 0.5;
                i_term_ =
                    std::clamp(i_term_ + config_.ki * trapezoidal_error * dt, -config_.max_i_term, config_.max_i_term);
            }
        }

        // 5. 1st-Order Low-Pass Filtered Gyro Rate for D-Term
        if (BNO055_UNLIKELY(!initialized_)) {
            filtered_gyro_ = gyro_z_deg;
        } else if (BNO055_LIKELY(config_.cutoff_freq_hz > 0.0)) {
            const double tau = 1.0 / (2.0 * M_PI * config_.cutoff_freq_hz);
            const double alpha = dt / (tau + dt);
            filtered_gyro_ = filtered_gyro_ + alpha * (gyro_z_deg - filtered_gyro_);
        } else {
            filtered_gyro_ = gyro_z_deg;
        }
        out.gyro_filtered = filtered_gyro_;

        double d_term = 0.0;
        if (BNO055_LIKELY(gyro_z_deg != 0.0 || !initialized_)) {
            d_term = -config_.kd * filtered_gyro_;
        } else {
            const double error_dot = (out.error_deg - prev_error_) / dt;
            d_term = config_.kd * error_dot;
        }

        // 6. Kinematic Feedforward Term
        const double ff_term = config_.kff * target_yaw_rate_deg;

        prev_error_ = out.error_deg;

        // 7. Unconstrained Control Output
        double raw_output = std::clamp(ff_term + p_term + i_term_ + d_term, config_.min_output, config_.max_output);

        // 8. Slew-Rate Limiter (Max Acceleration Constraint to protect motors)
        if (BNO055_LIKELY(initialized_ && config_.max_slew_rate > 0.0)) {
            const double max_change = config_.max_slew_rate * dt;
            const double delta = std::clamp(raw_output - prev_output_, -max_change, max_change);
            raw_output = prev_output_ + delta;
        }

        prev_output_ = raw_output;
        initialized_ = true;
        out.correction = raw_output;

        // 9. Differential wheel outputs
        out.left_motor = std::clamp(base_velocity - out.correction, 0.0, 1.0);
        out.right_motor = std::clamp(base_velocity + out.correction, 0.0, 1.0);

        return out;
    }

    /**
     * @brief Direct Quaternion Update Overload
     */
    [[nodiscard]] inline Output update(const Quat& q_target, const Quat& q_current, double dt, double gyro_z_deg = 0.0,
                                       double base_velocity = 0.5, double target_yaw_rate_deg = 0.0) noexcept {
        const double target_heading_deg = fastExtractYawDeg(q_target);
        const double current_heading_deg = fastExtractYawDeg(q_current);
        return update(target_heading_deg, current_heading_deg, dt, gyro_z_deg, base_velocity, target_yaw_rate_deg);
    }

private:
    Config config_;
    double i_term_;
    double prev_error_;
    double prev_output_;
    double filtered_gyro_;
    bool initialized_;
};

}  // namespace bno055lib

#endif  // BNO055_HEADING_CONTROLLER_HPP
