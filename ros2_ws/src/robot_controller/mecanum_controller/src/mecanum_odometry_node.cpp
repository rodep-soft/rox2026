#include "mecanum_controller/mecanum_odometry.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "actuator_msgs/msg/actuator_state.hpp"
#include "actuator_msgs/msg/actuator_state_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

class MecanumOdometryNode : public rclcpp::Node
{
public:
  MecanumOdometryNode()
  : Node("mecanum_odometry_node")
  {
    configure_parameters();

    // pose_covariance_x_ / y_ / yaw_ が確定してから pose_cov_* を初期化する
    pose_cov_x_ = pose_covariance_x_;
    pose_cov_y_ = pose_covariance_y_;
    pose_cov_yaw_ = pose_covariance_yaw_;

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    state_sub_ = create_subscription<actuator_msgs::msg::ActuatorStateArray>(
      state_topic_, 10,
      [this](const actuator_msgs::msg::ActuatorStateArray::SharedPtr msg) {receive_state(*msg);});

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {receive_imu(*msg);});

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 20);

    last_update_ = now();
    timer_ = create_wall_timer(
      std::chrono::duration<double, std::milli>(publish_period_ms_),
      [this]() {update();});

    RCLCPP_INFO(get_logger(), "MecanumOdometryNode initialized with IMU-discrepancy slip fusion.");
  }

private:
  static void require_positive(const std::string & name, const double value)
  {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(name + " must be finite and greater than zero");
    }
  }

  void configure_parameters()
  {
    wheel_radius_m_ = declare_parameter("wheel_radius", 0.075);
    robot_length_m_ = declare_parameter("robot_length", 0.355);
    robot_width_m_ = declare_parameter("robot_width", 0.353);
    publish_period_ms_ = declare_parameter("publish_period_ms", 20.0);
    feedback_timeout_ms_ = declare_parameter("feedback_timeout_ms", 250.0);
    scale_x_ = declare_parameter("velocity_scale_x", 1.0);
    scale_y_ = declare_parameter("velocity_scale_y", 1.0);
    scale_yaw_ = declare_parameter("velocity_scale_yaw", 1.0);
    filter_alpha_ = declare_parameter("velocity_filter_alpha", 0.35);
    publish_tf_ = declare_parameter("publish_tf", true);

    slip_enabled_ = declare_parameter("slip_compensation.enabled", true);
    accel_threshold_x_ = declare_parameter("slip_compensation.acceleration_threshold_x_m_s2", 1.0);
    accel_threshold_y_ = declare_parameter("slip_compensation.acceleration_threshold_y_m_s2", 0.7);
    accel_threshold_yaw_ = declare_parameter(
      "slip_compensation.angular_acceleration_threshold_rad_s2", 2.0);
    max_covariance_multiplier_ = declare_parameter(
      "slip_compensation.maximum_covariance_multiplier", 10.0);

    pose_covariance_x_ = declare_parameter("pose_covariance_x", 0.02);
    pose_covariance_y_ = declare_parameter("pose_covariance_y", 0.05);
    pose_covariance_yaw_ = declare_parameter("pose_covariance_yaw", 0.10);
    twist_covariance_x_ = declare_parameter("twist_covariance_x", 0.03);
    twist_covariance_y_ = declare_parameter("twist_covariance_y", 0.10);
    twist_covariance_yaw_ = declare_parameter("twist_covariance_yaw", 0.15);
    // odom_drift_rate: pose covariance を毎ステップ増加させる比率 (twist covに対する倍率)。
    odom_drift_rate_ = declare_parameter("odom_drift_rate", 0.03);

    state_topic_ = declare_parameter<std::string>(
      "state_array_topic",
      "/hardware/actuator_state_array");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data");
    odom_topic_ = declare_parameter<std::string>("odometry_topic", "/wheel/odometry");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    const auto ids = declare_parameter<std::vector<int64_t>>("wheel_logical_ids", {0, 1, 2, 3});
    if (ids.size() != wheel_ids_.size()) {
      throw std::invalid_argument("wheel_logical_ids must contain exactly four values");
    }
    for (std::size_t index = 0; index < ids.size(); ++index) {
      if (ids[index] < 0 || ids[index] > 65535) {
        throw std::invalid_argument("wheel_logical_ids values must be in [0, 65535]");
      }
      wheel_ids_[index] = static_cast<uint16_t>(ids[index]);
      if (std::count(wheel_ids_.cbegin(), wheel_ids_.cbegin() + index + 1, wheel_ids_[index]) > 1) {
        throw std::invalid_argument("wheel_logical_ids values must be unique");
      }
    }

    for (const auto & parameter : std::vector<std::pair<std::string, double>>{
      {"wheel_radius", wheel_radius_m_}, {"publish_period_ms", publish_period_ms_},
      {"feedback_timeout_ms", feedback_timeout_ms_}, {"velocity_scale_x", scale_x_},
      {"velocity_scale_y", scale_y_}, {"velocity_scale_yaw", scale_yaw_},
      {"acceleration_threshold_x", accel_threshold_x_},
      {"acceleration_threshold_y", accel_threshold_y_},
      {"acceleration_threshold_yaw", accel_threshold_yaw_},
      {"maximum_covariance_multiplier", max_covariance_multiplier_}})
    {
      require_positive(parameter.first, parameter.second);
    }
    if (!std::isfinite(robot_length_m_) || !std::isfinite(robot_width_m_) ||
      robot_length_m_ < 0.0 || robot_width_m_ < 0.0 || robot_length_m_ + robot_width_m_ <= 0.0)
    {
      throw std::invalid_argument("robot dimensions must be non-negative with a positive sum");
    }
    if (!std::isfinite(filter_alpha_) || filter_alpha_ <= 0.0 || filter_alpha_ > 1.0) {
      throw std::invalid_argument("velocity_filter_alpha must be in (0, 1]");
    }
  }

  void receive_state(const actuator_msgs::msg::ActuatorStateArray & message)
  {
    std::array<double, mecanum_odometry::WHEEL_COUNT> received{};
    std::array<bool, mecanum_odometry::WHEEL_COUNT> found{};
    for (const auto & actuator : message.actuators) {
      const auto iterator = std::find(wheel_ids_.cbegin(), wheel_ids_.cend(), actuator.logical_id);
      if (iterator == wheel_ids_.cend()) {
        continue;
      }
      const auto index = static_cast<std::size_t>(std::distance(wheel_ids_.cbegin(), iterator));
      if (actuator.state == actuator_msgs::msg::ActuatorState::STATE_READY &&
        std::isfinite(actuator.velocity))
      {
        received[index] = actuator.velocity;
        found[index] = true;
      }
    }
    feedback_valid_ = std::all_of(found.cbegin(), found.cend(), [](bool value) {return value;});
    // メッセージの header.stamp を計測時刻として使う。
    // now() を使うと EKF が「いつ計測されたか」を誤解し、タイムスタンプベースの融合が乱れる。
    // header.stamp が未設定 (0) の場合は受信時刻にフォールバックする。
    const auto & msg_stamp = message.header.stamp;
    feedback_stamp_ = (msg_stamp.sec != 0 || msg_stamp.nanosec != 0) ?
      rclcpp::Time(msg_stamp) : now();
    if (feedback_valid_) {
      wheel_velocity_ = received;
    }
  }

  mecanum_odometry::BodyVelocity measured_velocity() const
  {
    const double rotation_radius_m = (robot_length_m_ + robot_width_m_) / 2.0;
    auto velocity = mecanum_odometry::calculate_body_velocity(
      wheel_velocity_, wheel_radius_m_, rotation_radius_m);
    velocity.x_m_s *= scale_x_;
    velocity.y_m_s *= scale_y_;
    velocity.yaw_rad_s *= scale_yaw_;
    return velocity;
  }

  mecanum_odometry::BodyVelocity filtered_velocity(
    const mecanum_odometry::BodyVelocity & measured)
  {
    if (!filter_initialized_) {
      filtered_ = measured;
      filter_initialized_ = true;
      return filtered_;
    }
    const double old_weight = 1.0 - filter_alpha_;
    filtered_.x_m_s = filter_alpha_ * measured.x_m_s + old_weight * filtered_.x_m_s;
    filtered_.y_m_s = filter_alpha_ * measured.y_m_s + old_weight * filtered_.y_m_s;
    filtered_.yaw_rad_s = filter_alpha_ * measured.yaw_rad_s + old_weight * filtered_.yaw_rad_s;
    return filtered_;
  }

  void receive_imu(const sensor_msgs::msg::Imu & msg)
  {
    imu_accel_x_ = msg.linear_acceleration.x;
    imu_accel_y_ = msg.linear_acceleration.y;
    imu_angular_vel_z_ = msg.angular_velocity.z;
    imu_stamp_ = msg.header.stamp;
    imu_received_ = true;
  }

  mecanum_odometry::AxisCovarianceScale covariance_multiplier(
    const mecanum_odometry::BodyVelocity & velocity, const double dt_s)
  {
    if (!slip_enabled_ || !previous_velocity_initialized_) {
      previous_velocity_ = velocity;
      previous_velocity_initialized_ = true;
      return {};  // x=y=yaw=1.0
    }

    const mecanum_odometry::BodyVelocity wheel_acceleration{
      (velocity.x_m_s - previous_velocity_.x_m_s) / dt_s,
      (velocity.y_m_s - previous_velocity_.y_m_s) / dt_s,
      (velocity.yaw_rad_s - previous_velocity_.yaw_rad_s) / dt_s};
    previous_velocity_ = velocity;

    // IMUが受信できている場合は、車輪加速度とIMU実測加速度の不一致度(スリップ量)で共分散を計算
    const double imu_age_s = (now() - imu_stamp_).seconds();
    if (imu_received_ && imu_age_s >= 0.0 && imu_age_s < 0.2) {
      const mecanum_odometry::BodyVelocity imu_accel{
        imu_accel_x_,
        imu_accel_y_,
        wheel_acceleration.yaw_rad_s // yaw角加速度は車輪差分またはジャイロ微分
      };
      return mecanum_odometry::calculate_imu_discrepancy_multipliers(
        wheel_acceleration, imu_accel,
        accel_threshold_x_, accel_threshold_y_, accel_threshold_yaw_,
        max_covariance_multiplier_);
    }

    // IMU未受信時は従来の車輪加速度によるフォールバック
    return mecanum_odometry::calculate_covariance_multipliers(
      wheel_acceleration, accel_threshold_x_, accel_threshold_y_, accel_threshold_yaw_,
      max_covariance_multiplier_);
  }

  void update()
  {
    const auto now_stamp = now();
    const double dt_s = (now_stamp - last_update_).seconds();
    last_update_ = now_stamp;
    if (dt_s <= 0.0 || dt_s > 1.0) {
      return;
    }

    const double feedback_age_ms = (now_stamp - feedback_stamp_).seconds() * 1000.0;
    const bool usable = feedback_valid_ && feedback_age_ms >= 0.0 &&
      feedback_age_ms <= feedback_timeout_ms_;
    mecanum_odometry::BodyVelocity velocity;
    mecanum_odometry::AxisCovarianceScale slip_scale;
    if (usable) {
      // スリップ検出はフィルタ前の生速度に対して行う。
      // フィルタ後では急加速が平滑化されてスリップを見逃す可能性がある。
      const auto raw_velocity = measured_velocity();
      slip_scale = covariance_multiplier(raw_velocity, dt_s);
      velocity = filtered_velocity(raw_velocity);
      const double cos_yaw = std::cos(yaw_rad_);
      const double sin_yaw = std::sin(yaw_rad_);
      position_x_m_ += (velocity.x_m_s * cos_yaw - velocity.y_m_s * sin_yaw) * dt_s;
      position_y_m_ += (velocity.x_m_s * sin_yaw + velocity.y_m_s * cos_yaw) * dt_s;
      // yaw は std::remainder で正規化しない。EKFやquaternion変換は連続値を期待するケースがある。
      // 正規化は orientation を TF2 quaternion で作る際に暗黙的に行われる。
      yaw_rad_ += velocity.yaw_rad_s * dt_s;
    } else {
      filter_initialized_ = false;
      previous_velocity_initialized_ = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Wheel feedback is incomplete, not ready, or stale; integration is paused");
    }

    // odometry のタイムスタンプは実際の計測時刻 (feedback_stamp_) を使う。
    // robot_localization はタイムスタンプを使ってセンサフュージョンを時系列で正確に行うため、
    // ここで now() (タイマー発火時刻) を使うと最大 feedback_timeout_ms_ 分の遅延誤差が生じる。
    // TF は now_stamp を使う (未来タイムスタンプを送ると tf2 が拒否するため)。
    const auto odom_stamp = usable ? feedback_stamp_ : now_stamp;
    publish(odom_stamp, now_stamp, velocity, slip_scale, usable);
  }


  void publish(
    const rclcpp::Time & stamp, const rclcpp::Time & tf_stamp,
    const mecanum_odometry::BodyVelocity & velocity,
    const mecanum_odometry::AxisCovarianceScale & slip_scale, const bool feedback_usable)
  {
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, yaw_rad_);

    // --- 6×6 covariance matrices (row-major, indices: x=0,y=1,z=2,roll=3,pitch=4,yaw=5) ---
    //
    // EKFに突っ込む前提での設計方針:
    //   - twist covariance: ホイールで直接計測した速度の不確かさ → EKFの observation noise
    //     各軸独立のスリップスケールを適用。フィードバック無効時は巨大値でEKFに無視させる。
    //   - velocity_scale²: velocity_scale_x/y/yaw は速度をスケールする補正係数なので、
    //     分散は scale の二乗を乗じる必要がある (σ²(scale*v) = scale² * σ²(v))。
    //   - pose covariance: odom フレームでの積分誤差累積 (differential モードでは未参照)
    //   - z / roll / pitch: 平面ロボット。1e9 でEKFが事実上無視する大きさ。
    //
    // nav_msgs/Odometry::pose.covariance の index 対応:
    //   [row*6+col] → row/col: 0=x, 1=y, 2=z, 3=roll, 4=pitch, 5=yaw

    constexpr double kUnobservedVariance = 1e9;
    constexpr double kInvalidFeedbackVariance = 1e12;

    // ---------- twist covariance ----------
    // 各軸の最終的な twist 分散:
    //   σ²_twist_x   = twist_covariance_xy  * scale_x²  * slip_scale.x
    //   σ²_twist_y   = twist_covariance_xy  * scale_y²  * slip_scale.y
    //   σ²_twist_yaw = twist_covariance_yaw * scale_yaw² * slip_scale.yaw
    //
    // 連帯責任排除: スリップは軸ごとに独立評価。X がスリップしても yaw の分散は増えない。
    nav_msgs::msg::Odometry message;
    message.header.stamp = stamp;
    message.header.frame_id = odom_frame_;
    message.child_frame_id = base_frame_;
    message.pose.pose.position.x = position_x_m_;
    message.pose.pose.position.y = position_y_m_;
    message.pose.pose.orientation.x = orientation.x();
    message.pose.pose.orientation.y = orientation.y();
    message.pose.pose.orientation.z = orientation.z();
    message.pose.pose.orientation.w = orientation.w();
    message.twist.twist.linear.x = velocity.x_m_s;
    message.twist.twist.linear.y = velocity.y_m_s;
    message.twist.twist.angular.z = velocity.yaw_rad_s;

    if (feedback_usable) {
      // scale² を乗じて速度補正係数の影響を分散に反映する (XとYで完全独立)
      message.twist.covariance[0] = twist_covariance_x_ * scale_x_ * scale_x_ * slip_scale.x;
      message.twist.covariance[7] = twist_covariance_y_ * scale_y_ * scale_y_ * slip_scale.y;
      message.twist.covariance[35] = twist_covariance_yaw_ * scale_yaw_ * scale_yaw_ *
        slip_scale.yaw;
    } else {
      message.twist.covariance[0] = kInvalidFeedbackVariance;
      message.twist.covariance[7] = kInvalidFeedbackVariance;
      message.twist.covariance[35] = kInvalidFeedbackVariance;
    }
    // twist: 非観測軸 (vz, wx, wy) — 平面ロボットなので巨大な不確かさ
    message.twist.covariance[14] = kUnobservedVariance;  // vz
    message.twist.covariance[21] = kUnobservedVariance;  // wx (roll rate)
    message.twist.covariance[28] = kUnobservedVariance;  // wy (pitch rate)

    // ---------- pose covariance ----------
    // 積分誤差を軸別に累積。twist covと同じ scale² 補正を適用する。
    // フィードバック無効時は積分を止めているので pose cov は増加しない。
    if (feedback_usable) {
      pose_cov_x_ += twist_covariance_x_ * scale_x_ * scale_x_ * slip_scale.x * odom_drift_rate_;
      pose_cov_y_ += twist_covariance_y_ * scale_y_ * scale_y_ * slip_scale.y * odom_drift_rate_;
      pose_cov_yaw_ += twist_covariance_yaw_ * scale_yaw_ * scale_yaw_ * slip_scale.yaw *
        odom_drift_rate_;
    }
    // pose: 観測軸
    message.pose.covariance[0] = pose_cov_x_;     // x
    message.pose.covariance[7] = pose_cov_y_;     // y
    message.pose.covariance[35] = pose_cov_yaw_;  // yaw
    // pose: 非観測軸
    message.pose.covariance[14] = kUnobservedVariance;  // z
    message.pose.covariance[21] = kUnobservedVariance;  // roll
    message.pose.covariance[28] = kUnobservedVariance;  // pitch

    odom_pub_->publish(message);

    // TF は tf_stamp (now) を使う。
    // odometry は計測時刻 (stamp) を使うが、TF に過去のタイムスタンプを送ると
    // tf2::lookupTransform が「過去に遡れない」エラーになるため現在時刻を使う。
    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = tf_stamp;
      t.header.frame_id = odom_frame_;
      t.child_frame_id = base_frame_;
      t.transform.translation.x = position_x_m_;
      t.transform.translation.y = position_y_m_;
      t.transform.translation.z = 0.0;
      t.transform.rotation.x = orientation.x();
      t.transform.rotation.y = orientation.y();
      t.transform.rotation.z = orientation.z();
      t.transform.rotation.w = orientation.w();
      tf_broadcaster_->sendTransform(t);
    }
  }


  rclcpp::Subscription<actuator_msgs::msg::ActuatorStateArray>::SharedPtr state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::array<uint16_t, mecanum_odometry::WHEEL_COUNT> wheel_ids_{};
  std::array<double, mecanum_odometry::WHEEL_COUNT> wheel_velocity_{};
  rclcpp::Time feedback_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_update_{0, 0, RCL_ROS_TIME};
  rclcpp::Time imu_stamp_{0, 0, RCL_ROS_TIME};
  double imu_accel_x_{0.0};
  double imu_accel_y_{0.0};
  double imu_angular_vel_z_{0.0};
  bool imu_received_{false};
  mecanum_odometry::BodyVelocity filtered_;
  mecanum_odometry::BodyVelocity previous_velocity_;
  bool feedback_valid_{false};
  bool filter_initialized_{false};
  bool previous_velocity_initialized_{false};
  double wheel_radius_m_, robot_length_m_, robot_width_m_;
  double publish_period_ms_, feedback_timeout_ms_;
  double scale_x_, scale_y_, scale_yaw_, filter_alpha_;
  bool publish_tf_;
  bool slip_enabled_;
  double accel_threshold_x_, accel_threshold_y_, accel_threshold_yaw_;
  double max_covariance_multiplier_;
  double pose_covariance_x_, pose_covariance_y_, pose_covariance_yaw_;
  double twist_covariance_x_, twist_covariance_y_, twist_covariance_yaw_;
  double odom_drift_rate_;
  double position_x_m_{0.0}, position_y_m_{0.0}, yaw_rad_{0.0};
  // pose covariance の累積値。configure_parameters() 後に base 値で初期化。
  // robot_localization の differential モードでは twist cov のみ参照されるため、
  // ここの値が絶対的な精度に影響することはないが、absolute pose モードのために設定する。
  double pose_cov_x_{0.0};
  double pose_cov_y_{0.0};
  double pose_cov_yaw_{0.0};
  std::string state_topic_, imu_topic_, odom_topic_, odom_frame_, base_frame_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MecanumOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
