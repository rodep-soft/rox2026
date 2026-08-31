#include "dribble_controller/dribble_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

namespace
{
constexpr int kMaxRollerRpmStepPerTick = 150;
} // namespace

DribbleControllerNode::DribbleControllerNode()
: Node("dribble_controller_node")
{
  const auto position_target_topic = declare_parameter<std::string>(
    "position_target_topic", "/edulite/target");
  const auto roller_target_topic =
    declare_parameter<std::string>("roller_target_topic", "/vesc/target");
  const auto command_period_ms =
    declare_parameter<int>("command_period_ms", 20);
  if (position_target_topic.empty() || roller_target_topic.empty()) {
    throw std::runtime_error("target topics must not be empty");
  }
  if (command_period_ms <= 0) {
    throw std::runtime_error("command_period_ms must be positive");
  }

  load_parameters();

  const auto emergency_stop_qos = rclcpp::QoS(1).reliable().transient_local();
  const auto command_qos =
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
  const auto request_qos =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

  position_command_pub_ = create_publisher<actuator_msgs::msg::ActuatorTarget>(
    position_target_topic, command_qos);
  roller_command_pub_ = create_publisher<actuator_msgs::msg::ActuatorTarget>(
    roller_target_topic, command_qos);
  shot_cycle_state_pub_ = create_publisher<robot_msgs::msg::ShotCycleState>(
    "/dribble/shot_cycle_state", rclcpp::QoS(1).reliable().transient_local());
  belt_clearance_request_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/spring/belt_clearance_request",
    rclcpp::QoS(1).reliable().transient_local());

  ball_detected_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/dribble/ball_detected", rclcpp::QoS(1).reliable().transient_local());

  belt_mode_pub_ = create_publisher<robot_msgs::msg::BeltMode>(
    "/belt/command_mode", request_qos);

  position_mode_sub_ = create_subscription<robot_msgs::msg::ArmPosition>(
    "/dribble/command_position", request_qos,
    std::bind(
      &DribbleControllerNode::position_mode_callback, this,
      std::placeholders::_1));

  dribble_enabled_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/dribble/command_enabled", command_qos,
    std::bind(
      &DribbleControllerNode::dribble_enabled_callback, this,
      std::placeholders::_1));

  shot_cycle_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/dribble/shot_cycle_request", request_qos,
    std::bind(
      &DribbleControllerNode::shot_cycle_callback, this,
      std::placeholders::_1));

  emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/system/emergency_stop", emergency_stop_qos,
    std::bind(
      &DribbleControllerNode::emergency_stop_callback, this,
      std::placeholders::_1));

  edulite_state_sub_ = create_subscription<actuator_msgs::msg::ActuatorState>(
    "/edulite/state", command_qos,
    std::bind(
      &DribbleControllerNode::edulite_state_callback, this,
      std::placeholders::_1));

  vesc_state_sub_ = create_subscription<actuator_msgs::msg::ActuatorState>(
    "/vesc/state", command_qos,
    std::bind(
      &DribbleControllerNode::vesc_state_callback, this,
      std::placeholders::_1));

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic_, command_qos,
    std::bind(
      &DribbleControllerNode::cmd_vel_callback, this,
      std::placeholders::_1));
  spring_operation_state_sub_ =
    create_subscription<robot_msgs::msg::SpringOperationState>(
    "/spring/operation_state",
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(
      &DribbleControllerNode::spring_operation_state_callback,
      this, std::placeholders::_1));

  control_timer_ = create_wall_timer(
    std::chrono::milliseconds(command_period_ms),
    std::bind(&DribbleControllerNode::control_timer_callback, this));

  parameter_callback_handle_ = add_on_set_parameters_callback(
    std::bind(
      &DribbleControllerNode::parameter_callback, this, std::placeholders::_1));
}

// ---------------------------------------------------------------------------------------------------------------
// callback関数一覧
// ---------------------------------------------------------------------------------------------------------------
/// @brief 4パターンの位置の情報を取得
void DribbleControllerNode::position_mode_callback(
  const robot_msgs::msg::ArmPosition::SharedPtr msg)
{
  const uint8_t target_mode = msg->position;
  if (emergency_stop_active_ ||
    (msg->position != robot_msgs::msg::ArmPosition::DRIBBLE &&
    msg->position != robot_msgs::msg::ArmPosition::OPEN &&
    msg->position != robot_msgs::msg::ArmPosition::HOME))
  {
    return;
  }
  // 今の状態以外に移行もしくは，ベルト発射中の場合はベルト発射を中断する
  if (target_mode != position_mode_ || shot_cycle_active_) {
    shot_cycle_active_ = false;
    pre_shot_state_ = PreShotState::IDLE;
    set_spring_clearance(false);
    is_arm_moving_ = true;
    arm_move_start_time_ = now();
    arm_move_start_pos_rad_ = arm_cmd_pos_rad_;
    arm_move_start_rpm_ = roller_cmd_rpm_;
    position_mode_ = target_mode;
    if (target_mode == robot_msgs::msg::ArmPosition::OPEN) {
      // Stop immediately instead of sending very low RPM commands until the next control tick.
      roller_cmd_rpm_ = 0;
      update_and_publish_roller_command();
    }
  }
}

/// @brief ドリブルをするかしないかを受信
void DribbleControllerNode::dribble_enabled_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  dribble_enabled_ = msg->data;
}

/// @brief ベルト発射のサイクルのフラグ受け取り
void DribbleControllerNode::shot_cycle_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  // 非常がかかっている，すでにサイクルが始まっている場合は無視
  if (!msg->data || emergency_stop_active_ || shot_cycle_active_ ||
    pre_shot_state_ != PreShotState::IDLE)
  {
    return;
  }

  // ショット前の元の姿勢を記憶
  pre_shot_saved_position_mode_ = position_mode_;

  if (position_mode_ != robot_msgs::msg::ArmPosition::DRIBBLE) {
    position_mode_ = robot_msgs::msg::ArmPosition::DRIBBLE;
    is_arm_moving_ = true;
    arm_move_start_time_ = now();
    arm_move_start_pos_rad_ = arm_cmd_pos_rad_;
    arm_move_start_rpm_ = roller_cmd_rpm_;
    pre_shot_state_ = PreShotState::MOVING_TO_DRIBBLE;
    RCLCPP_INFO(
      get_logger(),
      "Belt shot requested from mode %u: returning to DRIBBLE first",
      pre_shot_saved_position_mode_);
    return;
  }
  start_shot_cycle();
}

/// @brief ばね発射機構に退避のリクエスト送信
/// @param enabled true : 退避させる false : 退避させない
void DribbleControllerNode::set_spring_clearance(bool enabled)
{
  std_msgs::msg::Bool request;
  request.data = enabled;
  belt_clearance_request_pub_->publish(request);
}

/// @brief ベルト発射のシーケンススタート
void DribbleControllerNode::start_shot_cycle()
{
  RCLCPP_INFO(
    get_logger(),
    "Belt shot requested: starting roller spin-up and spring retraction");
  is_arm_moving_ = false;
  shot_cycle_active_ = true;
  shot_cycle_phase_ = robot_msgs::msg::ShotCycleState::BELT_SPINUP;
  shot_phase_start_time_ = now();
  shot_phase_start_pos_rad_ = arm_cmd_pos_rad_;
  position_mode_ = robot_msgs::msg::ArmPosition::DRIBBLE;

  // 上下のベルトが両方とも回転基準未満の場合だけ、自動で回転を開始する
  constexpr float stopped_threshold_rpm = 500.0f;
  belt_auto_started_ =
    std::abs(upper_belt_measured_rpm_) < stopped_threshold_rpm &&
    std::abs(under_belt_measured_rpm_) < stopped_threshold_rpm;
  if (belt_auto_started_) {
    robot_msgs::msg::BeltMode belt_msg;
    belt_msg.mode = static_cast<uint8_t>(shot_cycle_belt_spinup_level_);
    belt_mode_pub_->publish(belt_msg);
  }

  set_spring_clearance(true);
  update_and_publish_roller_command();
}

/// @brief ドリブル機構の位置と回転数を指定する関数
/// @param start_pos_rad セットする角度
/// @param start_rpm セットする回転数
void DribbleControllerNode::resume_arm_control(double start_pos_rad, int start_rpm)
{
  arm_cmd_pos_rad_ = start_pos_rad;
  if (shot_cycle_active_) {
    shot_phase_start_pos_rad_ = start_pos_rad;
    shot_phase_start_time_ = now();
    return;
  }
  is_arm_moving_ = true;
  arm_move_start_pos_rad_ = start_pos_rad;
  arm_move_start_time_ = now();
  arm_move_start_rpm_ = start_rpm;
}

/// @brief 非常停止の値を受信する関数
void DribbleControllerNode::emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  //値変更時に一度だけ実行
  if (msg->data == emergency_stop_active_) {
    return;
  }
  emergency_stop_active_ = msg->data;

  if (emergency_stop_active_) {
    //非常停止時はローラを止めて，アームを現在位置に固定する
    hold_arm_pos_rad_ = is_arm_ready_ ? arm_pos_rad_ : arm_cmd_pos_rad_;
    arm_cmd_pos_rad_ = hold_arm_pos_rad_;
    roller_cmd_rpm_ = 0;
    RCLCPP_WARN(
      get_logger(),
      "Emergency stop: holding dribble arm at %.3f rad", hold_arm_pos_rad_);
  } else {
    // 現在位置と0回転から再開する
    const double start_pos_rad = is_arm_ready_ ? arm_pos_rad_ : hold_arm_pos_rad_;
    resume_arm_control(start_pos_rad, 0);
    RCLCPP_INFO(
      get_logger(),
      "Emergency stop released: resuming from %.3f rad", start_pos_rad);
  }
  control_timer_callback();
}

/// @brief armのeduliteの状態を受信する関数
void DribbleControllerNode::edulite_state_callback(
  const actuator_msgs::msg::ActuatorState::SharedPtr msg)
{
  if (msg->logical_id != position_logical_id_) {
    return;
  }

  if (msg->state != actuator_msgs::msg::ActuatorState::STATE_READY) {
    // eduliteの起動ができていない場合
    if (is_arm_ready_) {
      RCLCPP_WARN(
        get_logger(), "Dribble EduLite disconnected; pausing arm commands");
    }
    is_arm_ready_ = false;
    return;
  }

  // 現在角度はREADYのフィードバックを受けるたびに更新する
  const bool was_arm_ready = is_arm_ready_;
  is_arm_ready_ = true;
  arm_pos_rad_ = msg->position;

  // READYへ切り替わったときだけ指令値を現在角度で初期化する
  if (was_arm_ready) {
    return;
  }
  arm_cmd_pos_rad_ = arm_pos_rad_;
  hold_arm_pos_rad_ = arm_pos_rad_;

  // 非常が解除されている状態で準備完了した場合，目標値のリセット
  if (!emergency_stop_active_) {
    resume_arm_control(arm_pos_rad_, roller_cmd_rpm_);
  }
  RCLCPP_INFO(
    get_logger(), "Dribble EduLite ready at %.3f rad", arm_pos_rad_);
}

/// @brief ローラやベルトの赤ブラシのデータを取得
void DribbleControllerNode::vesc_state_callback(
  const actuator_msgs::msg::ActuatorState::SharedPtr msg)
{
  if (msg->logical_id == upper_belt_logical_id_) {
    upper_belt_measured_rpm_ = msg->velocity;
  } else if (msg->logical_id == under_belt_logical_id_) {
    under_belt_measured_rpm_ = msg->velocity;
  }

  if (msg->logical_id == roller_logical_id_) {
    // ローラー電流値に一次ローパスフィルタを適用 (最新値係数:current_lpf_alpha_)
    if (!roller_current_initialized_) {
      filtered_roller_current_a_ = msg->current_a;
      roller_current_initialized_ = true;
    } else {
      filtered_roller_current_a_ = current_lpf_alpha_ * msg->current_a +
        (1.0 - current_lpf_alpha_) * filtered_roller_current_a_;
    }
    // 電流値によるボール保持判定 (ヒステリシス + 連続カウントによるディバウンスノイズフィルタ)
    const bool previous_has_ball = has_ball_;
    if (filtered_roller_current_a_ >= ball_detection_threshold_a_) {
      ball_detected_counter_++;
      ball_lost_counter_ = 0;
      if (ball_detected_counter_ >= ball_detection_debounce_count_) {
        has_ball_ = true;
      }
    } else if (filtered_roller_current_a_ <= ball_lost_threshold_a_) {
      ball_lost_counter_++;
      ball_detected_counter_ = 0;
      if (ball_lost_counter_ >= ball_lost_debounce_count_) {
        has_ball_ = false;
      }
    } else {
      ball_detected_counter_ = 0;
      ball_lost_counter_ = 0;
    }
    if (previous_has_ball != has_ball_) {
      if (has_ball_) {
        RCLCPP_INFO(
          get_logger(),
          ">>> BALL DETECTED (Current: %.2f A, Filtered: %.2f A) <<<",
          msg->current_a, filtered_roller_current_a_);
      } else {
        RCLCPP_INFO(
          get_logger(),
          "--- BALL LOST (Current: %.2f A, Filtered: %.2f A) ---",
          msg->current_a, filtered_roller_current_a_);
      }
    }
    if (!last_published_ball_state_.has_value() || *last_published_ball_state_ != has_ball_) {
      std_msgs::msg::Bool ball_msg;
      ball_msg.data = has_ball_;
      ball_detected_pub_->publish(ball_msg);
      last_published_ball_state_ = has_ball_;
    }
  }
}

/// @brief cmd_velを受信する関数
void DribbleControllerNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  //速度と現在時間を取得して，差分により加速度を求める
  const auto current_time = now();
  commanded_vx_m_s_ = msg->linear.x;
  commanded_wz_rad_s_ = msg->angular.z;
  if (last_cmd_vel_time_.nanoseconds() > 0) {
    const double dt = (current_time - last_cmd_vel_time_).seconds();
    if (dt > 0.001 && dt < 0.5) {  // 受信間隔がおかしい場合は無視
      const double raw_acc = (commanded_vx_m_s_ - last_commanded_vx_m_s_) / dt;
      commanded_ax_m_s2_ = cmd_vel_acc_lpf_alpha_ * raw_acc + (1.0 - cmd_vel_acc_lpf_alpha_) *
        commanded_ax_m_s2_;
    }
  }
  last_commanded_vx_m_s_ = commanded_vx_m_s_;
  last_cmd_vel_time_ = current_time;
}

/// @brief ばね発射機構の現在状態を受信
void DribbleControllerNode::spring_operation_state_callback(
  const robot_msgs::msg::SpringOperationState::SharedPtr msg)
{
  const bool was_slow = spring_operation_state_ == robot_msgs::msg::SpringOperationState::SLOW_FIRE;
  spring_operation_state_ = msg->state;
  const bool is_slow = spring_operation_state_ == robot_msgs::msg::SpringOperationState::SLOW_FIRE;
  if (was_slow == is_slow || shot_cycle_active_) {
    return;
  }
  is_arm_moving_ = true;
  arm_move_start_time_ = now();
  arm_move_start_pos_rad_ = arm_cmd_pos_rad_;
  arm_move_start_rpm_ = roller_cmd_rpm_;
}
// ------------------------------------------------------------------------------------------------------------------
// callback関数終了
// ------------------------------------------------------------------------------------------------------------------

/// @brief ローラの回転数を求めて，送信
void DribbleControllerNode::update_and_publish_roller_command()
{
  const int target_rpm = roller_target_rpm();
  if (emergency_stop_active_) {
    roller_cmd_rpm_ = 0;
  } else if (!shot_cycle_active_ &&
    position_mode_ == robot_msgs::msg::ArmPosition::OPEN && target_rpm == 0)
  {
    // OPEN is the ball-release posture. Do not carry the previous dribble or
    // motion-compensated RPM through the arm trajectory.
    roller_cmd_rpm_ = 0;
  } else if (is_arm_moving_ && !shot_cycle_active_ && dribble_enabled_ &&
    spring_operation_state_ != robot_msgs::msg::SpringOperationState::SLOW_FIRE)
  {
    const double mode_target_rad = get_target_position_rad();
    const double max_vel_rad_s = get_arm_move_max_vel_rad_s();
    const double max_rad_s2 = get_arm_move_max_accel_rad_s2();

    const double elapsed_sec = (now() - arm_move_start_time_).seconds();
    const auto trajectory = sample_trapezoidal_trajectory(
      arm_move_start_pos_rad_, mode_target_rad,
      elapsed_sec, max_vel_rad_s, max_rad_s2);
    if (trajectory.duration_sec > 0.0) {
      const double progress =
        std::clamp(elapsed_sec / trajectory.duration_sec, 0.0, 1.0);
      const double smooth_progress =
        progress * progress * progress * progress *
        (35.0 + progress * (-84.0 + progress * (70.0 - 20.0 * progress)));
      roller_cmd_rpm_ = static_cast<int>(std::round(
          arm_move_start_rpm_ +
          (target_rpm - arm_move_start_rpm_) * smooth_progress));
    } else {
      roller_cmd_rpm_ = target_rpm;
    }
  } else {
    // Limit the RPM change on every control tick.
    if (roller_cmd_rpm_ < target_rpm) {
      roller_cmd_rpm_ = std::min(
        target_rpm, roller_cmd_rpm_ + kMaxRollerRpmStepPerTick);
    } else if (roller_cmd_rpm_ > target_rpm) {
      roller_cmd_rpm_ = std::max(
        target_rpm, roller_cmd_rpm_ - kMaxRollerRpmStepPerTick);
    }
  }
  actuator_msgs::msg::ActuatorTarget roller_command;
  roller_command.logical_id = roller_logical_id_;
  roller_command.target = static_cast<float>(roller_cmd_rpm_);
  roller_command_pub_->publish(roller_command);
}

/// @brief 台形制御の目標値の更新
/// @param target_rad 最終の目標値
/// @return 現在の目標値
double DribbleControllerNode::update_arm_move_trajectory(double target_rad)
{
  if (!is_arm_moving_ || shot_cycle_active_) {
    return target_rad;
  }

  const double mode_target_rad = get_target_position_rad();
  const double max_vel_rad_s = get_arm_move_max_vel_rad_s();
  const double max_rad_s2 = get_arm_move_max_accel_rad_s2();
  const double elapsed_sec = (now() - arm_move_start_time_).seconds();
  const auto trajectory = sample_trapezoidal_trajectory(
    arm_move_start_pos_rad_, mode_target_rad,
    elapsed_sec, max_vel_rad_s, max_rad_s2);
  target_rad = trajectory.position_rad;
  if (elapsed_sec >= trajectory.duration_sec) {
    is_arm_moving_ = false;
    target_rad = mode_target_rad;
  }
  return target_rad;
}

/// @brief armの角度を送る関数
/// @param target_rad 目標角度
void DribbleControllerNode::publish_arm_target(double target_rad)
{
  actuator_msgs::msg::ActuatorTarget target;
  target.logical_id = position_logical_id_;
  target.target = static_cast<float>(target_rad);
  arm_cmd_pos_rad_ = target_rad;
  position_command_pub_->publish(target);
}

// ────────────────────────────────────────────────────────────────────────────
// タイマーコールバック（ステートマシン進行 + publish）
// ────────────────────────────────────────────────────────────────────────────

void DribbleControllerNode::control_timer_callback()
{
  //今のシュートサイクルを取得し，変化があった場合送信
  const uint8_t current_state =
    shot_cycle_active_ ? shot_cycle_phase_ : robot_msgs::msg::ShotCycleState::IDLE;
  if (current_state != last_published_shot_cycle_state_) {
    robot_msgs::msg::ShotCycleState state;
    state.state = current_state;
    shot_cycle_state_pub_->publish(state);
    last_published_shot_cycle_state_ = current_state;
  }

  // 移動速度や加速度に応じてローラの回転速度を変化させる
  const bool cmd_vel_fresh = last_cmd_vel_time_.nanoseconds() > 0 &&
    (now() - last_cmd_vel_time_).seconds() <= cmd_vel_timeout_sec_;
  if (!enable_motion_compensation_ || emergency_stop_active_ || !cmd_vel_fresh) {
    current_motion_compensation_rpm_ = 0;
  } else {
    const double forward_speed = std::max(0.0, commanded_vx_m_s_);
    const double backward_speed = std::max(0.0, -commanded_vx_m_s_);
    const double backward_acc = std::max(0.0, -commanded_ax_m_s2_);
    const double turning_speed = std::abs(commanded_wz_rad_s_);
    const double raw_compensation =
      backward_speed * backward_velocity_boost_rpm_per_mps_ +
      backward_acc * backward_acc_rpm_per_mps2_ +
      turning_speed * turning_boost_rpm_per_rad_s_ -
      forward_speed * forward_velocity_reduction_rpm_per_mps_;
    current_motion_compensation_rpm_ = std::clamp(
      static_cast<int>(std::round(raw_compensation)), -max_reduction_rpm_, max_boost_rpm_);
  }
  update_and_publish_roller_command();

  // armのeduliteがREADY以外の状態は送信処理をせずに終了
  if (!is_arm_ready_) {
    return;
  }
  // 非常停止中は一定角度を送信
  if (emergency_stop_active_) {
    publish_arm_target(hold_arm_pos_rad_);
    return;
  }

  if (pre_shot_state_ == PreShotState::MOVING_TO_DRIBBLE && !is_arm_moving_) { // open姿勢からdribble姿勢に移動が完了した場合
    pre_shot_state_ = PreShotState::WAITING;
    pre_shot_wait_start_time_ = now();
  } else if (pre_shot_state_ == PreShotState::WAITING &&
    (now() - pre_shot_wait_start_time_).seconds() >= prepare_from_open_delay_sec_)
  {// 所定時間待ち終わった場合
    pre_shot_state_ = PreShotState::IDLE;
    start_shot_cycle();
  }

  //　最終的な目標値を取得して，台形制御の計算を始める
  double position_command_rad = get_target_position_rad();
  position_command_rad = update_arm_move_trajectory(position_command_rad);

  // ベルト発射のサイクルが動いている場合
  if (shot_cycle_active_) {
    if (belt_auto_started_) {
      // ベルトがまだ回っていない場合
      robot_msgs::msg::BeltMode belt_msg;
      belt_msg.mode = static_cast<uint8_t>(shot_cycle_belt_spinup_level_);
      belt_mode_pub_->publish(belt_msg);
    }
    if (shot_cycle_phase_ == robot_msgs::msg::ShotCycleState::BELT_SPINUP) {
      // ベルトの回転が上がるのを一定時間待ち，ばねを退避して，退避完了したら一定時間待つ
      const bool spring_retracted = spring_operation_state_ ==
        robot_msgs::msg::SpringOperationState::BELT_CLEARANCE;
      const double elapsed_sec = (now() - shot_phase_start_time_).seconds();
      if (!spring_retracted && elapsed_sec >= belt_clearance_timeout_sec_) {
        // ばね退避に時間がかかりすぎたときのタイムアウト
        shot_cycle_active_ = false;
        set_spring_clearance(false);
        position_mode_ = pre_shot_saved_position_mode_;
        if (position_mode_ != robot_msgs::msg::ArmPosition::DRIBBLE) {
          is_arm_moving_ = true;
          arm_move_start_time_ = now();
          arm_move_start_pos_rad_ = arm_cmd_pos_rad_;
          arm_move_start_rpm_ = roller_cmd_rpm_;
        }
        if (belt_auto_started_) {
          belt_auto_started_ = false;
          robot_msgs::msg::BeltMode belt_msg;
          belt_msg.mode = robot_msgs::msg::BeltMode::STOP;
          belt_mode_pub_->publish(belt_msg);
        }
        RCLCPP_ERROR(
          get_logger(),
          "Shot Cycle aborted: spring clearance timed out after %.3f s",
          elapsed_sec);
      } else if (spring_retracted && (!belt_auto_started_ || elapsed_sec >= belt_shot_delay_sec_)) {
        shot_cycle_phase_ = robot_msgs::msg::ShotCycleState::FEEDING;
        shot_phase_start_time_ = now();
        shot_phase_start_pos_rad_ = arm_cmd_pos_rad_;
        position_mode_ = robot_msgs::msg::ArmPosition::FEED;
        RCLCPP_INFO(
          get_logger(),
          "Shot Cycle: spring retracted and %.3f s roller delay elapsed; "
          "starting FEED",
          elapsed_sec);
      }
    } else {
      // ベルトの回転時間待ち以降の処理
      const bool is_feeding = shot_cycle_phase_ == robot_msgs::msg::ShotCycleState::FEEDING;
      position_mode_ =
        is_feeding ? robot_msgs::msg::ArmPosition::FEED : robot_msgs::msg::ArmPosition::DRIBBLE;
      const double phase_target_rad = is_feeding ? feed_pos_rad_ : dribble_pos_rad_;
      const double phase_max_vel_rad_s = is_feeding ? feeding_max_rad_s_ : returning_max_rad_s_;
      const double phase_max_rad_s2 = is_feeding ? feeding_max_rad_s2_ : returning_max_rad_s2_;
      const double hold_duration_sec = is_feeding ? feed_duration_sec_ : 0.0;

      const double elapsed_sec = (now() - shot_phase_start_time_).seconds();
      const auto trajectory = sample_trapezoidal_trajectory(
        shot_phase_start_pos_rad_, phase_target_rad, elapsed_sec,
        phase_max_vel_rad_s, phase_max_rad_s2);
      position_command_rad = trajectory.position_rad;

      if (elapsed_sec >= trajectory.duration_sec + hold_duration_sec) {
        position_command_rad = phase_target_rad;
        shot_phase_start_pos_rad_ = phase_target_rad;
        shot_phase_start_time_ = now();

        if (is_feeding) {
          shot_cycle_phase_ = robot_msgs::msg::ShotCycleState::RETURNING;
          RCLCPP_INFO(get_logger(), "Shot Cycle: FEED -> RETURNING");
        } else {
          shot_cycle_active_ = false;
          set_spring_clearance(false);
          has_ball_ = false;
          ball_detected_counter_ = 0;
          ball_lost_counter_ = ball_detection_debounce_count_;
          position_mode_ = pre_shot_saved_position_mode_;
          if (position_mode_ != robot_msgs::msg::ArmPosition::DRIBBLE) {
            is_arm_moving_ = true;
            arm_move_start_time_ = now();
            arm_move_start_pos_rad_ = arm_cmd_pos_rad_;
            arm_move_start_rpm_ = roller_cmd_rpm_;
            RCLCPP_INFO(
              get_logger(),
              "Shot Cycle Completed: Restoring arm position to PRE-SHOT mode (%u)",
              position_mode_);
          } else {
            RCLCPP_INFO(
              get_logger(),
              "Shot Cycle Completed: Returned to DRIBBLE");
          }
          if (belt_auto_started_) {
            belt_auto_started_ = false;
            robot_msgs::msg::BeltMode belt_msg;
            belt_msg.mode = robot_msgs::msg::BeltMode::STOP;
            belt_mode_pub_->publish(belt_msg);
            RCLCPP_INFO(
              get_logger(),
              "Shot Cycle Completed: Belt auto-stopped");
          }
        }
      }
    }
  }

  publish_arm_target(position_command_rad);
}

// -------------------------------------------------------------------------------------------------//
// ローラの回転数を計算する関数
int DribbleControllerNode::roller_target_rpm() const
{
  if (emergency_stop_active_) {
    return 0;
  }
  if (spring_operation_state_ == robot_msgs::msg::SpringOperationState::SLOW_FIRE) {
    return slow_fire_dribble_rpm_;
  }
  if (shot_cycle_active_) {
    switch (shot_cycle_phase_) {
      case robot_msgs::msg::ShotCycleState::BELT_SPINUP:
        return shot_cycle_opening_rpm_;
      case robot_msgs::msg::ShotCycleState::FEEDING:
        return shot_cycle_feeding_rpm_;
      case robot_msgs::msg::ShotCycleState::RETURNING:
        return shot_cycle_returning_rpm_;
    }
  }
  if (!dribble_enabled_) {
    // 通常 OFF 時（何もしていない時）：当然 0 RPM（完全停止）
    return 0;
  }
  // ボールを出すとき (OPEN姿勢) はローラー回転を 0 RPM にする
  if (position_mode_ == robot_msgs::msg::ArmPosition::OPEN) {
    return 0;
  }
  int base_rpm = dribble_on_rpm_;
  // 前進時は減速し、後退・急減速・旋回時は加速する。
  if (enable_motion_compensation_) {
    return std::clamp(base_rpm + current_motion_compensation_rpm_, 0, 4000);
  }
  return base_rpm;
}
/// @brief 動かす際の目標の角度をセットする関数
/// @return 今回の動きでの目標角度
double DribbleControllerNode::get_target_position_rad() const
{
  if (spring_operation_state_ == robot_msgs::msg::SpringOperationState::SLOW_FIRE &&
    !shot_cycle_active_)
  {
    return slow_fire_dribble_position_rad_;
  }
  switch (position_mode_) {
    case robot_msgs::msg::ArmPosition::OPEN:
      return open_position_rad_;
    case robot_msgs::msg::ArmPosition::HOME:
      return home_position_rad_;
    case robot_msgs::msg::ArmPosition::FEED:
      return feed_pos_rad_;
    case robot_msgs::msg::ArmPosition::DRIBBLE:
    default:
      return dribble_pos_rad_;
  }
}
/// @brief 動かす際の角速度をセットする関数
/// @return 今回の動きでの角速度
double DribbleControllerNode::get_arm_move_max_vel_rad_s() const
{
  switch (position_mode_) {
    case robot_msgs::msg::ArmPosition::OPEN:
      return opening_max_rad_s_;
    case robot_msgs::msg::ArmPosition::FEED:
      return feeding_max_rad_s_;
    case robot_msgs::msg::ArmPosition::DRIBBLE:
      return dribbling_max_rad_s_;
    case robot_msgs::msg::ArmPosition::HOME:
      return returning_max_rad_s_;
    default:
      return returning_max_rad_s_;
  }
}
/// @brief 動かす際の角加速度をセットする関数
/// @return 今回の動きの角加速度
double DribbleControllerNode::get_arm_move_max_accel_rad_s2() const
{
  if (spring_operation_state_ == robot_msgs::msg::SpringOperationState::SLOW_FIRE) {
    return dribbling_max_rad_s2_;
  }
  switch (position_mode_) {
    case robot_msgs::msg::ArmPosition::OPEN:
      return opening_max_rad_s2_;
    case robot_msgs::msg::ArmPosition::FEED:
      return feeding_max_rad_s2_;
    case robot_msgs::msg::ArmPosition::DRIBBLE:
    case robot_msgs::msg::ArmPosition::HOME:
    default:
      return dribbling_max_rad_s2_;
  }
}
/// @brief 台形制御をする関数
/// @param start_rad 開始角度
/// @param target_rad 目標角度
/// @param elapsed_sec 移動開始からの経過時間
/// @param max_rad_s 最大角速度
/// @param max_rad_s2 最大角加速度
/// @return 現在の目標角度と移動完了までの総時間
DribbleControllerNode::TrajectorySample DribbleControllerNode::sample_trapezoidal_trajectory(
  double start_rad, double target_rad,
  double elapsed_sec,
  double max_rad_s,
  double max_rad_s2) const
{
  const double distance = std::abs(target_rad - start_rad);
  if (distance <= 1e-9) {
    return {target_rad, 0.0};
  }
  const double acc = std::max(1e-6, max_rad_s2);
  const double velocity = std::max(1e-6, max_rad_s);
  const double acc_time = std::min(velocity / acc, std::sqrt(distance / acc));
  const double peak_velocity = acc * acc_time;
  const double acc_distance = 0.5 * acc * acc_time * acc_time;
  const double cruise_time = std::max(0.0, (distance - 2.0 * acc_distance) / peak_velocity);
  const double duration = 2.0 * acc_time + cruise_time;
  const double time = std::clamp(elapsed_sec, 0.0, duration);

  double traveled = 0.0;
  if (time < acc_time) {
    traveled = 0.5 * acc * time * time;
  } else if (time < acc_time + cruise_time) {
    traveled = acc_distance + peak_velocity * (time - acc_time);
  } else {
    const double remaining = duration - time;
    traveled = distance - 0.5 * acc * remaining * remaining;
  }
  return {start_rad + std::copysign(traveled, target_rad - start_rad), duration};
}
/// @brief 台形制御の計算をリセットする関数
void DribbleControllerNode::restart_active_trajectory()
{
  // スタートの時間のリセットとスタートを現在地に変更
  const auto current_time = now();
  if (is_arm_moving_) {
    arm_move_start_time_ = current_time;
    arm_move_start_pos_rad_ = arm_cmd_pos_rad_;
  }
  if (shot_cycle_active_) {
    shot_phase_start_time_ = current_time;
    shot_phase_start_pos_rad_ = arm_cmd_pos_rad_;
  }
}

void DribbleControllerNode::load_parameters()
{
  // アーム位置
  dribble_pos_rad_ = declare_parameter("dribble_position_rad", dribble_pos_rad_);
  open_position_rad_ = declare_parameter("open_position_rad", open_position_rad_);
  home_position_rad_ = declare_parameter("home_position_rad", home_position_rad_);
  bottom_pos_rad_ = declare_parameter("bottom_position_rad", bottom_pos_rad_);
  feed_pos_rad_ = declare_parameter("feed_position_rad", feed_pos_rad_);
  slow_fire_dribble_position_rad_ = declare_parameter(
    "slow_fire_dribble_position_rad", slow_fire_dribble_position_rad_);
  // ショットサイクル
  feed_duration_sec_ = declare_parameter("feed_duration_sec", feed_duration_sec_);
  belt_shot_delay_sec_ = declare_parameter("belt_shot_delay_sec", belt_shot_delay_sec_);
  belt_clearance_timeout_sec_ = declare_parameter(
    "belt_clearance_timeout_sec", belt_clearance_timeout_sec_);
  prepare_from_open_delay_sec_ = declare_parameter(
    "prepare_from_open_delay_sec", prepare_from_open_delay_sec_);
  // アーム速度・加速度
  opening_max_rad_s_ = declare_parameter(
    "opening_max_rad_s", opening_max_rad_s_);
  feeding_max_rad_s_ = declare_parameter(
    "feeding_max_rad_s", feeding_max_rad_s_);
  returning_max_rad_s_ = declare_parameter(
    "returning_max_rad_s", returning_max_rad_s_);
  dribbling_max_rad_s_ = declare_parameter(
    "dribbling_max_rad_s", dribbling_max_rad_s_);
  opening_max_rad_s2_ = declare_parameter(
    "opening_max_rad_s2", opening_max_rad_s2_);
  feeding_max_rad_s2_ = declare_parameter(
    "feeding_max_rad_s2", feeding_max_rad_s2_);
  returning_max_rad_s2_ = declare_parameter(
    "returning_max_rad_s2", returning_max_rad_s2_);
  dribbling_max_rad_s2_ = declare_parameter(
    "dribbling_max_rad_s2", dribbling_max_rad_s2_);
  // ローラー・ボール検出
  dribble_on_rpm_ = declare_parameter("dribble_on_rpm", dribble_on_rpm_);
  slow_fire_dribble_rpm_ = declare_parameter(
    "slow_fire_dribble_rpm", slow_fire_dribble_rpm_);
  shot_cycle_opening_rpm_ = declare_parameter(
    "shot_cycle_opening_rpm", shot_cycle_opening_rpm_);
  shot_cycle_feeding_rpm_ = declare_parameter(
    "shot_cycle_feeding_rpm", shot_cycle_feeding_rpm_);
  shot_cycle_returning_rpm_ = declare_parameter(
    "shot_cycle_returning_rpm", shot_cycle_returning_rpm_);
  shot_cycle_belt_spinup_level_ = declare_parameter(
    "shot_cycle_belt_spinup_level", shot_cycle_belt_spinup_level_);
  ball_detection_threshold_a_ = declare_parameter(
    "ball_detection_threshold_a", ball_detection_threshold_a_);
  ball_lost_threshold_a_ = declare_parameter(
    "ball_lost_threshold_a", ball_lost_threshold_a_);
  current_lpf_alpha_ = declare_parameter("current_lpf_alpha", current_lpf_alpha_);
  ball_detection_debounce_count_ = declare_parameter(
    "ball_detection_debounce_count", ball_detection_debounce_count_);
  ball_lost_debounce_count_ = declare_parameter(
    "ball_lost_debounce_count", ball_lost_debounce_count_);
  // 走行中のドリブル補正
  enable_motion_compensation_ = declare_parameter(
    "enable_motion_compensation", enable_motion_compensation_);
  forward_velocity_reduction_rpm_per_mps_ = declare_parameter(
    "forward_velocity_reduction_rpm_per_mps", forward_velocity_reduction_rpm_per_mps_);
  backward_velocity_boost_rpm_per_mps_ = declare_parameter(
    "backward_velocity_boost_rpm_per_mps", backward_velocity_boost_rpm_per_mps_);
  backward_acc_rpm_per_mps2_ = declare_parameter(
    "backward_acc_rpm_per_mps2", backward_acc_rpm_per_mps2_);
  turning_boost_rpm_per_rad_s_ = declare_parameter(
    "turning_boost_rpm_per_rad_s", turning_boost_rpm_per_rad_s_);
  cmd_vel_acc_lpf_alpha_ = declare_parameter(
    "cmd_vel_acc_lpf_alpha", cmd_vel_acc_lpf_alpha_);
  cmd_vel_timeout_sec_ = declare_parameter("cmd_vel_timeout_sec", cmd_vel_timeout_sec_);
  max_boost_rpm_ = declare_parameter("max_boost_rpm", max_boost_rpm_);
  max_reduction_rpm_ = declare_parameter("max_reduction_rpm", max_reduction_rpm_);
  cmd_vel_topic_ = declare_parameter("cmd_vel_topic", cmd_vel_topic_);

  const auto load_id = [this](const char * name, uint16_t default_value) {
      const int value = declare_parameter<int>(name, default_value);
      if (value < 0 || value > 65535) {
        throw std::runtime_error(std::string(name) + " must be in [0, 65535]");
      }
      return static_cast<uint16_t>(value);
    };
  position_logical_id_ = load_id("position_logical_id", position_logical_id_);
  roller_logical_id_ = load_id("roller_logical_id", roller_logical_id_);
  upper_belt_logical_id_ = load_id("upper_belt_logical_id", upper_belt_logical_id_);
  under_belt_logical_id_ = load_id("under_belt_logical_id", under_belt_logical_id_);

  position_mode_ = robot_msgs::msg::ArmPosition::DRIBBLE;
  arm_cmd_pos_rad_ = dribble_pos_rad_;
}

rcl_interfaces::msg::SetParametersResult DribbleControllerNode::parameter_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  bool trajectory_changed = false;

  const auto reject = [&result](const std::string & reason) {
      result.successful = false;
      result.reason = reason;
    };
  const auto update_double = [&reject](
    const rclcpp::Parameter & parameter, double & target,
    double minimum, double maximum) {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        reject(parameter.get_name() + " must be a double");
        return false;
      }
      const double value = parameter.as_double();
      if (!std::isfinite(value) || value < minimum || value > maximum) {
        reject(parameter.get_name() + " is outside its valid range");
        return false;
      }
      target = value;
      return true;
    };
  const auto update_int = [&reject](
    const rclcpp::Parameter & parameter, int & target,
    int minimum, int maximum) {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
        reject(parameter.get_name() + " must be an integer");
        return false;
      }
      const auto value = parameter.as_int();
      if (value < minimum || value > maximum) {
        reject(parameter.get_name() + " is outside its valid range");
        return false;
      }
      target = static_cast<int>(value);
      return true;
    };

  constexpr double any_double_min = -std::numeric_limits<double>::max();
  constexpr double any_double_max = std::numeric_limits<double>::max();
  constexpr double positive_min = std::numeric_limits<double>::min();
  constexpr int any_int_min = std::numeric_limits<int>::min();
  constexpr int any_int_max = std::numeric_limits<int>::max();

  for (const auto & parameter : parameters) {
    const auto & name = parameter.get_name();
    if (name == "command_period_ms" ||
      name == "position_logical_id" || name == "roller_logical_id" ||
      name == "upper_belt_logical_id" || name == "under_belt_logical_id" ||
      name == "position_target_topic" || name == "roller_target_topic" ||
      name == "cmd_vel_topic")
    {
      reject(name + " requires a node restart");
      return result;
    }

    bool updated = true;
    bool affects_trajectory = false;
    if (name == "dribble_position_rad") {
      updated = update_double(parameter, dribble_pos_rad_, any_double_min, any_double_max);
      affects_trajectory = true;
    } else if (name == "open_position_rad") {
      updated = update_double(parameter, open_position_rad_, any_double_min, any_double_max);
      affects_trajectory = true;
    } else if (name == "home_position_rad") {
      updated = update_double(parameter, home_position_rad_, any_double_min, any_double_max);
      affects_trajectory = true;
    } else if (name == "bottom_position_rad") {
      updated = update_double(parameter, bottom_pos_rad_, any_double_min, any_double_max);
      affects_trajectory = true;
    } else if (name == "feed_position_rad") {
      updated = update_double(parameter, feed_pos_rad_, any_double_min, any_double_max);
      affects_trajectory = true;
    } else if (name == "slow_fire_dribble_position_rad") {
      updated = update_double(
        parameter, slow_fire_dribble_position_rad_, any_double_min,
        any_double_max);
      affects_trajectory = true;
    } else if (name == "feed_duration_sec") {
      updated = update_double(parameter, feed_duration_sec_, 0.0, any_double_max);
      affects_trajectory = true;
    } else if (name == "belt_shot_delay_sec") {
      updated = update_double(parameter, belt_shot_delay_sec_, 0.0, any_double_max);
    } else if (name == "belt_clearance_timeout_sec") {
      updated = update_double(parameter, belt_clearance_timeout_sec_, positive_min, any_double_max);
    } else if (name == "prepare_from_open_delay_sec") {
      updated = update_double(parameter, prepare_from_open_delay_sec_, 0.0, any_double_max);
    } else if (name == "opening_max_rad_s") {
      updated = update_double(parameter, opening_max_rad_s_, positive_min, any_double_max);
      affects_trajectory = true;
    } else if (name == "feeding_max_rad_s") {
      updated = update_double(parameter, feeding_max_rad_s_, positive_min, any_double_max);
      affects_trajectory = true;
    } else if (name == "returning_max_rad_s") {
      updated = update_double(parameter, returning_max_rad_s_, positive_min, any_double_max);
      affects_trajectory = true;
    } else if (name == "dribbling_max_rad_s") {
      updated = update_double(parameter, dribbling_max_rad_s_, positive_min, any_double_max);
      affects_trajectory = true;
    } else if (name == "opening_max_rad_s2") {
      updated = update_double(parameter, opening_max_rad_s2_, positive_min, any_double_max);
      affects_trajectory = true;
    } else if (name == "feeding_max_rad_s2") {
      updated = update_double(parameter, feeding_max_rad_s2_, positive_min, any_double_max);
      affects_trajectory = true;
    } else if (name == "returning_max_rad_s2") {
      updated = update_double(parameter, returning_max_rad_s2_, positive_min, any_double_max);
      affects_trajectory = true;
    } else if (name == "dribbling_max_rad_s2") {
      updated = update_double(parameter, dribbling_max_rad_s2_, positive_min, any_double_max);
      affects_trajectory = true;
    } else if (name == "ball_detection_threshold_a") {
      updated = update_double(parameter, ball_detection_threshold_a_, 0.0, any_double_max);
    } else if (name == "ball_lost_threshold_a") {
      updated = update_double(parameter, ball_lost_threshold_a_, 0.0, any_double_max);
    } else if (name == "current_lpf_alpha") {
      updated = update_double(parameter, current_lpf_alpha_, positive_min, 1.0);
    } else if (name == "cmd_vel_timeout_sec") {
      updated = update_double(parameter, cmd_vel_timeout_sec_, positive_min, any_double_max);
    } else if (name == "cmd_vel_acc_lpf_alpha") {
      updated = update_double(parameter, cmd_vel_acc_lpf_alpha_, positive_min, 1.0);
    } else if (name == "forward_velocity_reduction_rpm_per_mps") {
      updated = update_double(
        parameter, forward_velocity_reduction_rpm_per_mps_, 0.0, any_double_max);
    } else if (name == "backward_velocity_boost_rpm_per_mps") {
      updated = update_double(parameter, backward_velocity_boost_rpm_per_mps_, 0.0, any_double_max);
    } else if (name == "backward_acc_rpm_per_mps2") {
      updated = update_double(parameter, backward_acc_rpm_per_mps2_, 0.0, any_double_max);
    } else if (name == "turning_boost_rpm_per_rad_s") {
      updated = update_double(parameter, turning_boost_rpm_per_rad_s_, 0.0, any_double_max);
    } else if (name == "dribble_on_rpm") {
      updated = update_int(parameter, dribble_on_rpm_, 0, any_int_max);
    } else if (name == "slow_fire_dribble_rpm") {
      updated = update_int(parameter, slow_fire_dribble_rpm_, any_int_min, any_int_max);
    } else if (name == "shot_cycle_opening_rpm") {
      updated = update_int(parameter, shot_cycle_opening_rpm_, 0, any_int_max);
    } else if (name == "shot_cycle_feeding_rpm") {
      updated = update_int(parameter, shot_cycle_feeding_rpm_, 0, any_int_max);
    } else if (name == "shot_cycle_returning_rpm") {
      updated = update_int(parameter, shot_cycle_returning_rpm_, 0, any_int_max);
    } else if (name == "shot_cycle_belt_spinup_level") {
      updated = update_int(parameter, shot_cycle_belt_spinup_level_, 1, 4);
    } else if (name == "max_boost_rpm") {
      updated = update_int(parameter, max_boost_rpm_, 0, any_int_max);
    } else if (name == "max_reduction_rpm") {
      updated = update_int(parameter, max_reduction_rpm_, 0, any_int_max);
    } else if (name == "ball_detection_debounce_count") {
      updated = update_int(parameter, ball_detection_debounce_count_, 1, any_int_max);
    } else if (name == "ball_lost_debounce_count") {
      updated = update_int(parameter, ball_lost_debounce_count_, 1, any_int_max);
    } else if (name == "enable_motion_compensation") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
        reject(name + " must be a boolean");
        updated = false;
      } else {
        enable_motion_compensation_ = parameter.as_bool();
      }
    } else {
      continue;
    }

    if (!updated) {
      return result;
    }
    trajectory_changed = trajectory_changed || affects_trajectory;
  }

  if (trajectory_changed) {
    restart_active_trajectory();
  }
  return result;
}
