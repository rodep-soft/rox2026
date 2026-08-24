#include <gtest/gtest.h>
#include <chrono>
#include <memory>

#include "actuator_msgs/msg/actuator_target.hpp"
#include "actuator_msgs/msg/actuator_target_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "belt_controller/belt_controller.hpp"
#include "dribble_controller/dribble_controller.hpp"
#include "mecanum_controller/mecanum_controller_node.hpp"
#include "robot_msgs/msg/arm_position.hpp"
#include "robot_msgs/msg/belt_mode.hpp"
#include "robot_msgs/msg/spring_operation_state.hpp"
#include "spring_controller/spring_edulite_controller.hpp"

class RobotControllerTest : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }
};

TEST_F(RobotControllerTest, BeltControllerLevelAndEmergencyStopTest)
{
  auto belt_node = std::make_shared<BeltControllerNode>();
  auto test_node = std::make_shared<rclcpp::Node>("test_belt_client");

  float last_underbelt_rpm = -1.0f;
  float last_upperbelt_rpm = -1.0f;
  int received_command_count = 0;
  auto target_array_sub =
    test_node->create_subscription<actuator_msgs::msg::ActuatorTargetArray>(
    "/vesc/target_array", 1,
    [&last_underbelt_rpm, &last_upperbelt_rpm, &received_command_count](
      const actuator_msgs::msg::ActuatorTargetArray::SharedPtr msg) {
      for (const auto & target : msg->actuators) {
        if (target.logical_id == 11) {
          last_underbelt_rpm = target.target;
        } else if (target.logical_id == 10) {
          last_upperbelt_rpm = target.target;
        }
      }
      ++received_command_count;
    });

  auto pub_belt_mode = test_node->create_publisher<std_msgs::msg::UInt8>("/belt/mode", 1);
  auto pub_estop = test_node->create_publisher<std_msgs::msg::Bool>(
    "/emergency_stop", rclcpp::QoS(1).reliable().transient_local());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(belt_node);
  executor.add_node(test_node);

  const auto rpm_update = belt_node->set_parameters_atomically(
  {
    rclcpp::Parameter("underbelt_level_3_rpm", 4100),
    rclcpp::Parameter("upperbelt_level_3_rpm", 3900)});
  ASSERT_TRUE(rpm_update.successful);
  EXPECT_FALSE(
    belt_node->set_parameter(
      rclcpp::Parameter("underbelt_level_3_rpm", -1)).successful);
  EXPECT_FALSE(
    belt_node->set_parameter(
      rclcpp::Parameter("qos_depth", 2)).successful);
  EXPECT_TRUE(
    belt_node->set_parameter(
      rclcpp::Parameter("qos_depth", 1)).successful);

  // Level 3を受信した時点で、上下個別のRPMを即時送信する。
  std_msgs::msg::UInt8 mode_msg;
  mode_msg.data = 3;
  pub_belt_mode->publish(mode_msg);

  auto start = std::chrono::steady_clock::now();
  while ((last_underbelt_rpm != 4100 || last_upperbelt_rpm != 3900) &&
    std::chrono::steady_clock::now() - start < std::chrono::seconds(2))
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(last_underbelt_rpm, 4100);
  EXPECT_EQ(last_upperbelt_rpm, 3900);

  // 通常時は周期送信しない。
  const int count_during_normal_operation = received_command_count;
  start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(120)) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  EXPECT_EQ(received_command_count, count_during_normal_operation);

  // 選択中レベルのparameter変更は、新しい上下RPMを即時送信する。
  const auto active_level_update = belt_node->set_parameters_atomically(
  {
    rclcpp::Parameter("underbelt_level_3_rpm", 4200),
    rclcpp::Parameter("upperbelt_level_3_rpm", 3800)});
  ASSERT_TRUE(active_level_update.successful);
  start = std::chrono::steady_clock::now();
  while ((last_underbelt_rpm != 4200 || last_upperbelt_rpm != 3800) &&
    std::chrono::steady_clock::now() - start < std::chrono::seconds(2))
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(last_underbelt_rpm, 4200);
  EXPECT_EQ(last_upperbelt_rpm, 3800);

  // 非常停止時は即時にゼロを送り、その後もタイマーでゼロを送り続ける。
  std_msgs::msg::Bool estop_msg;
  estop_msg.data = true;
  pub_estop->publish(estop_msg);

  start = std::chrono::steady_clock::now();
  while ((last_underbelt_rpm != 0 || last_upperbelt_rpm != 0) &&
    std::chrono::steady_clock::now() - start < std::chrono::seconds(2))
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(last_underbelt_rpm, 0);
  EXPECT_EQ(last_upperbelt_rpm, 0);

  const int count_at_emergency_stop = received_command_count;
  start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(120)) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  EXPECT_GE(received_command_count - count_at_emergency_stop, 2);

  // 非常停止解除時は、選択中レベルの指令を即時再送する。
  estop_msg.data = false;
  pub_estop->publish(estop_msg);
  start = std::chrono::steady_clock::now();
  while ((last_underbelt_rpm != 4200 || last_upperbelt_rpm != 3800) &&
    std::chrono::steady_clock::now() - start < std::chrono::seconds(2))
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(last_underbelt_rpm, 4200);
  EXPECT_EQ(last_upperbelt_rpm, 3800);
}

TEST_F(RobotControllerTest, SpringControllerReadyFireAndEmergencyStopTest)
{
  auto spring_node = std::make_shared<SpringEduliteController>();
  auto test_node = std::make_shared<rclcpp::Node>("test_spring_client");

  float last_position_rad = 0.0f;
  int received_command_count = 0;
  auto target_sub = test_node->create_subscription<actuator_msgs::msg::ActuatorTarget>(
    "/edulite/target", 1,
    [&last_position_rad, &received_command_count](
      const actuator_msgs::msg::ActuatorTarget::SharedPtr msg) {
      if (msg->logical_id == 4) {
        last_position_rad = msg->target;
        ++received_command_count;
      }
    });

  auto state_pub = test_node->create_publisher<actuator_msgs::msg::ActuatorState>(
    "/edulite/state", 1);
  auto fire_pub = test_node->create_publisher<std_msgs::msg::Bool>(
    "/spring/fire_request", 1);
  auto emergency_stop_pub = test_node->create_publisher<std_msgs::msg::Bool>(
    "/emergency_stop", rclcpp::QoS(1).reliable().transient_local());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(spring_node);
  executor.add_node(test_node);

  actuator_msgs::msg::ActuatorState state_msg;
  state_msg.logical_id = 4;
  state_msg.state = actuator_msgs::msg::ActuatorState::STATE_READY;
  state_msg.position_reference_set = true;
  state_pub->publish(state_msg);

  auto start = std::chrono::steady_clock::now();
  while (received_command_count == 0 &&
    std::chrono::steady_clock::now() - start < std::chrono::seconds(2))
  {
    state_pub->publish(state_msg);
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  std_msgs::msg::Bool fire_msg;
  fire_msg.data = true;
  fire_pub->publish(fire_msg);
  start = std::chrono::steady_clock::now();
  while (std::abs(last_position_rad - (-6.283185307f)) > 0.01f &&
    std::chrono::steady_clock::now() - start < std::chrono::seconds(2))
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_NEAR(last_position_rad, -6.283185307f, 0.01f);

  fire_msg.data = false;
  fire_pub->publish(fire_msg);
  std_msgs::msg::Bool emergency_stop_msg;
  emergency_stop_msg.data = true;
  emergency_stop_pub->publish(emergency_stop_msg);
  executor.spin_some();

  const float position_before_rejected_fire = last_position_rad;
  fire_msg.data = true;
  fire_pub->publish(fire_msg);
  start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(100)) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  EXPECT_NEAR(last_position_rad, position_before_rejected_fire, 0.001f);
}

TEST_F(RobotControllerTest, DribbleControllerEnableAndEmergencyStopTest)
{
  auto dribble_node = std::make_shared<DribbleControllerNode>();
  auto test_node = std::make_shared<rclcpp::Node>("test_dribble_client");

  float last_dribble_rpm = -1.0f;
  auto target_sub = test_node->create_subscription<actuator_msgs::msg::ActuatorTarget>(
    "/vesc/target", 1,
    [&last_dribble_rpm](const actuator_msgs::msg::ActuatorTarget::SharedPtr msg) {
      if (msg->logical_id == 12) {
        last_dribble_rpm = msg->target;
      }
    });

  auto pub_dribble_enable = test_node->create_publisher<std_msgs::msg::Bool>("/dribble/enabled", 1);
  auto pub_estop = test_node->create_publisher<std_msgs::msg::Bool>(
    "/emergency_stop", rclcpp::QoS(1).reliable().transient_local());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(dribble_node);
  executor.add_node(test_node);

  const auto rpm_update = dribble_node->set_parameter(rclcpp::Parameter("dribble_on_rpm", 900));
  ASSERT_TRUE(rpm_update.successful);
  const auto slow_fire_rpm_update = dribble_node->set_parameter(
      rclcpp::Parameter("slow_fire_dribble_rpm", -750));
  ASSERT_TRUE(slow_fire_rpm_update.successful);
  EXPECT_FALSE(
      dribble_node->set_parameter(rclcpp::Parameter("dribble_on_rpm", -1))
          .successful);
  EXPECT_FALSE(
    dribble_node->set_parameter(rclcpp::Parameter("qos_depth", 2)).successful);
  EXPECT_TRUE(
    dribble_node->set_parameter(rclcpp::Parameter("qos_depth", 1)).successful);

  // 1. runtime parameterで変更した900 RPMを出力する。
  std_msgs::msg::Bool enable_msg;
  enable_msg.data = true;
  pub_dribble_enable->publish(enable_msg);

  auto start = std::chrono::steady_clock::now();
  while (last_dribble_rpm != 900 &&
    std::chrono::steady_clock::now() - start < std::chrono::seconds(2))
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(last_dribble_rpm, 900);

  // 2. 非常停止で 0 RPM に落ちるかのテスト
  std_msgs::msg::Bool estop_msg;
  estop_msg.data = true;
  pub_estop->publish(estop_msg);

  start = std::chrono::steady_clock::now();
  while (last_dribble_rpm != 0 &&
    std::chrono::steady_clock::now() - start < std::chrono::seconds(2))
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(last_dribble_rpm, 0);
}

TEST_F(RobotControllerTest, DribbleControllerPositionSequenceTest)
{
  auto dribble_node = std::make_shared<DribbleControllerNode>();
  auto test_node = std::make_shared<rclcpp::Node>("test_dribble_position_client");

  float last_position_rad = 999.0f;
  auto position_sub = test_node->create_subscription<actuator_msgs::msg::ActuatorTarget>(
    "/edulite/target", 1,
    [&last_position_rad](const actuator_msgs::msg::ActuatorTarget::SharedPtr msg) {
      if (msg->logical_id == 5) {
        last_position_rad = msg->target;
      }
    });

  auto position_mode_pub = test_node->create_publisher<robot_msgs::msg::ArmPosition>(
    "/dribble/command_position", 1);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(dribble_node);
  executor.add_node(test_node);

  // 状態0: 初期姿勢 DRIBBLE (-0.86 rad)
  auto start = std::chrono::steady_clock::now();
  while (std::abs(last_position_rad - (-0.86f)) > 0.01f &&
    std::chrono::steady_clock::now() - start < std::chrono::seconds(2))
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_NEAR(last_position_rad, -0.86f, 0.01f);

  const auto position_update = dribble_node->set_parameters_atomically(
  {
    rclcpp::Parameter("open_position_rad", -0.5),
    rclcpp::Parameter("opening_max_velocity_rad_s", 10.0)});
  ASSERT_TRUE(position_update.successful);

  // 状態1: runtime parameterで変更したOPEN位置 (-0.5 rad) へ遷移
  robot_msgs::msg::ArmPosition mode_msg;
  mode_msg.position = robot_msgs::msg::ArmPosition::OPEN;
  position_mode_pub->publish(mode_msg);

  start = std::chrono::steady_clock::now();
  while (std::abs(last_position_rad - (-0.5f)) > 0.01f &&
    std::chrono::steady_clock::now() - start < std::chrono::seconds(2))
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_NEAR(last_position_rad, -0.5f, 0.01f);

  // 状態2: FEED 位置 (1.3 rad) へ遷移
  mode_msg.position = robot_msgs::msg::ArmPosition::FEED;
  position_mode_pub->publish(mode_msg);

  start = std::chrono::steady_clock::now();
  while (std::abs(last_position_rad - 1.3f) > 0.01f &&
    std::chrono::steady_clock::now() - start < std::chrono::seconds(2))
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_NEAR(last_position_rad, 1.3f, 0.01f);
}

TEST_F(RobotControllerTest, DribbleControllerShotCycleDelayTest) {
  auto dribble_node = std::make_shared<DribbleControllerNode>();
  auto test_node =
      std::make_shared<rclcpp::Node>("test_dribble_shot_timeout_client");

  float last_position_rad = 999.0f;
  auto position_sub = test_node->create_subscription<actuator_msgs::msg::ActuatorTarget>(
    "/edulite/target", 1,
    [&last_position_rad](const actuator_msgs::msg::ActuatorTarget::SharedPtr msg) {
      if (msg->logical_id == 5) {
        last_position_rad = msg->target;
      }
    });

  uint8_t last_published_belt_mode = 255;
  auto belt_mode_sub = test_node->create_subscription<robot_msgs::msg::BeltMode>(
    "/belt/command_mode", 1,
    [&last_published_belt_mode](const robot_msgs::msg::BeltMode::SharedPtr msg) {
      last_published_belt_mode = msg->mode;
    });

  auto pub_shot_cycle = test_node->create_publisher<std_msgs::msg::Bool>(
    "/dribble/shot_cycle_request", 1);
  auto spring_state_pub =
      test_node->create_publisher<robot_msgs::msg::SpringOperationState>(
          "/spring/operation_state",
          rclcpp::QoS(1).reliable().transient_local());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(dribble_node);
  executor.add_node(test_node);

  // 初期化待ち (DRIBBLE姿勢: -0.86 rad)
  auto start = std::chrono::steady_clock::now();
  while (std::abs(last_position_rad - (-0.86f)) > 0.01f &&
    std::chrono::steady_clock::now() - start < std::chrono::seconds(2))
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_NEAR(last_position_rad, -0.86f, 0.01f);

  // 設定したローラ待機時間の経過後にFEEDへ進む。
  const auto param_res = dribble_node->set_parameter(
      rclcpp::Parameter("belt_shot_delay_sec", 0.0));
  ASSERT_TRUE(param_res.successful);

  // ベルトとローラを自動起動し、ばね収納完了後にFEEDへ向かう。
  std_msgs::msg::Bool shot_msg;
  shot_msg.data = true;
  pub_shot_cycle->publish(shot_msg);

  robot_msgs::msg::SpringOperationState spring_state;
  spring_state.state = robot_msgs::msg::SpringOperationState::BELT_CLEARANCE;
  spring_state_pub->publish(spring_state);

  // FEED (1.3 rad) に到達することを確認
  start = std::chrono::steady_clock::now();
  while (std::abs(last_position_rad - 1.3f) > 0.01f &&
    std::chrono::steady_clock::now() - start < std::chrono::seconds(2))
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_NEAR(last_position_rad, 1.3f, 0.01f);

  // FEED保持時間後、DRIBBLE (-0.86 rad) に自動復帰することを確認
  start = std::chrono::steady_clock::now();
  while (std::abs(last_position_rad - (-0.86f)) > 0.01f &&
    std::chrono::steady_clock::now() - start < std::chrono::seconds(3))
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_NEAR(last_position_rad, -0.86f, 0.01f);

  // 自動起動したベルトはシーケンス完了時に停止する。
  EXPECT_EQ(last_published_belt_mode, robot_msgs::msg::BeltMode::STOP);
}

TEST_F(RobotControllerTest, MecanumControllerKinematicsAndEmergencyStopTest)
{
  auto mecanum_node = std::make_shared<MecanumControllerNode>();
  auto test_node = std::make_shared<rclcpp::Node>("test_mecanum_client");

  float fl_vel = 0.0f, fr_vel = 0.0f;
  int received_command_count = 0;
  auto target_array_sub =
    test_node->create_subscription<actuator_msgs::msg::ActuatorTargetArray>(
    "/edulite/target_array", 1,
    [&fl_vel, &fr_vel, &received_command_count](
      const actuator_msgs::msg::ActuatorTargetArray::SharedPtr msg) {
      for (const auto & target : msg->actuators) {
        if (target.logical_id == 0) {
          fl_vel = target.target;
        } else if (target.logical_id == 1) {
          fr_vel = target.target;
        }
      }
      ++received_command_count;
    });

  auto pub_cmd_vel = test_node->create_publisher<geometry_msgs::msg::Twist>("/mecanum/cmd_vel", 1);
  auto pub_estop = test_node->create_publisher<std_msgs::msg::Bool>(
    "/emergency_stop", rclcpp::QoS(1).reliable().transient_local());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(mecanum_node);
  executor.add_node(test_node);

  // 1. 前進指令 (vx = 1.0) の運動学計算テスト
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 1.0;
  pub_cmd_vel->publish(twist);

  auto start = std::chrono::steady_clock::now();
  while (fl_vel == 0.0f &&
    std::chrono::steady_clock::now() - start < std::chrono::seconds(2))
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  // 直径150 mm（半径0.075 m）なので、fl: -13.33 rad/s, fr: 13.33 rad/s
  EXPECT_NEAR(fl_vel, -13.333f, 0.1f);
  EXPECT_NEAR(fr_vel, 13.333f, 0.1f);

  // 2. 非常停止時に全輪 0 rad/s になるかのテスト
  std_msgs::msg::Bool estop_msg;
  estop_msg.data = true;
  pub_estop->publish(estop_msg);

  start = std::chrono::steady_clock::now();
  while ((fl_vel != 0.0f || fr_vel != 0.0f) &&
    std::chrono::steady_clock::now() - start < std::chrono::seconds(2))
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_NEAR(fl_vel, 0.0f, 0.001f);
  EXPECT_NEAR(fr_vel, 0.0f, 0.001f);

  const int count_at_emergency_stop = received_command_count;
  start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(70)) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  EXPECT_GE(received_command_count - count_at_emergency_stop, 2);
  EXPECT_NEAR(fl_vel, 0.0f, 0.001f);
  EXPECT_NEAR(fr_vel, 0.0f, 0.001f);

  // 3. 上限超過時は全輪を同じ比率で縮小し、最大50 rad/sに収める。
  estop_msg.data = false;
  pub_estop->publish(estop_msg);
  twist.linear.x = 4.0;
  twist.linear.y = 1.0;
  pub_cmd_vel->publish(twist);

  start = std::chrono::steady_clock::now();
  while (std::abs(fl_vel + 50.0f) > 0.1f &&
    std::chrono::steady_clock::now() - start < std::chrono::seconds(2))
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_NEAR(fl_vel, -50.0f, 0.1f);
  EXPECT_NEAR(fr_vel, 30.0f, 0.1f);
}
