#include <gtest/gtest.h>
#include <chrono>
#include <memory>

#include "actuator_msgs/msg/actuator_target_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "arm_position_controller/arm_position_controller.hpp"
#include "belt_controller/belt_controller.hpp"
#include "dribbler_controller/dribbler_controller.hpp"
#include "mecanum_controller/mecanum_controller_node.hpp"

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

  int16_t last_underbelt_rpm = -1;
  auto sub_underbelt = test_node->create_subscription<std_msgs::msg::Int16>(
    "/underbelt/target/rpm", 1,
    [&last_underbelt_rpm](const std_msgs::msg::Int16::SharedPtr msg) {
      last_underbelt_rpm = msg->data;
    });

  auto pub_belt_mode = test_node->create_publisher<std_msgs::msg::UInt8>("/belt/mode", 1);
  auto pub_estop = test_node->create_publisher<std_msgs::msg::Bool>(
    "/emergency_stop", rclcpp::QoS(1).reliable().transient_local());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(belt_node);
  executor.add_node(test_node);

  // 1. Level 3 (4000 RPM) のテスト
  std_msgs::msg::UInt8 mode_msg;
  mode_msg.data = 3;
  pub_belt_mode->publish(mode_msg);

  auto start = std::chrono::steady_clock::now();
  while (last_underbelt_rpm != 4000 &&
    std::chrono::steady_clock::now() - start < std::chrono::seconds(2))
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(last_underbelt_rpm, 4000);

  // 2. 非常停止発動のテスト
  std_msgs::msg::Bool estop_msg;
  estop_msg.data = true;
  pub_estop->publish(estop_msg);

  start = std::chrono::steady_clock::now();
  while (last_underbelt_rpm != 0 &&
    std::chrono::steady_clock::now() - start < std::chrono::seconds(2))
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(last_underbelt_rpm, 0);
}

TEST_F(RobotControllerTest, DribblerControllerEnableAndEmergencyStopTest)
{
  auto dribble_node = std::make_shared<DribblerControllerNode>();
  auto test_node = std::make_shared<rclcpp::Node>("test_dribbler_client");

  int16_t last_dribble_rpm = -1;
  auto sub_dribble = test_node->create_subscription<std_msgs::msg::Int16>(
    "/dribble/target/rpm", 1,
    [&last_dribble_rpm](const std_msgs::msg::Int16::SharedPtr msg) {
      last_dribble_rpm = msg->data;
    });

  auto pub_dribble_enable = test_node->create_publisher<std_msgs::msg::Bool>("/dribble/enabled", 1);
  auto pub_estop = test_node->create_publisher<std_msgs::msg::Bool>(
    "/emergency_stop", rclcpp::QoS(1).reliable().transient_local());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(dribble_node);
  executor.add_node(test_node);

  // 1. Dribble ON (2000 RPM) のテスト
  std_msgs::msg::Bool enable_msg;
  enable_msg.data = true;
  pub_dribble_enable->publish(enable_msg);

  auto start = std::chrono::steady_clock::now();
  while (last_dribble_rpm != 2000 &&
    std::chrono::steady_clock::now() - start < std::chrono::seconds(2))
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(last_dribble_rpm, 2000);

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

TEST_F(RobotControllerTest, ArmPositionControllerStateTransitionSequenceTest)
{
  auto arm_node = std::make_shared<ArmPositionControllerNode>();
  auto test_node = std::make_shared<rclcpp::Node>("test_arm_client");

  float last_arm_pos = 999.0f;
  auto sub_arm_pos = test_node->create_subscription<std_msgs::msg::Float32>(
    "/dribble/position_command", 1,
    [&last_arm_pos](const std_msgs::msg::Float32::SharedPtr msg) {
      last_arm_pos = msg->data;
    });

  auto pub_arm_mode = test_node->create_publisher<std_msgs::msg::UInt8>(
    "/dribble/position_mode", 1);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(arm_node);
  executor.add_node(test_node);

  // 状態0: 初期姿勢 DRIBBLE (0.35 rad)
  auto start = std::chrono::steady_clock::now();
  while (std::abs(last_arm_pos - 0.35f) > 0.01f &&
    std::chrono::steady_clock::now() - start < std::chrono::seconds(2))
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_NEAR(last_arm_pos, 0.35f, 0.01f);

  // 状態1: OPEN 位置 (-1.0 rad) へ遷移
  std_msgs::msg::UInt8 mode_msg;
  mode_msg.data = 1; // OPEN
  pub_arm_mode->publish(mode_msg);

  start = std::chrono::steady_clock::now();
  while (std::abs(last_arm_pos - (-1.0f)) > 0.01f &&
    std::chrono::steady_clock::now() - start < std::chrono::seconds(2))
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_NEAR(last_arm_pos, -1.0f, 0.01f);

  // 状態2: FEED 位置 (1.3 rad) へ遷移
  mode_msg.data = 2; // FEED
  pub_arm_mode->publish(mode_msg);

  start = std::chrono::steady_clock::now();
  while (std::abs(last_arm_pos - 1.3f) > 0.01f &&
    std::chrono::steady_clock::now() - start < std::chrono::seconds(2))
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_NEAR(last_arm_pos, 1.3f, 0.01f);
}

TEST_F(RobotControllerTest, MecanumControllerKinematicsAndEmergencyStopTest)
{
  auto mecanum_node = std::make_shared<MecanumControllerNode>();
  auto test_node = std::make_shared<rclcpp::Node>("test_mecanum_client");

  float fl_vel = 0.0f, fr_vel = 0.0f;
  auto target_array_sub =
    test_node->create_subscription<actuator_msgs::msg::ActuatorTargetArray>(
    "/edulite/target_array", 1,
    [&fl_vel, &fr_vel](const actuator_msgs::msg::ActuatorTargetArray::SharedPtr msg) {
      for (const auto & target : msg->actuators) {
        if (target.logical_id == 0) {
          fl_vel = target.target;
        } else if (target.logical_id == 1) {
          fr_vel = target.target;
        }
      }
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
  // fl: -20 rad/s, fr: 20 rad/s
  EXPECT_NEAR(fl_vel, -20.0f, 0.1f);
  EXPECT_NEAR(fr_vel, 20.0f, 0.1f);

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
}
