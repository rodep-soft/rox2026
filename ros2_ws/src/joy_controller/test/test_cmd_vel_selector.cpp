#include <gtest/gtest.h>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "joy_controller/cmd_vel_selector_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace joy_controller
{

class CmdVelSelectorNodeTest : public ::testing::Test
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

  void SetUp() override
  {
    node_ = std::make_shared<CmdVelSelectorNode>();
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
  }

  std::shared_ptr<CmdVelSelectorNode> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

TEST_F(CmdVelSelectorNodeTest, TestCmdVelSelection)
{
  geometry_msgs::msg::Twist received_cmd_vel;
  bool cmd_vel_received = false;

  auto sub = node_->create_subscription<geometry_msgs::msg::Twist>(
    "/mecanum/cmd_vel", rclcpp::QoS(10),
    [&received_cmd_vel, &cmd_vel_received](const geometry_msgs::msg::Twist::SharedPtr msg) {
      received_cmd_vel = *msg;
      cmd_vel_received = true;
    });

  // 1. モードを 1 (DRIVE / 手動) に設定
  auto mode_msg = std::make_shared<std_msgs::msg::UInt8>();
  mode_msg->data = 1;
  node_->operation_mode_callback(mode_msg);

  // 2. 手動スピードコマンドを送信
  auto manual_twist = std::make_shared<geometry_msgs::msg::Twist>();
  manual_twist->linear.x = 1.5;
  node_->manual_cmd_vel_callback(manual_twist);

  executor_->spin_some();
  EXPECT_TRUE(cmd_vel_received);
  EXPECT_DOUBLE_EQ(received_cmd_vel.linear.x, 1.5);

  // 3. モードを 2 (SHOT_CYCLE / 自動) に切替
  cmd_vel_received = false;
  mode_msg->data = 2;
  node_->operation_mode_callback(mode_msg);

  auto auto_twist = std::make_shared<geometry_msgs::msg::Twist>();
  auto_twist->linear.x = 0.8;
  node_->auto_cmd_vel_callback(auto_twist);

  executor_->spin_some();
  EXPECT_TRUE(cmd_vel_received);
  EXPECT_DOUBLE_EQ(received_cmd_vel.linear.x, 0.8);
}

}  // namespace joy_controller
