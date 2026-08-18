#include <gtest/gtest.h>
#include <memory>

#include "auto_game1/auto_game1_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace auto_game1
{

class AutoGame1StateMachineTest : public ::testing::Test
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
    node_ = std::make_shared<AutoGame1Node>();
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
  }

  std::shared_ptr<AutoGame1Node> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

// 初期状態および Joyボタンによる段階的発進シーケンスのテスト
TEST_F(AutoGame1StateMachineTest, TestStateMachineTransitions)
{
  // 1. 初期状態は AUTO_STOP
  EXPECT_EQ(node_->current_state_, State::AUTO_STOP);

  // 2. Createボタン (Index 8) を入力して AUTO_STOP から DRIBBLE_ON (Standby) へ移行
  auto joy_msg = std::make_shared<sensor_msgs::msg::Joy>();
  joy_msg->buttons.resize(12, 0);
  joy_msg->buttons[8] = 1;  // Create ボタン ON

  node_->joy_callback(joy_msg);
  EXPECT_EQ(node_->current_state_, State::DRIBBLE_ON);

  // 3. ○ボタン (Index 2) でドリブルON要求
  bool dribble_enabled_published = false;
  auto dribble_sub = node_->create_subscription<std_msgs::msg::Bool>(
    "/dribble/enabled", rclcpp::QoS(10),
    [&dribble_enabled_published](const std_msgs::msg::Bool::SharedPtr msg) {
      if (msg->data) {
        dribble_enabled_published = true;
      }
    });

  joy_msg->buttons[8] = 0;
  joy_msg->buttons[2] = 1;  // Circle ボタン ON
  node_->joy_callback(joy_msg);

  executor_->spin_some();
  EXPECT_TRUE(dribble_enabled_published);
  EXPECT_EQ(node_->current_state_, State::DRIBBLE_ON);

  // 4. ×ボタン (Index 1) で自律移動 GO_TO_KICK_START へ遷移
  joy_msg->buttons[2] = 0;
  joy_msg->buttons[1] = 1;  // Cross ボタン ON
  node_->joy_callback(joy_msg);
  EXPECT_EQ(node_->current_state_, State::GO_TO_KICK_START);

  // 5. 手動介入トピック /operation_mode = 1 (DRIVE) 受信で AUTO_STOP へ割り込み緊急停止
  auto op_mode_msg = std::make_shared<std_msgs::msg::UInt8>();
  op_mode_msg->data = 1;  // DRIVE 手動モード
  node_->operation_mode_callback(op_mode_msg);
  EXPECT_EQ(node_->current_state_, State::AUTO_STOP);
}

}  // namespace auto_game1
