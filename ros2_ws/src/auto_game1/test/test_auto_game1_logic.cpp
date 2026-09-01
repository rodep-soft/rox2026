#include <gtest/gtest.h>
#include <cmath>
#include <memory>

#include "auto_game1/auto_game1_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "tf2/utils.h"

namespace auto_game1
{

class AutoGame1NodeTest : public ::testing::Test
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
  }

  std::shared_ptr<AutoGame1Node> node_;
};

// 1. 2D距離計算のテスト
TEST_F(AutoGame1NodeTest, TestComputeDistance2D)
{
  geometry_msgs::msg::Point p1;
  p1.x = 0.0;
  p1.y = 0.0;

  geometry_msgs::msg::Point p2;
  p2.x = 3.0;
  p2.y = 4.0;

  double dist = node_->compute_distance_2d(p1, p2);
  EXPECT_NEAR(dist, 5.0, 1e-6);
}

// 2. サイド反転 (SIDE_A ⇔ SIDE_B) の座標およびYaw角反転テスト
TEST_F(AutoGame1NodeTest, TestApplySideTransform)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 2.5;
  pose.pose.position.y = 1.0;
  
  // Yaw = 0.5 rad
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.5);
  pose.pose.orientation = tf2::toMsg(q);

  // SIDE_A (通常モード)
  node_->current_side_ = Side::SIDE_A;
  auto pose_side_a = node_->apply_side_transform(pose);
  EXPECT_DOUBLE_EQ(pose_side_a.pose.position.x, 2.5);
  EXPECT_DOUBLE_EQ(pose_side_a.pose.position.y, 1.0);
  EXPECT_NEAR(tf2::getYaw(pose_side_a.pose.orientation), 0.5, 1e-6);

  // SIDE_B (左右反転モード)
  node_->current_side_ = Side::SIDE_B;
  auto pose_side_b = node_->apply_side_transform(pose);
  EXPECT_DOUBLE_EQ(pose_side_b.pose.position.x, -2.5);
  EXPECT_DOUBLE_EQ(pose_side_b.pose.position.y, 1.0);
  EXPECT_NEAR(tf2::getYaw(pose_side_b.pose.orientation), -0.5, 1e-6);
}

// 3. Map速度からBase速度への回転変換テスト
TEST_F(AutoGame1NodeTest, TestTransformMapVelocityToBase)
{
  // ロボットが Map 上で 90度 (PI/2) 左を向いている場合
  double robot_yaw = M_PI_2;
  double v_x_map = 1.0;  // Mapの東方向に向かって 1.0 m/s
  double v_y_map = 0.0;

  auto twist = node_->transform_map_velocity_to_base(v_x_map, v_y_map, 0.0, robot_yaw);

  // ロボットから見ると「右（マイナスY方向）」に移動していることになる
  EXPECT_NEAR(twist.linear.x, 0.0, 1e-5);
  EXPECT_NEAR(twist.linear.y, -1.0, 1e-5);
}

// 4. ボタン押し下げ判定機能のテスト
TEST_F(AutoGame1NodeTest, TestButtonPressed)
{
  sensor_msgs::msg::Joy joy_msg;
  joy_msg.buttons = {0, 1, 0, 1};

  EXPECT_FALSE(node_->button_pressed(joy_msg, 0));
  EXPECT_TRUE(node_->button_pressed(joy_msg, 1));
  EXPECT_FALSE(node_->button_pressed(joy_msg, 2));
  EXPECT_TRUE(node_->button_pressed(joy_msg, 3));

  // 配列範囲外の安全アクセス
  EXPECT_FALSE(node_->button_pressed(joy_msg, -1));
  EXPECT_FALSE(node_->button_pressed(joy_msg, 10));
}

}  // namespace auto_game1
