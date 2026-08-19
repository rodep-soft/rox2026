#define ROS2_NODE_TESTING
#include <gtest/gtest.h>

#include <chrono>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <thread>

// Include the source files directly to compile and test the node classes.
// Note that main() in these files is guarded by #ifndef ROS2_NODE_TESTING.
#include "../src/ros2/ros2_lifecycle_publisher_node.cpp"
#include "../src/ros2/ros2_publisher_node.cpp"

class ROS2NodeTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }

    static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(ROS2NodeTest, TestStandardPublisherNodeMock) {
    rclcpp::NodeOptions options;
    options.append_parameter_override("device", "/dev/i2c-mock");
    options.append_parameter_override("address", 0x28);
    options.append_parameter_override("publish_rate", 50.0);

    // Create standard node (should initialize BNO055 in mock mode)
    auto node = std::make_shared<BNO055PublisherNode>(options);

    bool msg_received = false;
    auto sub_node = std::make_shared<rclcpp::Node>("test_sub_node");
    auto sub = sub_node->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", 10, [&msg_received](const sensor_msgs::msg::Imu::SharedPtr msg) {
            (void)msg;
            msg_received = true;
        });

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(sub_node);

    auto start = std::chrono::steady_clock::now();
    while (!msg_received && (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(500))) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    EXPECT_TRUE(msg_received);
}

TEST_F(ROS2NodeTest, TestLifecyclePublisherNodeMock) {
    rclcpp::NodeOptions options;
    options.append_parameter_override("device", "/dev/i2c-mock");
    options.append_parameter_override("address", 0x28);
    options.append_parameter_override("publish_rate", 50.0);

    // Create lifecycle node
    auto node = std::make_shared<bno055_ros2::BNO055LifecyclePublisherNode>(options);

    bool msg_received = false;
    auto sub_node = std::make_shared<rclcpp::Node>("test_sub_node_lifecycle");
    auto sub = sub_node->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", 10, [&msg_received](const sensor_msgs::msg::Imu::SharedPtr msg) {
            (void)msg;
            msg_received = true;
        });

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.add_node(sub_node);

    // 1. Verify Unconfigured state (should not publish)
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(100)) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_FALSE(msg_received);

    // 2. Trigger configure transition -> state should become Inactive
    auto state = node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    ASSERT_EQ(state.id(), lifecycle_msgs::msg::State::STATE_INACTIVE);

    // 3. Verify Inactive state (should not publish)
    start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(100)) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_FALSE(msg_received);

    // 4. Trigger activate transition -> state should become Active
    state = node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    ASSERT_EQ(state.id(), lifecycle_msgs::msg::State::STATE_ACTIVE);

    // 5. Verify Active state (should start publishing)
    start = std::chrono::steady_clock::now();
    while (!msg_received && (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(500))) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_TRUE(msg_received);

    // Reset received flag
    msg_received = false;

    // 6. Trigger deactivate transition -> state should become Inactive
    state = node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    ASSERT_EQ(state.id(), lifecycle_msgs::msg::State::STATE_INACTIVE);

    // 7. Verify Inactive state again (should stop publishing)
    start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(100)) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_FALSE(msg_received);
}
