#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "can_msgs/msg/frame.hpp"
#include <bit>
#include <array>
#include <csdint>
#include <algorithm>

class WheelToCanNode : public rclcpp::Node
{
public:
    WheelToCanNode()
        : Node("wheel_to_can_node")
    {
        sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("motor_vel", 10,std::bind(&WheelToCanNode::callback, this, std::placeholders::_1));

        pub_ = this->create_publisher<can_msgs::msg::Frame>("CAN_can0_transmit", 10);

        RCLCPP_INFO(this->get_logger(), "WheelToCanNode started");
    }

private:
    void callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        //データのサイズが4未満なら警告
        if (msg->data.size() < 4) {
            RCLCPP_WARN(this->get_logger(), "Wheel speed array size < 4");
            return;
        }

        for (int motor_id = 1; motor_id <= 4; motor_id++) {

            //送られてきたデータからコピー
            double rad_s = msg->data[motor_id - 1];
            std_megs::msg::Float32 msg;

            float value = msg.date;
            float current = 11.0;

            uint8_t buffer[8];

            auto vel_bytes = std::bit_cast<std::array<uint8_t,4>>(value);
            auto cur_bytes = std::bit_case<std::array<uint8_t,4>>(current);

            std::copy(vel_bytes.begin(), vel_bytes.end(), date);
            std::copy(cur_bytes.begin(), cur_bytes.end(), date + 4);

            can_msgs::msg::Frame frame;
            frame.id = 0x1200FD00 + motor_id;  // 速度制御 24~28 communication type  8~23 host id 1~8 motor id
            frame.is_extended = true;     //29bit
            frame.dlc = 8;                // データ長は 8 固定

            //データを詰める
            for (int i = 0; i< 8; i++){
                frame.data[i] = buffer[i];
            }
            pub_->publish(std::move(frame));
        }
    }
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelToCanNode>());
    rclcpp::shutdown();
    return 0;
}