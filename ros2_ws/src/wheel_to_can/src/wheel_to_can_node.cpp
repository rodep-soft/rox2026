#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "can_msgs/msg/frame.hpp"
#include <bit>
#include <array>
#include <cstdint>
#include <algorithm>
#include <chrono>
#include <thread>

class WheelToCanNode : public rclcpp::Node
{
public:
    WheelToCanNode()
        : Node("wheel_to_can_node")
    {
        sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("motor_vel", 10,std::bind(&WheelToCanNode::callback, this, std::placeholders::_1));

        pub_ = this->create_publisher<can_msgs::msg::Frame>("CAN_can0_transmit", 10);

        RCLCPP_INFO(this->get_logger(), "WheelToCanNode started");

        //少しだけ待ってから初期化を実行
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        init_motors(); //モーターの有効化
    }

    ~WheelToCanNode(){
        RCLCPP_INFO(this->get_logger(), "Stopping all motors");
        stop_motors(); //モーターを止める
    }


private:
    void init_motors(){
        for (int motor_id = 1; motor_id <= 4; motor_id++){
            //6.8 Command 6: Set Operation Mode Velocity modeに設定
            can_msgs::msg::Frame frame_mode;
            frame_mode.id = motor_id;
            frame_mode.is_extended = false; //11bit
            frame_mode.dlc = 8; //データ長は 8 固定

            for (int i = 0; i < 6; i++){
                frame_mode.data[i] = 0xFF; //前半6バイトはすべて 0xFF
            }

            frame_mode.data[6] = 0x02; //Velocity mode
            frame_mode.data[7] = 0xFC; //Command 1 の識別子

            pub_->publish(std::move(frame_mode));

            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            //6.3 Command 1: モーターの有効化
            can_msgs::msg::Frame frame_enable;
            frame_enable.id = motor_id;
            frame_enable.is_extended = false; // 11bit
            frame_enable.dlc = 8;             // データ長は 8 固定

            for (int i = 0; i < 7; i++){
                frame_enable.data[i] = 0xFF;   // 前半7バイトはすべて 0xFF
            }

            frame_enable.data[7] = 0xFC;    // Command 1 の識別子

            pub_->publish(std::move(frame_enable));

            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            RCLCPP_INFO(this->get_logger(), "Motor %d: Velocity Mode Set & Enabled", motor_id);
        
        }
    }

    void stop_motors(){
        for (int motor_id = 1; motor_id <= 4; motor_id++) {
            can_msgs::msg::Frame frame_stop;
            frame_stop.id = motor_id;          // 上位3ビットは0、そのままモーターid
            frame_stop.is_extended = false;   // 11bit 
            frame_stop.dlc = 8;               // データ長は 8 固定
        for (int i = 0; i < 7; i++) {
                frame_stop.data[i] = 0xFF; //7バイトFFにする
            }

        frame_stop.data[7] = 0xFD;

        pub_->publish(std::move(frame_stop));

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        RCLCPP_INFO(this->get_logger(), "Motor %d: Sent Stop Command", motor_id);
        }
    }

    void callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        //データのサイズが4未満なら警告
        if (msg->data.size() < 4) {
            RCLCPP_WARN(this->get_logger(), "Wheel speed array size < 4");
            return;
        }

        for (int motor_id = 1; motor_id <= 4; motor_id++) {

            //送られてきたデータからコピー
            float value =  msg->data[motor_id - 1];
            float current = 11.0;

            uint8_t buffer[8];

            auto vel_bytes = std::bit_cast<std::array<uint8_t,4>>(value);
            auto cur_bytes = std::bit_cast<std::array<uint8_t,4>>(current);

            std::copy(vel_bytes.begin(), vel_bytes.end(),buffer);
            std::copy(cur_bytes.begin(), cur_bytes.end(), buffer + 4);

            can_msgs::msg::Frame frame;
            frame.id = 0x200 + motor_id;    // MIT 6.13 Velocity Mode Control
            frame.is_extended = false;     //11bit
            frame.dlc = 8;                // データ長は 8 固定

            //データを詰める
            for (int i = 0; i< 8; i++){
                frame.data[i] = buffer[i];
            }
            pub_->publish(std::move(frame));
        }
    }
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelToCanNode>());
    rclcpp::shutdown();
    return 0;
}