#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "can_msgs/msg/frame.hpp"
#include <bit>
#include <array>
#include <cstdint>
#include <algorithm>
#include <chrono>
#include <thread>
#include <cstring>

class WheelToCanNode : public rclcpp::Node
{
public:
  WheelToCanNode()
  : Node("wheel_to_can_node")
  {
    sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("motor_vel", 10,
      std::bind(&WheelToCanNode::callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<can_msgs::msg::Frame>("CAN/can0/transmit", 10);

    RCLCPP_INFO(this->get_logger(), "WheelToCanNode started");

        //少しだけ待ってから初期化を実行
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    init_motors();     //モーターの有効化
  }

  ~WheelToCanNode()
  {
    RCLCPP_INFO(this->get_logger(), "Stopping all motors");
    stop_motors();     //モーターを止める
  }

private:
  void init_motors()
  {
    for (int motor_id = 1; motor_id <= 4; motor_id++) {
            //4.1.9 Communication type 18: Single parameter write
      can_msgs::msg::Frame frame_mode;
      frame_mode.id = 0x1200FD00 + motor_id;
      frame_mode.is_extended = true;       //29bit
      frame_mode.dlc = 8;       //データ長は 8 固定

            // 運転モードを表すインデックス 0x7005 (run_mode) に 2 (Velocity mode) を書き込む
      frame_mode.data[0] = 0x05;       // Index下位
      frame_mode.data[1] = 0x70;       // Index上位
      frame_mode.data[2] = 0x00;
      frame_mode.data[3] = 0x00;
      frame_mode.data[4] = 0x02;       // 2: 速度モード (Velocity Mode)
      frame_mode.data[5] = 0x00;
      frame_mode.data[6] = 0x00;
      frame_mode.data[7] = 0x00;

      pub_->publish(std::move(frame_mode));

      std::this_thread::sleep_for(std::chrono::milliseconds(10));

            //4.1.4Communication Type 3: Motor enabled to run
      can_msgs::msg::Frame frame_enable;
      frame_enable.id = 0x0300FD00 + motor_id;
      frame_enable.is_extended = true;       // 29bit
      frame_enable.dlc = 8;                   // データ長は 8 固定

      for (int i = 0; i < 8; i++) {
        frame_enable.data[i] = 0;
      }

      pub_->publish(std::move(frame_enable));

      std::this_thread::sleep_for(std::chrono::milliseconds(10));

            //4.1.9 Communication type 18: 電流制御
      can_msgs::msg::Frame frame_cur_max;
      frame_cur_max.id = 0x1200FD00 + motor_id;
      frame_cur_max.is_extended = true;       // 29bit
      frame_cur_max.dlc = 8;                   // データ長は 8 固定

      frame_cur_max.data[0] = 0x06;       // Index下位
      frame_cur_max.data[1] = 0x70;       // Index上位
      frame_cur_max.data[2] = 0x00;
      frame_cur_max.data[3] = 0x00;
      float current_limit = 11.0f;       // 11A

      uint8_t current_buffer[4];

      auto cur_bytes = std::bit_cast<std::array<uint8_t, 4>>(current_limit);

      std::copy(cur_bytes.begin(), cur_bytes.end(), current_buffer);

      for (int i = 0; i < 4; i++) {
        frame_cur_max.data[i + 4] = current_buffer[i];
      }

      pub_->publish(std::move(frame_cur_max));

      std::this_thread::sleep_for(std::chrono::milliseconds(10));

             //4.1.9 Communication type 18: 加速度設定
      can_msgs::msg::Frame frame_acc_rad;
      frame_acc_rad.id = 0x1200FD00 + motor_id;
      frame_acc_rad.is_extended = true;       // 29bit
      frame_acc_rad.dlc = 8;                   // データ長は 8 固定

      frame_acc_rad.data[0] = 0x22;       // Index下位
      frame_acc_rad.data[1] = 0x70;       // Index上位
      frame_acc_rad.data[2] = 0x00;
      frame_acc_rad.data[3] = 0x00;
      float acc = 20.0f;       // 20 rad/s^2

      uint8_t acc_buffer[4];

      auto acc_bytes = std::bit_cast<std::array<uint8_t, 4>>(acc);

      std::copy(acc_bytes.begin(), acc_bytes.end(), acc_buffer);

      for (int i = 0; i < 4; i++) {
        frame_acc_rad.data[i + 4] = acc_buffer[i];
      }

      pub_->publish(std::move(frame_acc_rad));

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  void stop_motors()
  {
    for (int motor_id = 1; motor_id <= 4; motor_id++) {
      can_msgs::msg::Frame frame_vel_zero;
      frame_vel_zero.id = 0x1200FD00 + motor_id;                // 上位3ビットは0、そのままモーターid
      frame_vel_zero.is_extended = true;         // 29bit
      frame_vel_zero.dlc = 8;                     // データ長は 8 固定


      frame_vel_zero.data[0] = 0x0A;       //下位bit
      frame_vel_zero.data[1] = 0x70;       //上位bit
      for (int i = 2; i < 8; i++) {
        frame_vel_zero.data[i] = 0x00;         //速度を０に
      }

      pub_->publish(std::move(frame_vel_zero));

      std::this_thread::sleep_for(std::chrono::milliseconds(20));

      can_msgs::msg::Frame frame_stop;
      frame_stop.id = 0x0400FD00 + motor_id;
      frame_stop.is_extended = true;
      frame_stop.dlc = 8;

      for (int i = 0; i < 8; i++) {
        frame_stop.data[i] = 0x00;
      }

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

      float value = msg->data[motor_id - 1];

      uint8_t buffer[4];

      auto vel_bytes = std::bit_cast<std::array<uint8_t, 4>>(value);

      std::copy(vel_bytes.begin(), vel_bytes.end(), buffer);


      can_msgs::msg::Frame frame;
      frame.id = 0x1200FD00 + motor_id;        // 速度制御 24~28 communication type  8~23 host id 1~8 motor id
      frame.is_extended = true;           //29bit
      frame.dlc = 8;                      // データ長は 8 固定

            //データを詰める
      frame.data[0] = 0x0A;       //下位bit
      frame.data[1] = 0x70;       //上位bit
      frame.data[2] = 0;
      frame.data[3] = 0;
      for (int i = 0; i < 4; i++) {
        frame.data[i + 4] = buffer[i];
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
