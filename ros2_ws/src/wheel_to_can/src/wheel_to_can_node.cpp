#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "can_msgs/msg/frame.hpp"
#include <bit>
#include <array>
#include <cstdint>
#include <algorithm>
#include <chrono>
#include <thread>
#include <utility>
#include <functional>

using namespace std::chrono_literals;

class WheelToCanNode : public rclcpp::Node
{
public:
    WheelToCanNode()
        : Node("wheel_to_can_node"), is_timeout_(false) // タイムアウトフラグの初期化
    {
         // パラメータ読み込み
        this->declare_parameter<double>("current_limit", 11.0);
        this->declare_parameter<double>("acceleration", 20.0);
        this->declare_parameter<double>("max_velocity", 50.0);
        this->declare_parameter<double>("min_velocity", -50.0);

        current_limit_ = this->get_parameter("current_limit").as_double();
        acceleration_ = this->get_parameter("acceleration").as_double();
        max_velocity_ = this->get_parameter("max_velocity").as_double();
        min_velocity_ = this->get_parameter("min_velocity").as_double();
        sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "motor_vel", 10, std::bind(&WheelToCanNode::callback, this, std::placeholders::_1));
        
        can_sub_ = this->create_subscription<can_msgs::msg::Frame>("/CAN/can0/receive",10,
        std::bind(
            &WheelToCanNode::can_receive_callback,this,std::placeholders::_1
            )
        );


        pub_ = this->create_publisher<can_msgs::msg::Frame>("CAN/can0/transmit", 10);

        last_vel_received_time_ = this->now();     // 初期化時に現在の時刻を設定

        timeout_timer_ = this->create_wall_timer(
            100ms, std::bind(&WheelToCanNode::check_timeout, this));    // 100msごとにタイムアウトチェックを実行

        RCLCPP_INFO(this->get_logger(), "WheelToCanNode started");

        // 少しだけ待ってから初期化を実行
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        init_motors(); // モーターの有効化
        is_initialized_ = true;
    }

    void stop_motors() {
        for (int motor_id = 1; motor_id <= 4; motor_id++) {
            can_msgs::msg::Frame frame_vel_zero;
            frame_vel_zero.id = 0x1200FD00 + motor_id;          // 上位3ビットは0、そのままモーターid
            frame_vel_zero.is_extended = true;           // 29bit 
            frame_vel_zero.dlc = 8;                      // データ長は 8 固定

            frame_vel_zero.data[0] = 0x0A;       // 下位bit
            frame_vel_zero.data[1] = 0x70;      // 上位bit
            for (int i = 2; i < 8; i++) {
                frame_vel_zero.data[i] = 0x00;      // 速度を０に
            }

            pub_->publish(std::move(frame_vel_zero));

            // std::this_thread::sleep_for(std::chrono::milliseconds(10));

            RCLCPP_INFO(this->get_logger(), "Motor %d: Sent Stop Command", motor_id);
        }
    }

    ~WheelToCanNode() {
    }

private:

    double current_limit_;
    double acceleration_;
    double max_velocity_;
    double min_velocity_;

    bool is_timeout_;       // タイムアウト状態を管理するフラグ
    bool is_initialized_ = false; // 初期化が完了したかどうかを管理するフラグ


    void init_motors() {                     
        for (int motor_id = 1; motor_id <= 4; motor_id++) {
            // 4.1.9 Communication type 18: Single parameter write
            can_msgs::msg::Frame frame_mode;
            frame_mode.id = 0x1200FD00 + motor_id;
            frame_mode.is_extended = true;      // 29bit
            frame_mode.dlc = 8;         // データ長は 8 固定

            // 運転モードを表すインデックス 0x7005 (run_mode) に 2 (Velocity mode) を書き込む
            frame_mode.data[0] = 0x05;      // Index下位
            frame_mode.data[1] = 0x70;      // Index上位
            frame_mode.data[2] = 0x00;
            frame_mode.data[3] = 0x00;
            frame_mode.data[4] = 0x02;      // 2: 速度モード (Velocity Mode)
            frame_mode.data[5] = 0x00;
            frame_mode.data[6] = 0x00;
            frame_mode.data[7] = 0x00;
            
            pub_->publish(std::move(frame_mode));
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            
            // 4.1.4 Communication Type 3: Motor enabled to run
            can_msgs::msg::Frame frame_enable;
            frame_enable.id = 0x0300FD00 + motor_id;
            frame_enable.is_extended = true;        // 29bit
            frame_enable.dlc = 8;                    // データ長は 8 固定

            for (int i = 0; i < 8; i++) {
                frame_enable.data[i] = 0; 
            }

            pub_->publish(std::move(frame_enable));
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            // 4.1.9 Communication type 18: 電流制御
            can_msgs::msg::Frame frame_cur_max;
            frame_cur_max.id = 0x1200FD00 + motor_id;
            frame_cur_max.is_extended = true;       // 29bit
            frame_cur_max.dlc = 8;                  // データ長は 8 固定    

            frame_cur_max.data[0] = 0x06;        // Index下位
            frame_cur_max.data[1] = 0x70;           // Index上位
            frame_cur_max.data[2] = 0x00;
            frame_cur_max.data[3] = 0x00;
            float current_limit = current_limit_; // yamlから取得


            auto cur_bytes = std::bit_cast<std::array<uint8_t, 4>>(current_limit);
            for (int i = 0; i < 4; i++) {
                frame_cur_max.data[i + 4] = cur_bytes[i];
            }

            pub_->publish(std::move(frame_cur_max));
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

             // 4.1.9 Communication type 18: 加速度設定
            can_msgs::msg::Frame frame_acc_rad;
            frame_acc_rad.id = 0x1200FD00 + motor_id;
            frame_acc_rad.is_extended = true;       // 29bit
            frame_acc_rad.dlc = 8;                  // データ長は 8 固定    

            frame_acc_rad.data[0] = 0x22;       // Index下位
            frame_acc_rad.data[1] = 0x70;       // Index上位
            frame_acc_rad.data[2] = 0x00;
            frame_acc_rad.data[3] = 0x00;
            float acc = static_cast<float>(acceleration_);


            auto acc_bytes = std::bit_cast<std::array<uint8_t, 4>>(acc);
            for (int i = 0; i < 4; i++) {
                frame_acc_rad.data[i + 4] = acc_bytes[i];
            }

            pub_->publish(std::move(frame_acc_rad));
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void can_receive_callback( const can_msgs::msg::Frame::SharedPtr msg)
    {
        for(int motor_id = 1; motor_id <= 4; motor_id++)
        {
            uint32_t target_id =
                (motor_id << 16) | 0xFE;

           if(msg->id == target_id)
            {
                RCLCPP_INFO(this->get_logger(), "Received CAN message from motor %d", motor_id);
                if(!is_initialized_)
                {
                    RCLCPP_INFO(this->get_logger(), "Initializing motors after receiving first CAN message.");  
                    init_motors();
                    is_initialized_ = true;
                }

    return;
}
        }
    }

    void callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // データのサイズが4未満なら警告
        if (msg->data.size() < 4) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Wheel speed array size < 4");
            return;
        }

        last_vel_received_time_ = this->now();

        // タイムアウトから復帰した場合、フラグを下ろす
        if (is_timeout_) {
            RCLCPP_INFO(this->get_logger(), "Recovered from timeout. Resuming motors.");
            is_timeout_ = false;
        }

    for (int motor_id = 1; motor_id <= 4; motor_id++) {

            float value = std::clamp(
                msg->data[motor_id - 1],
                static_cast<float>(min_velocity_),
                static_cast<float>(max_velocity_)
            );

            auto vel_bytes = std::bit_cast<std::array<uint8_t, 4>>(value);

            can_msgs::msg::Frame frame;
            frame.id = 0x1200FD00 + motor_id;       // 速度制御 24~28 communication type  8~23 host id 1~8 motor id
            frame.is_extended = true;           // 29bit
            frame.dlc = 8;                      // データ長は 8 固定

            // データを詰める
            frame.data[0] = 0x0A;       // 下位bit
            frame.data[1] = 0x70;       // 上位bit
            frame.data[2] = 0; 
            frame.data[3] = 0;
            for (int i = 0; i < 4; i++) {
                frame.data[i + 4] = vel_bytes[i];
            }
            pub_->publish(std::move(frame));
        }
    }

    void check_timeout() {
        auto duration_since_last_rx = this->now() - last_vel_received_time_;

        // まだタイムアウト処理をしていない(フラグがfalse)場合のみ停止コマンドを送る
        if (duration_since_last_rx > rclcpp::Duration(1.0s) && !is_timeout_) {
            RCLCPP_WARN(this->get_logger(), "motor_vel timeout! Stopping motors...");
            
            stop_motors();
            
            is_timeout_ = true;     // フラグを立てて、連続で停止コマンドを送らないようにする
            RCLCPP_INFO(this->get_logger(), "Timeout flag set.");
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timeout_timer_;
    rclcpp::Time last_vel_received_time_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WheelToCanNode>();
    rclcpp::spin(node);
    node->stop_motors();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    rclcpp::shutdown();
    return 0;
}
