#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "spring_controller/spring_controller_state.h"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"

class SpringMot : public rclcpp::Node
{
public:
    SpringMot()
        : Node("spring_mot")
    {
        this->declare_parameter("max_speed", 20.0);
        this->declare_parameter("loading_speed", 5.0);
        this->declare_parameter("limit_sw_index", 0);

        max_speed_ = this->get_parameter("max_speed").as_double();
        loading_speed_ = this->get_parameter("loading_speed").as_double();
        limit_sw_index_ = this->get_parameter("limit_sw_index").as_int();

        spr_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/spring_cmd",
            10,
            std::bind(&SpringMot::spring_callback, this, std::placeholders::_1));

        limit_sw_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "/limit_sw",
            10,
            std::bind(&SpringMot::limit_callback, this, std::placeholders::_1));

        speed_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "/edulite_speed",
            10);

        // 10msごとにモーター制御管理コールバックを呼び出す
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&SpringMot::timer_callback, this));
    }

private:
    State state_ = State::STOP; // 制御状態を表す
    double max_speed_;          // 発射時のモーター回転速度
    double loading_speed_;      // 装填時のモーター回転速度
    int limit_sw_ = 0;          // リミットスイッチの状態を表す変数
    int limit_sw_index_;        // リミットスイッチのインデックス番号

    int fire_count_ = 0; // 発射中にどれぐらい時間がたったか

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr spr_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr limit_sw_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    void spring_callback(const std_msgs::msg::UInt8::SharedPtr msg)
    { // 制御コマンドの受け取り
            state_ = static_cast<State>(msg->data); // 受け取ったコマンドに応じて状態を変更
    }
    void limit_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
    { // リミットスイッチのデータ受け取り
        if (msg->data.size() > 0)
        {
            limit_sw_ = msg->data[limit_sw_index_];
        }
    }

    void timer_callback()
    { // モーター制御管理コールバック
        std_msgs::msg::Float32 speed;

        switch (state_)
        {
        case STOP:
            speed.data = 0.0;
            break;
        case LOAD://チャタリングや通り過ぎの制御も
            if (limit_sw_ == 1)
            { // リミットスイッチが押されている
                speed.data = 0.0;
                if (state_ == FIRE)
                {                    // 発射命令がある
                    state_ = LOAD;   // 装填状態に戻す
                    fire_count_ = 0; // 発射カウントをリセット
                }
            }
            else
            { // リミットスイッチが押されていない
                speed.data = loading_speed_;
            }
            break;
        case FIRE:
            speed.data = max_speed_;
            fire_count_++;

            if (fire_count_ > 10)
            {
                state_ = LOAD;
            }
            break;
        default:
            speed.data = 0.0;
            break;
        }

        speed_pub_->publish(speed);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpringMot>());
    rclcpp::shutdown();
    return 0;
}