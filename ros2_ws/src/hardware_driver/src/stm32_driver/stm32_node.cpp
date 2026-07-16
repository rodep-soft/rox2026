/*
stm32とCANFrameを送受信するノード
*/
#include <memory>
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "can_msgs/can_msg.hpp"

class Stm32Node : public Node
{
public:
    Stm32Node()
        : Node("stm32_driver_node")
    {
        this->declare_parameter<std::string>("can_pub_topic", "/CAN/can0/transmit");
        this->declare_parameter<std::string>("can_sub_topic", "/CAN/can0/receive");

        this->declare_parameter<std::string>("limit_sw_topic", "/limit_sw");
        this->declare_parameter<std::string>("brushless_pub_topic", "/rpm");

        this->declare_parameter<int32_t>("keep_alive_period_ms", 50);


        std::string can_pub_topic_ = this->get_parameter("can_pub_topic").as_string();
        std::string can_sub_topic_ = this->get_parameter("can_sub_topic").as_string();

        std::string limit_sw_topic_ = this->get_parameter("limit_sw_topic").as_string();
        std::string brushless_pub_topic_ = this->get_parameter("brushless_pub_topic").as_string();

        int32_t keep_alive_period_ms = this->get_parameter("keep_alive_period_ms").as_int();

        // ros2とstm32間で送受信するためのpublisherとsubscriber
        can_pub_ = this->create_publisher<can_msgs::msg::Frame>(
            can_pub_topic_, 10);
        can_sub_ = this->create_subscription<can_msgs::msg::Frame>(
            can_sub_topic_, 10,
            std::bind(&Stm32Node::canCallback, this, std::placeholders::_1));

        // stm32から来たリミットスイッチとモータの現在のrpmを取得
        limit_sw_pub_ = this->create_publisher<std_msgs::msg::UInt8>(limit_sw_topic_, 10);
        brushless_motor_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(brushless_pub_topic_, 10);

        // タイムアウトの検知とノードへのパブリッシュをするタイマー
        alived_timer_ = this->create_wall_timer(
            keep_alive_period_ms,
            std::bind(&Stm32Node::alive_timer_callback, this));

        publish_timer_ = this->create_wall_timer(
            10ms,
            std::bind(&Stm32Node::publish_timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "stm32_driver_node has been started.");

        // タイムアウト検知時には，非常停止を強制的に入れる？
    }

private:
    // canFrameの受信
    void can_callback(const can_msgs::msg::Frame::SharedPtr msg);

    // モータの目標回転速度を送る
    void motor_target_callback(const std_msgs::msg::Float32MultiArray &msg);

    // ledのコマンドを送る
    void led_callback(const std_msgs::msg::Bool &msg);

    // タイムアウト処理をするかの判断
    void alive_timer_callback();

    // 一定周期でモータの現在のrpmを送信する
    void publish_timer_callback();

    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr current_rpm_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_rpm_pub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr led_cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr limit_sw_pub;

    rclcpp::TimerBase::SharedPtr alived_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};

// canのcallback関数：feedback
//  canのトピックに呼ばれたら実行されるのでidによるフィルタが必要
//  解析したIDを元に必要な
void Stm32Node::can_callback(const can_msgs::msg::Frame::SharedPtr msg)
{
    if (msg.id >= protocol::MOTOR_RPM_BASE &&
        msg.id < protocol::MOTOR_RPM_BASE + MOTOR_NUM)
    {
        uint8_t raw_data = msg->data[0];
        uint8_t limit_status = raw_data & 0x07;

        auto output_msg = std_msgs::msg::UInt8();
        output_msg.data = limit_status;

        limit_swpub->publish(output_msg);

        RCLCPP_INFO(this->get_logger(), "Published limit switch state: %d", limit_status);
    }
}

// タイムアウト時の処理をどうするか？

void Stm32Node::motor_target_callback(const std_msgs::msg::Float32MultiArray &msg)
{ // 三つ分受け取り，一つずつのIDに分割
    for (size_t i = 0; i < msg.data.size() && i < protocol::MOTOR_NUM; i++)
    {
        auto frame = protocol::make_motor_marget_frame(i, msg.data[i]);
        can_pub_->publish(std::move(frame));
    }
}

void Stm32Node::led_callback()
{
}

//
void Stm32Node::alive_timer_callback()
{
    auto frame =
        protocol::encodeKeepAlive();
    can_pub_->publish(std::move(frame));
}

void Stm32Node::publish_timer_callback()
{
    Float32MultiArray msg;

    msg.data.assign(
        current_rpm_.begin(),
        current_rpm_.end());

    rpm_pub_->publish(std::move(msg));
}

int main(int argc, char argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Stm32Node>());
    rclcpp::shutdown();
    return 0;
}