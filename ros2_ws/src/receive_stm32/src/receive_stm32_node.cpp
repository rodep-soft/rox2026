#include <memory>
#include <vector>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

class LimitSwNode : public rclcpp::Node
{
public:
    LimitSwNode() : Node("limit_sw_node")
    {
        // can_sub_ のアンダーバーを追加、canCallback の大文字小文字を修正
        can_sub_ = this->create_subscription<can_msgs::msg::Frame>(
            "/CAN/CAN0/receive", 10,
            std::bind(&LimitSwNode::canCallback, this, std::placeholders::_1));
            
        // UIit8MultiArray -> UInt8MultiArray に修正
        limit_sw_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/limit_sw", 10);

        // ログ出力のドット(.)をカンマ(,)に修正、末尾にセミコロン(;)を追加
        RCLCPP_INFO(this->get_logger(), "Limit Switch Node has been started.");
    }

private:
    void canCallback(const can_msgs::msg::Frame::SharedPtr msg)
    {
        if (msg->id == 0x202) {
            uint8_t raw_data = msg->data[0];
            uint8_t limit_status = raw_data & 0x07;

            // 送信用メッセージの作成
            auto output_msg = std_msgs::msg::UInt8MultiArray();
            output_msg.data.push_back(limit_status);

            // トピックへパブリッシュ
            limit_sw_pub_->publish(output_msg);
            
            RCLCPP_DEBUG(this->get_logger(), "Published limit switch state: %d", limit_status);
        }
    }

    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr limit_sw_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LimitSwNode>());
    rclcpp::shutdown();
    return 0;
}