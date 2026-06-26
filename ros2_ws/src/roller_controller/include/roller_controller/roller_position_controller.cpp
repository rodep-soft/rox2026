#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64.hpp"

class RollerPositionController : public rclcpp::Node
{
public:
    RollerPositionController()
    : Node("roller_position_controller")
    {
        // 1. パラメータの宣言と初期値の設定（configファイルから上書き可能になります）
        this->declare_parameter<double>("storage_angle", 0.0);
        this->declare_parameter<double>("intake_angle", 30.0);
        this->declare_parameter<double>("shoot_angle", 60.0);
        
        this->declare_parameter<int>("enable_button", 6);   // L2
        this->declare_parameter<int>("storage_button", 1);  // 〇
        this->declare_parameter<int>("intake_button", 0);   // ×
        this->declare_parameter<int>("shoot_button", 2);    // △

        // 初期目標角度を初期位置(storage_angle)に設定
        current_target_angle_ = this->get_parameter("storage_angle").as_double();

        // 2. パブリッシャの作成 (std_msgs::msg::Float64型)
        publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "/roller/robostride/target_position", 
            10
        );

        // 3. サブスクライバの作成
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy",
            10,
            std::bind(&RollerPositionController::topic_callback, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(this->get_logger(), "roller_position_controller が起動しました。");
    }

private:
    void topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // 最新のパラメータ値を読み込む
        int enable_btn_idx = this->get_parameter("enable_button").as_int();
        int storage_btn_idx = this->get_parameter("storage_button").as_int();
        int intake_btn_idx = this->get_parameter("intake_button").as_int();
        int shoot_btn_idx = this->get_parameter("shoot_button").as_int();

        // 境界チェック（配列の要素数を超えてアクセスしないための安全対策）
        if (static_cast<size_t>(enable_btn_idx) >= msg->buttons.size() ||
            static_cast<size_t>(storage_btn_idx) >= msg->buttons.size() ||
            static_cast<size_t>(intake_btn_idx) >= msg->buttons.size() ||
            static_cast<size_t>(shoot_btn_idx) >= msg->buttons.size()) 
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "設定されたボタン番号がJoyメッセージのサイズを超えています。");
            return;
        }

        // --- ボタン判定 ---
        // L2ボタンが押されているか確認（押されていなければ何もしない＝前回の目標角度を維持）
        if (msg->buttons[enable_btn_idx] == 1) 
        {
            // 優先順位に従って判定 (初期位置 > 吸着位置 > 射出位置)
            if (msg->buttons[storage_btn_idx] == 1) 
            {
                current_target_angle_ = this->get_parameter("storage_angle").as_double();
            }
            else if (msg->buttons[intake_btn_idx] == 1) 
            {
                current_target_angle_ = this->get_parameter("intake_angle").as_double();
            }
            else if (msg->buttons[shoot_btn_idx] == 1) 
            {
                current_target_angle_ = this->get_parameter("shoot_angle").as_double();
            }
        }

        // --- 目標角度をPublishする ---
        auto output_msg = std_msgs::msg::Float64();
        output_msg.data = current_target_angle_;
        publisher_->publish(output_msg);
    }

    // メンバ変数
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    double current_target_angle_; // 前回の目標角度を保持する変数
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RollerPositionController>());
    rclcpp::shutdown();
    return 0;
}