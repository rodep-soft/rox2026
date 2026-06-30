#include <memory>
#include "angle_motor_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "angle_motor/angle_motor_node.hpp"

RollerControllerNode::RollerControllerNode() : Node("roller_controller_node") {
    declareParameters();
    getParameters();

    current_command_.mode = RotationMode::Stop;
    current_command_.pwm_value = stop_pwm_;

    // パブリッシャ（マブチ用仕様）とサブスクライバの初期化
    pwm_publisher_ = this->create_publisher<std_msgs::msg::Int16>("/mabuchi555/pwm_value", 10);
    joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&RollerControllerNode::joyCallback, this, std::placeholders::1));

    RCLCPP_INFO(this->get_logger(), "RollerControllerNode has been initialized with Mabuchi specs.");
}

void RollerControllerNode::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
    // 異常設定への安全チェック（配列の範囲外アクセス防止）
    if (enable_button_ < 0 || static_cast<size_t>(enable_button_) >= joy_msg->buttons.size() ||
        positive_button_ < 0 || static_cast<size_t>(positive_button_) >= joy_msg->buttons.size() ||
        negative_button_ < 0 || static_cast<size_t>(negative_button_) >= joy_msg->buttons.size() ||
        stop_button_ < 0 || static_cast<size_t>(stop_button_) >= joy_msg->buttons.size()) 
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Button index out of bounds! Forced to Stop.");
        current_command_.mode = RotationMode::Stop;
        current_command_.pwm_value = stop_pwm_;
    } 
    else 
    {
        // 判定ロジックと計算の呼び出し
        current_command_.mode = determineRotationMode(joy_msg, current_command_.mode);
        current_command_.pwm_value = getPwmValueFromMode(current_command_.mode);
    }

    // PWM値をパブリッシュ
    auto pwm_msg = std_msgs::msg::Int16();
    pwm_msg.data = current_command_.pwm_value;
    pwm_publisher_->publish(pwm_msg);
}

// ボタン入力から次のモードを判定する関数（優先順位: Stop > Negative > Positive）
RotationMode RollerControllerNode::determineRotationMode(const sensor_msgs::msg::Joy::SharedPtr joy_msg, RotationMode previous_mode) const {
    if (isButtonPressed(joy_msg, enable_button_)) {
        if (isButtonPressed(joy_msg, stop_button_)) {
            return RotationMode::Stop;
        } else if (isButtonPressed(joy_msg, negative_button_)) {
            return RotationMode::NegativeRotate;
        } else if (isButtonPressed(joy_msg, positive_button_)) {
            return RotationMode::PositiveRotate;
        }
    }
    return previous_mode;
}

// モードに対応するPWM値を返す関数
int16_t RollerControllerNode::getPwmValueFromMode(RotationMode mode) const {
    switch (mode) {
        case RotationMode::Stop:           return stop_pwm_;
        case RotationMode::PositiveRotate: return positive_pwm_;
        case RotationMode::NegativeRotate: return negative_pwm_;
        default:                           return 0;
    }
}

// 安全にボタン状態をチェックする関数
bool RollerControllerNode::isButtonPressed(const sensor_msgs::msg::Joy::SharedPtr joy_msg, int button_index) const {
    if (button_index < 0 || static_cast<size_t>(button_index) >= joy_msg->buttons.size()) {
        return false;
    }
    return joy_msg->buttons[button_index] == 1;
}

// パラメータ宣言
void RollerControllerNode::declareParameters() {
    this->declare_parameter<int>("enable_button", 6);
    this->declare_parameter<int>("positive_button", 2);
    this->declare_parameter<int>("negative_button", 0);
    this->declare_parameter<int>("stop_button", 1);
    this->declare_parameter<int>("positive_pwm", 200);
    this->declare_parameter<int>("negative_pwm", -200);
    this->declare_parameter<int>("stop_pwm", 0);
}

// パラメータ取得
void RollerControllerNode::getParameters() {
    this->get_parameter("enable_button", enable_button_);
    this->get_parameter("positive_button", positive_button_);
    this->get_parameter("negative_button", negative_button_);
    this->get_parameter("stop_button", stop_button_);
    this->get_parameter("positive_pwm", positive_pwm_);
    this->get_parameter("negative_pwm", negative_pwm_);
    this->get_parameter("stop_pwm", stop_pwm_);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RollerControllerNode>());
    rclcpp::shutdown();
    return 0;
}