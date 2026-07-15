#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"

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

    this->declare_parameter<int>("enable_button", 4);       // L2
    this->declare_parameter<int>("storage_button", 1);      // 〇
    this->declare_parameter<int>("intake_button", 0);       // ×
    this->declare_parameter<int>("shoot_button", 2);        // △

    // 起動時に1回だけパラメータを読み込んで、クラスの保管庫（メンバ変数）に保存する
    enable_btn_idx_ = this->get_parameter("enable_button").as_int();
    storage_btn_idx_ = this->get_parameter("storage_button").as_int();
    intake_btn_idx_ = this->get_parameter("intake_button").as_int();
    shoot_btn_idx_ = this->get_parameter("shoot_button").as_int();

    // 初期目標角度を初期位置(storage_angle)に設定
    current_target_angle_ = this->get_parameter("storage_angle").as_double();

    // 2. パブリッシャの作成 (robstride_can_node(position node)に合わせてFloat32/命令topic)
    publisher_ = this->create_publisher<std_msgs::msg::Float32>(
      "/robstride/position_cmd",
      10
    );

    // 3. サブスクライバの作成
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy_second",
      10,
      std::bind(&RollerPositionController::Joycallback, this, std::placeholders::_1)
    );

    // 4. 起動時の初期位置をPublishする
    auto initial_msg = std_msgs::msg::Float32();
    initial_msg.data = static_cast<float>(current_target_angle_);
    publisher_->publish(initial_msg);

    RCLCPP_INFO(this->get_logger(), "roller_position_controller が起動しました。");
  }

private:
  // コールバック関数
  void Joycallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // 【修正】境界チェック：0未満（マイナス）の異常値も確実にはじく安全対策を追加
    if (enable_btn_idx_ < 0 || static_cast<size_t>(enable_btn_idx_) >= msg->buttons.size() ||
      storage_btn_idx_ < 0 || static_cast<size_t>(storage_btn_idx_) >= msg->buttons.size() ||
      intake_btn_idx_ < 0 || static_cast<size_t>(intake_btn_idx_) >= msg->buttons.size() ||
      shoot_btn_idx_ < 0 || static_cast<size_t>(shoot_btn_idx_) >= msg->buttons.size())
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(), 1000, "設定されたボタン番号が不正、またはJoyメッセージのサイズを超えています。");
      return;
    }

    // --- ボタン判定 ---
    if (msg->axes[enable_btn_idx_] <= -0.95) {
      RCLCPP_INFO(this->get_logger(), "反応した");
      // 優先順位に従って判定 (初期位置 > 吸着位置 > 射出位置)
      if (msg->buttons[storage_btn_idx_] == 1) {
        current_target_angle_ = this->get_parameter("storage_angle").as_double();
      } else if (msg->buttons[intake_btn_idx_] == 1) {
        current_target_angle_ = this->get_parameter("intake_angle").as_double();
      } else if (msg->buttons[shoot_btn_idx_] == 1) {
        current_target_angle_ = this->get_parameter("shoot_angle").as_double();
      }
    }

    // --- 目標角度をPublishする ---
    auto output_msg = std_msgs::msg::Float32();
    output_msg.data = static_cast<float>(current_target_angle_);
    publisher_->publish(output_msg);
  }

  // メンバ変数（クラスの保管庫）
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
  double current_target_angle_;   // 前回の目標角度を保持する変数

  int enable_btn_idx_;
  int storage_btn_idx_;
  int intake_btn_idx_;
  int shoot_btn_idx_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RollerPositionController>());
  rclcpp::shutdown();
  return 0;
}
