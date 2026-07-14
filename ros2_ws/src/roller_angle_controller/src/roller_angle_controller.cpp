#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"

class RollerPositionController : public rclcpp::Node
{
public:
  RollerPositionController()
  : Node("roller_position_controller")
  {
    DeclareParameters();
    GetParameters();

    current_target_angle_ = storage_angle_;

    publisher_ = this->create_publisher<std_msgs::msg::Float32>(
      "/robstride/position_cmd",
      10
    );

    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy",
      10,
      std::bind(&RollerPositionController::Joycallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "roller_position_controller が起動しました。");
  }

private:
  void DeclareParameters()
  {
    this->declare_parameter<double>("storage_angle", 0.0);
    this->declare_parameter<double>("intake_angle", 10.0);
    this->declare_parameter<double>("shoot_angle", 2.0);

    this->declare_parameter<int>("enable_button", 4);
    this->declare_parameter<int>("storage_button", 1);
    this->declare_parameter<int>("intake_button", 0);
    this->declare_parameter<int>("shoot_button", 2);

  }

  void GetParameters()
  {
    storage_angle_ = this->get_parameter("storage_angle").as_double();
    intake_angle_ = this->get_parameter("intake_angle").as_double();
    shoot_angle_ = this->get_parameter("shoot_angle").as_double();

    enable_button_idx_ = this->get_parameter("enable_button").as_int();
    storage_btn_idx_ = this->get_parameter("storage_button").as_int();
    intake_btn_idx_ = this->get_parameter("intake_button").as_int();
    shoot_btn_idx_ = this->get_parameter("shoot_button").as_int();

  }

  // 設定されたJoyボタンが届いているかの確認
  bool IsJoyMessageValid(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (!msg) {
      return false;
    }

    if (enable_button_idx_ < 0 || static_cast<size_t>(enable_button_idx_) >= msg->buttons.size() ||
      storage_btn_idx_ < 0 || static_cast<size_t>(storage_btn_idx_) >= msg->buttons.size() ||
      intake_btn_idx_ < 0 || static_cast<size_t>(intake_btn_idx_) >= msg->buttons.size() ||
      shoot_btn_idx_ < 0 || static_cast<size_t>(shoot_btn_idx_) >= msg->buttons.size())
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(), 1000, "設定されたボタン番号が不正、またはJoyメッセージのサイズを超えています。");
      return false;
    }

    return true;
  }

  void Joycallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    std::ostringstream received;
    received << "Received: " << subscription_->get_topic_name() << " axes=[";
    for (size_t i = 0; i < msg->axes.size(); ++i) {
      if (i > 0) {
        received << ", ";
      }
      received << msg->axes[i];
    }
    received << "] buttons=[";
    for (size_t i = 0; i < msg->buttons.size(); ++i) {
      if (i > 0) {
        received << ", ";
      }
      received << msg->buttons[i];
    }
    received << "]";
    RCLCPP_INFO(this->get_logger(), "%s", received.str().c_str());

    if (!IsJoyMessageValid(msg)) {
      return;
    }

    // 角度の安全装置
    if (msg->buttons[enable_button_idx_] == 1) {
      // 一旦、strage < intake < shoot(絶対に変更する！！！)
      if (msg->buttons[storage_btn_idx_] == 1) {
        current_target_angle_ = storage_angle_;
      }

      if (msg->buttons[intake_btn_idx_] == 1) {
        current_target_angle_ = intake_angle_;
      }

      if (msg->buttons[shoot_btn_idx_] == 1) {
        current_target_angle_ = shoot_angle_;
      }
    }

    auto output_msg = std_msgs::msg::Float32();
    output_msg.data = static_cast<float>(current_target_angle_);
    publisher_->publish(output_msg);
  }

  // メンバ変数（クラスの保管庫）
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;

  double storage_angle_;
  double intake_angle_;
  double shoot_angle_;
  double current_target_angle_;

  int enable_button_idx_;
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
