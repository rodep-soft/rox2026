#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "spring_controller/spring_controller_state.h"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"

class SpringEduliteController : public rclcpp::Node
{ // なんか位置制御がわんちゃんいけそうだから変更かも
public:
  SpringEduliteController()
  : Node("spring_edulite_controller")
  {
    this->declare_parameter("fire_speed_", -20.0);
    this->declare_parameter("loading_speed", -5.0);
    this->declare_parameter("limit_sw_index", 1);
    this->declare_parameter("fire_duration", 5.0);
    this->declare_parameter("publish_edulite", "/spring_edulite_speed");

    fire_speed_ = this->get_parameter("fire_speed_").as_double();
    loading_speed_ = this->get_parameter("loading_speed").as_double();
    limit_sw_index_ = this->get_parameter("limit_sw_index").as_int();
    fire_duration_ = this->get_parameter("fire_duration").as_double();
    std::string publish_edulite_ = this->get_parameter("publish_edulite").as_string();


    spr_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
      "/spring_cmd",
      10,
      std::bind(&SpringEduliteController::spring_callback, this, std::placeholders::_1));

    limit_sw_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
      "/limit_sw",
      10,
      std::bind(&SpringEduliteController::limit_callback, this, std::placeholders::_1));

    speed_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      publish_edulite_,
      10);

    // 10msごとにモーター制御管理コールバックを呼び出す
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&SpringEduliteController::timer_callback, this));
  }

private:
  State now_state_ = State::STOP;      // 制御状態を表す
  State target_state_ = State::STOP;   // 目標状態

  double fire_speed_;       // 発射時のモーター回転速度
  double loading_speed_;   // 装填時のモーター回転速度
  int limit_sw_index_;     // limit_swのインデックス番号
  bool limit_sw_ = false;       // limit_swの状態

  rclcpp::Time fire_start;   // 発射開始時の時間
  double fire_duration_;   // 発射時間の長さ[s]

  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr spr_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr limit_sw_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void spring_callback(const std_msgs::msg::UInt8::SharedPtr msg)
  {                                                    // 制御コマンドの受け取り
    target_state_ = static_cast<State>(msg->data);     // 受け取ったコマンドに応じて状態を変更
    if (target_state_ == State::STOP) { // 停止コマンドは即時適用
      now_state_ = State::STOP;
    } else if (target_state_ == State::LOAD && now_state_ == State::STOP) { // 停止状態から発射命令が来た場合は装填状態に遷移
      now_state_ = State::LOAD;
    }
  }

  void limit_callback(const std_msgs::msg::UInt8::SharedPtr msg)
  {   // limit_swのnbit目のデータを受け取る
    if (limit_sw_index_ < 8) {   // indexが有効な範囲内か確認
      limit_sw_ = (msg->data >> limit_sw_index_) & 1;       // 指定されたビットの値を取得
    }
  }

  void timer_callback()
  {   // モーター制御コールバック
    std_msgs::msg::Float32 speed;

    switch (now_state_) {
      case State::STOP:
        speed.data = 0.0;
        break;
      case State::LOAD:
        if (limit_sw_ == true) {   // limit_swが押されている
          speed.data = 0.0;
          // 装填完了の状態
          if (target_state_ == State::FIRE) { // 発射命令がある
            now_state_ = State::FIRE;         // 装填状態に戻す
            fire_start = this->now();                  // 発射開始時間を記録
          }
        } else { // limit_swが押されていない
          // 装填中は発射の命令を無視
          speed.data = loading_speed_;
        }
        break;
      case State::FIRE:   // 発射中
        speed.data = fire_speed_;
        if ((this->now() - fire_start).seconds() >= fire_duration_) {
          now_state_ = State::LOAD;
        }
        break;
      default:
        speed.data = 0.0;
        break;
    }

    RCLCPP_DEBUG(
      this->get_logger(), "now_state: %d, target_state: %d, speed: %f, limit_sw: %d", now_state_, target_state_, speed.data,
      limit_sw_);
    speed_pub_->publish(speed);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpringEduliteController>());
  rclcpp::shutdown();
  return 0;
}
