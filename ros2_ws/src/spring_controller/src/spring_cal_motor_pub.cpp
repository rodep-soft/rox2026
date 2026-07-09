#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"

class SprMot : public rclcpp::Node
{
    public :
    SprMot()
    : Node ("spr_mot")
    {
        this->declare_parameter("max_speed",20.0);
        max_speed_ =
            this->get_parameter("max_speed").as_double();
        spr_sub_ = 
        this->create_subscription<std_msgs::msg::UInt8>(
            "/spring_cmd",
            10,
            std::bind(&SprMot::spring_callback,this,std::placeholders::_1));
        
        limit_sub_ = 
        this -> create_subscription<std_msgs::msg::UInt8MultiArray>(
            "/limit_sw",
            10,
            std::bind(&SprMot::limit_callback,this,std::placeholders::_1));

        spe_pub_ = 
        this->create_publisher<std_msgs::msg::Float32>(
            "/edulite_speed",
            10
            );

        timer_ = 
        this -> create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&SprMot::timer_callback,this));
    }

    private:
    enum State{
        LOAD, //装填中
        FIRE //発射中
    };

    State state_ = LOAD; //state_→状態を表す
    double max_speed_;
    bool fire_request_ = false; //発射命令がtrueかfalseか
    bool limit_on_ = false; //リミットスイッチが押されているかどうか 
    //limit_on_がtrue　→　発射準備完了
    
    int fire_count_ = 0; //発射中にどれぐらい時間がたったか
    void spring_callback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        if(msg -> data == 1){
            fire_request_ = true;
        }
    }

    void limit_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg){
        if(msg->data.size() > 0)
        {
            limit_on_ = msg -> data[0];
        }
    }

    void timer_callback()
    {
        std_msgs::msg::Float32 speed;

        

        switch(state_){
        case LOAD:
            if(limit_on_){ //リミットスイッチが押されている
                speed.data = 0.0;
        
            if(fire_request_){ //発射命令がある
                fire_request_ = false;
                fire_count_ = 0;
                state_ = FIRE;
            }
            }
        else{ //リミットスイッチが押されていない
            speed.data = max_speed_;
        }

        break;

        case FIRE:
            speed.data = max_speed_;
            fire_count_ ++;

            if(fire_count_ > 10){
                state_ = LOAD;
        }
        break;

        default:
            speed.data = 0.0;
            break;
    }
    

    spe_pub_ -> publish(speed);
}
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr spr_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr limit_sub_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr spe_pub_;

    rclcpp::TimerBase::SharedPtr timer_;


};

int main(int argc,char *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<SprMot>());
    rclcpp::shutdown();
    return 0;
}