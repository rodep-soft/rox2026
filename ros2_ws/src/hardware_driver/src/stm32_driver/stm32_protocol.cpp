#include <cstdint>
#include "can_msgs/msg/frame.hpp"

can_msgs::msg::Frame make_motor_target_frame(uint8_t motor , float rpm)
{
    Frame frame;
    frame.id = MOTOR_TARGET_BASE + motor;
    frame.dlc = 4;
    std::memcpy(frame.data.data(),&rpm,sizeof(float));
    return frame;
}

can_msgs::msg::Frame make_alive_frame(){

    return frame;
}

can_msgs::msg::Frame make_led_frame(bool enable){

    return frame;
}

bool decodeMotorCurrent(const Frame & frame,uint8_t & motor,float & rpm)
{
    if(frame.id < MOTOR_CURRENT_BASE)
        return false;

    if(frame.id >=
        MOTOR_CURRENT_BASE + MOTOR_NUM)
        return false;

    motor =
        frame.id - MOTOR_CURRENT_BASE;

    std::memcpy(
        &rpm,
        frame.data.data(),
        sizeof(float));

    return true;
}
