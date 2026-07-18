#include "edulite05_protcol.hpp"

Ed05CanframeCreater::Ed05CanframeCreater(uint32_t motor_id)
{
    set_motor_data(motor_id, run_mode);
}

void Ed05CanframeCreater::set_motor_data(uint32_t motor_id)
{
    motor_id_ = motor_id;
    //run_mode_ = run_mode;
}
/*
void Ed05CanframeCreater::gen_enable_frame(uint32_t* canid, uint8_t* data, int* dlc)
{
    *canid = encode_can_id(motor_id);
    *dlc = 8;
    data[0] = 0x3; // Enable command
    for (int i = 1; i < 8; i++)
    {
        data[i] = 0x00; // Fill the rest of the data with zeros
    }
}
    */