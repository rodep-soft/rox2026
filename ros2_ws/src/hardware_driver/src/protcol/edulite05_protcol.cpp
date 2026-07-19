#include "edulite05_protcol.hpp"

Ed05CanframeCreater::Ed05CanframeCreater(uint8_t motor_id) : motor_id_(motor_id)
{
    //set_param_value();
}

std::vector<Canframe> Ed05CanframeCreater::create_init_frame()
{
    std::vector<Canframe> frames;

    Canframe frame;
    // run mode を設定;
    frame.id = encode_can_id(0x12);
    frame.dlc = dlc_;
    frame.data = encode_commtype18_data(targets_info[0]);
    frames.push_back(frame);

    // motor enable
    frame.id = encode_can_id(0x03);
    frame.data = 0 //bitcast<uint8_t>は必要？
    frames.push_back(frame);

    // その他制御量の設定
    for (int i = 2; i < targets_info.data; i++) { //2: runmode, vel/pos より後のtargetに絞る
        frame.id = encode_can_id(0x12);
        frame.data = encode_commtype18_data(targets_info[i]);
        frames.push_back(frame);
    }

    return frames;
}

Canframe Ed05CanframeCreater::create_control_frame(float value)
{
    Canframe frame;
    frame.id = encode_can_id(0x12);
    frame.dlc = dlc_;

    ControlTargetInfo[0].value = value;
    frame.data = encode_commtype18_data(targets_info[1]);

    return frame;
}

uint32_t encode_can_id(uint8_t commtype_index)
{
    uint32_t id;
    id = commtype_index << 24 | host_id_ << 8 | motor_id_; //多分違う．AIはbit_castをおすすめしたけど...
    
    return id;
}

uint8_t[8] encode_commtype18_data(ControlTargetInfo target_info)
{
    uint8_t[8] data;  //arrayのほうがよかったり？
    uint16_t index = target_info.index;
    float value = std::clamp(target_info.value, target_info.min_value, target_info.max_value);

    data[0] 
}