#include "edulite05_protcol.hpp"
#include <cmath>

Ed05CanframeCreater::Ed05CanframeCreater(uint8_t motor_id) : motor_id_(motor_id)
{
    //set_param_value();
}

std::vector<Canframe> Velocity::create_init_frame()
{
    std::vector<Canframe> frames;

    //runmode 
    frames.push_back(set_runmode(1));

    // motor enable
    frames.push_back(set_enable());

    // その他制御量の設定
    for (int i = 1; i < targets_info.size; i++) { //velより後のtargetに絞る info.size? .data?
        //frame.id = encode_can_id(0x12);
        //frame.data = encode_commtype18_data(targets_info[i]);
        frames.push_back(set_target_value(targets_info[i]));
    }

    return frames;
}

std::vector<Canframe> Position::create_init_frame()
{
    std::vector<Canframe> frames;

    //disable
    frames.push_back(set_disable());
    //runmode: operation
    frames.push_back(set_runmode(0));
    //ここでenable？
    frames.push_back(set_enable());
    //mechanical zero
    frames.push_back(set_mechanicalzero());
    frames.push_back(set_disable());
    frames.push_back(set_runmode(1));
    frames.push_back(set_enable());
    for(int i = 1; i < targets_info.size; i ++) {
        frames.push_back(create_control_frame(targets_info[i]));
    }
    
    return frames;
}
/*
Canframe Ed05CanframeCreater::create_control_frame(float value)
{
    /*
    Canframe frame;
    frame.id = encode_can_id(0x12);
    frame.dlc = dlc_;

    ControlTargetInfo[0].value = value;
    frame.data = encode_commtype18_data(targets_info[1]);
    //この関数も継承して
    //ここでvalueを範囲に丸め込むのがよいのでは？
    return set_target_value(targets_info[0]);
}
*/
Canframe Velocity::create_control_frame(float value)
{
    targets_info[0].value = std::clamp(value, -50, 50);
    return set_target_value(targets_info[0]);
}

Canframe Position::create_control_frame(float value)
{
    targets_info[0].value = std::clamp(value, -1 * M_PI, M_PI);
    return set_target_value(targets_info[0]);
}

uint32_t Ed05CanframeCreater::encode_can_id(uint8_t commtype_index)
{
    uint32_t id;
    id = commtype_index << 24 | host_id_ << 8 | motor_id_; //多分違う．AIはbit_castをおすすめしたけど...
    
    return id;
}

uint8_t[8] Ed05CanframeCreater::encode_commtype18_data(ControlTargetInfo target_info)
{
    uint8_t[8] data;  //arrayのほうがよかったり？
    uint16_t index = target_info.index;
    //ここで値が範囲外なら知らせておくべきか？
    //float value = std::clamp(target_info.value, target_info.min_value, target_info.max_value);
    float value = std::clamp(target_info.value);
    //data[0] = index | 0xFF;
    encode_data(static_cast<uint32_t> index, 2); 
    uint8_t[2] = 0; //ここbitcastいるかな？ もしくは0x00 2 = 0x0000 は？
    uint8_t[3] = 0;
    encode_data(static_cast<uint32_t> value, 4);

    return data;
}

void Ed05CanframeCreater::encode_data(uint8_t array[], uint32_t data, int byte_num){
    for (int i = 0; i < byte_num; i++){
        array[i] = data | 0xFF;
        data = data << 8;
    }
}

Ed05CanframeCreater::set_runmode(int value)
{
    Canframe frame;
    frame.id = encode_can_id(0x12);
    frame.dlc = dlc_;
    frame.data = encode_commtype18_data(value);
    return frame;
}

Ed05CanframeCreater::set_enable()
{
    Canframe frame;
    frame.id = encode_can_id(0x03);
    frame.dlc = dlc_;
    frame.data = static_cast<uint8_t[8]> 0;
    return frame;
}

Ed05CanframeCreater::set_target_value(ControlTargetInfo target_info)
{
    Canframe frame;
    frame.id = encode_can_id(0x12);
    frame.dlc = dlc_;
    frame.data = encode_commtype18_data(target_info);
    return frame;
}

Ed05CanframeCreater::set_disable()
{
    Canframe frame;
    frame.id = encode_can_id(0x04);
    frame.dlc = dlc_;
    frame.data = 0;
    return frame;
}

Ed05CanframeCreater::set_mechanicalzero()
{
    Canframe frame;
    frame.id = encode_can_id(0x06);
    frame.dlc = dlc_;
    frame.data = 0;
    return frame;
}

