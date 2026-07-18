#include <stdint.h>

struct can_frame
{
    id;
    dlc;
    data;
};

/*
struct motor 
{
    uint8_t motor_id;
    Runmode run_mode;
};

enum class Runmode
{
    velocity,
    position,
    operation, //??
    disable,
};
*/

class Ed05CanframeCreater
{
    public:
        Ed05CanframeCreater(uint32_t motor_id);
        ~Ed05CanframeCreater();

        virtual can_frame create_init_frame() = 0;
        can_frame create_update_mainparam_frame(float data);

        set_param_value(char param_name, float value); //加速度制限とか変更したかったらこれ使ってがんば

    private:
        int motor_id_;
        //int run_mode_;

        int dlc_ = 8;
        uint8_t host_id_ = 0xFD;

        //int Index //commtype18で使うindex pair使うつもり いやでも，index data name３つを同時に管理したいな
        enum class ControlTarget
        {
            main,
        };
        struct ControlTargetInfo
        {
            uint8_t[2] index; //commtype18で使用
            float value;
            float max_value;
            float min_value;
        };
        std::unordered_map<ControlTarget, ControlTargetInfo> control_info;


        void set_motor_data(uint32_t motor_id, ); //motor_id, runmode, main_param_indexを設定  dlcやhost_idもここでやってもいいけど必要ないからいいや
        //void gen_runmode_frame(int mode);  //comm type 18 : operation mode
        //void gen_enable_frame(uint32_t* canid, uint8_t* data);  //comm type 3 : enable
        //void gen_disable_frame();
        
        uint32_t encode_can_id(int commtype_index);
        uint8_t[8] encode_commtype18_data(float value);  //comm type 18

        //int encode_data(uint8_t* data);


}

class Velocity : public Ed05CanframeCreater
{
    public:
    can_frame create_init_frame(); //オーバーライド 
    //canframe create_update_mainparam_frame();

    private:
    Index   //index, data, 項目名の３つをどう管理するか？
    vel = 0;
    int acc_ = 100;
    int cur_ = 11;//?
    enum class ControlTarget
    {
        main,
        acc,
        cur,//?
    };
    
    std::unordered_map<ControlTarget, ControlTargetInfo> control_info = {
        {ControlTarget::main, {0x700A, 0.0, 50.0, -50.0}},
        {ControlTarget::acc, {0x0, 100, 1000, 0}},
        {ControlTarget::cur,{, , 11, 0}}
    }

    


    
}

class Position : public Ed05CanframeCreater
{

}
