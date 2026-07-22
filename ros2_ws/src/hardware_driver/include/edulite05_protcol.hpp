#include <stdint.h>

struct Canframe
{
    id;
    dlc;  //dlc 8bit固定だけど，やっぱこっちで書いたほうがいいかな？ 
    data;
    //extendedのほうも書いたほうがいい？
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
        Ed05CanframeCreater(uint8_t motor_id) : motor_id_(motor_id);
        ~Ed05CanframeCreater();

        virtual std::vector<Canframe> create_init_frame() = 0;
        //Canframe create_control_frame(float value);
        virtual Canframe create_control_frame(float value) = 0;

        //set_param_value(char[8] param_name, float value); //加速度制限とか変更したかったらこれ使ってがんば;

    private:
        uint8_t motor_id_;
        int dlc_ = 8;
        uint8_t host_id_ = 0xFD;
        runmode = 0; //runmodeを設定するコマンドで使用．vel:

        /*
        enum class ControlTarget
        {
            main,
        };
        */
        struct ControlTargetInfo  
        {
            char[12] name;
            uint16_t index; //commtype18で使用
            float value;
            //float max_value;   //maxとmin消すべきかも．vel/posだけ処理中でやるべきかも
            //float min_value;
        };
        //std::unordered_map<ControlTarget, ControlTargetInfo> control_info;


        //void set_motor_data(uint32_t motor_id, ); //motor_id, runmode, main_param_indexを設定  dlcやhost_idもここでやってもいいけど必要ないからいいや
        //void gen_runmode_frame(int mode);  //comm type 18 : operation mode
        //void gen_enable_frame(uint32_t* canid, uint8_t* data);  //comm type 3 : enable
        //void gen_disable_frame();
        
        uint32_t encode_can_id(uint8_t commtype_index);
        uint8_t[8] encode_commtype18_data(ControlTargetInfo target_info);  //comm type 18
        void encode_data(uint8_t array[], uint32_t data, int byte_num); //リトルエンディアン

        Canframe set_runmode(int value);  //setするわけじゃないから名前変えるべき
        Canframe set_enable();            // でもcanframeCreaterだから自明っちゃ自明なんよな
        Canframe set_target_value(ControlTargetInfo target_info);
        Canframe set_disable();
        Canframe set_mechanicalzero();
        //int encode_data(uint8_t* data);


}

class Velocity : public Ed05CanframeCreater
{
    public:

    private:

    std::vector<Canframe> create_init_frame();
    std::vector<Canframe> create_control_frame(float value);

    std::array<ControlTargetInfo, 3> targets_info = {
        //{"runmode", 0x7005, 2.0, 6, 0}, //これここにいれるのキモくはある
        {"vel", 0x700A, 0.0}, //ここにはメインのtarget  後を文字列で検索のがきれいではあるんよな
        {"acc", 0x7022, 100},
        {"limit_cur", 0x7018, 5},
    }    
}

class Position : public Ed05CanframeCreater
{
    private:
    std::vector<Canframe> create_init_frame();
    std::vector<Canframe> create_control_frame(float value);

    std::array<ControlTargetInfo, 6> targets_info = {
        //{"runmode", 0x7005, 1.0, 6, 0},
        {"loc_ref", 0x7016, 0},
        {"limit_cur", 0x7018, 5},
        {"vel_max", 0x7024, 50},
        {"acc_set", 0x7025, 100},
        {"zero_sta", 0x7029, 1},


    }
}
