#pragma once
#include <math.h>
#include "tsubarobo_undercarriage/fry_lib/vector.hpp"
#include "tsubarobo_undercarriage/fry_lib/math.hpp"
#include "robomas_driver/msg/motor_cmd.hpp"
#include "robomas_driver/msg/motor_cmd_array.hpp"
#include <cstdint>

namespace constant{
    
}


constexpr float cos30 = FRY::sqrt(3)/2;
constexpr float sin30 = 0.5;

constexpr float max_acceleration_ = 500.0;//to do
constexpr float max_velocity_ = 900.0;//to do

class motor{
    private:
    const FRY::vec2d direction;//モーターの向いている方向ベクトル
    float TARGET;//TARGET
    // float LAST_TARGET; 

    public:
    motor(float x,float y)
    :direction(FRY::vec2d(x,y)),TARGET(0)
    {}//初期化
    void set_target(float power,double dt){
        if(std::fabs(power) > max_velocity_){
            power = max_velocity_ * (power > 0 ? 1 : -1);
        }
        float velocity_difference = power - this->TARGET;
        float max_velocity_change = max_acceleration_ * dt;

        // 最大加速度の制限を超えないように次の速度を計算
        if (std::fabs(velocity_difference) < max_velocity_change) {
            
        } else {
            power = (velocity_difference > 0 ? 1 : -1) * max_velocity_change + this->TARGET;
        }
        // this->LAST_TARGET = (this->TARGET);
        this->TARGET = power;
    }//TARGETを代入
    float make_frame()
    {
        return this->TARGET;
    }
    FRY::vec2d get_vec2d(){
        return this->direction;
    }


};


enum class turn_direction{left_turn, no_turn, right_turn};//回転するかしないか
enum class motor_name{right_front_motor, left_front_motor,back_motor};//モーターの名前
enum class motor_mode{disable,velocity};


class undercarriage{
    private:
    FRY::vec2d direction;//進みたい方向
    
    // motor right_front_motor;
    // motor left_front_motor;
    // motor back_motor;

    motor motors[3];//right_front_motor,left_front_motor,back_motor
    //四輪オムニ    
    motor_mode MODE;
    public:
    
    undercarriage()
    :direction(FRY::vec2d(0,0)),motors{motor(cos30,sin30),motor(-cos30,sin30),motor(0,-1)}
    {
        MODE = motor_mode::disable;
    }//初期化
    void set_motor_power(turn_direction turn_dir,double dt);//4タイヤがうまく回るようにする
    void set_direction(float x,float y);//行きたい方向を設定
    std::unique_ptr<robomas_driver::msg::MotorCmdArray> make_robomas_Frame();//robomas_driversに送るメッセージを作成
    void make_mode(motor_mode motor_state);//modeを設定
    void update(float x,float y,turn_direction turn_dir,double dt);//他の関数を全部融合させた
};

inline void undercarriage::set_direction(float x,float y){
    this->direction.x = x;
    this->direction.y = y;
}

inline std::unique_ptr<robomas_driver::msg::MotorCmdArray> undercarriage::make_robomas_Frame(){
    robomas_driver::msg::MotorCmdArray TARGET_FRAME;
    uint8_t i = 0;
    for(motor m : this->motors){
        TARGET_FRAME.cmds[i].id = i+1;
        TARGET_FRAME.cmds[i].type = "M3508";
        TARGET_FRAME.cmds[i].value = m.make_frame();
        if(this->MODE == motor_mode::velocity){
            TARGET_FRAME.cmds[i].mode = 1;
        }
        else if(this->MODE == motor_mode::disable){
            TARGET_FRAME.cmds[i].mode = 0;
        }
        i++;
    }
    
    return std::make_unique<robomas_driver::msg::MotorCmdArray>(TARGET_FRAME);
}



inline void undercarriage::set_motor_power(turn_direction turn_dir,double dt){
    constexpr float MAX_OF_TARGET = 100.0;
    constexpr float TURN_TARGET = 20;
    //多分TARGETの最大値になるはず
    float TURN_POWER = 0;

    if(turn_dir == turn_direction::left_turn){
        TURN_POWER = -TURN_TARGET;
    }
    else if(turn_dir == turn_direction::right_turn){
        TURN_POWER = TURN_TARGET;
    }
    uint8_t i = 0;
    for(motor m : motors){
        float power = (this->direction * m.get_vec2d()) * MAX_OF_TARGET;
        m.set_target(power + TURN_POWER,dt);
        i++;        
    }
}

inline void undercarriage::make_mode(motor_mode motor_state){
    this->MODE = motor_state;
}

inline void undercarriage::update(float x,float y,turn_direction turn_dir,double dt)
{
    this->set_direction(x,y);
    this->set_motor_power(turn_dir,dt);
}




