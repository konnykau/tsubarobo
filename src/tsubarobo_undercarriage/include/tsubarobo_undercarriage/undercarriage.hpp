#pragma once
#include <math.h>
#include "tsubarobo_undercarriage/fry_lib/vector.hpp"
#include "tsubarobo_undercarriage/fry_lib/math.hpp"
#include "robomas_driver/msg/motor_cmd.hpp"
#include "robomas_driver/msg/motor_cmd_array.hpp"
#include <cstdint>


constexpr float cos30 = FRY::sqrt(3)/2;
constexpr float sin30 = 0.5;

constexpr float max_acceleration_ = 3000.0;//to do
constexpr float max_velocity_ = 5000.0;//to do

class motor{
    private:
    const FRY::vec2d direction;//モーターの向いている方向ベクトル
    float TARGET;//TARGET
    // float LAST_TARGET; 
    rclcpp::Logger logger_;
    public:
    motor(float x,float y)
    :direction(FRY::vec2d(x,y)),TARGET(0),logger_(rclcpp::get_logger("motor"))
    {}//初期化
    void set_target(float power,double dt){
        if(std::fabs(power) > max_velocity_){
            power = max_velocity_ * (power > 0 ? 1 : -1);
            RCLCPP_WARN(logger_,"%f , %f vector motor power over max_velocity_",this->direction.x,this->direction.y);
        }
        float velocity_difference = power - this->TARGET;
        float max_velocity_change = max_acceleration_ * dt;

        // 最大加速度の制限を超えないように次の速度を計算
        if (std::fabs(velocity_difference) < max_velocity_change) {
            
        } else {
            power = (velocity_difference > 0 ? 1 : -1) * max_velocity_change + this->TARGET;
            RCLCPP_WARN(logger_,"%f , %f vector motor acceleration over max_acceleration_",this->direction.x,this->direction.y);
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
enum class motor_mode{disable,velocity,position};//モーターのmode


class undercarriage{
    private:
    FRY::vec2d direction;//進みたい方向
    
    // motor right_front_motor;
    // motor left_front_motor;
    // motor back_motor;

    motor motors[3];//right_front_motor,left_front_motor,back_motor
    uint8_t id_[3] = {1,2,3};
    //四輪オムニ    
    motor_mode MODE;
    rclcpp::Logger logger_;
    public:
    
    undercarriage()
    :direction(FRY::vec2d(0,0)),motors{motor(sin30,cos30),motor(sin30,-cos30),motor(-1,0)},logger_(rclcpp::get_logger("undercarriage"))
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
        robomas_driver::msg::MotorCmd cmd;
        cmd.id = this->id_[i];
        cmd.type = "M3508";
        cmd.mode = 1;
        if(this->MODE == motor_mode::velocity){
            cmd.value = m.make_frame();
        }
        else{// if(this->MODE == motor_mode::disable)
            cmd.value = 0;
        }
        RCLCPP_INFO(logger_,"motor id:%d, value:%f",cmd.id,cmd.value);
        TARGET_FRAME.cmds.push_back(cmd);
        i++;
    }
    RCLCPP_INFO(logger_,"-------------------");
    
    return std::make_unique<robomas_driver::msg::MotorCmdArray>(TARGET_FRAME);
}



inline void undercarriage::set_motor_power(turn_direction turn_dir,double dt){
    constexpr float MAX_OF_TARGET = 3000.0;
    constexpr float TURN_TARGET = 1000;
    //多分TARGETの最大値になるはず
    float TURN_POWER = 0;

    if(turn_dir == turn_direction::left_turn){
        TURN_POWER -= TURN_TARGET;
    }
    else if(turn_dir == turn_direction::right_turn){
        TURN_POWER += TURN_TARGET;
    }
    uint8_t i = 0;
    for(motor &m : motors){
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




