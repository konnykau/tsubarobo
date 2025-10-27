#pragma once
#include <math.h>
#include "tsubarobo_undercarriage/undercarriage.hpp"
#include "robomas_driver/msg/motor_cmd.hpp"
#include "robomas_driver/msg/motor_cmd_array.hpp"
#include <cstdint>

// constexpr float launcher_max_acceleration_ = 500.0;//to do
constexpr float rolling_velocity_ = 40000.0;//to do

class launcher{
    private:
    rclcpp::Logger logger_;
    float TARGET;
    uint8_t id_[2];
    bool rolling_direction[2];
    bool is_rolling_;
    motor_mode MODE;

    std::unique_ptr<robomas_driver::msg::MotorCmdArray> make_robomas_Frame(){
        robomas_driver::msg::MotorCmdArray TARGET_FRAME;
        for(uint8_t i = 0;i < 2;i++){
            robomas_driver::msg::MotorCmd cmd;
            cmd.id = this->id_[i];
            cmd.type = "LAUNCHER";
            cmd.mode = 1;
            if(this->MODE == motor_mode::disable){
                cmd.value = 0;
            }
            else{// if(this->MODE == motor_mode::velocity)
                if(this->rolling_direction[i]){
                    cmd.value = this->TARGET;
                }
                else{
                    cmd.value = -this->TARGET;
                }
            }
            TARGET_FRAME.cmds.push_back(cmd);
        }
        return std::make_unique<robomas_driver::msg::MotorCmdArray>(TARGET_FRAME);
    }//make_robomas_Frame
    void roll_launcher(){
        if(this->is_rolling_){
            this->set_target(rolling_velocity_);
        }
        else{
            this->set_target(0.0);
        }
    }//roll_launcher
    
    void set_rolling(bool rolling_state){
        this->is_rolling_ = rolling_state;
    }//set_rolling

    public:

    launcher()
    :logger_(rclcpp::get_logger("launcher")),TARGET(0),id_{4,5},rolling_direction{true,false},is_rolling_(false),MODE(motor_mode::disable)
    {}//初期化

    void set_target(float power){
        this->TARGET = power;
    }//TARGETを代入
    
    void make_mode(motor_mode motor_state){
        this->MODE = motor_state;
    }//modeを設定

    std::unique_ptr<robomas_driver::msg::MotorCmdArray> update(bool rolling_state){
        this->set_rolling(rolling_state);
        this->roll_launcher();
        return this->make_robomas_Frame();
    }//他の関数を全部融合させた

    bool get_is_rolling(){
        return this->is_rolling_;
    }//rollingしているかどうかを返す


};

constexpr float pos_angle_a_ = 245.0;//to do
constexpr float pos_angle_b_ = 0.0;//to do

class loading_mechanism{
    private:
    rclcpp::Logger logger_;
    bool position_;
    uint8_t id_ = 6;
    motor_mode MODE;
    public:
    loading_mechanism()
    :logger_(rclcpp::get_logger("loading_mechanism")),position_(false)
    {}//初期化
    std::unique_ptr<robomas_driver::msg::MotorCmdArray> make_robomas_Frame(){
        robomas_driver::msg::MotorCmdArray TARGET_FRAME;
        robomas_driver::msg::MotorCmd cmd;
        cmd.id = this->id_;
        cmd.type = "M2006";
        cmd.mode = 2;
        if(MODE == motor_mode::disable){
            cmd.value = 0;
        }
        else{// if(this->MODE == motor_mode::position)
            if(this->position_){
                cmd.value = pos_angle_a_;
            }
            else{
                cmd.value = pos_angle_b_;
            }
        }   
        TARGET_FRAME.cmds.push_back(cmd);
        return std::make_unique<robomas_driver::msg::MotorCmdArray>(TARGET_FRAME);
    }//make_robomas_Frame
    void set_position(bool position_state){
        this->position_ = position_state;
    }//set_position
    bool get_position(){
        return this->position_;
    }//get_position
    std::unique_ptr<robomas_driver::msg::MotorCmdArray> update(){
        this->set_position(!this->position_);
        return this->make_robomas_Frame();
    }//update  
    void make_mode(motor_mode motor_state){
        this->MODE = motor_state;
    }//modeを設定
};