#include <sensor_msgs/msg/joy.hpp>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
#include "tsubarobo_undercarriage/undercarriage.hpp"
#include "tsubarobo_undercarriage/launcher.hpp"
#include "robomas_driver/msg/motor_cmd.hpp"
#include "robomas_driver/msg/motor_cmd_array.hpp"

using std::placeholders::_1;


class Undercarriage_Node: public rclcpp::Node
{
public:
  Undercarriage_Node()
  : Node("tsubarobo_undercarriage"),tsubarobo_undercarriage(undercarriage()),tsubarobo_launcher(launcher())
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&Undercarriage_Node::topic_callback, this, _1));
      
    robomas_pub_1 = this->create_publisher<robomas_driver::msg::MotorCmdArray>("/rm_cmd_array", 10);
    last_update_time_ = this->now(); 
  }
  

private:
  void topic_callback(const sensor_msgs::msg::Joy & msg)
  {  
    if(msg.buttons[7]){//startボタン
      tsubarobo_undercarriage.make_mode(motor_mode::velocity);
      tsubarobo_launcher.make_mode(motor_mode::velocity);
      tsubarobo_loading_mechanism.make_mode(motor_mode::position);
      
    }//mode velにする
    if(msg.buttons[6]){//backボタン
      tsubarobo_undercarriage.make_mode(motor_mode::disable);
      tsubarobo_launcher.make_mode(motor_mode::disable);
      tsubarobo_loading_mechanism.make_mode(motor_mode::position);
    }//mode disにする
///////////////////////////////ここの上がstartボタン、backボタンによるmodeの調整
///////////////////////////////ここの下から平行移動、回転をするための個々のモーターのターゲットを決めるif文

    auto current_time = this->now();
    double dt = (current_time - last_update_time_).seconds();  // 前回からの経過時間
    last_update_time_ = current_time;

    if(msg.axes[2] == -1){//ZL(left shouldderボタン)
        this->tsubarobo_undercarriage.update(-msg.axes[0],msg.axes[1],turn_direction::left_turn,dt);
    }//left turn
    else if(msg.axes[5] == -1){//ZR(right shouldderボタン)
        this->tsubarobo_undercarriage.update(-msg.axes[0],msg.axes[1],turn_direction::right_turn,dt);
    }//right turn
    else{
        this->tsubarobo_undercarriage.update(-msg.axes[0],msg.axes[1],turn_direction::no_turn,dt);
    }//平行移動//x軸はjoyの入力時点で反転していた

  robomas_pub_1->publish(this->tsubarobo_undercarriage.make_robomas_Frame());

  //////////////////////////////////////////////////////////////////////以上足回り

  //////////////////////////////////////////////////////////////////////以下射出
    if(msg.buttons[0]){//Aボタン
      if(!a_button_pushing_){
        a_button_pushing_ = true;
        if(this->tsubarobo_launcher.get_is_rolling()){//rolling中なら止める
          robomas_pub_1->publish(this->tsubarobo_launcher.update(false,dt));
          RCLCPP_INFO(this->get_logger(),"stop rolling");
        }
        else{//rollingしていないなら始める
          robomas_pub_1->publish(this->tsubarobo_launcher.update(true,dt));
          RCLCPP_INFO(this->get_logger(),"start rolling");
        }
      }
      else{
        // ボタンが押され続けている場合は何もしない
      }
    }
    else{//Aボタンが押されていないとき
      a_button_pushing_ = false;
    }
  //////////////////////////////////////////////////////////////////////以上射出
  //////////////////////////////////////////////////////////////////////以下装填
  if(msg.buttons[1]){//Bボタン
      if(!b_button_pushing_){
        b_button_pushing_ = true;
        robomas_pub_1->publish(this->tsubarobo_loading_mechanism.update());
        RCLCPP_INFO(this->get_logger(),"change loading position");
      }
      else{
        // ボタンが押され続けている場合は何もしない
      }
    }
    else{//Bボタンが押されていないとき
      b_button_pushing_ = false;
    }
    


}
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  rclcpp::Publisher<robomas_driver::msg::MotorCmdArray>::SharedPtr robomas_pub_1;
  undercarriage tsubarobo_undercarriage;
  launcher tsubarobo_launcher;
  bool a_button_pushing_ = false;
  loading_mechanism tsubarobo_loading_mechanism;
  bool b_button_pushing_ = false;
  rclcpp::Time last_update_time_;  // 前回の更新時刻
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Undercarriage_Node>());
  rclcpp::shutdown();
  return 0;
}
