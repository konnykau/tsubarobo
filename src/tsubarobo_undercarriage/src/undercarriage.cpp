#include <sensor_msgs/msg/joy.hpp>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
#include "tsubarobo_undercarriage/undercarriage.hpp"
#include "robomas_driver/msg/motor_cmd.hpp"
#include "robomas_driver/msg/motor_cmd_array.hpp"

using std::placeholders::_1;


class Undercarriage_Node: public rclcpp::Node
{
public:
  Undercarriage_Node()
  : Node("tsubarobo_undercarriage"),tsubarobo_undercarriage(undercarriage())
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
      
    }//mode velにする
    if(msg.buttons[6]){//backボタン
      tsubarobo_undercarriage.make_mode(motor_mode::disable);
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
/////////////////////////////
  robomas_pub_1->publish(this->tsubarobo_undercarriage.make_robomas_Frame());

  //////////////////////////////////////////////////////////////////////


}
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  rclcpp::Publisher<robomas_driver::msg::MotorCmdArray>::SharedPtr robomas_pub_1;
  undercarriage tsubarobo_undercarriage;
  rclcpp::Time last_update_time_;  // 前回の更新時刻
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Undercarriage_Node>());
  rclcpp::shutdown();
  return 0;
}
