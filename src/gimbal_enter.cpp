//
// Created by maackia on 23-2-7.
//

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/float64.hpp"
#include "gary_msgs/msg/dr16_receiver.hpp"
#include "gary_msgs/msg/auto_aim.hpp"
#include "gary_gimbal/gimbal_enter.hpp"

using namespace std::chrono_literals;

float k_rc = 0.08 , k_mk = 0.08; //k_为倍数参数,k_rc最大实践值0.084

double judge_transgress(double gimble_enter){ //越界判断
    while (gimble_enter > PI){
        gimble_enter = gimble_enter - 2*PI;
    }
    while (gimble_enter < -PI){
        gimble_enter = gimble_enter + 2*PI;
    }
    return gimble_enter; //算法缺陷：若拨出10pi，运算后最终也是0pi
}

class GimbalEnterTask : public rclcpp::Node
{
public:
    GimbalEnterTask():Node("gimbal_enter"){
        this->declare_parameter("gimbal_pitch_max",0.72477);//TODO 写入至配置文件
        this->declare_parameter("gimbal_pitch_min",-0.45125);

        rc_sub_ = this->create_subscription<gary_msgs::msg::DR16Receiver>("/remote_control",rclcpp::SystemDefaultsQoS(),std::bind(&GimbalEnterTask::rc_callback,this,std::placeholders::_1));
        auto_aim_sub_ = this->create_subscription<gary_msgs::msg::AutoAIM>("/autoaim/target",rclcpp::SystemDefaultsQoS(),std::bind(&GimbalEnterTask::auto_aim_callback,this,std::placeholders::_1));//TODO topic待定
        yaw_enter_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/gimbal_yaw_enter",rclcpp::SystemDefaultsQoS());
        pitch_enter_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/gimbal_pitch_enter",rclcpp::SystemDefaultsQoS());
    }
private:
    void rc_callback(const gary_msgs::msg::DR16Receiver::SharedPtr msg){
        enter::RC_control = *msg;
        if (enter::RC_control.sw_right == enter::RC_control.SW_MID || enter::RC_control.sw_right == enter::RC_control.SW_UP){
            std_msgs::msg::Float64 yaw_enter;
            std_msgs::msg::Float64 pitch_enter;
            if (enter::RC_control.mouse_x == 0 && enter::RC_control.mouse_y == 0){ //判断鼠标无动作,使用遥控器
                this->yaw_set += enter::RC_control.ch_right_x*k_rc; //最大值1684中间值1024最小值364
                this->pitch_set += enter::RC_control.ch_right_y*k_rc;
                if (this->pitch_set >= this->get_parameter("gimbal_pitch_max").as_double() - 0.05) this->pitch_set = this->get_parameter("gimbal_pitch_max").as_double() - 0.05;
                if (this->pitch_set <= this->get_parameter("gimbal_pitch_min").as_double() + 0.05) this->pitch_set = this->get_parameter("gimbal_pitch_min").as_double() + 0.05;
            }
            else {
                this->yaw_set += enter::RC_control.mouse_x*k_mk; //+-32767静止值0
                this->pitch_set += enter::RC_control.mouse_y*k_mk;
            }
            yaw_enter.data = this->yaw_set;
            pitch_enter.data = this->pitch_set;
            yaw_enter_publisher_->publish(yaw_enter);
            pitch_enter_publisher_->publish(pitch_enter);
        }
    }

    void auto_aim_callback(const gary_msgs::msg::AutoAIM::SharedPtr msg){
        enter::autoAim = *msg;
        std_msgs::msg::Float64 yaw_enter;
        std_msgs::msg::Float64 pitch_enter;

        if (enter::autoAim.target_id != enter::autoAim.TARGET_ID0_NONE && enter::RC_control.sw_right == enter::RC_control.SW_UP){
            this->yaw_set += enter::autoAim.yaw;
            this->pitch_set += enter::autoAim.pitch;

            yaw_enter.data = judge_transgress(this->yaw_set);
            pitch_enter.data = this->pitch_set;
            yaw_enter_publisher_->publish(yaw_enter);
            pitch_enter_publisher_->publish(pitch_enter);
        }
    }
    rclcpp::Subscription<gary_msgs::msg::DR16Receiver>::SharedPtr rc_sub_;
    rclcpp::Subscription<gary_msgs::msg::AutoAIM>::SharedPtr auto_aim_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_enter_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pitch_enter_publisher_;
    double pitch_set;
    double yaw_set;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GimbalEnterTask>());
    return 0;
}