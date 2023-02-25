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

class GimbalEnterTask : public rclcpp::Node
{
public:
    GimbalEnterTask():Node("gimbal_enter"){
        rc_sub_ = this->create_subscription<gary_msgs::msg::DR16Receiver>("/remote_control",rclcpp::SystemDefaultsQoS(),std::bind(&GimbalEnterTask::rc_callback,this,std::placeholders::_1));
        auto_aim_sub_ = this->create_subscription<gary_msgs::msg::AutoAIM>("/auto_aim",rclcpp::SystemDefaultsQoS(),std::bind(&GimbalEnterTask::auto_aim_callback,this,std::placeholders::_1));//TODO topic待定
        yaw_enter_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/gimbal_yaw_enter",rclcpp::SystemDefaultsQoS());
        pitch_enter_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/gimbal_pitch_enter",rclcpp::SystemDefaultsQoS());
        gimbal_status_publisher_ = this->create_publisher<std_msgs::msg::Int16>("gimbal_status",rclcpp::SystemDefaultsQoS());
    }
private:
    void rc_callback(const gary_msgs::msg::DR16Receiver::SharedPtr msg){
        enter::RC_control = *msg;

        if (enter::RC_control.sw_right == enter::RC_control.SW_MID){
            std_msgs::msg::Float64 yaw_enter;
            std_msgs::msg::Float64 pitch_enter;
            yaw_enter.data = enter::RC_control.ch_right_x;
            yaw_enter_publisher_->publish(yaw_enter);
            pitch_enter.data = enter::RC_control.ch_right_y;
            pitch_enter_publisher_->publish(pitch_enter);
        }

        if (enter::RC_control.sw_right == enter::RC_control.SW_DOWN && enter::RC_control.sw_left == enter::RC_control.SW_DOWN){
            static short i=0;
            if (enter::RC_control.ch_left_x == 1 && enter::RC_control.ch_left_y == -1 && enter::RC_control.ch_right_x == -1 && enter::RC_control.ch_right_y == -1){
                i++;
            }
            if (i >= 2000){
                std_msgs::msg::Int16 gimbal_status;
                gimbal_status.data = 1;
                gimbal_status_publisher_->publish(gimbal_status);
            } else if (i >= 4000 || i == 0){
                std_msgs::msg::Int16 gimbal_status;
                gimbal_status.data = 0;
                gimbal_status_publisher_->publish(gimbal_status);
                i = 0;
            }
        }
    }
    void auto_aim_callback(const gary_msgs::msg::AutoAIM::SharedPtr msg){
        enter::autoAim = *msg;
        if (enter::autoAim.target_id != enter::autoAim.TARGET_ID0_NONE && enter::RC_control.sw_right == enter::RC_control.SW_UP){
            std_msgs::msg::Float64 yaw_enter;
            std_msgs::msg::Float64 pitch_enter;
            yaw_enter.data = enter::autoAim.yaw;
            yaw_enter_publisher_->publish(yaw_enter);
            pitch_enter.data = enter::autoAim.pitch;
            pitch_enter_publisher_->publish(pitch_enter);
        }
    }
    rclcpp::Subscription<gary_msgs::msg::DR16Receiver>::SharedPtr rc_sub_;
    rclcpp::Subscription<gary_msgs::msg::AutoAIM>::SharedPtr auto_aim_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_enter_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pitch_enter_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr gimbal_status_publisher_;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GimbalEnterTask>());
    return 0;
}