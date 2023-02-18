//
// Created by maackia on 23-2-7.
//
#include <string.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int16.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "gary_gimbal/gimbal_control.hpp"

using namespace std::chrono_literals;

void gimbal::init() {
    yaw.given_current = pitch.given_current = 0;
    yaw.add_angle = pitch.add_angle = 0;
    yaw.motor_mode = yaw.last_motor_mode = GIMBAL_MOTOR_RAW;
    pitch.motor_mode = pitch.last_motor_mode = GIMBAL_MOTOR_RAW;
}

void gimbal::behaviour_set() {
//         if (enter::RC_control.sw_left == enter::RC_control.SW_UP){
//             mode.behaviour = GIMBAL_ZERO_FORCE;
//         }
//         if (enter::RC_control.sw_left == enter::RC_control.SW_MID){
//             mode.behaviour = GIMBAL_ABSOLUTE_ANGLE;
//         }
}

void gimbal::set_mode(){
    behaviour_set();
    if (mode.behaviour == GIMBAL_ZERO_FORCE){
        yaw.motor_mode = GIMBAL_MOTOR_RAW;
        pitch.motor_mode =GIMBAL_MOTOR_RAW;
    }
    else if (mode.behaviour == GIMBAL_ABSOLUTE_ANGLE){
        yaw.motor_mode = GIMBAL_MOTOR_GYRO;
        pitch.motor_mode = GIMBAL_MOTOR_GYRO;
    }
}

void gimbal::behaviour_control_set(float add_yaw,float add_pitch){
    if (mode.behaviour == GIMBAL_ZERO_FORCE){
        add_yaw = 0.0f;
        add_pitch = 0.0f;
    }
//        if (mode.behaviour == GIMBAL_ABSOLUTE_ANGLE){
//            add_yaw = enter::RC_control.ch_right_x;
//            add_pitch = enter::RC_control.ch_right_y;
//        }
}

void gimbal::set_control(){
    behaviour_control_set(yaw.add_angle,pitch.add_angle);
    if (yaw.motor_mode == GIMBAL_MOTOR_RAW){
        yaw.raw_cmd_current = yaw.add_angle;
    }
    else if (yaw.motor_mode == GIMBAL_MOTOR_GYRO){
        yaw.absolute_angle_set = yaw.absolute_angle_set + yaw.add_angle;//TODO 防止超限
    }

    if (pitch.motor_mode == GIMBAL_MOTOR_RAW){
        pitch.raw_cmd_current = pitch.add_angle;
    }
    else if (pitch.motor_mode == GIMBAL_MOTOR_GYRO){
        pitch.absolute_angle_set = pitch.absolute_angle_set + pitch.add_angle;//TODO 防止超限
    }
}

void gimbal::control_loop(){
    if (yaw.motor_mode == GIMBAL_MOTOR_RAW){
        yaw.given_current = yaw.raw_cmd_current;
    }
    else if (yaw.motor_mode == GIMBAL_MOTOR_GYRO){
        yaw.given_current = yaw.absolute_angle_set;
    }


    if (pitch.motor_mode == GIMBAL_MOTOR_RAW){
        pitch.given_current = pitch.raw_cmd_current;
    }
    else if (pitch.motor_mode == GIMBAL_MOTOR_GYRO){
        pitch.given_current = pitch.absolute_angle_set;
    }
}

class GimbalTask : public rclcpp::Node
{
public:
    GimbalTask():Node("gimbal_control"){
        gimbal::init();
        yaw_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/gimbal_yaw_pid/set",rclcpp::SystemDefaultsQoS());
        pitch_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/gimbal_pitch_pid/set",rclcpp::SystemDefaultsQoS());
        state_sub_ = this->create_subscription<std_msgs::msg::Int16>("/gimbal_enter",rclcpp::SystemDefaultsQoS(),std::bind(&GimbalTask::state_callback,this,std::placeholders::_1));//TODO 信息类型待定
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/gimbal_imu_broadcaster/imu",rclcpp::SystemDefaultsQoS(),std::bind(&GimbalTask::imu_callback,this,std::placeholders::_1));
        yaw_encoder_sub_ = this->create_subscription<std_msgs::msg::Float64>("/gimbal_yaw/encoder",rclcpp::SystemDefaultsQoS(),std::bind(&GimbalTask::yaw_encoder_callback,this,std::placeholders::_1));
        pitch_encoder_sub_ = this->create_subscription<std_msgs::msg::Float64>("/gimbal_pitch/encoder",rclcpp::SystemDefaultsQoS(),std::bind(&GimbalTask::pitch_encoder_callback,this,std::placeholders::_1));
        timer_ = this->create_wall_timer(1ms, std::bind(&GimbalTask::timer_callback, this));
    }
private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        gimbal::Imu = *msg;//TODO 四元数转换 ROLL,YAW反了
        double yaw,pitch,roll;
        tf2::Quaternion imu_quaternion(gimbal::Imu.orientation.x,gimbal::Imu.orientation.y,gimbal::Imu.orientation.z,gimbal::Imu.orientation.w);
        tf2::Matrix3x3 m(imu_quaternion);
        m.getRPY(roll,pitch,yaw);
        RCLCPP_INFO(this->get_logger(),"yaw:%lf pitch:%lf roll:%lf",yaw,pitch,roll);
    }
    void yaw_encoder_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        gimbal::YawEncoder = *msg;
    }
    void pitch_encoder_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        gimbal::PitchEncoder = *msg;
    }
    void state_callback(const std_msgs::msg::Int16::SharedPtr msg)
    {
        gimbal::state = *msg;//TODO 接受数据调整模式
    }
    void timer_callback()
    {

        gimbal::set_mode();
        gimbal::set_control();
        gimbal::control_loop();

        if (gimbal::mode.behaviour != GIMBAL_ZERO_FORCE){
            std_msgs::msg::Float64 pid;
            pid.data = gimbal::yaw.absolute_angle_set;
            yaw_publisher_->publish(pid);
            pid.data = gimbal::pitch.absolute_angle_set;
            pitch_publisher_->publish(pid);
        }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pitch_publisher_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_encoder_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pitch_encoder_sub_;
};


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GimbalTask>());
    return 0;
}