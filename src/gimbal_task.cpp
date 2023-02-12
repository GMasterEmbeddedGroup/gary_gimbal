//
// Created by maackia on 23-2-7.
//
#include <string.h>
#include "rclcpp/rclcpp.hpp"
#include "gary_msgs/msg/dr16_receiver.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "../include/gary_gimbal/gimbal_task.hpp"

using namespace std::chrono_literals;

class GimbalTask : public rclcpp::Node
{
public:
    GimbalTask():Node("gimbal_control"){
        gimbal::init();
        yaw_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/gimbal_yaw_pid/set",rclcpp::SystemDefaultsQoS());//TODO 节点名暂定
        pitch_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/gimbal_pitch_pid/set",rclcpp::SystemDefaultsQoS());//TODO 节点名暂定
        rc_sub_ = this->create_subscription<gary_msgs::msg::DR16Receiver>("/remote_control",rclcpp::SystemDefaultsQoS(),std::bind(&GimbalTask::rc_callback, this, std::placeholders::_1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/gimbal_imu_broadcaster/imu",rclcpp::SystemDefaultsQoS(),std::bind(&GimbalTask::imu_callback,this,std::placeholders::_1));
        timer_ = this->create_wall_timer(1ms, std::bind(&GimbalTask::timer_callback, this));
    }
private:
    void rc_callback(const gary_msgs::msg::DR16Receiver::SharedPtr msg)
    {
        gimbal::RC_control = *msg;
    }
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        gimbal::Imu = *msg;
    }
    void timer_callback()
    {
        gimbal::set_mode();
        gimbal::set_control();
        //gimbal::control_loop();
        if (gimbal::yaw.motor_mode == GIMBAL_MOTOR_RAW){
            client_ = this->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");
            auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
            request->stop_controllers.emplace_back("/gimbal_yaw_control");
            request->strictness = request->BEST_EFFORT;
            auto result = client_->async_send_request(request);
        }
        if (gimbal::pitch.motor_mode == GIMBAL_MOTOR_RAW){
            client_ = this->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");
            auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
            request->stop_controllers.emplace_back("/gimbal_pitch_control");
            request->strictness = request->BEST_EFFORT;
            auto result = client_->async_send_request(request);
        }

        std_msgs::msg::Float64 pid;
        pid.data = gimbal::yaw.absolute_angle_set;
        yaw_publisher_->publish(pid);
        pid.data = gimbal::pitch.absolute_angle_set;
        pitch_publisher_->publish(pid);

    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pitch_publisher_;
    rclcpp::Subscription<gary_msgs::msg::DR16Receiver>::SharedPtr rc_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr client_;
};


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GimbalTask>());
    return 0;
}