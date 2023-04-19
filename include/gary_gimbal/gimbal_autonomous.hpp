#pragma once

#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "gary_msgs/msg/dr16_receiver.hpp"
#include "gary_msgs/msg/auto_aim.hpp"
#include "std_msgs/msg/float64.hpp"
#include "gary_msgs/msg/robot_hurt.hpp"
#include "control_msgs/msg/dynamic_joint_state.hpp"


using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using namespace std::chrono_literals;

namespace gary_gimbal {
    typedef enum{
        MANUAL = 0,
        LOW_SPEED = 1,
        HIGH_SPEED = 2,
        ROTATE = 3,
        AUTO_AIM = 4,
        ZERO_FORCE
    } GimbalStatusEnum;

    constexpr auto no_target_duration_limit = 3000ms;

    class GimbalAutonomous : public rclcpp_lifecycle::LifecycleNode {
    public:
        explicit GimbalAutonomous(const rclcpp::NodeOptions & options);

    private:
        CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

        CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

        CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

        CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

        CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

        void update();

        //callback group
        rclcpp::CallbackGroup::SharedPtr cb_group;

        //callbacks
        void rc_callback(gary_msgs::msg::DR16Receiver::SharedPtr msg);
        void autoaim_callback(gary_msgs::msg::AutoAIM::SharedPtr msg);
        void robot_hurt_callback(gary_msgs::msg::RobotHurt::SharedPtr msg);
        void joint_callback(control_msgs::msg::DynamicJointState::SharedPtr msg);

        //params
        double gimbal_pitch_max{};
        double gimbal_pitch_min{};
        double k_rc{};
        double k_mouse{};
        double k_autoaim{};
        std::string remote_control_topic;
        std::string autoaim_topic;
        std::string yaw_set_topic;
        std::string pitch_set_topic;
        std::string robot_hurt_topic;
        std::string joint_topic;

        //publishers and subscribers
        rclcpp::Subscription<gary_msgs::msg::DR16Receiver>::SharedPtr rc_sub;
        rclcpp::Subscription<gary_msgs::msg::AutoAIM>::SharedPtr autoaim_sub;
        rclcpp::Subscription<gary_msgs::msg::RobotHurt>::SharedPtr robot_hurt_sub;
        rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr joint_subscriber;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr yaw_set_publisher;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr pitch_set_publisher;

        double pitch_set{};
        double yaw_set{};

        gary_msgs::msg::DR16Receiver::_sw_right_type rc_sw_right;
        GimbalStatusEnum GimbalStatus;

        control_msgs::msg::DynamicJointState joint_state;

        //timer
        rclcpp::TimerBase::SharedPtr timer_update;

        std::chrono::steady_clock::time_point last_target_timestamp;

        double rolling_counter;
        double pitch_counter;
        double update_freq;
        double rotate_time;
        double yaw_encoder_bias{};
    };
}
