#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "gary_msgs/msg/dual_loop_pid_with_filter.hpp"


using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


namespace gary_gimbal {
    class GimbalControl : public rclcpp_lifecycle::LifecycleNode {

    public:
        explicit GimbalControl(const rclcpp::NodeOptions & options);

    private:
        CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

        CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

        CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

        CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

        CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

        //callback group
        rclcpp::CallbackGroup::SharedPtr cb_group;

        //callbacks
        void imu_callback(sensor_msgs::msg::Imu::SharedPtr msg);
        void joint_callback(control_msgs::msg::DynamicJointState::SharedPtr msg);
        void gimbal_yaw_callback(std_msgs::msg::Float64::SharedPtr msg);
        void gimbal_pitch_callback(std_msgs::msg::Float64::SharedPtr msg);
        void yaw_pid_callback(gary_msgs::msg::DualLoopPIDWithFilter::SharedPtr msg);

        //params
        double gimbal_pitch_max{};
        double gimbal_pitch_min{};
        double gimbal_yaw_ecd_transform{};
        double pitch_soft_limit{};
        std::string pitch_publish_topic;
        std::string yaw_pid_subscribe_topic;
        std::string yaw_publish_topic;
        std::string pitch_subscribe_topic;
        std::string yaw_subscribe_topic;
        std::string imu_subscribe_topic;
        std::string joint_subscribe_topic;

        //publishers and subscribers
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr yaw_pid_publisher;
        rclcpp::Subscription<gary_msgs::msg::DualLoopPIDWithFilter>::SharedPtr yaw_pid_subscriber;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr pitch_pid_publisher;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_cmd_sub;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pitch_cmd_sub;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
        rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr joint_sub;

        double imu_yaw_angle_pre{};
        double motor_yaw_angle_pre{};
        bool imu_data_available{};
        bool joint_data_available{};
        float gimbal_yaw_position{};
    };
}
