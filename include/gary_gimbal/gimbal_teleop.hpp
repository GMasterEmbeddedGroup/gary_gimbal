#pragma once


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "gary_msgs/msg/dr16_receiver.hpp"
#include "gary_msgs/msg/auto_aim.hpp"
#include "std_msgs/msg/float64.hpp"


using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace gary_gimbal {
    class GimbalTeleop : public rclcpp_lifecycle::LifecycleNode {
    public:
        explicit GimbalTeleop(const rclcpp::NodeOptions & options);

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
        void rc_callback(gary_msgs::msg::DR16Receiver::SharedPtr msg);
        void autoaim_callback(gary_msgs::msg::AutoAIM::SharedPtr msg);

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

        //publishers and subscribers
        rclcpp::Subscription<gary_msgs::msg::DR16Receiver>::SharedPtr rc_sub;
        rclcpp::Subscription<gary_msgs::msg::AutoAIM>::SharedPtr autoaim_sub;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr yaw_set_publisher;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr pitch_set_publisher;

        double pitch_set{};
        double yaw_set{};
        bool use_autoaim{};
    };
}
