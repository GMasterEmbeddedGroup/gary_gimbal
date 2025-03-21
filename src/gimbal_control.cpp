#include "gary_gimbal/gimbal_control.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace gary_gimbal;


GimbalControl::GimbalControl(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("gimbal_control",
                                                                                                   options) {
    //declare params
    this->declare_parameter("gimbal_pitch_max", 0.0);
    this->declare_parameter("gimbal_pitch_min", 0.0);
    this->declare_parameter("gimbal_yaw_ecd_transform", 0.0);
    this->declare_parameter("pitch_soft_limit", 0.0);
    this->declare_parameter("pitch_publish_topic", "/gimbal_pitch_pid/cmd");
    this->declare_parameter("yaw_pid_subscribe_topic", "/gimbal_yaw_pid/pid");
    this->declare_parameter("yaw_publish_topic", "/yaw_pitch_pid/cmd");
    this->declare_parameter("pitch_subscribe_topic", "/gimbal_pitch_set");
    this->declare_parameter("yaw_subscribe_topic", "/gimbal_yaw_set");
    this->declare_parameter("imu_subscribe_topic", "/gimbal_imu_broadcaster/imu");
    this->declare_parameter("joint_subscribe_topic", "/dynamic_joint_states");
}

CallbackReturn GimbalControl::on_configure(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //create callback group
    rclcpp::CallbackGroup::SharedPtr cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = cb_group;

    //get gimbal_pitch_max
    this->gimbal_pitch_max = this->get_parameter("gimbal_pitch_max").as_double();

    //get gimbal_pitch_min
    this->gimbal_pitch_min = this->get_parameter("gimbal_pitch_min").as_double();

    //get gimbal_yaw_ecd_transform
    this->gimbal_yaw_ecd_transform = this->get_parameter("gimbal_yaw_ecd_transform").as_double();

    //get pitch_soft_limit
    this->pitch_soft_limit = this->get_parameter("pitch_soft_limit").as_double();

    //get pitch_publish_topic
    this->pitch_publish_topic = this->get_parameter("pitch_publish_topic").as_string();
    this->pitch_pid_publisher = this->create_publisher<std_msgs::msg::Float64>(this->pitch_publish_topic,
                                                                               rclcpp::SystemDefaultsQoS());

    //get yaw_pid_subscribe_topic
    this->yaw_pid_subscribe_topic = this->get_parameter("yaw_pid_subscribe_topic").as_string();
    this->yaw_pid_subscriber = this->create_subscription<gary_msgs::msg::DualLoopPIDWithFilter>(
            this->yaw_pid_subscribe_topic, rclcpp::SystemDefaultsQoS(),
            std::bind(&GimbalControl::yaw_pid_callback, this, std::placeholders::_1), sub_options);

    //get yaw_publish_topic
    this->yaw_publish_topic = this->get_parameter("yaw_publish_topic").as_string();
    this->yaw_pid_publisher = this->create_publisher<std_msgs::msg::Float64>(this->yaw_publish_topic,
                                                                             rclcpp::SystemDefaultsQoS());

    //get pitch_subscribe_topic
    this->pitch_subscribe_topic = this->get_parameter("pitch_subscribe_topic").as_string();
    this->pitch_cmd_sub = this->create_subscription<std_msgs::msg::Float64>(
            this->pitch_subscribe_topic, rclcpp::SystemDefaultsQoS(),
            std::bind(&GimbalControl::gimbal_pitch_callback, this, std::placeholders::_1), sub_options);

    //get yaw_subscribe_topic
    this->yaw_subscribe_topic = this->get_parameter("yaw_subscribe_topic").as_string();
    this->yaw_cmd_sub = this->create_subscription<std_msgs::msg::Float64>(
            this->yaw_subscribe_topic, rclcpp::SystemDefaultsQoS(),
            std::bind(&GimbalControl::gimbal_yaw_callback, this, std::placeholders::_1), sub_options);

    //get imu_subscribe_topic
    this->imu_subscribe_topic = this->get_parameter("imu_subscribe_topic").as_string();
    this->imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            this->imu_subscribe_topic, rclcpp::SystemDefaultsQoS(),
            std::bind(&GimbalControl::imu_callback, this, std::placeholders::_1), sub_options);

    //get joint_subscribe_topic
    this->joint_subscribe_topic = this->get_parameter("joint_subscribe_topic").as_string();
    this->joint_sub = this->create_subscription<control_msgs::msg::DynamicJointState>(
            this->joint_subscribe_topic, rclcpp::SystemDefaultsQoS(),
            std::bind(&GimbalControl::joint_callback, this, std::placeholders::_1), sub_options);

    RCLCPP_INFO(this->get_logger(), "configured");
    return CallbackReturn::SUCCESS;
}


CallbackReturn GimbalControl::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //destroy objects
    this->yaw_pid_publisher.reset();
    this->pitch_pid_publisher.reset();
    this->yaw_cmd_sub.reset();
    this->pitch_cmd_sub.reset();
    this->imu_sub.reset();
    this->joint_sub.reset();

    RCLCPP_INFO(this->get_logger(), "cleaning up");
    return CallbackReturn::SUCCESS;
}


CallbackReturn GimbalControl::on_activate(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //activate lifecycle publisher
    this->yaw_pid_publisher->on_activate();
    this->pitch_pid_publisher->on_activate();

    RCLCPP_INFO(this->get_logger(), "activated");
    return CallbackReturn::SUCCESS;
}


CallbackReturn GimbalControl::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //deactivate lifecycle publisher
    this->yaw_pid_publisher->on_deactivate();
    this->pitch_pid_publisher->on_deactivate();

    RCLCPP_INFO(this->get_logger(), "deactivated");
    return CallbackReturn::SUCCESS;
}


CallbackReturn GimbalControl::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //destroy objects
    if (this->yaw_pid_publisher.get() != nullptr) this->yaw_pid_publisher.reset();
    if (this->pitch_pid_publisher.get() != nullptr) this->pitch_pid_publisher.reset();
    if (this->yaw_cmd_sub.get() != nullptr) this->yaw_cmd_sub.reset();
    if (this->pitch_cmd_sub.get() != nullptr) this->pitch_cmd_sub.reset();
    if (this->imu_sub.get() != nullptr) this->imu_sub.reset();
    if (this->joint_sub != nullptr) this->joint_sub.reset();

    RCLCPP_INFO(this->get_logger(), "shutdown");
    return CallbackReturn::SUCCESS;
}


CallbackReturn GimbalControl::on_error(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //destroy objects
    if (this->yaw_pid_publisher.get() != nullptr) this->yaw_pid_publisher.reset();
    if (this->pitch_pid_publisher.get() != nullptr) this->pitch_pid_publisher.reset();
    if (this->yaw_cmd_sub.get() != nullptr) this->yaw_cmd_sub.reset();
    if (this->pitch_cmd_sub.get() != nullptr) this->pitch_cmd_sub.reset();
    if (this->imu_sub.get() != nullptr) this->imu_sub.reset();
    if (this->joint_sub != nullptr) this->joint_sub.reset();

    RCLCPP_INFO(this->get_logger(), "error");
    return CallbackReturn::SUCCESS;
}

void GimbalControl::yaw_pid_callback(gary_msgs::msg::DualLoopPIDWithFilter::SharedPtr msg) {
    this->gimbal_yaw_position = msg->outer_feedback;
}


void GimbalControl::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    double yaw, pitch, roll;

    if(msg->orientation.x == 0.0f && msg->orientation.y == 0.0f && msg->orientation.z == 0.0f && msg->orientation.w == 0.0f) return;

    //quat to euler
    tf2::Quaternion imu_quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf2::Matrix3x3 m(imu_quaternion);
    m.getRPY(roll, pitch, yaw);

    //log first data
    this->imu_yaw_angle_pre = -roll;
    this->imu_data_available = true;
    this->imu_sub.reset();
}


void GimbalControl::joint_callback(control_msgs::msg::DynamicJointState::SharedPtr msg) {

    //get yaw encoder
    for (unsigned long i = 0; i < msg->joint_names.size(); ++i) {
        if (msg->joint_names[i] == "gimbal_yaw") {
            bool offline = true;
            float encoder = 0.0;

            for (unsigned long j = 0; j < msg->interface_values[i].interface_names.size(); ++j) {
                if (msg->interface_values[i].interface_names[j] == "offline" && msg->interface_values[i].values[j] == 0.0f) offline = false;
            }

            for (unsigned long j = 0; j < msg->interface_values[i].interface_names.size(); ++j) {
                if (msg->interface_values[i].interface_names[j] == "encoder") {
                    encoder = msg->interface_values[i].values[j];
                }
            }

            //log first data
            if(!offline) {
                this->motor_yaw_angle_pre = encoder;
                this->joint_data_available = true;
                this->joint_sub.reset();
            }
        }
    }
}


void GimbalControl::gimbal_yaw_callback(std_msgs::msg::Float64::SharedPtr msg) {

    //calc head position
    if (this->imu_data_available && this->joint_data_available) {
        double motor_imu_transform = this->motor_yaw_angle_pre - this->imu_yaw_angle_pre;
        double relative_transform = motor_imu_transform + this->gimbal_yaw_ecd_transform;
        while (relative_transform > M_PI) relative_transform -= 2 * M_PI;
        while (relative_transform < -M_PI) relative_transform += 2 * M_PI;

        std_msgs::msg::Float64 pid_set;
        pid_set.data = relative_transform + msg->data;
        if (this->yaw_pid_publisher->is_activated()) {
            while (pid_set.data - this->gimbal_yaw_position > M_PI) pid_set.data -= 2 * M_PI;
            while (pid_set.data - this->gimbal_yaw_position < -M_PI) pid_set.data += 2 * M_PI;
            this->yaw_pid_publisher->publish(pid_set);
        }
    } else {
        //if data unavailable, forward the command
        //if (this->yaw_pid_publisher->is_activated()) this->yaw_pid_publisher->publish(*msg);
    }
}


void GimbalControl::gimbal_pitch_callback(std_msgs::msg::Float64::SharedPtr msg) {

    double angle_set = msg->data;

    //limit
    if (angle_set >= this->gimbal_pitch_max - this->pitch_soft_limit) {
        angle_set = this->gimbal_pitch_max - this->pitch_soft_limit;
    }
    if (angle_set <= this->gimbal_pitch_min + this->pitch_soft_limit) {
        angle_set = this->gimbal_pitch_min + this->pitch_soft_limit;
    }

    std_msgs::msg::Float64 pid_set;
    pid_set.data = angle_set;
    if (this->pitch_pid_publisher->is_activated()) this->pitch_pid_publisher->publish(pid_set);
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<GimbalControl> gimbal_control = std::make_shared<GimbalControl>(rclcpp::NodeOptions());

    exe.add_node(gimbal_control->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();

    return 0;
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gary_gimbal::GimbalControl)
