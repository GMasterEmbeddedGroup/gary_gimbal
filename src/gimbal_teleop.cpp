#include "gary_gimbal/gimbal_teleop.hpp"


using namespace std::chrono_literals;
using namespace gary_gimbal;


GimbalTeleop::GimbalTeleop(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("gimbal_teleop",
                                                                                                 options) {
    //declare params
    this->declare_parameter("gimbal_pitch_max", 0.0);
    this->declare_parameter("gimbal_pitch_min", 0.0);
    this->declare_parameter("k_rc", 1.0);
    this->declare_parameter("k_mouse", 1.0);
    this->declare_parameter("k_autoaim", 1.0);
    this->declare_parameter("remote_control_topic", "/remote_control");
    this->declare_parameter("autoaim_topic", "/autoaim/target");
    this->declare_parameter("yaw_set_topic", "/gimbal_yaw_set");
    this->declare_parameter("pitch_set_topic", "/gimbal_pitch_set");
}

CallbackReturn GimbalTeleop::on_configure(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //create callback group
    this->cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = cb_group;

    //get gimbal_pitch_max
    this->gimbal_pitch_max = this->get_parameter("gimbal_pitch_max").as_double();

    //get gimbal_pitch_min
    this->gimbal_pitch_min = this->get_parameter("gimbal_pitch_min").as_double();

    //get k_rc
    this->k_rc = this->get_parameter("k_rc").as_double();

    //get k_mouse
    this->k_mouse = this->get_parameter("k_mouse").as_double();

    //get k_autoaim
    this->k_autoaim = this->get_parameter("k_autoaim").as_double();

    //get remote_control_topic
    this->remote_control_topic = this->get_parameter("remote_control_topic").as_string();
    this->rc_sub = this->create_subscription<gary_msgs::msg::DR16Receiver>(
            this->remote_control_topic, rclcpp::SystemDefaultsQoS(),
            std::bind(&GimbalTeleop::rc_callback, this, std::placeholders::_1), sub_options);

    //get autoaim_topic
    this->autoaim_topic = this->get_parameter("autoaim_topic").as_string();
    this->autoaim_sub = this->create_subscription<gary_msgs::msg::AutoAIM>(
            this->autoaim_topic, rclcpp::SystemDefaultsQoS(),
            std::bind(&GimbalTeleop::autoaim_callback, this, std::placeholders::_1), sub_options);

    //get yaw_set_topic
    this->yaw_set_topic = this->get_parameter("yaw_set_topic").as_string();
    this->yaw_set_publisher = this->create_publisher<std_msgs::msg::Float64>(this->yaw_set_topic,
                                                                             rclcpp::SystemDefaultsQoS());

    //get pitch_set_topic
    this->pitch_set_topic = this->get_parameter("pitch_set_topic").as_string();
    this->pitch_set_publisher = this->create_publisher<std_msgs::msg::Float64>(this->pitch_set_topic,
                                                                               rclcpp::SystemDefaultsQoS());

    RCLCPP_INFO(this->get_logger(), "configured");
    return CallbackReturn::SUCCESS;
}


CallbackReturn GimbalTeleop::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //destroy objects
    this->rc_sub.reset();
    this->autoaim_sub.reset();
    this->yaw_set_publisher.reset();
    this->pitch_set_publisher.reset();

    RCLCPP_INFO(this->get_logger(), "cleaning up");
    return CallbackReturn::SUCCESS;
}


CallbackReturn GimbalTeleop::on_activate(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //activate lifecycle publisher
    this->yaw_set_publisher->on_activate();
    this->pitch_set_publisher->on_activate();

    RCLCPP_INFO(this->get_logger(), "activated");
    return CallbackReturn::SUCCESS;
}


CallbackReturn GimbalTeleop::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //deactivate lifecycle publisher
    this->yaw_set_publisher->on_deactivate();
    this->pitch_set_publisher->on_deactivate();

    RCLCPP_INFO(this->get_logger(), "deactivated");
    return CallbackReturn::SUCCESS;
}


CallbackReturn GimbalTeleop::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //destroy objects
    if (this->rc_sub.get() != nullptr) this->rc_sub.reset();
    if (this->autoaim_sub.get() != nullptr) this->autoaim_sub.reset();
    if (this->yaw_set_publisher.get() != nullptr) this->yaw_set_publisher.reset();
    if (this->pitch_set_publisher.get() != nullptr) this->pitch_set_publisher.reset();

    RCLCPP_INFO(this->get_logger(), "shutdown");
    return CallbackReturn::SUCCESS;
}


CallbackReturn GimbalTeleop::on_error(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //destroy objects
    if (this->rc_sub.get() != nullptr) this->rc_sub.reset();
    if (this->autoaim_sub.get() != nullptr) this->autoaim_sub.reset();
    if (this->yaw_set_publisher.get() != nullptr) this->yaw_set_publisher.reset();
    if (this->pitch_set_publisher.get() != nullptr) this->pitch_set_publisher.reset();

    RCLCPP_INFO(this->get_logger(), "error");
    return CallbackReturn::SUCCESS;
}


void GimbalTeleop::rc_callback(gary_msgs::msg::DR16Receiver::SharedPtr msg) {
    std_msgs::msg::Float64 yaw_msg;
    std_msgs::msg::Float64 pitch_msg;

    if (msg->sw_right == gary_msgs::msg::DR16Receiver::SW_MID || msg->sw_right == gary_msgs::msg::DR16Receiver::SW_UP){

        //判断鼠标无动作,使用遥控器
        if (msg->mouse_x == 0 && msg->mouse_y == 0){
            //最大值1684 中间值1024 最小值364
            this->yaw_set += msg->ch_right_x * this->k_rc;
            this->pitch_set += msg->ch_right_y * this->k_rc;
        }
        else {
            //+-32767静止值0
            this->yaw_set += msg->mouse_x * this->k_mouse;
            this->pitch_set += msg->mouse_y * this->k_mouse;
        }

        //limit
        if (this->pitch_set >= this->gimbal_pitch_max) this->pitch_set = this->gimbal_pitch_max;
        if (this->pitch_set <= this->gimbal_pitch_min) this->pitch_set = this->gimbal_pitch_min;

        yaw_msg.data = this->yaw_set;
        pitch_msg.data = this->pitch_set;
        if (this->yaw_set_publisher->is_activated()) this->yaw_set_publisher->publish(yaw_msg);
        if (this->pitch_set_publisher->is_activated()) this->pitch_set_publisher->publish(pitch_msg);
    }
    this->use_autoaim = msg->sw_right == gary_msgs::msg::DR16Receiver::SW_UP || msg->mouse_press_r;
}


void GimbalTeleop::autoaim_callback(gary_msgs::msg::AutoAIM::SharedPtr msg) {
    std_msgs::msg::Float64 yaw_msg;
    std_msgs::msg::Float64 pitch_msg;

    //have target and use autoaim
    if (msg->target_id != gary_msgs::msg::AutoAIM::TARGET_ID0_NONE && this->use_autoaim){
        this->yaw_set -= msg->yaw * this->k_autoaim;
        this->pitch_set -= msg->pitch * this->k_autoaim;

        //pitch limit
        if (this->pitch_set >= this->gimbal_pitch_max) this->pitch_set = this->gimbal_pitch_max;
        if (this->pitch_set <= this->gimbal_pitch_min) this->pitch_set = this->gimbal_pitch_min;

        yaw_msg.data = this->yaw_set;
        pitch_msg.data = this->pitch_set;
        if (this->yaw_set_publisher->is_activated()) this->yaw_set_publisher->publish(yaw_msg);
        if (this->pitch_set_publisher->is_activated()) this->pitch_set_publisher->publish(pitch_msg);
    }
}


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<GimbalTeleop> gimbal_teleop = std::make_shared<GimbalTeleop>(rclcpp::NodeOptions());

    exe.add_node(gimbal_teleop->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();

    return 0;
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gary_gimbal::GimbalTeleop)
