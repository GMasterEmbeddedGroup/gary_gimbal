#include "gary_gimbal/gimbal_autonomous.hpp"

using namespace gary_gimbal;


GimbalAutonomous::GimbalAutonomous(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("gimbal_autonomous",
                                                                                                 options) {
    //declare params
    this->declare_parameter("gimbal_pitch_max", 0.0);
    this->declare_parameter("gimbal_pitch_min", 0.0);
    this->declare_parameter("rotate_time", 3.0);
    this->declare_parameter("k_rc", 1.0);
    this->declare_parameter("k_mouse", 1.0);
    this->declare_parameter("k_autoaim", 1.0);
    this->declare_parameter("remote_control_topic", "/remote_control");
    this->declare_parameter("autoaim_topic", "/autoaim/target");
    this->declare_parameter("yaw_set_topic", "/gimbal_yaw_set");
    this->declare_parameter("pitch_set_topic", "/gimbal_pitch_set");

    rc_sw_right = gary_msgs::msg::DR16Receiver::SW_DOWN;
    GimbalStatus = MANUAL;
    last_target_timestamp = std::chrono::steady_clock::now();

    rolling_counter = 0.0;
    update_freq = 100.0;
    pitch_counter = 0.0;
    rotate_time = 3.0;
}

CallbackReturn GimbalAutonomous::on_configure(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //create callback group
    this->cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = cb_group;

    //get gimbal_pitch_max
    this->gimbal_pitch_max = this->get_parameter("gimbal_pitch_max").as_double();

    //get gimbal_pitch_min
    this->gimbal_pitch_min = this->get_parameter("gimbal_pitch_min").as_double();
    pitch_counter = gimbal_pitch_min;

    this->rotate_time = this->get_parameter("rotate_time").as_double();

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
            std::bind(&GimbalAutonomous::rc_callback, this, std::placeholders::_1), sub_options);

    //get autoaim_topic
    this->autoaim_topic = this->get_parameter("autoaim_topic").as_string();
    this->autoaim_sub = this->create_subscription<gary_msgs::msg::AutoAIM>(
            this->autoaim_topic, rclcpp::SystemDefaultsQoS(),
            std::bind(&GimbalAutonomous::autoaim_callback, this, std::placeholders::_1), sub_options);

    //get yaw_set_topic
    this->yaw_set_topic = this->get_parameter("yaw_set_topic").as_string();
    this->yaw_set_publisher = this->create_publisher<std_msgs::msg::Float64>(this->yaw_set_topic,
                                                                             rclcpp::SystemDefaultsQoS());

    //get pitch_set_topic
    this->pitch_set_topic = this->get_parameter("pitch_set_topic").as_string();
    this->pitch_set_publisher = this->create_publisher<std_msgs::msg::Float64>(this->pitch_set_topic,
                                                                               rclcpp::SystemDefaultsQoS());

    rolling_counter = 0.0;

    RCLCPP_INFO(this->get_logger(), "configured");
    return CallbackReturn::SUCCESS;
}


CallbackReturn GimbalAutonomous::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //destroy objects
    this->rc_sub.reset();
    this->autoaim_sub.reset();
    this->yaw_set_publisher.reset();
    this->pitch_set_publisher.reset();

    RCLCPP_INFO(this->get_logger(), "cleaning up");
    return CallbackReturn::SUCCESS;
}


CallbackReturn GimbalAutonomous::on_activate(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //activate lifecycle publisher
    this->yaw_set_publisher->on_activate();
    this->pitch_set_publisher->on_activate();

    last_target_timestamp = std::chrono::steady_clock::now();

    this->timer_update = this->create_wall_timer(1000ms / update_freq,[this] { update(); });

    RCLCPP_INFO(this->get_logger(), "activated");
    return CallbackReturn::SUCCESS;
}


CallbackReturn GimbalAutonomous::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //deactivate lifecycle publisher
    this->yaw_set_publisher->on_deactivate();
    this->pitch_set_publisher->on_deactivate();

    RCLCPP_INFO(this->get_logger(), "deactivated");
    return CallbackReturn::SUCCESS;
}


CallbackReturn GimbalAutonomous::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //destroy objects
    if (this->rc_sub.get() != nullptr) this->rc_sub.reset();
    if (this->autoaim_sub.get() != nullptr) this->autoaim_sub.reset();
    if (this->yaw_set_publisher.get() != nullptr) this->yaw_set_publisher.reset();
    if (this->pitch_set_publisher.get() != nullptr) this->pitch_set_publisher.reset();

    RCLCPP_INFO(this->get_logger(), "shutdown");
    return CallbackReturn::SUCCESS;
}


CallbackReturn GimbalAutonomous::on_error(const rclcpp_lifecycle::State &previous_state) {
    RCL_UNUSED(previous_state);

    //destroy objects
    if (this->rc_sub.get() != nullptr) this->rc_sub.reset();
    if (this->autoaim_sub.get() != nullptr) this->autoaim_sub.reset();
    if (this->yaw_set_publisher.get() != nullptr) this->yaw_set_publisher.reset();
    if (this->pitch_set_publisher.get() != nullptr) this->pitch_set_publisher.reset();

    RCLCPP_INFO(this->get_logger(), "error");
    return CallbackReturn::SUCCESS;
}


void GimbalAutonomous::rc_callback(gary_msgs::msg::DR16Receiver::SharedPtr msg) {
    std_msgs::msg::Float64 yaw_msg;
    std_msgs::msg::Float64 pitch_msg;

    this->rc_sw_right = msg->sw_right;

    if (GimbalStatus == MANUAL) {

        //判断鼠标无动作,使用遥控器
        if (msg->mouse_x == 0 && msg->mouse_y == 0) {
            //最大值1684 中间值1024 最小值364
            this->yaw_set += msg->ch_right_x * this->k_rc;
            this->pitch_set += msg->ch_right_y * this->k_rc;
        } else {
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

    if(rc_sw_right == gary_msgs::msg::DR16Receiver::SW_MID){
        if(GimbalStatus != MANUAL){
            RCLCPP_INFO(this->get_logger(),"Switched to manual mode!");
        }
        GimbalStatus = MANUAL;
    }else if(rc_sw_right == gary_msgs::msg::DR16Receiver::SW_DOWN){
        if(GimbalStatus != ZERO_FORCE){
            RCLCPP_INFO(this->get_logger(),"Switched to zero-force mode!");
        }
        GimbalStatus = ZERO_FORCE;
    }else if(rc_sw_right != gary_msgs::msg::DR16Receiver::SW_UP){
        GimbalStatus = ZERO_FORCE;
        auto clock = rclcpp::Clock();
        RCLCPP_ERROR_THROTTLE(this->get_logger(),clock,3000,"Cannot receive DR16 message.");
    }else{
        if(GimbalStatus == ZERO_FORCE || GimbalStatus == MANUAL){
            RCLCPP_INFO(this->get_logger(),"Switched to auto-aim mode!");
            GimbalStatus = AUTO_AIM;
        }
    }
}


void GimbalAutonomous::autoaim_callback(gary_msgs::msg::AutoAIM::SharedPtr msg) {
    std_msgs::msg::Float64 yaw_msg;
    std_msgs::msg::Float64 pitch_msg;

    //have target and use autoaim
    if(msg->target_id != gary_msgs::msg::AutoAIM::TARGET_ID0_NONE && GimbalStatus == AUTO_AIM) {
        auto now_target_timestamp = std::chrono::steady_clock::now();
        last_target_timestamp = now_target_timestamp;

        this->yaw_set -= msg->yaw * this->k_autoaim;
        this->pitch_set -= msg->pitch * this->k_autoaim;

        //pitch limit
        if (this->pitch_set >= this->gimbal_pitch_max) this->pitch_set = this->gimbal_pitch_max;
        if (this->pitch_set <= this->gimbal_pitch_min) this->pitch_set = this->gimbal_pitch_min;

        yaw_msg.data = this->yaw_set;
        pitch_msg.data = this->pitch_set;
        if (this->yaw_set_publisher->is_activated()) this->yaw_set_publisher->publish(yaw_msg);
        if (this->pitch_set_publisher->is_activated()) this->pitch_set_publisher->publish(pitch_msg);

    }else if(GimbalStatus != MANUAL){
        if(msg->target_id == gary_msgs::msg::AutoAIM::TARGET_ID0_NONE){
            /*@deprecated: Moved to function update()*/
//            auto now_target_timestamp = std::chrono::steady_clock::now();
//            if(now_target_timestamp - last_target_timestamp > no_target_duration_limit){
//                GimbalStatus = ROTATE;
//            }
        } else {
            GimbalStatus = AUTO_AIM;
        }
    }
}

void GimbalAutonomous::update() {
    std_msgs::msg::Float64 yaw_msg;
    std_msgs::msg::Float64 pitch_msg;

    if(GimbalStatus != MANUAL && GimbalStatus != ZERO_FORCE) {
        auto now_target_timestamp = std::chrono::steady_clock::now();
        if (now_target_timestamp - last_target_timestamp > no_target_duration_limit) {
            GimbalStatus = ROTATE;
        }
    }
    if(GimbalStatus == AUTO_AIM){
        //do nothing.
        //let auto_aim callback handle this.
        return;
    }else if(GimbalStatus == MANUAL){
        //do nothing.
        //let rc callback handle this.
        return;
    } else if(GimbalStatus == ROTATE){
        rolling_counter += (M_PI * 2.0) / (rotate_time * update_freq);

        static bool pitch_reverse = false;


        auto gimbal_pitch_upper = (gimbal_pitch_max+gimbal_pitch_min)/2.0;
        auto gimbal_pitch_lower = gimbal_pitch_min;

        auto pitch_diff = (gimbal_pitch_upper - gimbal_pitch_lower) / update_freq;
        if(!pitch_reverse){
            pitch_counter += pitch_diff;
            if(pitch_counter + pitch_diff > gimbal_pitch_upper){
                pitch_reverse = true;
            }
        } else {
            pitch_counter -= (gimbal_pitch_upper - gimbal_pitch_lower) / update_freq;
            if(pitch_counter - pitch_diff < gimbal_pitch_lower){
                pitch_reverse = false;
            }
        }

        this->yaw_set = rolling_counter;
        this->pitch_set = pitch_counter;

        yaw_msg.data = this->yaw_set;
        pitch_msg.data = this->pitch_set;
        if (this->yaw_set_publisher->is_activated()) this->yaw_set_publisher->publish(yaw_msg);
        if (this->pitch_set_publisher->is_activated()) this->pitch_set_publisher->publish(pitch_msg);

    }
}


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<GimbalAutonomous> gimbal_teleop = std::make_shared<GimbalAutonomous>(rclcpp::NodeOptions());

    exe.add_node(gimbal_teleop->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();

    return 0;
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gary_gimbal::GimbalAutonomous)
