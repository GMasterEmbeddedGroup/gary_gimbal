//
// Created by maackia on 23-2-7.
//

#ifndef BUILD_GIMBAL_CONTROL_HPP
#define BUILD_GIMBAL_CONTROL_HPP

typedef enum
{
    GIMBAL_MOTOR_RAW = 0, //电机原始值控制
    GIMBAL_MOTOR_GYRO,    //电机陀螺仪角度控制
    GIMBAL_MOTOR_ENCONDE, //电机编码值角度控制
    GIMBAL_MOTOR_VISION_PID,
} gimbal_motor_mode_e;

typedef enum
{
    GIMBAL_ZERO_FORCE = 0,
    GIMBAL_ABSOLUTE_ANGLE,
    GIMBAL_INIT,
    GIMBAL_CALI,
    GIMBAL_RELATIVE_ANGLE,
    GIMBAL_MOTIONLESS,
} gimbal_behaviour_e;

namespace gimbal
{
    sensor_msgs::msg::Imu Imu;//TODO 初始化
    std_msgs::msg::Float64 YawEncoder;
    std_msgs::msg::Float64 PitchEncoder;
    std_msgs::msg::Int16 state;

    class Mode
    {
    public:
        gimbal_motor_mode_e motor_mode;
        gimbal_motor_mode_e last_motor_mode;
        gimbal_behaviour_e behaviour;
        gimbal_behaviour_e last_behaviour;
    };

    class Motor : public Mode{
    public:
        float absolute_angle_set;
        float current_set;
        float raw_cmd_current;
        float given_current;
        float add_angle;
    };

    Motor yaw;
    Motor pitch;
    Mode mode;

    void init();

    void behaviour_set();

    void set_mode();

    void behaviour_control_set(float add_yaw,float add_pitch);

    void set_control();

    void control_loop();
}

#endif //BUILD_GIMBAL_CONTROL_HPP
