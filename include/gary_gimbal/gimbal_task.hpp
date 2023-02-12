//
// Created by maackia on 23-2-7.
//

#ifndef BUILD_GIMBAL_TASK_HPP
#define BUILD_GIMBAL_TASK_HPP

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
    GIMBAL_INIT,
    GIMBAL_CALI,
    GIMBAL_ABSOLUTE_ANGLE,
    GIMBAL_RELATIVE_ANGLE,
    GIMBAL_MOTIONLESS,
} gimbal_behaviour_e;

namespace gimbal
{
    gary_msgs::msg::DR16Receiver RC_control;//TODO 初始化
    sensor_msgs::msg::Imu Imu;//TODO 初始化

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
        int16_t given_current;
        float add_angle;
    };

    Motor yaw;
    Motor pitch;
    Mode mode;

    void init(){
        yaw.given_current = pitch.given_current = 0;
        yaw.add_angle = pitch.add_angle = 0;
        yaw.motor_mode = yaw.last_motor_mode = GIMBAL_MOTOR_RAW;
        pitch.motor_mode = pitch.last_motor_mode = GIMBAL_MOTOR_RAW;
    }

     void behaviour_set(){
         if (RC_control.sw_left == RC_control.SW_UP){
             mode.behaviour = GIMBAL_ZERO_FORCE;
         }
         if (RC_control.sw_left == RC_control.SW_MID){
             mode.behaviour = GIMBAL_ABSOLUTE_ANGLE;
         }
    }

    void set_mode(){
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

    void behaviour_control_set(float add_yaw,float add_pitch){
        if (mode.behaviour == GIMBAL_ZERO_FORCE){
            add_yaw = 0.0f;
            add_pitch = 0.0f;
        }
        if (mode.behaviour == GIMBAL_ABSOLUTE_ANGLE){
            add_yaw = RC_control.ch_right_x;
            add_pitch = RC_control.ch_right_y;
        }
    }
    void set_control(){
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

    void control_loop(){
        if (yaw.motor_mode == GIMBAL_MOTOR_RAW){
            yaw.current_set = yaw.raw_cmd_current;
            yaw.given_current = (int16_t)yaw.current_set;
        }
        else if (yaw.motor_mode == GIMBAL_MOTOR_GYRO){
            //TODO
        }


        if (pitch.motor_mode == GIMBAL_MOTOR_RAW){
            pitch.current_set = pitch.raw_cmd_current;
            pitch.given_current = (int16_t)pitch.current_set;
        }
        else if (pitch.motor_mode == GIMBAL_MOTOR_GYRO){

        }
    }
}

#endif //BUILD_GIMBAL_TASK_HPP
