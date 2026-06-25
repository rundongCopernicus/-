/*
 * motor.h
 *
 *  Created on: 2026年4月10日
 *      Author: lenovo
 */

#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_

#include "zf_common_headfile.h"


typedef struct motor1
{
        int target_speed;
        int duty;
        float encoder_speed;
        int encoder_raw;
        int32 total_encoder;
        int32 encoder_miles;

}motor1;

// 转向电机结构体
typedef struct servo_motor
{
        int encoder_raw;        // 转向编码器原始值
        int32 total_encoder;    // 转向编码器累计值
        float current_angle;    // 当前转向角度（根据编码器计算）
}servo_motor;

// 转向电机控制结构体
typedef struct servo_motor_ctrl
{
    float target_angle;      // 目标转向角度
    float current_angle;     // 当前转向角度（从编码器计算）
    int duty;                // 输出PWM占空比
} servo_motor_ctrl_t;

extern struct motor1 motor_1;
extern struct motor1 motor_r;
extern servo_motor servo_motor_data;
extern servo_motor_ctrl_t servo_motor_ctrl;
extern uint8 start;


void Encoder_init(void);
void Data_encoder(void);
void Servo_Encoder_init(void);      // 转向编码器初始化
void Data_Servo_encoder(void);      // 读取转向编码器数据
void Motor_init();
void Motor_SetSpeed(pwm_channel_enum pin1,gpio_pin_enum pin2, int pwm,uint8 just,uint8 lose);
void car_start(void);
void Motor_Control(int Speed_L, int Speed_R);
void Servo_init(void);
void Servo_Motor_Control(float target_angle);  // 转向电机位置控制（替换原来的Servo_SetAngle）

#define MotorL_pwm ATOM0_CH0_P21_2
#define MotorL_turn P21_5

#define MotorR_pwm ATOM0_CH2_P21_4
#define MotorR_turn P21_3

// 转向电机引脚定义（DRV8301驱动板）
#define Servo_pwm   ATOM0_CH7_P02_7  // 转向电机PWM
#define Servo_DIR   P02_6            // 转向电机方向引脚（请根据你的实际接线确认）

#define SERVO_CENTER_DUTY 0          // 转向电机中心占空比（0表示停止）
#define SERVO_MAX_ANGLE 30.0f        // 转向最大角度（降低到30度，减小转弯幅度）
#define SERVO_DUTY_RANGE 5000        // PWM占空比范围

#define pid_limit 5000


#endif /* CODE_MOTOR_H_ */
