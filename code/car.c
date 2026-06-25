/*
 * car.c
 *
 *  Created on: 2026年6月9日
 *      Author: jred
 */

 #include "zf_common_headfile.h"

#include "motor.h"

// 小车停止函数
// 将左右电机速度都设为0，实现小车紧急停止
void car_stop(void)
{
    Motor_Control(0, 0);  // 调用电机控制函数，左右速度都设为0
}

// 小车前进函数
// speed: 前进速度，范围0-100（百分比）
void car_go_forward(int speed)
{
    if(speed < 0) speed = 0;     // 限制速度下限为0
    if(speed > 100) speed = 100; // 限制速度上限为100
    Motor_Control(speed, speed);  // 左右电机同速前进
}

// 小车后退函数
// speed: 后退速度，范围0-100（百分比）
void car_go_backward(int speed)
{
    if(speed < 0) speed = 0;     // 限制速度下限为0
    if(speed > 100) speed = 100; // 限制速度上限为100
    Motor_Control(-speed, -speed);  // 左右电机同速后退（负值）
}

// 小车左转函数
// speed: 基础速度，范围0-100
// turn_ratio: 转向比例，范围0.0-1.0，值越小转向越急
void car_turn_left(int speed, float turn_ratio)
{
    if(speed < 0) speed = 0;      // 限制速度下限
    if(speed > 100) speed = 100;   // 限制速度上限
    if(turn_ratio < 0.0f) turn_ratio = 0.0f;   // 限制转向比下限
    if(turn_ratio > 1.0f) turn_ratio = 1.0f;   // 限制转向比上限

    // 左转原理：左轮速度降低，右轮速度保持
    // turn_ratio越小，左轮相对右轮越慢，转弯越急
    int speed_l = (int)((float)speed * turn_ratio);  // 左轮速度 = 基础速度 * 转向比
    int speed_r = speed;                              // 右轮速度 = 基础速度（全速）
    Motor_Control(speed_l, speed_r);  // 执行左右电机控制
}

// 小车右转函数
// speed: 基础速度，范围0-100
// turn_ratio: 转向比例，范围0.0-1.0，值越小转向越急
void car_turn_right(int speed, float turn_ratio)
{
    if(speed < 0) speed = 0;      // 限制速度下限
    if(speed > 100) speed = 100;   // 限制速度上限
    if(turn_ratio < 0.0f) turn_ratio = 0.0f;   // 限制转向比下限
    if(turn_ratio > 1.0f) turn_ratio = 1.0f;   // 限制转向比上限

    // 右转原理：左轮速度保持，右轮速度降低
    // turn_ratio越小，右轮相对左轮越慢，转弯越急
    int speed_l = speed;                              // 左轮速度 = 基础速度（全速）
    int speed_r = (int)((float)speed * turn_ratio);  // 右轮速度 = 基础速度 * 转向比
    Motor_Control(speed_l, speed_r);  // 执行左右电机控制
}

