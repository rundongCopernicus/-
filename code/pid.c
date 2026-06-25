/*
 * pid.c
 *
 *  Created on: 2026年4月11日
 *      Author: lenovo
 */


#include "zf_common_headfile.h"

// Servo PID parameters for steering control
// Kp: Proportional gain - main response to heading error
// Ki: Integral gain - corrects steady-state error (keep small to avoid overshoot)
// Kd: Derivative gain - dampens oscillation
// LowPass: Low-pass filter coefficient for derivative term (0.8 = 80% old + 20% new)
// 航向角到转向角度的映射PID
// 调参记录：
// 测试1: Kp=0.5, Ki=0.0  (保守起步，观察是否反应太慢)
// 测试2: Kp=0.8, Ki=0.1  (如果测试1太慢，尝试这组)
// 测试3: Kp=1.0, Ki=0.15 (如果测试2还不够快)
// 注意：Kp>1.2通常会导致震荡！
// 当前使用：降低Kp避免震荡，增大Kd抑制震荡
PID servo_pid=PID_CREATE(0.5, 0.0, 0.5, 0.7);

// 转向电机位置PID参数（用于转向电机位置闭环控制）
// 注意：当前使用开环控制，此PID暂时不用
// 如果以后添加转向编码器，可以取消注释启用
// Kp: 比例增益，控制响应速度
// Ki: 积分增益，消除稳态误差
// Kd: 微分增益，减少超调
// 这些参数需要根据实际电机和机械结构调试！
// PID servo_position_pid=PID_CREATE(50.0, 0.5, 10.0, 0.7);
PID servo_position_pid =
    PID_CREATE(50.0f, 0.5f, 10.0f, 0.7f);

// 增量式PID参数：更保守的参数避免累加过快
PID motor_pid_1=PID_CREATE(3,0.05,1,0.3);
PID motor_pid_r=PID_CREATE(3,0.05,1,0.3);


float PID_NORMAL(PID *PID,float NowData,float Point)
{
    // 计算角度误差，处理±180°边界跳变
    PID->Error = Point - NowData;

    // 将误差归一化到[-180, 180]范围
    while(PID->Error > 180.0f) PID->Error -= 360.0f;
    while(PID->Error < -180.0f) PID->Error += 360.0f;

    PID->OUT_D = (PID->Error-PID->OUT_P)*PID->LowPass+PID->OUT_D*(1-PID->LowPass);
    PID->OUT_P=PID->Error;
    return (PID->Kp * PID->OUT_P + PID->Ki * PID->OUT_D);
}

// 位置式PID：直接输出duty值
float PID_Position(PID *PID, float NowData, float Point)
{
    // 计算误差
    PID->Error = Point - NowData;

    // 比例项
    PID->OUT_P = PID->Error;

    // 积分项（带限幅，防止积分饱和）
    PID->OUT_I += PID->Error;
    if(PID->OUT_I > 100) PID->OUT_I = 100;
    if(PID->OUT_I < -100) PID->OUT_I = -100;

    // 微分项（带滤波）
    float diff = PID->Error - PID->LastError;
    PID->OUT_D = PID->OUT_D * 0.7 + diff * 0.3;

    // 更新历史误差
    PID->LastError = PID->Error;

    // 计算输出
    return PID->Kp * PID->OUT_P + PID->Ki * PID->OUT_I + PID->Kd * PID->OUT_D;
}

// 增量式PID：输出增量值
float PID_Increase(PID *PID, float NowData, float Point)
{
    // 计算当前误差（目标 - 实际）
    PID->Error = Point - NowData;

    // 计算增量
    // ΔU = Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2*e(k-1)+e(k-2))
    float increment = PID->Kp * (PID->Error - PID->LastError)
                    + PID->Ki * PID->Error
                    + PID->Kd * (PID->Error - 2 * PID->LastError + PID->PrevError);

    // 更新历史误差
    PID->PrevError = PID->LastError;
    PID->LastError = PID->Error;

    return increment;
}

