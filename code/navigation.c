/*
 * navigation.c
 *
 *  Created on: 2026年6月21日
 *      Author: jred
 */

#include "zf_common_headfile.h"
#include "navigation.h"
#include "motor.h"
#include "fuction.h"
#include "flash.h"
#include <math.h>


#define NAG_DISTANCE_RATIO 1.0f   // 编码器脉冲到距离的比例，根据实际车辆标定
#define NAG_TURN_KP 0.8f          // 轨迹航向误差P控制系数
#define NAG_FORWARD_SPEED   40   // 轨迹跟踪时的基础前进速度

int32 Nav_read[Read_MaxSize];  // 保存的路径点，每5cm一个，1000点 = 50m
int8 Nav_direction[Read_MaxSize]; // 保存的方向数据（1=前进，-1=倒车，0=停止）
int8 back = 0;
Nag N;
float diff;
int8 current_direction;
float servo_angle;

// 计算角度差值，处理跨越±180度边界的情况
static float angle_diff(float target, float current)
{

    diff = target - current;

   // 将误差归一化到[-180, 180]范围
    while(diff > 180.0f)
{
    diff -= 360.0f;
}

while(diff < -180.0f)
{
    diff += 360.0f;
}
    return diff;
}

// 根据编码器读数更新当前位置
void Nag_Position_Update(void)  //累计里程和位置更新

{
    float delta_count = ((float)motor_1.encoder_raw + (float)motor_r.encoder_raw) * 0.5f;
    N.last_distance = delta_count;
    N.Mileage_All += delta_count*0.5;

    float forward = delta_count * NAG_DISTANCE_RATIO;
    float rad = angle_Z * PI / 180.0f;
    N.pos_x += forward * cosf(rad);
    N.pos_y += forward * sinf(rad);
}

// 控制航向以跟随目标轨迹
static void Nag_ControlHeading(void)
{
    static float last_servo_angle = 0;  // 记录上次舵机角度，用于变化率限制
    static float filtered_error = 0;     // 滤波后的误差

    // 计算航向误差（目标 - 当前）
    float error = angle_diff(N.Angle_Run, angle_Z);
    N.yaw_error = error;

    filtered_error =error;

    // 死区处理：误差小于5度时不转向
    if(filtered_error > -2.0f && filtered_error < 2.0f)
    {
        filtered_error = 0;
    }

    // 保守的比例系数，避免过冲
    servo_angle = filtered_error * 0.7f;

    // 严格限制舵机变化率，每次最多1度
    float angle_change = servo_angle - last_servo_angle;
    if(angle_change > 3.0f) angle_change = 3.0f;
    if(angle_change < -3.0f) angle_change = -3.0f;
    servo_angle = last_servo_angle + angle_change;
    last_servo_angle = servo_angle;


    // 限幅到舵机物理范围
    servo_angle = Limit_init(-SERVO_MAX_ANGLE, servo_angle, SERVO_MAX_ANGLE);
    N.servo_angle_out = servo_angle;  // 保存输出角度用于调试

    // 使用转向电机控制函数
    if(current_direction==1)
    {
    Servo_Motor_Control(servo_angle);
    }
    else
    {
        if(servo_angle > 5.0f) servo_angle = servo_angle-5.0f;
        if(servo_angle < -5.0f) servo_angle = servo_angle+5.0f;
        Servo_Motor_Control(servo_angle);
        back = 1;

    }

}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：Nag_Read - 读取并记录偏航角轨迹
// 功能说明：读取偏航角轨迹记录线程，通过N.End_f状态切换
// 参数说明：void
// 使用示例：由用户代码调用
// 备注信息：
//-------------------------------------------------------------------------------------------------------------------
void Nag_Read()
{
    switch(N.End_f)
    {
        case 0: Run_Nag_Save();  // 默认执行函数
            break;
        case 1:
            flash_Nag_Write();  // 写入最后一页以确保Flash存储
            N.End_f++;
            break;
        case 2:
            N.End_f = 0;  // 重置结束标志
            N.Nag_SystemRun_Index = 0;  // 退出记录模式，回到待机模式
            break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：Nag_Run - 执行基于偏航角的导航
// 功能说明：N.Final_Out包含生成的偏航误差
// 参数说明：void
// 使用示例：由用户代码调用
// 备注信息：
//-------------------------------------------------------------------------------------------------------------------
void Nag_Run()
{
    static int8 last_direction = 1;  // 记录上一次的方向

    Run_Nag_GPS();  // 读取偏航角并更新目标
    if(N.Nag_Stop_f)  // 停止导航
    {
        N.Final_Out = 0;
        Motor_Control(0,0);  // 停止电机
        Servo_Motor_Control(0);  // 舵机回中
        return;
    }

    // 回放模式下：舵机转向 + 根据方向信息控制前进或倒车
    Nag_ControlHeading();  // 控制舵机转向（内部只控制舵机，不控制电机）

    // 获取当前点的方向信息
    current_direction = 1;  // 默认前进
    if(N.Run_index < Read_MaxSize && N.Run_index < N.Save_index)
    {
        current_direction = Nav_direction[N.Run_index];
    }

    // 检测方向切换
    if(current_direction != last_direction)
    {
        // 方向改变时：先停车
        Motor_Control(0, 0);
        N.Angle_Run=angle_Z;
        int32 i=0;
        while(i<2000)
        {
            system_delay_ms(1);
        Servo_Motor_Control(0);
        i++;
        }
        last_direction = current_direction;  // 更新方向
    }

    // 根据记录的方向控制电机（正转或反转）
    int speed = NAG_FORWARD_SPEED * current_direction;
    Motor_Control(speed, speed);  // 两轮等速，靠舵机转向
    N.Final_Out = N.yaw_error;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：Run_Nag_Save - 记录期间保存偏航角
// 功能说明：读取当前YAW并定期存储到Flash中
// 参数说明：void
// 使用示例：由用户代码调用
// 备注信息：
//-------------------------------------------------------------------------------------------------------------------
void Run_Nag_Save()
{
    // 缓冲区满时写入Flash以防止溢出
    if(N.size > MaxSize)
    {
        flash_Nag_Write();
        N.size = 0;   // 重置计数器用于下一+页
        N.Flash_page_index--;   // Flash页索引递减
        zf_assert(N.Flash_page_index > Nag_End_Page);  // 防止越界
    }

    // 每隔固定距离保存航向和方向
    if(N.Mileage_All >= Nag_Set_mileage || N.Mileage_All <= -Nag_Set_mileage)
    {
       // 保存航向角
       int32 Save = (int32)(Nag_Yaw * 100);  // angle_Z放大100倍保存
       flash_union_buffer[N.size++].int32_type = Save;

       // 检测运动方向（基于编码器速度）
       float avg_speed = ((float)motor_1.encoder_raw + (float)motor_r.encoder_raw) * 0.5f;
       int8 direction = 0;  // 默认停止
       if(avg_speed > 1.0f)  // 前进（阈值避免噪声）
           direction = 1;
       else if(avg_speed < -1.0f)  // 倒车
           direction = -1;

       // 保存方向信息到Flash
       flash_union_buffer[N.size++].int8_type = direction;

       N.Save_index++;
       if(N.Mileage_All > 0) N.Mileage_All -= Nag_Set_mileage;
       else N.Mileage_All += Nag_Set_mileage;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：Run_Nag_GPS - 跟随保存的轨迹
// 功能说明：从Flash读取存储的YAW并更新目标航向（根据速度动态调整索引增长）
// 参数说明：void
// 使用示例：由用户代码调用
// 备注信息：速度越快，索引增长越快；速度越慢，索引增长越慢
//-------------------------------------------------------------------------------------------------------------------
void Run_Nag_GPS()
{
    unsigned short prospect = 0;

    float dynamic_mileage=Nag_Set_mileage;

    if(N.Mileage_All >= dynamic_mileage || N.Mileage_All <= -dynamic_mileage)
    {
        // 检查是否到达轨迹终点
        if(N.Run_index > N.Save_index)
        {
            N.Nag_Stop_f++;
            return;
        }
        N.Run_index++;  // 如果需要跑多圈，这里重置为0
        prospect = N.Run_index;  // 前瞻索引
        if(prospect > N.Save_index - 2) prospect = N.Save_index - 2;  // 边界检查
        N.Angle_Run = (Nav_read[prospect] / 100.0f);  // 恢复缩放后的角度
        if(N.Mileage_All > 0) N.Mileage_All -= dynamic_mileage;
        else N.Mileage_All += dynamic_mileage;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：Init_Nag - 初始化导航系统
// 参数说明：void
// 使用示例：在系统启动时调用
// 备注信息：
//-------------------------------------------------------------------------------------------------------------------
void Init_Nag()
{
    memset(&N, 0, sizeof(N));
    memset(Nav_direction, 0, sizeof(Nav_direction));  // 清空方向数组
    N.Flash_page_index = Nag_Start_Page;
    N.target_speed = NAG_FORWARD_SPEED;
    N.pos_x = 0.0f;
    N.pos_y = 0.0f;
    N.yaw_error = 0.0f;
    N.last_distance = 0.0f;
    N.Angle_Run = angle_Z;  // 【修复】初始化时使用当前航向，避免使用未定义的旧值

    // 清除转向PID的历史误差，防止积分饱和影响启动
    servo_pid.OUT_P = 0;
    servo_pid.OUT_I = 0;
    servo_pid.OUT_D = 0;
    servo_pid.PrevError = 0;
    servo_pid.LastError = 0;
    servo_pid.Error = 0;

    flash_buffer_clear();
    flash_Nag_Reset();  // 重置Flash读取标志
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：Nag_System - 主导航系统包装器
// 功能说明：根据N.Nag_SystemRun_Index调用记录或回放模式
// 参数说明：void
// 使用示例：在中断或主循环中调用
// 备注信息：
//-------------------------------------------------------------------------------------------------------------------
void Nag_System()
{
    Nag_Position_Update();

    // 待机模式或停止状态：复位舵机到中心位置，停止电机
    if(!N.Nag_SystemRun_Index || N.Nag_Stop_f)
    {
        Servo_Motor_Control(0);  // 转向电机回中
        // 不要在这里控制电机，让car_start()函数处理
        return;
    }

    switch(N.Nag_SystemRun_Index)
    {
       case 1: Nag_Read();  // 记录模式
            break;
       case 3: Nag_Run();   // 回放模式
            break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称：NagFlashRead - 从Flash读取轨迹（一次性操作）
// 功能说明：从Flash页读取整个保存的轨迹
// 参数说明：void
// 使用示例：在进入回放模式前调用一次
// 备注信息：
//-------------------------------------------------------------------------------------------------------------------
void NagFlashRead()
{
    if(N.Save_state) return;  // 已经读取过
    flash_Nag_Read();
    unsigned char page_trun = 0;

    for(int index = 0; index <= N.Save_index; index++)
    {
        if(index >= N.Save_index)
        {
            N.Save_state = 1;
            break;
        }
        int temp_index = index - (500 * page_trun);
        if(temp_index > MaxSize)  // 到达页边界
        {
            N.Flash_page_index--;   // 移动到下一页
            page_trun++;
            flash_Nag_Read();  // 从新页重新读取
        }
        // 读取航向角和方向数据（每个点占用2个位置：角度+方向）
        int actual_buffer_index = (index - (500 * page_trun)) * 2;
        Nav_read[index] = flash_union_buffer[actual_buffer_index].int32_type;
        Nav_direction[index] = flash_union_buffer[actual_buffer_index + 1].int8_type;
    }
    N.Nag_SystemRun_Index++;
}



