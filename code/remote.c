/*
 * remote.c
 *
 *  Created on: 2026年6月9日
 *      Author: jred
 */

#include "zf_common_headfile.h"
#include "motor.h"

// 遥控器控制主函数
// 读取SBUS接收机数据，根据通道值控制舵机和电机

float telecontrol_flag =0; // 遥控标志位（用于判别遥控优先级）

uint16_t ch1,ch2,ch3,ch4,ch5,ch6;// SBUS通道数据变量
int servo1;// 舵机角度取整变量

// 遥控器初始化函数
void remote_init(void)
{
    uart_receiver_init();  // SBUS接收机初始化
}

// 遥控器测试函数
// 打印各通道数据，用于调试
void remote_test(void)
{
    if (1 == uart_receiver.finsh_flag)  // 帧完成标志：表示收到完整的一帧数据
    {
        if (1 == uart_receiver.state)  // state=1表示遥控器连接正常
        {
            printf("CH1-CH6 data: ");
            for (int i = 0; i < 6; i++)
            {
                printf("%d ", uart_receiver.channel[i]);  // 串口输出6个通道数据
            }
            printf("\r\n");
        }
        else
        {
            printf("Remote control has been disconnected.\r\n");  // 串口输出失控提示
        }
        uart_receiver.finsh_flag = 0;  // 帧完成标志复位，准备接收下一帧数据
    }
}


// 遥控器控制主函数
// 根据SBUS通道值控制舵机和电机
void remote_use(void)
{
    // 读取各通道数据
    ch1 = uart_receiver.channel[0]; // 通道1：舵机/转向控制
    ch2 = uart_receiver.channel[1]; // 通道2：油门/前进后退
    ch3 = uart_receiver.channel[2]; //
    ch4 = uart_receiver.channel[3];
    ch5 = uart_receiver.channel[4];
    ch6 = uart_receiver.channel[5];

    // 舵机角度计算：将通道值转换为舵机角度偏移量
    
    int motor_speed; // 电机速度变量
    // 检查是否有新的遥控器数据帧
    if(1 == uart_receiver.finsh_flag)  // 帧完成标志：表示收到完整的一帧数据
    {
        // 检查遥控器连接状态
        if(1 == uart_receiver.state)   // state=1表示遥控器连接正常
        {
            // ==================== 舵机控制（通道1） ====================
            // 通道1对应遥控器右摇杆左右方向（转向控制）
            // SBUS通道值范围：192-1792，中心值约992
            // ch1中间值为1048，最大值为1846，最小值为252
            servo1 = (int)(((1000 - ch1) / 100)*3.15); // 舵机通道数据取整
            if (ch1>=1048 && ch1 <= 1846)  // 摇杆右打
            {
                // 舵机随着摇杆右打角度增大
                Servo_Motor_Control(SERVO_MID_ANGLE - servo1);  // 舵机右打，角度根据摇杆位置调整
            }
            else if (ch1>=252 && ch1 <= 1048)  // 摇杆左打
            {
                // 舵机随着摇杆左打角度增大
                Servo_Motor_Control(SERVO_MID_ANGLE + servo1);  // 舵机左打，角度根据摇杆位置调整
            }
            else  // 摇杆在中间区域
            {
                Servo_Motor_Control(SERVO_MID_ANGLE);   // 舵机回中
            }

            // ==================== 电机控制（通道2） ====================
            // 通道2对应遥控器左摇杆上下方向（油门/前进后退）
            // ch2中间值为1048，最大值为1840，最小值为248
            if (ch2>=1048 && ch2 <= 1840)  // 摇杆上推
            {
                // 电机前进，速度随摇杆上推增大
                motor_speed = (int)((ch2 - 1048) / 100 * 100); // 计算电机速度，范围0-100%
                Motor_Control(motor_speed,motor_speed); // 电机前进
            }
            else if (ch2>=248 && ch2 <= 1048)  // 摇杆下拉
            {
                // 电机后退，速度随摇杆下拉增大
                motor_speed = (int)((1048 - ch2) / 100 * 100); // 计算电机速度，范围0-100%
                Motor_Control(-motor_speed,-motor_speed); // 电机后退
            }
            else  // 摇杆在中间区域
            {
                Motor_Control(0,0);    // 电机停止
                Servo_Motor_Control(SERVO_MID_ANGLE);   // 舵机回中（停止时同时回正方向）
            }
        }
        else  // 遥控器失控（state=0）
        {
            printf("Remote control has been disconnected.\r\n");  // 串口输出失控提示信息
        }

        uart_receiver.finsh_flag = 0;  // 清除帧完成标志，准备接收下一帧数据
    }
}
