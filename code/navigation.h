/*
 * navigation.h
 *
 *  Created on: 2026年6月21日
 *      Author: jred
 */

#ifndef CODE_NAVIGATION_H_
#define CODE_NAVIGATION_H_

#include "zf_common_headfile.h"

//==================== 用户参数配置 ====================//
#define MaxSize 1020    // Flash存储数组最大容量

#define Read_MaxSize 10000  // 最大读取容量，10000个点应该足够

// 页面范围 <0 - 11>（TC264 DFLASH共12页，有效范围0-11）
#define Nag_End_Page 2      // Flash结束页（递减终点）
#define Nag_Start_Page 11   // Flash起始页（从页11递减到页2，共10页可用）

#define Nag_Set_mileage 100 // 里程间隔（每5cm一个编码器单位）
#define Nag_Prev 200        // 前瞻缓冲区
#define Nag_Yaw angle_Z     // 偏航角数据源（使用当前航向）

#define L_Mileage motor_1.encoder_miles   // 左编码器里程
#define R_Mileage motor_r.encoder_miles   // 右编码器里程
//=====================================================//

typedef struct{
       float Final_Out;         // 最终控制输出
       float Mileage_All;       // 总里程累加器
       float Angle_Run;         // 要跟随的目标航向
       bool Nag_Stop_f;         // 导航停止标志
       uint8 Flash_read_f;      // 导航读取标志
       uint16 size;             // 导航数组大小/通用计数器
       uint16 Run_index;        // 回放索引
       uint16 Save_count;       // 保存计数器
       uint16 Save_index;       // 保存标志/索引
       uint8 Save_state;        // 保存状态
       uint8 End_f;             // 结束标志

       // Flash相关
       uint8 Flash_page_index;      // Flash页索引
       uint8 Flash_Save_Page_Index; // Flash保存页索引
       uint8 Nag_SystemRun_Index;   // 导航执行模式

       // 里程计与跟踪状态
       float pos_x;             // 当前X位置（编码器计数单位）
       float pos_y;             // 当前Y位置（编码器计数单位）
       float yaw_error;         // 当前航向误差
       float yaw_error_raw;     // 原始航向误差（未经启动保护限制）
       float target_speed;      // 轨迹跟踪参考速度
       float last_distance;     // 上一周期累积距离
       float servo_angle_out;   // 舵机角度输出（用于调试）
       uint8 startup_protection_active;  // 启动保护是否激活（调试用）

       // 预留将来使用
       int Prev_mile[Nag_Prev]; // 前瞻缓冲区
}Nag;

extern Nag N;   // 主导航结构体，全局可用
extern int32 Nav_read[Read_MaxSize]; // 保存的轨迹数据（每5cm），1000点 = 50m
extern int8 Nav_direction[Read_MaxSize]; // 保存的方向数据（1=前进，-1=倒车，0=停止）
extern float diff;
extern int8 back;
extern int8 current_direction;


void Nag_Run();         // 执行基于偏航角的导航
void Run_Nag_GPS();     // 读取并跟随保存的偏航轨迹

void NagFlashRead();    // 从Flash读取目标轨迹
void Run_Nag_Save();    // 记录期间保存偏航角
void Nag_Read();        // 偏航角读取包装函数
void Nag_Position_Update(void);  // 更新当前位置/里程计

//****************************//
void Init_Nag();        // 初始化导航：清空Flash缓冲区，重置计数器
void Nag_System();      // 主导航系统包装器（调用记录或回放模式）




#endif /* CODE_NAVIGATION_H_ */
