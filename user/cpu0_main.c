/*********************************************************************************************************************
* TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC264 开源库的一部分
*
* TC264 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          cpu0_main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.8.0
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-15       pudding            first version
********************************************************************************************************************/

#include "isr_config.h"
#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中
#define    BOMA_2             (P20_7)
#define    BOMA_1             (P11_2)
#define KEY1                    (P20_6)
// Main button definitions:
// BOMA_1: 进入记录模式
// BOMA_2: 读取 Flash 并进入回放模式
// KEY1: 结束记录并写入 Flash
// KEY2: 启动或停止电机前进

// *************************** 例程硬件连接说明 ***************************
// 使用逐飞科技 英飞凌TriCore 调试下载器连接
//      直接将下载器正确连接在核心板的调试下载接口即可
// 使用 USB-TTL 模块连接
//      模块管脚            单片机管脚
//      USB-TTL-RX          查看 zf_common_debug.h 文件中 DEBUG_UART_TX_PIN 宏定义的引脚 默认 P14_0
//      USB-TTL-TX          查看 zf_common_debug.h 文件中 DEBUG_UART_RX_PIN 宏定义的引脚 默认 P14_1
//      USB-TTL-GND         核心板电源地 GND
//      USB-TTL-3V3         核心板 3V3 电源
// 接入正交编码器连接
//      模块管脚            单片机管脚
//      A                   ENCODER_QUADDEC_A 宏定义的引脚 默认 B4
//      B                   ENCODER_QUADDEC_B 宏定义的引脚 默认 B5
//      GND                 核心板电源地 GND
//      3V3                 核心板 3V3 电源
// 接入方向编码器连接
//      模块管脚            单片机管脚
//      LSB                 ENCODER_DIR_PULSE 宏定义的引脚 默认 B6
//      DIR                 ENCODER_DIR_DIR 宏定义的引脚 默认 B7
//      GND                 核心板电源地 GND
//      3V3                 核心板 3V3 电源



// *************************** 例程测试说明 ***************************
// 1.核心板烧录完成本例程，单独使用核心板与调试下载器或者 USB-TTL 模块，并连接好编码器，在断电情况下完成连接
// 2.将调试下载器或者 USB-TTL 模块连接电脑，完成上电
// 3.电脑上使用串口助手打开对应的串口，串口波特率为 zf_common_debug.h 文件中 DEBUG_UART_BAUDRATE 宏定义 默认 115200，核心板按下复位按键
// 4.可以在串口助手上看到如下串口信息：
//      ENCODER_QUADDEC counter     x .
//      ENCODER_DIR counter         x .
// 5.转动编码器就会看到数值变化
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查


// **************************** 代码区域 ****************************



// ============================================
int core0_main(void)
{   
    clock_init();
    debug_init();
    ips200_init(IPS200_TYPE_SPI);
    ips200_clear();
    imu963ra_init();
    gyroOffset_init();
    gpio_init(BOMA_2, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(BOMA_1, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP);
    Init_Nag();
    Encoder_init();
    Servo_Encoder_init();  // 初始化转向编码器
    Motor_init();
    Servo_init();
    remote_init();
    pit_ms_init(CCU60_CH1, 5);
    pit_ms_init(CCU60_CH0, 1);
    cpu_wait_event_ready();

    
    
    
    while (1)
    {
        if(!gpio_get_level(KEY1)){
            N.End_f=1;
        }

        if(!gpio_get_level(BOMA_1)){
            // 进入记录模式前，重置所有导航变量
            Init_Nag();  // 清空所有状态
            telecontrol_flag = 1;  // 开启遥控模式 ，在中断中反映出来
            N.Nag_SystemRun_Index=1;  // 进入记录模式
        }

        if(!gpio_get_level(BOMA_2)){
            telecontrol_flag = 0;  // 进入导航模式
            if(N.Nag_SystemRun_Index != 3) {  // 避免重复读取
                NagFlashRead();  // 先从Flash读取路径数据
                N.Mileage_All = 0;  //  清零里程累积，避免索引疯涨
                N.Run_index = 0;    //  重置运行索引

                // angle_Z现在已经在[-180, 180]范围内，不需要再归一化

                // 读取第一个目标角度
                if(N.Save_index > 0) {
                    N.Angle_Run = (Nav_read[0] / 100.0f);
                } else {
                    N.Angle_Run = angle_Z;
                }

                N.Nag_SystemRun_Index=3;  // 然后进入回放模式
            }
        }


        // 显示系统状态（IPS200屏幕分辨率：320x240 或 240x320）
        ips200_show_string(0, 20, "Mode:");
        ips200_show_int(50, 20, N.Nag_SystemRun_Index, 1);  // 0=待机 1=记录 3=回放

        ips200_show_string(0, 180, "Target:");
        ips200_show_float(70, 180, N.Angle_Run, 3, 2);

        ips200_show_string(0, 40, "angle_E:");
        ips200_show_float(70, 40, angle_Z, 3, 2);

        ips200_show_string(0, 60, "YawErr:");
        ips200_show_float(70, 60, N.yaw_error, 3, 2);

        ips200_show_string(0, 80, "YawRaw:");
        ips200_show_float(70, 80, N.yaw_error_raw, 3, 2);


        ips200_show_string(0, 120, "Speed_L:");
        ips200_show_float(70, 120, motor_1.encoder_speed, 3, 1);

        ips200_show_string(0, 140, "Speed_R:");
        ips200_show_float(70, 140, motor_r.encoder_speed, 3, 1);

        ips200_show_string(0, 120, "EncL:");
        ips200_show_int(50, 120, motor_1.encoder_raw, 5);  // 左轮编码器原始值

        ips200_show_string(0, 140, "EncR:");
        ips200_show_int(50, 140, motor_r.encoder_raw, 5);  // 右轮编码器原始值

        ips200_show_string(0,20*6,"PosX"); ips200_show_float(8*sizeof("PosX"),20*6,N.pos_x,5,1);
        ips200_show_string(0,20*7,"PosY"); ips200_show_float(8*sizeof("PosY"),20*7,N.pos_y,5,1);


        ips200_show_string(0, 160, "back:");
        ips200_show_int(70, 160, back, 5);  // 记录的总点数

        ips200_show_string(0, 200, "SaveIdx:");
        ips200_show_int(70, 200, N.Save_index, 5);  // 记录的总点数

        ips200_show_string(0, 220, "RunIdx:");
        ips200_show_int(70, 220, N.Run_index, 5);  // 当前回放索引

        ips200_show_string(0, 240, "Mile:");
        ips200_show_float(50, 240, N.Mileage_All, 5, 1);  // 当前里程累计

        ips200_show_string(0, 260, "ServoAng:");
        ips200_show_float(85, 260, N.servo_angle_out, 3, 2);  // 显示舵机输出角度

        ips200_show_string(0, 280, "diff:");
        ips200_show_float(85, 280, diff, 3, 2);  // 显示舵机输出角度


        // 延时，降低主循环刷新频率，避免CPU占用过高
        system_delay_ms(50);
    }
}

/*IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU60_CH0);

    encoder_data_quaddec = encoder_get_count(ENCODER_QUADDEC);                  // 获取编码器计数
    encoder_data_dir = encoder_get_count(ENCODER_DIR);                          // 获取编码器计数

    encoder_clear_count(ENCODER_QUADDEC);                                       // 清空编码器计数
    encoder_clear_count(ENCODER_DIR);                                           // 清空编码器计数
}*/

//项目中所有初始化函数都放在 code_init 函数中，方便管理


#pragma section all restore
// **************************** 代码区域 ****************************
// *************************** 例程常见问题说明 ***************************
// 遇到问题时请按照以下问题检查列表检查
// 问题1：串口没有数据
//      查看串口助手打开的是否是正确的串口，检查打开的 COM 口是否对应的是调试下载器或者 USB-TTL 模块的 COM 口
//      如果是使用逐飞科技 英飞凌TriCore 调试下载器连接，那么检查下载器线是否松动，检查核心板串口跳线是否已经焊接，串口跳线查看核心板原理图即可找到
//      如果是使用 USB-TTL 模块连接，那么检查连线是否正常是否松动，模块 TX 是否连接的核心板的 RX，模块 RX 是否连接的核心板的 TX
// 问题2：串口数据乱码
//      查看串口助手设置的波特率是否与程序设置一致，程序中 zf_common_debug.h 文件中 DEBUG_UART_BAUDRATE 宏定义为 debug uart 使用的串口波特率
// 问题3：数值一直在正负一跳转
//      不要把方向编码器接在正交编码器模式的接口下
// 问题4：数值不随编码转动变化
//      如果使用主板测试，主板必须要用电池供电
//      检查编码器是否是正常的，线是否松动，编码器是否发热烧了，是否接错线
