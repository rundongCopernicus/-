
/*
 * imu.c
 *
 *  Created on: 2026Äê6ÔÂ10ÈÕ
 *      Author: jred
 */

#include "zf_common_headfile.h"
#include "IMU.h"

gyro_param_t_imu gyro_offset;
IMU_param_t IMU_Data;


int gyro_offset_flag=0;
float z_360 = 0;

// é›¶ç¥¨åˆå?åŒ? //
float IMU_gyro_init()
{
    gyro_offset.xdata = 0;
    gyro_offset.ydata = 0;
    gyro_offset.zdata = 0;
    f¡¤or(uint16_t i = 0;i<1000;i++)
    {
        gyro_offset.xdata += imu963ra_gyro_x;
        gyro_offset.ydata += imu963ra_gyro_y;
        gyro_offset.zdata += imu963ra_gyro_z;
        system_delay_ms(5);

    }
    gyro_offset.xdata /=1000;
    gyro_offset.ydata /=1000;
    gyro_offset.zdata /=1000;

    return gyro_offset_flag=1;
}




void IMU_GetValues()
{
    IMU_Data.gyro_x =((float)imu963ra_gyro_x - gyro_offset.xdata) * PI/180/16.4f;
    IMU_Data.gyro_y =((float)imu963ra_gyro_y - gyro_offset.ydata) * PI/180/16.4f;
    IMU_Data.gyro_z =((float)imu963ra_gyro_z - gyro_offset.zdata) * PI/180/16.4f;
}


void IMU_YAW_integral()
{
    IMU_GetValues();

    if(IMU_Data.gyro_z<0.015&&IMU_Data.gyro_z>-0.015)
    {
        z_360-=0;
    }
    else
    {
    IMU_Handle_360();
    }

}
void IMU_Handle_360()
{
        z_360-=RAD_TO_ANGLE(IMU_Data.gyro_z*0.005);


       if(z_360>360||z_360<-360)
        {
            z_360=0;
        }

}
void IMU_init()
{
    imu963ra_init();
    IMU_gyro_init();

}

void IMU_SNOW()
{
    ips200_show_string(5, 230,"z_360");      ips200_show_float(5, 250,z_360,3,6);


}
