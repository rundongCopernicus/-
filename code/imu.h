/*
 * imu.h
 *
 *  Created on: 2026年6月10日
 *      Author: jred
 */

#ifndef CODE_IMU_H_
#define CODE_IMU_H_

#include "zf_common_headfile.h"

//全局变量
extern int gyro_offset_flag;


typedef struct{
        float xdata;
        float ydata;
        float zdata;
}gyro_param_t_imu;

typedef struct{
        float acc_x;
        float acc_y;
        float acc_z;

        float gyro_x;
        float gyro_y;
        float gyro_z;

}IMU_param_t;






float IMU_gyro_init(void);
void IMU_GetValues();
void IMU_YAW_integral(void);
void IMU_Handle_360(void);
void IMU_init(void);
void IMU_SNOW(void);

#endif /* CODE_IMU_H_ */
