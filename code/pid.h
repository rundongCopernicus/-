/*
 * pid.h
 *
 *  Created on: 2026ÔøΩÔøΩ4ÔøΩÔøΩ11ÔøΩÔøΩ
 *      Author: lenovo
 */

#ifndef CODE_PID_H_
#define CODE_PID_H_

#include "zf_common_headfile.h"

typedef struct PID
{
   float Kp;
   float Ki;
   float Kd;
   float LowPass;

   float OUT_P;
   float OUT_I;
   float OUT_D;

   float PrevError;
   float LastError;
   float Error;
   float LastData;
}PID;

#define PID_CREATE(_KP,_KI,_KD,_LOSS_PASS) \
{ \
    .Kp = _KP, \
    .Ki = _KI, \
    .Kd = _KD, \
    .LowPass = _LOSS_PASS, \
    .OUT_P = 0, \
    .OUT_I = 0, \
    .OUT_D = 0, \
    .PrevError = 0, \
    .LastError = 0, \
    .Error = 0, \
    .LastData = 0 \
}

float PID_Increase (PID *PID,float NowData,float Point);
float PID_Position(PID *PID, float NowData, float Point);
float PID_NORMAL(PID *PID, float NowData, float Point);

extern PID motor_pid_1;
extern PID motor_pid_r;
extern PID servo_pid;
extern PID servo_position_pid;  // ËΩ?êëÁîµÊú∫‰ΩçÁΩÆPID

#endif /* CODE_PID_H_ */
