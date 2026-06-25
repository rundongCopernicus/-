/*
 * remote.h
 *
 *  Created on: 2026年6月9日
 *      Author: jred
 */

#ifndef CODE_REMOTE_H_
#define CODE_REMOTE_H_
#define SERVO_MIN_ANGLE     -45.0f      //舵机最小值
#define SERVO_MID_ANGLE     0.0f        //舵机中值

extern float telecontrol_flag;          //遥控标志位


void remote_init(void);
void remote_test(void);
void remote_use(void);



#endif /* CODE_REMOTE_H_ */
