/*
 * remote.h
 *
 *  Created on: 2026Äê6ÔÂ9ÈÕ
 *      Author: jred
 */

#ifndef CODE_REMOTE_H_
#define CODE_REMOTE_H_
#define SERVO_MIN_ANGLE     -45.0f      //????????
#define SERVO_MID_ANGLE     0.0f        //???????????

extern float telecontrol_flag;          //?????

void remote_init(void);
void remote_test(void);
void remote_use(void);



#endif /* CODE_REMOTE_H_ */
