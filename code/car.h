/*
 * car.h
 *
 *  Created on: 2026年6月9日
 *      Author: jred
 */

#ifndef CODE_CAR_H_
#define CODE_CAR_H_


// 小车控制相关函数和数据结构

#ifndef CAR_SPEED_DEFAULT
#define CAR_SPEED_DEFAULT  50      // 默认行驶速度
#endif

// 小车停止
void car_stop(void);

// 小车前进
// speed: 前进速度 (0-100)
void car_go_forward (int speed);

// 小车后退
// speed: 后退速度 (0-100)
void car_go_backward(int speed);

// 小车左转
// speed: 转弯速度, turn_ratio: 内外轮速度比 (0.0-1.0, 越小转得越急)
void car_turn_left(int speed, float turn_ratio);

// 小车右转
// speed: 转弯速度, turn_ratio: 内外轮速度比 (0.0-1.0, 越小转得越急)
void car_turn_right(int speed, float turn_ratio);







#endif /* CODE_CAR_H_ */
