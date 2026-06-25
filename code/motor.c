/*
 * motor.c
 *
 *  Created on: 2026年6月9日
 *      Author: jred
 */

#include "zf_common_headfile.h"
#include "motor.h"
#include "pid.h"
#include "fuction.h"

motor1 motor_1;
motor1 motor_r;
servo_motor servo_motor_data;  // 转向电机编码器数据
servo_motor_ctrl_t servo_motor_ctrl = {0};  // 转向电机控制数据
uint8 start=0;

void Motor_init()
{
    gpio_init(MotorL_turn, GPO, 1, GPO_PUSH_PULL);
    pwm_init(MotorL_pwm, 5000, 0);
    gpio_init(MotorR_turn, GPO, 1, GPO_PUSH_PULL);
    pwm_init(MotorR_pwm, 5000, 0);

}
void Servo_init(void)
{
    // 初始化转向电机PWM（频率10kHz，DRV8301驱动板）
    pwm_init(Servo_pwm, 5000, 0);

    // 初始化转向电机方向引脚
    gpio_init(Servo_DIR, GPO, 1, GPO_PUSH_PULL);

    // 初始化转向电机控制结构体
    servo_motor_ctrl.target_angle = 0;
    servo_motor_ctrl.current_angle = 0;
    servo_motor_ctrl.duty = 0;
}

void Servo_Motor_Control(float target_angle)
{
    int duty;

    if(target_angle > SERVO_MAX_ANGLE)
        target_angle = SERVO_MAX_ANGLE;

    if(target_angle < -SERVO_MAX_ANGLE)
        target_angle = -SERVO_MAX_ANGLE;

    duty = (int)PID_Position(
                &servo_position_pid,
                servo_motor_data.current_angle,
                target_angle);

    duty = Limit_init(
                -5000,
                duty,
                5000);

    servo_motor_ctrl.target_angle = target_angle;
    servo_motor_ctrl.current_angle =
            servo_motor_data.current_angle;
    servo_motor_ctrl.duty = duty;

    Motor_SetSpeed(
            Servo_pwm,
            Servo_DIR,
            duty,
            1,
            0);
}

void car_start(void)
{

}


void Motor_SetSpeed(pwm_channel_enum pin1, gpio_pin_enum pin2, int pwm, uint8 just, uint8 lose)
{
    if(pwm >= 0)
    {
        gpio_set_level(pin2, lose);  // 娣囶喗顒滈敍姘劀鏉烆剚妞傞弬鐟版倻瀵洝鍓兼稉锟?0
        pwm_set_duty(pin1, pwm);
    }
    else
    {
        gpio_set_level(pin2, just);  // 娣囶喗顒滈敍姘冀鏉烆剚妞傞弬鐟版倻瀵洝鍓兼稉锟?1
        pwm_set_duty(pin1, -pwm);
    }
}

void Motor_Control(int Speed_L, int Speed_R)
{
    motor_1.target_speed = Speed_L;
    motor_r.target_speed = Speed_R;

    // 閻╊喗鐖ｉ柅鐔峰娑擄拷0閺冭绱濆〒鍛存祩duty閸滃ID閸樺棗褰剁拠顖氭▕
    if(Speed_L == 0)
    {
        motor_1.duty = 0;
        motor_pid_1.LastError = 0;
        motor_pid_1.PrevError = 0;
        motor_pid_1.Error = 0;
    }
    else
    {
        float increment_L = PID_Increase(&motor_pid_1,
                                         (float)motor_1.encoder_speed * 5,  // 閹垹顦茬紓鈺傛杹閸ョ姴鐡?
                                         (float)motor_1.target_speed);

        // 闂勬劕鍩楅崡鏇燁偧婢х偤鍣洪惃鍕亣鐏忓骏绱濋梼鍙夘剾缁鳖垰濮炴潻鍥ф彥
        if(increment_L > 50) increment_L = 50;
        if(increment_L < -50) increment_L = -50;

        motor_1.duty += increment_L;

        if(motor_1.encoder_speed < 1 && motor_1.encoder_speed > -1)  // 娴ｈ法鏁ら懠鍐ㄦ纯閸掋倖鏌囬弰顖氭儊闂堟瑦顒?
        {
            if(motor_1.duty > 0 && motor_1.duty < 500) motor_1.duty = 500;
            if(motor_1.duty < 0 && motor_1.duty > -500) motor_1.duty = -500;
        }

        motor_1.duty = Limit_init(-pid_limit, motor_1.duty, pid_limit);
    }

    if(Speed_R == 0)
    {
        motor_r.duty = 0;
        motor_pid_r.LastError = 0;
        motor_pid_r.PrevError = 0;
        motor_pid_r.Error = 0;
    }
    else
    {
        float increment_R = PID_Increase(&motor_pid_r,
                                         (float)motor_r.encoder_speed * 5,  // 閹垹顦茬紓鈺傛杹閸ョ姴鐡?
                                         (float)motor_r.target_speed);

        // 闂勬劕鍩楅崡鏇燁偧婢х偤鍣洪惃鍕亣鐏忓骏绱濋梼鍙夘剾缁鳖垰濮炴潻鍥ф彥
        if(increment_R > 50) increment_R = 50;
        if(increment_R < -50) increment_R = -50;

        motor_r.duty += increment_R;

        // 閸斻劍锟戒焦顒撮崠楦克夐崑鍖＄窗閸欘亜婀悽鍨簚闂堟瑦顒涢弮鎯八夐崑锟?
        if(motor_r.encoder_speed < 1 && motor_r.encoder_speed > -1)  // 娴ｈ法鏁ら懠鍐ㄦ纯閸掋倖鏌囬弰顖氭儊闂堟瑦顒?
        {
            if(motor_r.duty > 0 && motor_r.duty < 500) motor_r.duty = 500;
            if(motor_r.duty < 0 && motor_r.duty > -500) motor_r.duty = -500;
        }

        motor_r.duty = Limit_init(-pid_limit, motor_r.duty, pid_limit);
    }

    Motor_SetSpeed(MotorL_pwm, MotorL_turn, motor_1.duty, 0, 1);
    Motor_SetSpeed(MotorR_pwm, MotorR_turn, motor_r.duty, 0, 1);
}



void Encoder_init(void)
{
    encoder_quad_init(TIM4_ENCODER, TIM4_ENCODER_CH1_P02_8, TIM4_ENCODER_CH2_P00_9);
    encoder_quad_init(TIM2_ENCODER, TIM2_ENCODER_CH1_P33_7, TIM2_ENCODER_CH2_P33_6);
}

void Servo_Encoder_init(void)
{
    encoder_quad_init(TIM5_ENCODER, TIM5_ENCODER_CH1_P10_3, TIM5_ENCODER_CH2_P10_1);
    servo_motor_data.encoder_raw = 0;
    servo_motor_data.total_encoder = 0;
    servo_motor_data.current_angle = 0;
}

void Data_encoder(void)
{
    motor_1.encoder_raw=encoder_get_count(TIM4_ENCODER);
    motor_1.encoder_miles+=motor_1.encoder_raw;
    encoder_clear_count(TIM4_ENCODER);
    motor_1.encoder_speed=motor_1.encoder_raw;
    motor_1.total_encoder+=motor_1.encoder_raw;

    motor_r.encoder_raw=-encoder_get_count(TIM2_ENCODER);  // 右轮编码器取反
    motor_r.encoder_miles+=motor_r.encoder_raw;
    encoder_clear_count(TIM2_ENCODER);
    motor_r.encoder_speed=motor_r.encoder_raw;
    motor_r.total_encoder+=motor_r.encoder_raw;

}

// 读取转向编码器数据
// 每5ms调用一次（在中断中）
void Data_Servo_encoder(void)
{
    servo_motor_data.encoder_raw = encoder_get_count(TIM5_ENCODER);
    servo_motor_data.total_encoder += servo_motor_data.encoder_raw;
    encoder_clear_count(TIM5_ENCODER);

    // 将编码器值转换为角度
    // 这个系数需要根据实际机械结构标定！
    // 标定方法：
    // 1. 手动转动转向轮到最大角度（比如45度）
    // 2. 读取 servo_motor_data.total_encoder 的值
    // 3. 系数 = 45.0 / total_encoder的值
    // 例如：转动45度，编码器累计4500，则系数 = 45.0/4500 = 0.01
    servo_motor_data.current_angle = (float)servo_motor_data.total_encoder * 0.01f;
}

