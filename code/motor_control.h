#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_

#include "zf_common_headfile.h"

//====================================================右电机左电机控制端口定义====================================================
// 右电机方向控制端口 - DRV8701电机，PWM+方向模式
#define MOTOR_RIGHT_DIR     P02_4               //  右电机方向控制端口
#define MOTOR_RIGHT_PWM     ATOM0_CH5_P02_5     // 右电机PWM，速度控制端口

#define MOTOR_LEFT_DIR      P02_6               // 左电机方向控制端口
#define MOTOR_LEFT_PWM      ATOM0_CH7_P02_7     // 左电机PWM，速度控制端口
// 转向舵机PWM控制端口
#define SERVO_PWM_PIN       ATOM1_CH1_P33_9     // 转向舵机PWM控制端口

// 编码器端口定义 - 用于速度传感器
#define ENCODER_LEFT        TIM5_ENCODER            // 左电机编码器
#define ENCODER_LEFT_CH1    TIM5_ENCODER_CH1_P10_3  // 左电机编码器通道1
#define ENCODER_LEFT_CH2    TIM5_ENCODER_CH2_P10_1  // 左电机编码器通道2

#define ENCODER_RIGHT       TIM6_ENCODER            // 右电机编码器
#define ENCODER_RIGHT_CH1   TIM6_ENCODER_CH1_P20_3  // 右电机编码器通道1
#define ENCODER_RIGHT_CH2   TIM6_ENCODER_CH2_P20_0  // 右电机编码器通道2
// DRV8701电机参数
#define MOTOR_PWM_FREQ      17000               // 电机PWM频率 17KHz，DRV8701推荐10-25KHz
#define MOTOR_MAX_DUTY      8000                // 最大占空比 (PWM_DUTY_MAX = 10000)
#define MOTOR_MIN_DUTY      500                 // 最小启动占空比，防止低速时电机停止
// 舵机参数
#define SERVO_PWM_FREQ      50                  // 舵机PWM频率 50Hz (20ms周期)
#define SERVO_CENTER_DUTY   750                 // 舵机中心占空比 (1.5ms/20ms * 10000 = 750)
#define SERVO_LEFT_MAX      500                 // 舵机最左最大占空比 (1.0ms)
#define SERVO_RIGHT_MAX     1000                // 舵机最右最大占空比 (2.0ms)
#define SERVO_MAX_ANGLE     45                  // 舵机最大转角 (+-45度)
//====================================================数据结构====================================================
// 电机结构体定义（DRV8701驱动模式）
typedef struct
{
    pwm_channel_enum pwm_pin;                   // PWM通道，速度控制端口
    gpio_pin_enum dir_pin;                      // 方向控制端口
    encoder_index_enum encoder;                 // 编码器索引
    int16 encoder_count;                        // 编码器计数
    int16 target_speed;                         // 目标速度，单位为毫米每秒
    int16 current_speed;                        // 当前速度，单位为毫米每秒
    int32 pwm_duty;                             // 当前PWM占空比（速度或制动）
    uint8 direction;                            // 当前方向，1=前进，0=后退
} motor_t;

// 舵机结构体定义
typedef struct
{
    pwm_channel_enum pwm_pin;                   // PWM通道
    int16 current_angle;                        // 当前角度 (-SERVO_MAX_ANGLE ~ SERVO_MAX_ANGLE)
    uint32 current_duty;                        // 当前占空比
} servo_t;

// 车辆控制结构体定义（电机+舵机+速度+角度）
typedef struct
{
    motor_t left_motor;                         // 左电机
    motor_t right_motor;                        // 右电机
    servo_t steering_servo;                     // 转向舵机
    int16 base_speed;                           // 基础速度
    int16 target_angle;                         // 目标转向角度
} car_control_t;

//====================================================全局变量====================================================
extern car_control_t car;

//====================================================函数声明====================================================
void motor_init(void);                                          // 电机和编码器初始化
void motor_set_duty(motor_t *motor, int32 duty);               // 设置电机占空比
void motor_update_speed(void);                                  // 更新速度传感器数据
void servo_init(void);                                          // 舵机初始化
void servo_set_angle(servo_t *servo, int16 angle);             // 设置舵机角度 (-45 ~ 45度)
void servo_set_duty(servo_t *servo, uint32 duty);              // 设置舵机占空比
void car_set_speed(int16 left_speed, int16 right_speed);        // 设置左右电机速度
void car_set_angle(int16 angle);                                // 设置转向角度
void car_stop(void);                                            // 停止
void car_forward(int16 speed);                                  // 前进
void car_backward(int16 speed);                                 // 后退
void car_turn(int16 speed, int16 angle);                        // 转向，speed: 速度, angle: 转向角度

#endif // _MOTOR_CONTROL_H_
