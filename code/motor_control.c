#include "motor_control.h"
#include <math.h>

//====================================================车辆控制结构体====================================================
car_control_t car;

//====================================================函数实现====================================================
/**
 * @brief  限制数值范围
 * @param  value  输入值
 * @param  min    最小值
 * @param  max    最大值
 * @return 限制后的值
 */
static int32 limit(int32 value, int32 min, int32 max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

//====================================================舵机函数实现====================================================
/**
 * @brief  舵机初始化
 * @param  无
 * @return 无
 */
void servo_init(void)
{
    // 初始化舵机PWM，频率50Hz，占空比为中心位置
    pwm_init(SERVO_PWM_PIN, SERVO_PWM_FREQ, SERVO_CENTER_DUTY);
}

/**
 * @brief  设置舵机占空比
 * @param  servo  舵机结构体指针
 * @param  duty   占空比 (500-1000, 对应1.0ms-2.0ms)
 * @return 无
 */
void servo_set_duty(servo_t *servo, uint32 duty)
{
    // 限制范围
    if (duty < SERVO_LEFT_MAX)
        duty = SERVO_LEFT_MAX;
    else if (duty > SERVO_RIGHT_MAX)
        duty = SERVO_RIGHT_MAX;
    
    servo->current_duty = duty;
    pwm_set_duty(servo->pwm_pin, duty);
}

/**
 * @brief  设置舵机角度
 * @param  servo  舵机结构体指针
 * @param  angle  角度 (-45 ~ 45度，单位为度)
 * @return 无
 */
void servo_set_angle(servo_t *servo, int16 angle)
{
    // 限制范围
    angle = limit(angle, -SERVO_MAX_ANGLE, SERVO_MAX_ANGLE);
    servo->current_angle = angle;
    
    int32 duty = SERVO_CENTER_DUTY+(SERVO_RIGHT_MAX-SERVO_CENTER_DUTY)*angle/45;
    
    servo_set_duty(servo, (uint32)duty);
}

/**
 * @brief  电机和编码器初始化
 * @param  无
 * @return 无
 */
void motor_init(void)
{
    // ========== 右电机左电机初始化，DRV8701驱动模式 ==========
    car.left_motor.pwm_pin       = MOTOR_LEFT_PWM;
    car.left_motor.dir_pin       = MOTOR_LEFT_DIR;
    car.left_motor.encoder       = ENCODER_LEFT;
    car.left_motor.encoder_count = 0;
    car.left_motor.target_speed  = 0;
    car.left_motor.current_speed = 0;
    car.left_motor.pwm_duty      = 0;
    car.left_motor.direction     = 1;
    
    // ========== 右电机左电机初始化，DRV8701驱动模式 ==========
    car.right_motor.pwm_pin       = MOTOR_RIGHT_PWM;
    car.right_motor.dir_pin       = MOTOR_RIGHT_DIR;
    car.right_motor.encoder       = ENCODER_RIGHT;
    car.right_motor.encoder_count = 0;
    car.right_motor.target_speed  = 0;
    car.right_motor.current_speed = 0;
    car.right_motor.pwm_duty      = 0;
    car.right_motor.direction     = 1;
    
    // ========== 转向舵机初始化 ==========
    car.steering_servo.pwm_pin      = SERVO_PWM_PIN;
    car.steering_servo.current_angle = 0;
    car.steering_servo.current_duty  = SERVO_CENTER_DUTY;
    
    // ========== 车辆控制结构体初始化 ==========
    car.base_speed   = 0;
    car.target_angle = 0;
    
    // ========== 硬件初始化 ==========
    // 初始化电机PWM，DRV8701: PWM频率17KHz
    pwm_init(MOTOR_LEFT_PWM, MOTOR_PWM_FREQ, 0);
    pwm_init(MOTOR_RIGHT_PWM, MOTOR_PWM_FREQ, 0);
    
    // 初始化电机方向控制端口，DRV8701: DIR引脚
    gpio_init(MOTOR_LEFT_DIR, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(MOTOR_RIGHT_DIR, GPO, GPIO_LOW, GPO_PUSH_PULL);
    
    // 初始化转向舵机PWM
    servo_init();
    
    // 初始化编码器
    encoder_dir_init(ENCODER_LEFT, ENCODER_LEFT_CH1, ENCODER_LEFT_CH2);
    encoder_dir_init(ENCODER_RIGHT, ENCODER_RIGHT_CH1, ENCODER_RIGHT_CH2);
}

/**
 * @brief  设置电机占空比，适用于DRV8701: PWM+DIR驱动模式
 * @param  motor  电机结构体指针
 * @param  duty   占空比 (-MOTOR_MAX_DUTY ~ MOTOR_MAX_DUTY)
 * @return 无
 * @note   适用于DRV8701驱动模式，DIR引脚控制方向，PWM引脚控制占空比
 */
void motor_set_duty(motor_t *motor, int32 duty)
{
    // 限制范围
    duty = limit(duty, -MOTOR_MAX_DUTY, MOTOR_MAX_DUTY);
    motor->pwm_duty = duty;
    
    if (duty > 0)
    {
        // 正转: DIR=0, PWM=duty
        gpio_set_level(motor->dir_pin, GPIO_LOW);
        motor->direction = 0;
        pwm_set_duty(motor->pwm_pin, (uint32)duty);
    }
    else if (duty < 0)
    {
        // 反转: DIR=1, PWM=abs(duty)
        gpio_set_level(motor->dir_pin, GPIO_HIGH);
        motor->direction = 1;
        pwm_set_duty(motor->pwm_pin, (uint32)(-duty));
    }
    else
    {
        // 停止: PWM=0 (关闭电机)
        pwm_set_duty(motor->pwm_pin, 0);
    }
}

/**
 * @brief  更新电机速度，基于编码器计数，建议10ms调用一次
 * @param  无
 * @return 无
 * @note   建议10ms调用一次，基于编码器计数更新电机速度
 */
void motor_update_speed(void)
{
    // 获取编码器计数
    car.left_motor.encoder_count = encoder_get_count(car.left_motor.encoder);
    car.right_motor.encoder_count = -encoder_get_count(car.right_motor.encoder);
    
    // 更新当前速度
    car.left_motor.current_speed = car.left_motor.encoder_count;
    car.right_motor.current_speed = car.right_motor.encoder_count;
    
    // 清零编码器计数
    encoder_clear_count(car.left_motor.encoder);
    encoder_clear_count(car.right_motor.encoder);
}

/**
 * @brief  设置车辆左右电机速度
 * @param  left_speed   左电机速度
 * @param  right_speed  右电机速度
 * @return 无
 */
void car_set_speed(int16 left_speed, int16 right_speed)
{
    car.left_motor.target_speed = left_speed;
    car.right_motor.target_speed = right_speed;
    motor_set_duty(&car.left_motor, left_speed);
    motor_set_duty(&car.right_motor, right_speed);
}

/**
 * @brief  设置车辆转向角度
 * @param  angle  转向角度 (-45 ~ 45度)
 * @return 无
 */
void car_set_angle(int16 angle)
{
    car.target_angle = angle;
    servo_set_angle(&car.steering_servo, angle);
}

/**
 * @brief  停止车辆
 * @param  无
 * @return 无
 */
void car_stop(void)
{
    motor_set_duty(&car.left_motor, 0);
    motor_set_duty(&car.right_motor, 0);
    
    car.left_motor.target_speed = 0;
    car.right_motor.target_speed = 0;
    // 停止车辆
}

/**
 * @brief  前进
 * @param  speed  速度
 * @return 无
 */
void car_forward(int16 speed)
{
    car_set_speed(speed, speed);          // 前进: 左右电机速度相同
    servo_set_angle(&car.steering_servo, 0);  // 转向角度归零
}

/**
 * @brief  后退
 * @param  speed  速度
 * @return 无
 */
void car_backward(int16 speed)
{
    car_set_speed(-speed, -speed);        // 后退: 左右电机速度相同
    servo_set_angle(&car.steering_servo, 0);  // 转向角度归零
}

/**
 * @brief  转向
 * @param  speed  速度
 * @param  angle  转向角度 (-45 ~ 45度)
 * @return 无
 */
void car_turn(int16 speed, int16 angle)
{
    car_set_angle(angle);                 // 转向角度设置
    car_update_differential_speed(speed, angle); // 使用差速控制
}

/**
 * @brief  根据转向角度更新差速 (阿克曼转向几何)
 * @param  base_speed 基础速度
 * @param  angle      转向角度
 * @return 无
 */
void car_update_differential_speed(int16 base_speed, int16 angle)
{
    if (angle == 0)
    {
        car_set_speed(base_speed, base_speed);
        return;
    }

    // 角度转弧度
    float theta = (float)angle * 3.14159f / 180.0f;
    float tan_theta = tanf(theta);
    
    // 计算转弯半径 R = L / tan(theta)
    // 避免除零，虽然angle!=0已判断，但tan_theta可能极小
    if (fabsf(tan_theta) < 0.001f)
    {
        car_set_speed(base_speed, base_speed);
        return;
    }
    
    // 阿克曼几何差速公式
    // V_left  = V_center * (1 - W * tan(theta) / (2 * L))
    // V_right = V_center * (1 + W * tan(theta) / (2 * L))
    // 其中 W = CAR_TRACK_WIDTH, L = CAR_WHEELBASE
    
    float ratio = (float)CAR_TRACK_WIDTH * tan_theta / (2.0f * (float)CAR_WHEELBASE);
    
    int16 left_speed = (int16)((float)base_speed * (1.0f - ratio));
    int16 right_speed = (int16)((float)base_speed * (1.0f + ratio));
    
    car_set_speed(left_speed, right_speed);
}
