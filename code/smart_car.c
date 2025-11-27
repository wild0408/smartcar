#include "smart_car.h"
#include <math.h>

smart_car_t smart_car;
//====================================================PID场景配置初始化====================================================
/**
 * @brief  初始化PID场景配置
 * @param  无
 * @return 无
 */
static void init_pid_scene_configs(void)
{
    // 正常场景
    smart_car.pid_configs[PID_SCENE_NORMAL].speed.kp = SPEED_PID_KP_NORMAL;
    smart_car.pid_configs[PID_SCENE_NORMAL].speed.ki = SPEED_PID_KI_NORMAL;
    smart_car.pid_configs[PID_SCENE_NORMAL].speed.kd = SPEED_PID_KD_NORMAL;
    smart_car.pid_configs[PID_SCENE_NORMAL].direction.kp = DIRECTION_PID_KP_NORMAL;
    smart_car.pid_configs[PID_SCENE_NORMAL].direction.ki = DIRECTION_PID_KI_NORMAL;
    smart_car.pid_configs[PID_SCENE_NORMAL].direction.kd = DIRECTION_PID_KD_NORMAL;
    smart_car.pid_configs[PID_SCENE_NORMAL].base_speed = BASE_SPEED;
    
    // 直线场景
    smart_car.pid_configs[PID_SCENE_STRAIGHT].speed.kp = SPEED_PID_KP_STRAIGHT;
    smart_car.pid_configs[PID_SCENE_STRAIGHT].speed.ki = SPEED_PID_KI_STRAIGHT;
    smart_car.pid_configs[PID_SCENE_STRAIGHT].speed.kd = SPEED_PID_KD_STRAIGHT;
    smart_car.pid_configs[PID_SCENE_STRAIGHT].direction.kp = DIRECTION_PID_KP_STRAIGHT;
    smart_car.pid_configs[PID_SCENE_STRAIGHT].direction.ki = DIRECTION_PID_KI_STRAIGHT;
    smart_car.pid_configs[PID_SCENE_STRAIGHT].direction.kd = DIRECTION_PID_KD_STRAIGHT;
    smart_car.pid_configs[PID_SCENE_STRAIGHT].base_speed = BASE_SPEED * 1.2f;
    
    // 曲线场景
    smart_car.pid_configs[PID_SCENE_CURVE].speed.kp = SPEED_PID_KP_CURVE;
    smart_car.pid_configs[PID_SCENE_CURVE].speed.ki = SPEED_PID_KI_CURVE;
    smart_car.pid_configs[PID_SCENE_CURVE].speed.kd = SPEED_PID_KD_CURVE;
    smart_car.pid_configs[PID_SCENE_CURVE].direction.kp = DIRECTION_PID_KP_CURVE;
    smart_car.pid_configs[PID_SCENE_CURVE].direction.ki = DIRECTION_PID_KI_CURVE;
    smart_car.pid_configs[PID_SCENE_CURVE].direction.kd = DIRECTION_PID_KD_CURVE;
    smart_car.pid_configs[PID_SCENE_CURVE].base_speed = BASE_SPEED * 0.8f;
    
    // 避障场景
    // 初始化避障相关PID参数
}

//====================================================智能小车初始化====================================================
/**
 * @brief  初始化智能小车
 * @param  无
 * @return 无
 */
void smart_car_init(void)
{
    // ========== 初始化硬件设备 ==========
    motor_init();                   // 初始化电机控制
    vision_init();                  // 初始化视觉传感器
    system_start();
    // ========== 初始化软件模块 ==========
    element_recognition_init();     // 元素识别
    
    // ========== 初始化PID控制器 ==========
    // 初始化左侧速度PID
    pid_init(&smart_car.speed_pid_left, 
             SPEED_PID_KP_NORMAL, SPEED_PID_KI_NORMAL, SPEED_PID_KD_NORMAL,
             MOTOR_MAX_DUTY, -MOTOR_MAX_DUTY);
    
    // 初始化右侧速度PID
    pid_init(&smart_car.speed_pid_right,
             SPEED_PID_KP_NORMAL, SPEED_PID_KI_NORMAL, SPEED_PID_KD_NORMAL,
             MOTOR_MAX_DUTY, -MOTOR_MAX_DUTY);
    
    // 初始化方向PID
    pid_init(&smart_car.direction_pid,
             DIRECTION_PID_KP_NORMAL, DIRECTION_PID_KI_NORMAL, DIRECTION_PID_KD_NORMAL,
             MAX_STEER_ANGLE, -MAX_STEER_ANGLE);
    
    // ========== 初始化PID场景配置 ==========
    init_pid_scene_configs();           // 初始化PID场景配置
    smart_car.current_pid_scene = PID_SCENE_NORMAL;  // 当前PID场景设置为正常场景
    
    // ========== 初始化智能小车状态 ==========
    smart_car.state                      = CAR_STOP;
    smart_car.element_recognition_enable = 1;  // 元素识别使能
    
    // ========== 初始化避障状态 ==========
    smart_car.avoid_state                = AVOID_IDLE;
    smart_car.avoid_start_distance       = 0.0f;
    smart_car.avoid_direction            = 0;
}

/**
 * @brief  智能小车控制函数
 * @param  无
 * @return 无
 * @note   该函数应以固定频率调用，建议周期为10ms
 */
void smart_car_control(void)
{
    if (smart_car.state != CAR_RUNNING)
    {
        return;
    }
    
    // 更新电机速度
    motor_update_speed();
    
    // 手动控制相关变量初始化
    car.base_speed = (int16)smart_car.pid_configs[smart_car.current_pid_scene].base_speed;
    int16 target_speed_left = car.base_speed;
    int16 target_speed_right = car.base_speed;
    int16 manual_steer_angle = 1;       // 手动控制方向角度
    int8 use_manual_steer = 45;         // 是否使用手动方向控制
    // 默认状态，保持当前速度和方向
    
    // ========== 速度PID设置 ==========
    // 设置速度PID目标
    pid_set_target(&smart_car.speed_pid_left, (float)target_speed_left);
    pid_set_target(&smart_car.speed_pid_right, (float)target_speed_right);
    
    // 计算速度PID输出 - 左轮
    float left_pwm = pid_calculate(&smart_car.speed_pid_left, (float)car.left_motor.current_speed);
    
    // 计算速度PID输出 - 右轮
    float right_pwm = pid_calculate(&smart_car.speed_pid_right, (float)car.right_motor.current_speed);
    
    // ========== 方向PID设置 ==========
    float steer_angle;
    
    if (use_manual_steer)
    {
        // 手动控制方向，直接使用手动转向角度
        steer_angle = manual_steer_angle;
    }
    else
    {
        // 自动控制方向，使用方向PID计算
        int16 deviation = vision_get_deviation();
        pid_set_target(&smart_car.direction_pid, 0);  // 设置方向PID目标为0，表示期望偏差为0
        steer_angle = pid_calculate(&smart_car.direction_pid, (float)deviation);
    }
    
    // ========== 方向PWM输出 ==========
    // 设置方向PWM输出
    motor_set_duty(&car.left_motor, (int32)left_pwm);
    motor_set_duty(&car.right_motor, (int32)right_pwm);
    car_set_angle((int16)steer_angle);
}

/**
 * @brief  启动智能小车
 * @param  无
 * @return 无
 */
void smart_car_start(void)
{
    // 重置速度PID
    pid_reset(&smart_car.speed_pid_left);
    pid_reset(&smart_car.speed_pid_right);
    pid_reset(&smart_car.direction_pid);
    
    // 启动智能小车
    smart_car.state = CAR_RUNNING;
}

/**
 * @brief  停止智能小车
 * @param  无
 * @return 无
 */
void smart_car_stop(void)
{
    // 停止小车
    car_stop();
    
    // 停止智能小车
    smart_car.state = CAR_STOP;
}

/**
 * @brief  暂停智能小车
 * @param  无
 * @return 无
 */
void smart_car_pause(void)
{
    // 停止小车
    car_stop();
    smart_car.state = CAR_PAUSE;
}

/**
 * @brief  启动调试模式
 * @param  无
 * @return 无
 */
void smart_car_debug_mode(void)
{
    smart_car.state = CAR_DEBUG;
    
    // 调试模式示例代码
    // car_forward(2000);              // 前进2秒
    // system_delay_ms(2000);
    // car_turn(2000, 20);             // 右转2秒，角度20度
    // system_delay_ms(2000);
    // car_turn(2000, -20);            // 左转2秒，角度-20度
    // system_delay_ms(2000);
    // car_stop();
}

/**
 * @brief  启用元素识别
 * @param  无
 * @return 无
 */
void smart_car_enable_element_recognition(void)
{
    smart_car.element_recognition_enable = 1;
}

/**
 * @brief  禁用元素识别
 * @param  无
 * @return 无
 */
void smart_car_disable_element_recognition(void)
{
    smart_car.element_recognition_enable = 0;
}

/**
 * @brief  获取当前识别的元素类型
 * @param  无
 * @return 元素类型
 */
element_type_enum smart_car_get_current_element(void)
{
    return element_recog.current_element.type;
}
//====================================================PID参数设置====================================================
/**
 * @brief  设置PID场景
 * @param  scene  PID场景
 * @return 无
 */
void smart_car_set_pid_scene(pid_scene_enum scene)
{
    if (scene >= PID_SCENE_MAX) return;
    
    smart_car.current_pid_scene = scene;
    smart_car_load_pid_config(scene);
}

/**
 * @brief  获取当前PID场景
 * @param  无
 * @return 当前PID场景
 */
pid_scene_enum smart_car_get_pid_scene(void)
{
    return smart_car.current_pid_scene;
}

/**
 * @brief  加载PID配置
 * @param  scene  PID场景
 * @return 无
 */
void smart_car_load_pid_config(pid_scene_enum scene)
{
    if (scene >= PID_SCENE_MAX) return;
    
    pid_scene_config_t *config = &smart_car.pid_configs[scene];
    
    // 设置速度PID参数
    pid_set_params(&smart_car.speed_pid_left, 
                   config->speed.kp, 
                   config->speed.ki, 
                   config->speed.kd);
    
    pid_set_params(&smart_car.speed_pid_right, 
                   config->speed.kp, 
                   config->speed.ki, 
                   config->speed.kd);
    
    // 设置方向PID参数
    pid_set_params(&smart_car.direction_pid, 
                   config->direction.kp, 
                   config->direction.ki, 
                   config->direction.kd);
    
    // 设置基础速度
    car.base_speed = config->base_speed;
}

/**
 * @brief  保存PID配置
 * @param  scene  PID场景
 * @return 无
 */
void smart_car_save_pid_config(pid_scene_enum scene)
{
    if (scene >= PID_SCENE_MAX) return;
    
    pid_scene_config_t *config = &smart_car.pid_configs[scene];
    
    // 获取速度PID参数
    pid_get_params(&smart_car.speed_pid_left, 
                   &config->speed.kp, 
                   &config->speed.ki, 
                   &config->speed.kd);
    
    // 获取方向PID参数
    pid_get_params(&smart_car.direction_pid, 
                   &config->direction.kp, 
                   &config->direction.ki, 
                   &config->direction.kd);
    
    // 获取基础速度
    config->base_speed = car.base_speed;
}

/**
 * @brief  设置速度PID参数
 * @param  kp  比例系数
 * @param  ki  积分系数
 * @param  kd  微分系数
 * @return 无
 */
void smart_car_set_speed_pid(float kp, float ki, float kd)
{
    pid_set_params(&smart_car.speed_pid_left, kp, ki, kd);
    pid_set_params(&smart_car.speed_pid_right, kp, ki, kd);
}

/**
 * @brief  设置方向PID参数
 * @param  kp  比例系数
 * @param  ki  积分系数
 * @param  kd  微分系数
 * @return 无
 */
void smart_car_set_direction_pid(float kp, float ki, float kd)
{
    pid_set_params(&smart_car.direction_pid, kp, ki, kd);
}

/**
 * @brief  获取速度PID参数
 * @param  kp  比例系数
 * @param  ki  积分系数
 * @param  kd  微分系数
 * @return 无
 */
void smart_car_get_speed_pid(float *kp, float *ki, float *kd)
{
    pid_get_params(&smart_car.speed_pid_left, kp, ki, kd);
}

/**
 * @brief  获取方向PID参数
 * @param  kp  比例系数
 * @param  ki  积分系数
 * @param  kd  微分系数
 * @return 无
 */
void smart_car_get_direction_pid(float *kp, float *ki, float *kd)
{
    pid_get_params(&smart_car.direction_pid, kp, ki, kd);
}
