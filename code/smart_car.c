#include "smart_car.h"

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
    position_control_init();        // 位置控制
    element_recognition_init();     // 元素识别
    path_planning_init();           // 路径规划
    
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
    smart_car.display_enable             = 1;
    smart_car.position_control_enable    = 0;
    smart_car.element_recognition_enable = 1;  // 元素识别使能
    smart_car.path_planning_enable       = 0;  // 路径规划使能
    
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
    
    // 位置控制更新
    if (smart_car.position_control_enable)
    {
        position_update();
    }
    
    // 手动控制相关变量初始化
    int16 scene_base_speed = (int16)smart_car.pid_configs[smart_car.current_pid_scene].base_speed;
    int16 target_speed_left = scene_base_speed;
    int16 target_speed_right = scene_base_speed;
    int16 manual_steer_angle = 0;       // 手动控制方向角度
    uint8 use_manual_steer = 0;         // 是否使用手动方向控制
    
    // ========== 位置控制场景 ==========
    /*if (smart_car.position_control_enable && position_ctrl.mode == POSITION_MODE_ENABLE)
    {
        // 位置控制计算输出速度
        int16 position_output = position_control_calculate();
        
        if (position_is_reached())
        {
            // 位置控制目标点已到达，停止小车
            smart_car_stop();
            return;
        }
        
        target_speed_left = position_output;
        target_speed_right = position_output;
    }
    // ========== 元素识别场景 ==========
    else if (smart_car.element_recognition_enable)
    {
        element_recognition_process();
        
        // 元素识别处理
        if (smart_car.path_planning_enable)
        {
            // 路径自动决策
            path_auto_decision();
            
            // 路径执行状态检查
            if (path_planner.state == PATH_STATE_EXECUTING && 
                path_planner.current_node < path_planner.node_count)
            {
                path_node_t* current_node = &path_planner.nodes[path_planner.current_node];
                
                // 路径节点速度设置
                target_speed_left = current_node->target_speed;
                target_speed_right = current_node->target_speed;
                
                // 路径节点角度设置
                if (current_node->target_angle != 0)
                {
                    // 设置小车方向角度
                    car_set_angle(current_node->target_angle);
                }
            }
        }
        else
        {
            // 手动控制场景
            // 手动控制相关变量初始化
            if (element_recog.current_element.detected)
            {
                switch (element_recog.current_element.type)
                {
                    case ELEMENT_PARKING:
                        // 进入停车元素，启用位置控制
                        if (element_recog.current_element.state == ELEMENT_STATE_ENTERING)
                        {
                            position_stop_at(0.5f);  // 停车位置设置为0.5米
                            smart_car.position_control_enable = 1;
                        }
                        break;
                        
                    case ELEMENT_RAMP:
                        // 坡道元素，速度提升30%
                        target_speed_left = (int16)(scene_base_speed * 1.3f);
                        target_speed_right = (int16)(scene_base_speed * 1.3f);
                        break;
                        
                    case ELEMENT_OBSTACLE:
                        // 障碍物元素
                        if (element_recog.current_element.state == ELEMENT_STATE_ENTERING)
                        {
                            // 障碍物避让相关处理
                            if (smart_car.avoid_state == AVOID_IDLE)
                            {
                                // 记录避让起始位置
                                if (smart_car.position_control_enable)
                                {
                                    smart_car.avoid_start_distance = position_get_current_distance();
                                }
                                
                                // 设置避让方向和状态
                                smart_car.avoid_direction = 0;  // 0=左转避让
                                smart_car.avoid_state = AVOID_TURNING_LEFT;
                            }
                        }
                        
                        // 障碍物避让处理
                        if (smart_car.avoid_state == AVOID_TURNING_LEFT)
                        {
                            // 左转避让
                            target_speed_left = OBSTACLE_BYPASS_SPEED;
                            target_speed_right = OBSTACLE_BYPASS_SPEED;
                            manual_steer_angle = -OBSTACLE_AVOID_ANGLE;
                            use_manual_steer = 1;
                            
                            // 位置控制启用时，检查避让距离
                            if (smart_car.position_control_enable)
                            {
                                float traveled = position_get_current_distance() - smart_car.avoid_start_distance;
                                if (traveled > OBSTACLE_AVOID_DISTANCE)
                                {
                                    smart_car.avoid_state = AVOID_BYPASSING;
                                }
                            }
                        }
                        else if (smart_car.avoid_state == AVOID_BYPASSING)
                        {
                            // 障碍物绕行阶段
                            target_speed_left = OBSTACLE_BYPASS_SPEED;
                            target_speed_right = OBSTACLE_BYPASS_SPEED;
                            manual_steer_angle = 0;
                            use_manual_steer = 1;
                            
                            // 位置控制启用时，检查避让距离
                            if (element_recog.current_element.state == ELEMENT_STATE_PASSED)
                            {
                                smart_car.avoid_state = AVOID_RETURNING;
                            }
                        }
                        else if (smart_car.avoid_state == AVOID_RETURNING)
                        {
                            // 障碍物返回阶段
                            target_speed_left = OBSTACLE_BYPASS_SPEED;
                            target_speed_right = OBSTACLE_BYPASS_SPEED;
                            manual_steer_angle = OBSTACLE_AVOID_ANGLE;
                            use_manual_steer = 1;
                            
                            // 位置控制启用时，检查避让距离
                            if (vision.track_found)
                            {
                                smart_car.avoid_state = AVOID_IDLE;
                            }
                        }
                        else
                        {
                            // 默认避让速度降低40%
                            target_speed_left = (int16)(scene_base_speed * 0.6f);
                            target_speed_right = (int16)(scene_base_speed * 0.6f);
                        }
                        break;
                        
                    default:
                        // 默认状态，保持当前速度和方向
                        break;
                }
            }
        }
    }*/
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
        steer_angle = (float)manual_steer_angle;
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
    servo_set_angle(&car.steering_servo, (int16)steer_angle);
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
 * @brief  启用位置控制
 * @param  无
 * @return 无
 */
void smart_car_enable_position_control(void)
{
    smart_car.position_control_enable = 1;
    position_enable();
}

/**
 * @brief  禁用位置控制
 * @param  无
 * @return 无
 */
void smart_car_disable_position_control(void)
{
    smart_car.position_control_enable = 0;
    position_disable();
}

/**
 * @brief  设置目标距离
 * @param  distance  目标距离，单位为米
 * @return 无
 */
void smart_car_set_target_distance(float distance)
{
    position_set_target(distance);
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

/**
 * @brief  启用路径规划
 * @param  无
 * @return 无
 */
void smart_car_enable_path_planning(void)
{
    smart_car.path_planning_enable = 1;
    path_planning_reset();
}

/**
 * @brief  禁用路径规划
 * @param  无
 * @return 无
 */
void smart_car_disable_path_planning(void)
{
    smart_car.path_planning_enable = 0;
}

/**
 * @brief  设置决策模式
 * @param  mode  决策模式
 * @return 无
 */
void smart_car_set_decision_mode(decision_mode_enum mode)
{
    path_planner.mode = mode;
}

/**
 * @brief  打印当前路径规划
 * @param  无
 * @return 无
 */
void smart_car_print_current_path(void)
{
    path_print_plan();
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
