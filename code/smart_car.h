#ifndef _SMART_CAR_H_
#define _SMART_CAR_H_

#include "zf_common_headfile.h"
#include "motor_control.h"
#include "pid_control.h"
#include "vision_track.h"
#include "element_recognition.h"

#define BASE_SPEED              600        // 基础速度/10ms
#define MAX_STEER_ANGLE         45          // 最大转向角度

//====================================================PID参数配置====================================================
// PID场景枚举
typedef enum
{
    PID_SCENE_NORMAL = 0,       // 正常场景
    PID_SCENE_STRAIGHT,         // 直线场景
    PID_SCENE_CURVE,            // 曲线场景
    PID_SCENE_CIRCLE,           // 圆形场景
    PID_SCENE_RAMP,             // 坡道场景
    PID_SCENE_OBSTACLE,         // 障碍物场景
    PID_SCENE_PARKING,          // 停车场景
    PID_SCENE_DEBUG,            // 调试场景
    PID_SCENE_MAX
} pid_scene_enum;

// PID参数结构体
typedef struct
{
    float kp;                   // 比例系数
    float ki;                   // 积分系数
    float kd;                   // 微分系数
} pid_params_t;

// PID参数配置结构体
typedef struct
{
    pid_params_t speed;         // 速度PID参数
    pid_params_t direction;     // 方向PID参数
    int16 base_speed;           // 基础速度/10ms
} pid_scene_config_t;

//========================================================================================================
#define SPEED_PID_KP_NORMAL         3.0f       // 正常场景速度PID比例系数
#define SPEED_PID_KI_NORMAL         0.0f        // 正常场景速度PID积分系数
#define SPEED_PID_KD_NORMAL         0.0f        // 正常场景速度PID微分系数

#define DIRECTION_PID_KP_NORMAL     1.0f        // 正常场景方向PID比例系数
#define DIRECTION_PID_KI_NORMAL     0.0f       // 正常场景方向PID积分系数
#define DIRECTION_PID_KD_NORMAL     0.0f        // 正常场景方向PID微分系数
// 直线场景PID参数配置
#define SPEED_PID_KP_STRAIGHT       60.0f       // 直线场景速度PID比例系数
#define SPEED_PID_KI_STRAIGHT       2.5f        // 直线场景速度PID积分系数
#define SPEED_PID_KD_STRAIGHT       6.0f        // 直线场景速度PID微分系数

#define DIRECTION_PID_KP_STRAIGHT   1.5f        // 直线场景方向PID比例系数
#define DIRECTION_PID_KI_STRAIGHT   0.03f       // 直线场景方向PID积分系数
#define DIRECTION_PID_KD_STRAIGHT   4.0f        // 直线场景方向PID微分系数

// 曲线场景PID参数配置
#define SPEED_PID_KP_CURVE          45.0f       // 曲线场景速度PID比例系数
#define SPEED_PID_KI_CURVE          1.8f        // 曲线场景速度PID积分系数
#define SPEED_PID_KD_CURVE          4.5f        // 曲线场景速度PID微分系数

#define DIRECTION_PID_KP_CURVE      2.5f        // 曲线场景方向PID比例系数
#define DIRECTION_PID_KI_CURVE      0.08f       // 曲线场景方向PID积分系数
#define DIRECTION_PID_KD_CURVE      6.0f        // 曲线场景方向PID微分系数
//====================================================障碍物避让参数====================================================
#define OBSTACLE_AVOID_ANGLE    25          // 障碍物避让转向角度
#define OBSTACLE_AVOID_DISTANCE 0.3f        // 障碍物避让距离
#define OBSTACLE_BYPASS_SPEED   50        // 避让时速度/10ms
// 障碍物避让状态枚举
typedef enum
{
    AVOID_IDLE = 0,             // 空闲状态
    AVOID_TURNING_LEFT,         // 正在左转
    AVOID_TURNING_RIGHT,        // 正在右转
    AVOID_BYPASSING,            // 正在避让
    AVOID_RETURNING             // 正在返回
} obstacle_avoid_state_enum;

//====================================================车辆状态枚举====================================================
typedef enum
{
    CAR_STOP = 0,               // 停止
    CAR_RUNNING,                // 运行
    CAR_PAUSE,                  // 暂停
    CAR_DEBUG                   // 调试模式
} car_state_enum;

//====================================================智能小车主结构体====================================================
// 智能小车主结构体
typedef struct
{
    car_state_enum state;                       // 智能小车状态
    
    pid_t speed_pid_left;                       // 左轮速度PID
    pid_t speed_pid_right;                      // 右轮速度PID
    pid_t direction_pid;                        // 方向PID
    
    pid_scene_enum current_pid_scene;           // 当前PID场景
    pid_scene_config_t pid_configs[PID_SCENE_MAX]; // PID场景配置数组
    
    uint8 display_enable;                       // 显示使能
    uint8 position_control_enable;              // 位置控制使能
    uint8 element_recognition_enable;           // 元素识别使能
    uint8 path_planning_enable;                 // 路径规划使能
    
    obstacle_avoid_state_enum avoid_state;      // 障碍物避让状态
    float avoid_start_distance;                 // 障碍物避让起始距离
    uint8 avoid_direction;                      // 障碍物避让方向0=左转1=右转
} smart_car_t;

//====================================================智能小车主结构体====================================================
extern smart_car_t smart_car;

//====================================================智能小车功能函数====================================================
// 初始化智能小车
void smart_car_init(void);                                      // 初始化智能小车
void smart_car_control(void);                                   // 控制智能小车运动
void smart_car_start(void);                                     // 启动智能小车
void smart_car_stop(void);                                      // 停止智能小车
void smart_car_pause(void);                                     // 暂停智能小车
void smart_car_debug_mode(void);                                // 智能小车调试模式

// 元素识别使能
void smart_car_enable_element_recognition(void);                // 元素识别使能
void smart_car_disable_element_recognition(void);               // 元素识别使能
element_type_enum smart_car_get_current_element(void);          // 获取当前元素类型


// PID设置相关
void smart_car_set_pid_scene(pid_scene_enum scene);             // 设置PID场景
pid_scene_enum smart_car_get_pid_scene(void);                   // 获取当前PID场景
void smart_car_load_pid_config(pid_scene_enum scene);           // 加载PID配置
void smart_car_save_pid_config(pid_scene_enum scene);           // 保存PID配置

// PID设置相关
void smart_car_set_speed_pid(float kp, float ki, float kd);    // 设置速度PID
void smart_car_set_direction_pid(float kp, float ki, float kd);// 设置方向PID
void smart_car_get_speed_pid(float *kp, float *ki, float *kd); // 获取速度PID
void smart_car_get_direction_pid(float *kp, float *ki, float *kd);// 获取方向PID

#endif // _SMART_CAR_H_
