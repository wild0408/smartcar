/*********************************************************************************************************************
* TC264 Opensourec Library 智能小车驱动
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是智能小车路径规划模块头文件
*
* 文件名称          path_planning
* 版本信息          v1.0
* 修改记录
* 日期              作者                备注
* 2025-11-12       AI Assistant        first version
********************************************************************************************************************/

#ifndef _PATH_PLANNING_H_
#define _PATH_PLANNING_H_

#include "zf_common_headfile.h"
#include "element_recognition.h"
#include "position_control.h"

//====================================================路径类型====================================================
typedef enum
{
    PATH_STRAIGHT = 0,      // 直行
    PATH_LEFT_TURN,         // 左转
    PATH_RIGHT_TURN,        // 右转
    PATH_LEFT_CIRCLE,       // 左圆岛
    PATH_RIGHT_CIRCLE,      // 右圆岛
    PATH_CROSS_STRAIGHT,    // 十字直行
    PATH_CROSS_LEFT,        // 十字左转
    PATH_CROSS_RIGHT,       // 十字右转
    PATH_AVOID_LEFT,        // 左避障
    PATH_AVOID_RIGHT,       // 右避障
    PATH_RAMP,              // 坡道
    PATH_PARKING,           // 停车
    PATH_MAX
} path_type_enum;

//====================================================路径状态====================================================
typedef enum
{
    PATH_STATE_IDLE = 0,    // 空闲
    PATH_STATE_PLANNING,    // 规划中
    PATH_STATE_EXECUTING,   // 执行中
    PATH_STATE_COMPLETED,   // 完成
    PATH_STATE_FAILED       // 失败
} path_state_enum;

//====================================================决策模式====================================================
typedef enum
{
    DECISION_MODE_MANUAL = 0,   // 手动决策
    DECISION_MODE_AUTO,         // 自动决策
    DECISION_MODE_OPTIMAL       // 最优路径决策
} decision_mode_enum;

//====================================================路径节点====================================================
typedef struct
{
    path_type_enum type;        // 路径类型
    element_type_enum element;  // 对应元素
    float distance;             // 执行距离（米）
    uint16 duration;            // 执行时间（ms）
    int16 target_speed;         // 目标速度
    int16 target_angle;         // 目标角度
    uint8 priority;             // 优先级（0-255，越小越优先）
} path_node_t;

//====================================================路径规划器====================================================
typedef struct
{
    path_node_t nodes[20];      // 路径节点队列
    uint8 node_count;           // 节点数量
    uint8 current_node;         // 当前节点索引
    
    path_state_enum state;      // 规划状态
    decision_mode_enum mode;    // 决策模式
    
    uint32 start_time;          // 开始时间
    float start_position;       // 开始位置
    
    // 决策权重
    float distance_weight;      // 距离权重
    float time_weight;          // 时间权重
    float risk_weight;          // 风险权重
    
    // 统计信息
    uint8 circle_count;         // 圆岛计数
    uint8 obstacle_count;       // 障碍物计数
    uint8 cross_count;          // 十字计数
    
    // 路径选择偏好
    uint8 prefer_left;          // 偏好左转（0-100）
    uint8 prefer_right;         // 偏好右转（0-100）
    
} path_planner_t;

//====================================================路径成本计算====================================================
typedef struct
{
    float distance_cost;        // 距离成本
    float time_cost;            // 时间成本
    float risk_cost;            // 风险成本
    float total_cost;           // 总成本
} path_cost_t;

//====================================================全局变量====================================================
extern path_planner_t path_planner;

//====================================================函数声明====================================================
void path_planning_init(void);                              // 路径规划初始化
void path_planning_reset(void);                             // 重置规划器

// 路径规划
void path_plan_for_element(element_type_enum element);      // 为元素规划路径
void path_add_node(path_node_t node);                       // 添加路径节点
void path_clear_nodes(void);                                // 清空路径节点

// 路径执行
void path_execute_current_node(void);                       // 执行当前节点
uint8 path_is_node_completed(void);                         // 判断节点是否完成
void path_next_node(void);                                  // 切换到下一节点

// 决策算法
path_type_enum path_decide_circle(uint8 direction);         // 圆岛决策
path_type_enum path_decide_cross(void);                     // 十字决策
path_type_enum path_decide_obstacle(void);                  // 障碍物决策

// 成本计算
path_cost_t path_calculate_cost(path_node_t* node);         // 计算路径成本
path_type_enum path_select_optimal(path_node_t* options, uint8 count);  // 选择最优路径

// 智能决策
void path_auto_decision(void);                              // 自动决策
void path_update_preference(void);                          // 更新偏好

// 路径预设
void path_preset_left_circle(void);                         // 预设左圆岛路径
void path_preset_right_circle(void);                        // 预设右圆岛路径
void path_preset_cross_straight(void);                      // 预设十字直行
void path_preset_cross_left(void);                          // 预设十字左转
void path_preset_cross_right(void);                         // 预设十字右转
void path_preset_avoid_obstacle(uint8 side);                // 预设避障路径

// 辅助函数
const char* path_get_type_name(path_type_enum type);        // 获取路径类型名称
void path_print_plan(void);                                 // 打印路径规划
uint8 path_get_progress(void);                              // 获取执行进度（0-100）

#endif // _PATH_PLANNING_H_
