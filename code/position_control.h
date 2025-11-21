/*********************************************************************************************************************
* TC264 Opensourec Library 智能小车驱动
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是智能小车位置控制模块头文件
*
* 文件名称          position_control
* 版本信息          v1.0
* 修改记录
* 日期              作者                备注
* 2025-11-12       AI Assistant        first version
********************************************************************************************************************/

#ifndef _POSITION_CONTROL_H_
#define _POSITION_CONTROL_H_

#include "zf_common_headfile.h"
#include "pid_control.h"

//====================================================位置参数====================================================
#define ENCODER_PULSES_PER_METER    5000        // 编码器脉冲数/米（根据实际标定）
#define WHEEL_DIAMETER              65          // 轮子直径(mm)
#define ENCODER_PPR                 1024        // 编码器每转脉冲数
#define POSITION_TOLERANCE          50          // 位置允许误差（脉冲数）

//====================================================位置控制模式====================================================
typedef enum
{
    POSITION_MODE_DISABLE = 0,      // 位置控制禁用
    POSITION_MODE_ENABLE,           // 位置控制启用
    POSITION_MODE_REACHED           // 已到达目标位置
} position_mode_enum;

//====================================================数据结构====================================================
// 位置控制结构体
typedef struct
{
    int32 current_position;         // 当前位置（编码器累计脉冲数）
    int32 target_position;          // 目标位置（编码器累计脉冲数）
    int32 position_error;           // 位置误差
    
    float distance_traveled;        // 已行驶距离(米)
    float target_distance;          // 目标距离(米)
    
    position_mode_enum mode;        // 位置控制模式
    uint8 position_reached;         // 是否到达目标位置
    
    pid_t position_pid;             // 位置环PID
} position_control_t;

//====================================================全局变量====================================================
extern position_control_t position_ctrl;

//====================================================函数声明====================================================
void position_control_init(void);                           // 位置控制初始化
void position_reset(void);                                  // 重置位置
void position_update(void);                                 // 更新当前位置（在中断中调用）
void position_set_target(float distance_meters);            // 设置目标距离（米）
void position_set_target_pulses(int32 pulses);             // 设置目标位置（脉冲数）
float position_get_current_distance(void);                  // 获取当前距离（米）
int16 position_control_calculate(void);                     // 位置环PID计算
uint8 position_is_reached(void);                            // 判断是否到达目标
void position_enable(void);                                 // 启用位置控制
void position_disable(void);                                // 禁用位置控制

// 特殊位置控制
void position_stop_at(float distance_meters);               // 在指定距离停车
void position_move_distance(float distance_meters, int16 speed);  // 移动指定距离

#endif // _POSITION_CONTROL_H_
