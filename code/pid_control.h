/*********************************************************************************************************************
* TC264 Opensourec Library 智能小车驱动
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是智能小车PID控制器头文件
*
* 文件名称          pid_control
* 版本信息          v1.0
* 修改记录
* 日期              作者                备注
* 2025-11-12       AI Assistant        first version
********************************************************************************************************************/

#ifndef _PID_CONTROL_H_
#define _PID_CONTROL_H_

#include "zf_common_headfile.h"

//====================================================数据结构====================================================
// PID控制器结构体
typedef struct
{
    float kp;                   // 比例系数
    float ki;                   // 积分系数
    float kd;                   // 微分系数
    
    float target;               // 目标值
    float current;              // 当前值
    
    float error;                // 当前误差
    float last_error;           // 上次误差
    float integral;             // 积分累积
    
    float output;               // 输出值
    float output_max;           // 输出最大值
    float output_min;           // 输出最小值
    
    float integral_max;         // 积分限幅
} pid_t;

//====================================================函数声明====================================================
// 基础函数
void  pid_init(pid_t *pid, float kp, float ki, float kd, float output_max, float output_min);
void  pid_set_target(pid_t *pid, float target);
float pid_calculate(pid_t *pid, float current);
void  pid_reset(pid_t *pid);

// PID参数动态调整
void  pid_set_params(pid_t *pid, float kp, float ki, float kd);        // 设置PID参数
void  pid_set_kp(pid_t *pid, float kp);                                // 单独设置Kp
void  pid_set_ki(pid_t *pid, float ki);                                // 单独设置Ki
void  pid_set_kd(pid_t *pid, float kd);                                // 单独设置Kd
void  pid_set_output_limit(pid_t *pid, float max, float min);          // 设置输出限幅
void  pid_set_integral_limit(pid_t *pid, float limit);                 // 设置积分限幅

// PID参数获取
void  pid_get_params(pid_t *pid, float *kp, float *ki, float *kd);    // 获取PID参数
float pid_get_error(pid_t *pid);                                       // 获取当前误差
float pid_get_output(pid_t *pid);                                      // 获取当前输出

#endif // _PID_CONTROL_H_
