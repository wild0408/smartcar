/*********************************************************************************************************************
* TC264 Opensourec Library 智能小车驱动
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是智能小车PID控制器实现文件
*
* 文件名称          pid_control
* 版本信息          v1.0
* 修改记录
* 日期              作者                备注
* 2025-11-12       AI Assistant        first version
********************************************************************************************************************/

#include "pid_control.h"

/**
 * @brief  PID控制器初始化
 * @param  pid         PID结构体指针
 * @param  kp          比例系数
 * @param  ki          积分系数
 * @param  kd          微分系数
 * @param  output_max  输出最大值
 * @param  output_min  输出最小值
 * @return 无
 */
void pid_init(pid_t *pid, float kp, float ki, float kd, float output_max, float output_min)
{
    // PID参数
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    
    // 目标值和当前值
    pid->target  = 0;
    pid->current = 0;
    
    // 误差值
    pid->error      = 0;
    pid->last_error = 0;
    pid->integral   = 0;
    
    // 输出参数
    pid->output     = 0;
    pid->output_max = output_max;
    pid->output_min = output_min;
    
    // 积分限幅设为输出最大值的80%
    pid->integral_max = output_max * 0.8f;
}

/**
 * @brief  设置PID目标值
 * @param  pid     PID结构体指针
 * @param  target  目标值
 * @return 无
 */
void pid_set_target(pid_t *pid, float target)
{
    pid->target = target;
}

/**
 * @brief  PID计算
 * @param  pid      PID结构体指针
 * @param  current  当前值
 * @return PID输出值
 */
float pid_calculate(pid_t *pid, float current)
{
    pid->current = current;
    
    // 计算误差
    pid->error = pid->target - pid->current;
    
    // 比例项
    float p_out = pid->kp * pid->error;
    
    // 积分项（带限幅）
    pid->integral += pid->error;
    if (pid->integral > pid->integral_max)
        pid->integral = pid->integral_max;
    else if (pid->integral < -pid->integral_max)
        pid->integral = -pid->integral_max;
    float i_out = pid->ki * pid->integral;
    
    // 微分项
    float d_out = pid->kd * (pid->error - pid->last_error);
    
    // 计算总输出
    pid->output = p_out + i_out + d_out;
    
    // 输出限幅
    if (pid->output > pid->output_max)
        pid->output = pid->output_max;
    else if (pid->output < pid->output_min)
        pid->output = pid->output_min;
    
    // 保存当前误差
    pid->last_error = pid->error;
    
    return pid->output;
}

/**
 * @brief  重置PID控制器
 * @param  pid  PID结构体指针
 * @return 无
 */
void pid_reset(pid_t *pid)
{
    pid->error = 0;
    pid->last_error = 0;
    pid->integral = 0;
    pid->output = 0;
}

//====================================================PID参数动态调整====================================================
/**
 * @brief  设置PID参数
 * @param  pid  PID结构体指针
 * @param  kp   比例系数
 * @param  ki   积分系数
 * @param  kd   微分系数
 * @return 无
 */
void pid_set_params(pid_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/**
 * @brief  单独设置Kp参数
 * @param  pid  PID结构体指针
 * @param  kp   比例系数
 * @return 无
 */
void pid_set_kp(pid_t *pid, float kp)
{
    pid->kp = kp;
}

/**
 * @brief  单独设置Ki参数
 * @param  pid  PID结构体指针
 * @param  ki   积分系数
 * @return 无
 */
void pid_set_ki(pid_t *pid, float ki)
{
    pid->ki = ki;
}

/**
 * @brief  单独设置Kd参数
 * @param  pid  PID结构体指针
 * @param  kd   微分系数
 * @return 无
 */
void pid_set_kd(pid_t *pid, float kd)
{
    pid->kd = kd;
}

/**
 * @brief  设置输出限幅
 * @param  pid  PID结构体指针
 * @param  max  输出最大值
 * @param  min  输出最小值
 * @return 无
 */
void pid_set_output_limit(pid_t *pid, float max, float min)
{
    pid->output_max = max;
    pid->output_min = min;
}

/**
 * @brief  设置积分限幅
 * @param  pid    PID结构体指针
 * @param  limit  积分限幅值
 * @return 无
 */
void pid_set_integral_limit(pid_t *pid, float limit)
{
    pid->integral_max = limit;
}

//====================================================PID参数获取====================================================
/**
 * @brief  获取PID参数
 * @param  pid  PID结构体指针
 * @param  kp   比例系数指针
 * @param  ki   积分系数指针
 * @param  kd   微分系数指针
 * @return 无
 */
void pid_get_params(pid_t *pid, float *kp, float *ki, float *kd)
{
    *kp = pid->kp;
    *ki = pid->ki;
    *kd = pid->kd;
}

/**
 * @brief  获取当前误差
 * @param  pid  PID结构体指针
 * @return 当前误差值
 */
float pid_get_error(pid_t *pid)
{
    return pid->error;
}

/**
 * @brief  获取当前输出
 * @param  pid  PID结构体指针
 * @return 当前输出值
 */
float pid_get_output(pid_t *pid)
{
    return pid->output;
}
