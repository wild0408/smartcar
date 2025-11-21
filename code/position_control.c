/*********************************************************************************************************************
* TC264 Opensourec Library 智能小车驱动
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是智能小车位置控制模块实现文件
*
* 文件名称          position_control
* 版本信息          v1.0
* 修改记录
* 日期              作者                备注
* 2025-11-12       AI Assistant        first version
********************************************************************************************************************/

#include "position_control.h"
#include "motor_control.h"
#include <math.h>

//====================================================全局变量====================================================
position_control_t position_ctrl;

//====================================================外部函数====================================================
/**
 * @brief  位置控制初始化
 * @param  无
 * @return 无
 */
void position_control_init(void)
{
    position_ctrl.current_position = 0;
    position_ctrl.target_position = 0;
    position_ctrl.position_error = 0;
    position_ctrl.distance_traveled = 0.0f;
    position_ctrl.target_distance = 0.0f;
    position_ctrl.mode = POSITION_MODE_DISABLE;
    position_ctrl.position_reached = 0;
    
    // 初始化位置环PID（输出为速度）
    pid_init(&position_ctrl.position_pid,
             20.0f,     // Kp - 位置环比例系数
             0.1f,      // Ki - 位置环积分系数
             10.0f,     // Kd - 位置环微分系数
             3000,      // 最大输出速度
             -3000);    // 最小输出速度
}

/**
 * @brief  重置位置
 * @param  无
 * @return 无
 */
void position_reset(void)
{
    position_ctrl.current_position = 0;
    position_ctrl.distance_traveled = 0.0f;
    position_ctrl.position_reached = 0;
}

/**
 * @brief  更新当前位置
 * @param  无
 * @return 无
 * @note   在定时器中断中调用，使用编码器累计
 */
void position_update(void)
{
    // 累加左右轮编码器平均值
    int32 avg_encoder = (car.left_motor.encoder_count + car.right_motor.encoder_count) / 2;
    position_ctrl.current_position += avg_encoder;
    
    // 计算已行驶距离（米）
    position_ctrl.distance_traveled = (float)position_ctrl.current_position / ENCODER_PULSES_PER_METER;
    
    // 更新位置误差
    position_ctrl.position_error = position_ctrl.target_position - position_ctrl.current_position;
}

/**
 * @brief  设置目标距离（米）
 * @param  distance_meters  目标距离（米）
 * @return 无
 */
void position_set_target(float distance_meters)
{
    position_ctrl.target_distance = distance_meters;
    position_ctrl.target_position = (int32)(distance_meters * ENCODER_PULSES_PER_METER);
    position_ctrl.position_reached = 0;
}

/**
 * @brief  设置目标位置（脉冲数）
 * @param  pulses  目标位置脉冲数
 * @return 无
 */
void position_set_target_pulses(int32 pulses)
{
    position_ctrl.target_position = pulses;
    position_ctrl.target_distance = (float)pulses / ENCODER_PULSES_PER_METER;
    position_ctrl.position_reached = 0;
}

/**
 * @brief  获取当前距离（米）
 * @param  无
 * @return 当前距离
 */
float position_get_current_distance(void)
{
    return position_ctrl.distance_traveled;
}

/**
 * @brief  位置环PID计算
 * @param  无
 * @return 目标速度
 */
int16 position_control_calculate(void)
{
    if (position_ctrl.mode != POSITION_MODE_ENABLE)
    {
        return 0;
    }
    
    // 检查是否到达目标
    if (abs(position_ctrl.position_error) < POSITION_TOLERANCE)
    {
        position_ctrl.position_reached = 1;
        position_ctrl.mode = POSITION_MODE_REACHED;
        return 0;
    }
    
    // 位置环PID计算
    float output = pid_calculate(&position_ctrl.position_pid, 
                                 (float)position_ctrl.current_position);
    
    return (int16)output;
}

/**
 * @brief  判断是否到达目标位置
 * @param  无
 * @return 1-已到达，0-未到达
 */
uint8 position_is_reached(void)
{
    return position_ctrl.position_reached;
}

/**
 * @brief  启用位置控制
 * @param  无
 * @return 无
 */
void position_enable(void)
{
    position_ctrl.mode = POSITION_MODE_ENABLE;
    position_ctrl.position_reached = 0;
    pid_reset(&position_ctrl.position_pid);
}

/**
 * @brief  禁用位置控制
 * @param  无
 * @return 无
 */
void position_disable(void)
{
    position_ctrl.mode = POSITION_MODE_DISABLE;
}

/**
 * @brief  在指定距离停车
 * @param  distance_meters  距离（米）
 * @return 无
 */
void position_stop_at(float distance_meters)
{
    position_reset();
    position_set_target(distance_meters);
    position_enable();
}

/**
 * @brief  移动指定距离
 * @param  distance_meters  距离（米）
 * @param  speed            速度
 * @return 无
 */
void position_move_distance(float distance_meters, int16 speed)
{
    position_reset();
    
    // 根据方向设置目标
    if (distance_meters >= 0)
    {
        position_set_target(distance_meters);
    }
    else
    {
        position_set_target(distance_meters);  // 负距离表示后退
    }
    
    position_enable();
}
