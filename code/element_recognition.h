/*********************************************************************************************************************
* TC264 Opensourec Library 智能小车驱动
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是智能小车元素识别模块头文件
*
* 文件名称          element_recognition
* 版本信息          v1.0
* 修改记录
* 日期              作者                备注
* 2025-11-12       AI Assistant        first version
********************************************************************************************************************/

#ifndef _ELEMENT_RECOGNITION_H_
#define _ELEMENT_RECOGNITION_H_

#include "zf_common_headfile.h"
#include "vision_track.h"

//====================================================元素类型====================================================
typedef enum
{
    ELEMENT_NONE = 0,           // 无元素
    ELEMENT_CROSS,              // 十字路口
    ELEMENT_CIRCLE,             // 圆岛/环岛
    ELEMENT_RAMP,               // 坡道
    ELEMENT_OBSTACLE,           // 障碍物
    ELEMENT_PARKING,            // 停车点
    ELEMENT_ZEBRA_CROSSING,     // 斑马线
    ELEMENT_SPEED_BUMP,         // 减速带
    ELEMENT_FORK                // 岔路口
} element_type_enum;

//====================================================元素状态====================================================
typedef enum
{
    ELEMENT_STATE_NONE = 0,     // 无状态
    ELEMENT_STATE_FOUND,        // 发现元素
    ELEMENT_STATE_ENTERING,     // 进入元素
    ELEMENT_STATE_IN_ELEMENT,   // 在元素中
    ELEMENT_STATE_LEAVING,      // 离开元素
    ELEMENT_STATE_PASSED        // 已通过元素
} element_state_enum;

//====================================================十字路口参数====================================================
#define CROSS_WIDTH_THRESHOLD       120         // 十字宽度阈值
#define CROSS_DETECT_ROWS           3           // 连续检测行数

//====================================================圆岛参数====================================================
#define CIRCLE_CURVATURE_THRESHOLD  50          // 圆岛曲率阈值
#define CIRCLE_CONTINUOUS_ROWS      10          // 连续弯道行数

//====================================================坡道参数====================================================
#define RAMP_BRIGHTNESS_CHANGE      30          // 坡道亮度变化阈值
#define RAMP_EDGE_DISAPPEAR_ROWS    5           // 边界消失行数

//====================================================停车点参数====================================================
#define PARKING_RED_THRESHOLD       150         // 停车点红色阈值（彩色摄像头）
#define PARKING_AREA_MIN            200         // 停车标志最小面积
#define PARKING_WHITE_THRESHOLD     200         // 停车线白色阈值

//====================================================障碍物参数====================================================
#define OBSTACLE_BLACK_AREA_MIN     100         // 障碍物最小黑色区域
#define OBSTACLE_WIDTH_MIN          20          // 障碍物最小宽度

//====================================================数据结构====================================================
// 元素信息结构体
typedef struct
{
    element_type_enum type;         // 元素类型
    element_state_enum state;       // 元素状态
    uint8 detected;                 // 是否检测到
    uint8 confidence;               // 置信度 (0-100)
    uint16 distance;                // 距离元素的距离（像素行数）
    uint32 frame_count;             // 检测到的帧数
} element_info_t;

// 元素识别结构体
typedef struct
{
    element_info_t current_element;     // 当前元素
    element_info_t last_element;        // 上一个元素
    
    // 各元素专用参数
    struct {
        uint8 left_found;               // 发现左边界
        uint8 right_found;              // 发现右边界
        uint8 straight_count;           // 直道计数
    } cross;
    
    struct {
        int16 curvature;                // 曲率
        uint8 direction;                // 方向 (0-左, 1-右)
        uint8 continuous_rows;          // 连续弯道行数
    } circle;
    
    struct {
        uint8 brightness_changed;       // 亮度是否变化
        uint8 edge_lost_count;          // 边界丢失计数
    } ramp;
    
    struct {
        uint8 red_detected;             // 检测到红色
        uint16 red_area;                // 红色区域面积
        uint8 white_line_detected;      // 检测到白线
    } parking;
    
    struct {
        uint8 obstacle_left;            // 左侧障碍物
        uint8 obstacle_right;           // 右侧障碍物
        uint16 obstacle_area;           // 障碍物面积
    } obstacle;
    
} element_recognition_t;

//====================================================全局变量====================================================
extern element_recognition_t element_recog;

//====================================================函数声明====================================================
void element_recognition_init(void);                        // 元素识别初始化
void element_recognition_process(void);                     // 元素识别主处理函数

// 各元素识别函数
uint8 element_detect_cross(void);                           // 十字路口识别
uint8 element_detect_circle(void);                          // 圆岛识别
uint8 element_detect_ramp(void);                            // 坡道识别
uint8 element_detect_parking(void);                         // 停车点识别
uint8 element_detect_obstacle(void);                        // 障碍物识别
uint8 element_detect_zebra_crossing(void);                  // 斑马线识别

// 元素处理函数
void element_handle_cross(void);                            // 处理十字路口
void element_handle_circle(void);                           // 处理圆岛
void element_handle_ramp(void);                             // 处理坡道
void element_handle_parking(void);                          // 处理停车
void element_handle_obstacle(void);                         // 处理障碍物

// 辅助函数
void element_update_state(void);                            // 更新元素状态
uint8 element_is_current(element_type_enum type);           // 判断当前元素类型
void element_reset(void);                                   // 重置元素状态
const char* element_get_name(element_type_enum type);       // 获取元素名称

#endif // _ELEMENT_RECOGNITION_H_
