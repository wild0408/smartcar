/*********************************************************************************************************************
* TC264 Opensourec Library 智能小车驱动
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是TFT180显示功能扩展（仅包含智能小车特定的显示逻辑）
*
* 文件名称          display_tft180
* 版本信息          v1.0
* 修改记录
* 日期              作者                备注
* 2025-11-12       AI Assistant        first version
********************************************************************************************************************/

#ifndef _DISPLAY_TFT180_H_
#define _DISPLAY_TFT180_H_

#include "zf_common_headfile.h"
#include "zf_device_tft180.h"
#include "smart_car.h"
#include "vision_track.h"
#include "position_control.h"
#include "element_recognition.h"
#include "path_planning.h"

//====================================================函数声明====================================================
// 注意：基础显示功能请直接使用 zf_device_tft180.h 中的函数
// 本文件仅提供智能小车特定的组合显示功能

void smart_car_display_info_tft180(void);       // 在TFT180上显示小车综合信息
void vision_show_image_tft180(void);            // 在TFT180上显示视觉图像
void vision_show_image_with_lines_tft180(void); // 在TFT180上显示视觉图像（叠加边线和中线）
void display_path_info_tft180(void);            // 在TFT180上显示路径信息
void display_element_info_tft180(void);         // 在TFT180上显示元素信息

#endif // _DISPLAY_TFT180_H_
