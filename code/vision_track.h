
#ifndef _VISION_TRACK_H_
#define _VISION_TRACK_H_

#include "zf_common_headfile.h"

//====================================================  图像采集模块配置====================================================

#define USE_MT9V03X          // 使用MT9V03X摄像头           

//====================================================图像尺寸定义====================================================
#define IMAGE_WIDTH     MT9V03X_W       // 图像宽度188像素
#define IMAGE_HEIGHT    MT9V03X_H       // 图像高度120像素
// 图像数据缓冲区
extern uint8 image_data[IMAGE_HEIGHT][IMAGE_WIDTH];

//====================================================图像二值化模式====================================================
// 二值化模式枚举
typedef enum
{
    BINARIZE_FIXED = 0,         // 固定阈值
    BINARIZE_OTSU,              // 全局OTSU
    BINARIZE_ADAPTIVE,          // 自适应阈值
    BINARIZE_OTSU_ADAPTIVE      // OTSU+自适应结合
} binarize_mode_enum;

#define BINARIZE_MODE       BINARIZE_OTSU_ADAPTIVE  // 当前使用的二值化模式
#define THRESHOLD_VALUE     220         // 固定阈值的阈值
#define ADAPTIVE_BLOCK_SIZE 16          // 自适应阈值块大小
#define ADAPTIVE_OFFSET     10          // 自适应阈值偏移量
#define SCAN_START_ROW      110         // 扫描起始行
#define SCAN_END_ROW        30          // 扫描结束行
#define SCAN_STEP           1           // 扫描步进

#define TRACK_WIDTH_MIN     40          // 轨迹宽度最小值
#define TRACK_WIDTH_MAX     188         // 轨迹宽度最大值

// 边缘跳变阈值
#define EDGE_JUMP_LIMIT     30          // 边缘跳变阈值，像素级
#define GRADIENT_THRESHOLD  50          // 梯度阈值，用于边缘跳变
#define EDGE_SEARCH_MARGIN  10          // 边缘搜索边界像素数

//====================================================摄像头参数配置====================================================
// 摄像头参数配置（根据实际场景调整）
#define CAMERA_HEIGHT       20.0f       // 摄像头高度 (cm)
#define CAMERA_VIEW_ANGLE   45.0f       // 摄像头俯仰角度 (度，正向下为正)
#define CAMERA_FOV_H        100.0f      // 摄像头水平视场角 (度)
#define CAMERA_FOV_V        75.0f       // 摄像头垂直视场角 (度)

//====================================================轨迹信息结构体====================================================
// 小车轨迹信息结构体
typedef struct
{
    uint8 left_edge[IMAGE_HEIGHT];      // 轨迹左边缘 (像素坐标)
    uint8 right_edge[IMAGE_HEIGHT];     // 轨迹右边缘 (像素坐标)
    uint8 center_line[IMAGE_HEIGHT];    // 轨迹中心线 (像素坐标)
    uint8 track_width[IMAGE_HEIGHT];    // 轨迹宽度 (像素)
    
    // 轨迹实际坐标 (cm) - 以小车中心为原点，前方为X轴正方向，左侧为Y轴正方向
    float left_real_x[IMAGE_HEIGHT];
    float left_real_y[IMAGE_HEIGHT];
    float right_real_x[IMAGE_HEIGHT];
    float right_real_y[IMAGE_HEIGHT];
    float center_real_x[IMAGE_HEIGHT];
    float center_real_y[IMAGE_HEIGHT];
    
    uint8 valid_rows;                   // 有效行数
} track_info_t;

// 视觉跟踪结构体
typedef struct
{
    track_info_t track;                 // 轨迹信息
    int16 error;                        // 偏差值
    int16 last_error;                   // 上一次偏差值
    float deviation;                    // 偏差比例 (-1.0 ~ 1.0)
    uint8 track_found;                  // 是否找到轨迹
    uint8 image_ready;                  // 图像是否准备好
} vision_track_t;

// 视觉图像处理全局变量
extern vision_track_t vision;

//====================================================视觉图像处理====================================================
void vision_init(void);                                     // 视觉初始化
void vision_image_process(void);                            // 视觉处理
void vision_find_track_edge(void);                          // 寻找轨迹边缘
int16 vision_get_deviation(void);                           // 获取偏差值
void vision_show_image(void);                               // 显示图像
// 图像二值化阈值计算
uint8 otsu_threshold(uint8 *image, uint32 size);           // OTSU阈值计算
void image_binarization(uint8 threshold);                   // 图像二值化
void vision_pixel_to_world(uint8 row, uint8 col, float *real_x, float *real_y); // 像素坐标转换为实际坐标

#endif // _VISION_TRACK_H_
