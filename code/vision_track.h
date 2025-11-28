
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
#define THRESHOLD_VALUE     230         // 固定阈值的阈值
#define ADAPTIVE_BLOCK_SIZE 16          // 自适应阈值块大小
#define ADAPTIVE_OFFSET     10          // 自适应阈值偏移量
#define SCAN_START_ROW      110         // 扫描起始行
#define SCAN_END_ROW        50          // 扫描结束行
#define SCAN_STEP           1           // 扫描步进

#define TRACK_WIDTH_MIN     20          // 轨迹宽度最小值
#define TRACK_WIDTH_MAX     188         // 轨迹宽度最大值

// 边缘跳变阈值
#define EDGE_JUMP_LIMIT     30          // 边缘跳变阈值，像素级
#define GRADIENT_THRESHOLD  50          // 梯度阈值，用于边缘跳变
#define EDGE_SEARCH_MARGIN  1           // 边缘搜索边界像素数
//====================================================轨迹信息结构体====================================================
// 小车轨迹信息结构体
typedef struct
{
    uint8 left_edge[IMAGE_HEIGHT];      // 轨迹左边缘 (像素坐标)
    uint8 right_edge[IMAGE_HEIGHT];     // 轨迹右边缘 (像素坐标)
    uint8 center_line[IMAGE_HEIGHT];    // 轨迹中心线 (像素坐标)
    uint8 track_width[IMAGE_HEIGHT];    // 轨迹宽度 (像素)
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

//====================================================中线拟合配置====================================================
#define FIT_ENABLE              1           // 是否启用中线拟合 (1-启用, 0-禁用)
#define FIT_START_ROW           110         // 拟合起始行
#define FIT_END_ROW             30          // 拟合结束行
#define FIT_MIN_POINTS          10          // 拟合最少点数
#define EDGE_SMOOTH_ENABLE      1           // 边缘平滑使能
#define LOST_LINE_REPAIR_ENABLE    1           // 丢线补线使能
#define SLOPE_LIMIT             0.5f        // 斜率限制（防止边线突变）

//====================================================双最长白列算法相关定义====================================================
// 图像二值化颜色定义
#define IMG_WHITE 255
#define IMG_BLACK 0

// 双最长白列算法全局变量
extern int Longest_White_Column_Left[2];    // [0]长度，[1]列号
extern int Longest_White_Column_Right[2];   // [0]长度，[1]列号
extern int White_Column[IMAGE_WIDTH];       // 每列白点数量
extern uint8 Left_Lost_Flag[IMAGE_HEIGHT];  // 左丢线标志
extern uint8 Right_Lost_Flag[IMAGE_HEIGHT]; // 右丢线标志
extern int Left_Lost_Time;                  // 左丢线次数
extern int Right_Lost_Time;                 // 右丢线次数
extern int Both_Lost_Time;                  // 双边丢线次数
extern int Boundry_Start_Left;              // 左边界起始行
extern int Boundry_Start_Right;             // 右边界起始行
extern int Search_Stop_Line;                // 搜索截止行
// 环岛相关标志（如果项目中使用）
extern uint8 Right_Island_Flag;             // 右环岛标志
extern uint8 Left_Island_Flag;              // 左环岛标志
extern uint8 Island_State;                  // 环岛状态

// 二值化图像数组（用于双最长白列算法）
extern uint8 image_data[IMAGE_HEIGHT][IMAGE_WIDTH];

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

// 中线拟合和优化算法
void edge_smooth_filter(void);                              // 边缘平滑滤波
void centerline_least_square_fit(void);                     // 最小二乘法中线拟合
void lost_line_repair(void);                                // 丢线补线处理

#endif // _VISION_TRACK_H_
