
#ifndef _VISION_TRACK_H_
#define _VISION_TRACK_H_

#include "zf_common_headfile.h"

//====================================================图像采集及驱动相关====================================================

#define USE_MT9V03X                     // 使用MT9V03X摄像头驱动

//====================================================图像尺寸====================================================
#define IMAGE_WIDTH     MT9V03X_W       // 图像宽度188像素
#define IMAGE_HEIGHT    MT9V03X_H       // 图像高度120像素
// 图像数据缓冲区
extern uint8 image_data[IMAGE_HEIGHT][IMAGE_WIDTH];

//====================================================二值化阈值====================================================
// 二值化模式
typedef enum
{
    BINARIZE_FIXED = 0,         // 固定阈值
    BINARIZE_OTSU,              // 全局OTSU
    BINARIZE_ADAPTIVE,          // 自适应局部阈值
    BINARIZE_OTSU_ADAPTIVE      // OTSU+自适应混合
} binarize_mode_enum;

#define BINARIZE_MODE       BINARIZE_OTSU_ADAPTIVE  // 当前使用的二值化模式
#define THRESHOLD_VALUE     220         // 固定阈值模式的阈值
#define ADAPTIVE_BLOCK_SIZE 16          // 自适应阈值块大小
#define ADAPTIVE_OFFSET     10          // 自适应阈值偏移量
#define SCAN_START_ROW      110         // 扫描起始行
#define SCAN_END_ROW        30          // 扫描结束行
#define SCAN_STEP           1           // 扫描步长

#define TRACK_WIDTH_MIN     40          // 轨道宽度最小值
#define TRACK_WIDTH_MAX     188         // 轨道宽度最大值

// 边缘检测优化参数
#define EDGE_JUMP_LIMIT     30          // 边缘跳变限制（像素）
#define GRADIENT_THRESHOLD  50          // 梯度阈值（用于边缘检测）
#define EDGE_SEARCH_MARGIN  10          // 边缘搜索边界距离

//====================================================轨道信息结构体====================================================
// 轨道信息结构体
typedef struct
{
    uint8 left_edge[IMAGE_HEIGHT];      // 轨道左边缘
    uint8 right_edge[IMAGE_HEIGHT];     // 轨道右边缘
    uint8 center_line[IMAGE_HEIGHT];    // 轨道中心线
    uint8 track_width[IMAGE_HEIGHT];    // 轨道宽度
    uint8 valid_rows;                   // 有效行数
} track_info_t;

// 瀵邦亣鎶楅幒褍鍩楃紒鎾寸�?娴ｏ拷
typedef struct
{
    track_info_t track;                 // 轨道信息
    int16 error;                        // 偏差值
    int16 last_error;                   // 上次偏差值
    float deviation;                    // 偏差比例 (-1.0 ~ 1.0)
    uint8 track_found;                  // 是否找到轨道
    uint8 image_ready;                  // 图像是否准备好
} vision_track_t;

// 图像处理全局变量
extern vision_track_t vision;

//====================================================图像处理====================================================
void vision_init(void);                                     // 图像初始化
void vision_image_process(void);                            // 图像处理
void vision_find_track_edge(void);                          // 寻找轨道边缘
int16 vision_get_deviation(void);                           // 获取偏差值
void vision_show_image(void);                               // 显示图像
// 图像二值化及OTSU阈值计算
uint8 otsu_threshold(uint8 *image, uint32 size);           // OTSU阈值计算
void image_binarization(uint8 threshold);                   // 图像二值化

#endif // _VISION_TRACK_H_
