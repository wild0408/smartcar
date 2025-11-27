#include "vision_track.h"
#include <math.h>

#ifndef PI
#define PI 3.1415926535f
#endif
// 双最长白列算法全局变量
int Longest_White_Column_Left[2];    // [0]长度，[1]列号
int Longest_White_Column_Right[2];   // [0]长度，[1]列号
int White_Column[IMAGE_WIDTH];       // 每列白点数量
uint8 Left_Lost_Flag[IMAGE_HEIGHT];  // 左丢线标志
uint8 Right_Lost_Flag[IMAGE_HEIGHT]; // 右丢线标志
int Left_Lost_Time;                  // 左丢线次数
int Right_Lost_Time;                 // 右丢线次数
int Both_Lost_Time;                  // 双边丢线次数
int Boundry_Start_Left;              // 左边界起始行
int Boundry_Start_Right;             // 右边界起始行
int Search_Stop_Line;                // 搜索截止行

// 环岛相关标志（如果项目中使用）
uint8 Right_Island_Flag;             // 右环岛标志
uint8 Left_Island_Flag;              // 左环岛标志
uint8 Island_State;                  // 环岛状态
vision_track_t vision;

uint8 image_data[IMAGE_HEIGHT][IMAGE_WIDTH];

//====================================================图像处理====================================================
/**
 * @brief  OTSU阈值计算（优化版，使用整数运算）
 * @param  image  图像数据指针
 * @param  size   图像数据大小
 * @return 阈值
 */

 int my_adapt_threshold(uint8 *image, uint16 col, uint16 row)   //注意计算阈值的一定要是原图像
{
    #define GrayScale 256
    uint16 width = col;
    uint16 height = row;
    int pixelCount[GrayScale];
    float pixelPro[GrayScale];
    int i, j;
    int pixelSum = width * height/4;
    int threshold = 0;
    uint8* data = image;  //指向像素数据的指针
    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }
    uint32 gray_sum=0;
    //统计灰度级中每个像素在整幅图像中的个数
    for (i = 0; i < height; i+=2)
    {
        for (j = 0; j < width; j+=2)
        {
            pixelCount[(int)data[i * width + j]]++;  //将当前的点的像素值作为计数数组的下标
            gray_sum+=(int)data[i * width + j];       //灰度值总和
        }
    }
    //计算每个像素值的点在整幅图像中的比例
    for (i = 0; i < GrayScale; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
    }
    //遍历灰度级[0,255]
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
    w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
    for (j = 0; j < GrayScale; j++)
    {
        w0 += pixelPro[j];  //背景部分每个灰度值的像素点所占比例之和   即背景部分的比例
        u0tmp += j * pixelPro[j];  //背景部分 每个灰度值的点的比例 *灰度值
        w1=1-w0;
        u1tmp=gray_sum/pixelSum-u0tmp;
        u0 = u0tmp / w0;              //背景平均灰度
        u1 = u1tmp / w1;              //前景平均灰度
        u = u0tmp + u1tmp;            //全局平均灰度
        deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;
            threshold = j;
        }
        if (deltaTmp < deltaMax)
        {
            break;
        }
    }
    return threshold;
}

uint8 otsu_threshold(uint8 *image, uint32 size)
{
    uint32 histogram[256] = {0};
    uint32 i;
    
    // 计算直方图
    for (i = 0; i < size; i++)
    {
        histogram[image[i]]++;
    }
    
    // OTSU阈值计算（优化为整数运算，避免浮点）
    uint32 sum = 0;
    for (i = 0; i < 256; i++)
    {
        sum += i * histogram[i];
    }
    
    uint32 w0 = 0, w1;
    uint32 sum0 = 0;
    uint32 max_variance = 0;  // 使用整数存储方差×(w0*w1)
    uint8 threshold = 0;
    
    for (i = 0; i < 256; i++)
    {
        w0 += histogram[i];
        if (w0 == 0) continue;
        
        w1 = size - w0;
        if (w1 == 0) break;
        
        sum0 += i * histogram[i];
        
        // 使用整数运算：variance = w0 * w1 * (mean0 - mean1)^2
        // mean0 = sum0/w0, mean1 = (sum-sum0)/w1
        // (mean0-mean1) = (sum0*w1 - (sum-sum0)*w0) / (w0*w1)
        // variance*w0*w1 = (sum0*w1 - (sum-sum0)*w0)^2 / (w0*w1)
        int32 diff = (int32)(sum0 * w1) - (int32)((sum - sum0) * w0);
        uint32 variance = (uint32)((int64)diff * diff / (w0 * w1));
        
        if (variance > max_variance)
        {
            max_variance = variance;
            threshold = (uint8)i;
        }
    }
    
    return threshold;
}

/**
 * @brief  形态学腐蚀操作（去除小噪点）
 * @param  无
 * @return 无
 */
static void morphology_erode(void)
{
    uint8 row, col;
    static uint8 temp_image[IMAGE_HEIGHT][IMAGE_WIDTH];
    
    // 复制到临时缓冲区
    for (row = 0; row < IMAGE_HEIGHT; row++)
    {
        for (col = 0; col < IMAGE_WIDTH; col++)
        {
            temp_image[row][col] = image_data[row][col];
        }
    }
    
    // 3x3腐蚀核
    for (row = 1; row < IMAGE_HEIGHT - 1; row++)
    {
        for (col = 1; col < IMAGE_WIDTH - 1; col++)
        {
            // 如果3x3邻域内有黑点，则该点变黑
            if (temp_image[row-1][col] == 0 || temp_image[row+1][col] == 0 ||
                temp_image[row][col-1] == 0 || temp_image[row][col+1] == 0)
            {
                image_data[row][col] = 0;
            }
        }
    }
}

/**
 * @brief  形态学膨胀操作（填充小孔洞）
 * @param  无
 * @return 无
 */
static void morphology_dilate(void)
{
    uint8 row, col;
    static uint8 temp_image[IMAGE_HEIGHT][IMAGE_WIDTH];
    
    // 复制到临时缓冲区
    for (row = 0; row < IMAGE_HEIGHT; row++)
    {
        for (col = 0; col < IMAGE_WIDTH; col++)
        {
            temp_image[row][col] = image_data[row][col];
        }
    }
    
    // 3x3膨胀核
    for (row = 1; row < IMAGE_HEIGHT - 1; row++)
    {
        for (col = 1; col < IMAGE_WIDTH - 1; col++)
        {
            // 如果3x3邻域内有白点，则该点变白
            if (temp_image[row-1][col] == 255 || temp_image[row+1][col] == 255 ||
                temp_image[row][col-1] == 255 || temp_image[row][col+1] == 255)
            {
                image_data[row][col] = 255;
            }
        }
    }
}

/**
 * @brief  图像二值化处理
 * @param  threshold  阈值
 * @return 无
 */
void image_binarization(uint8 threshold)
{
    uint8 row, col;
    
    // 固定阈值二值化
    for (row = 0; row < IMAGE_HEIGHT; row++)
    {
        for (col = 0; col < IMAGE_WIDTH; col++)
        {
            image_data[row][col] = (mt9v03x_image[row][col] > threshold) ? 255 : 0;
        }
    }
    // 形态学滤波去噪（可选，根据效果开启）
    morphology_erode();   // 腐蚀：去除小噪点
    morphology_dilate();  // 膨胀：填充小孔洞
}

//====================================================图像处理====================================================
/**
 * @brief  视觉跟踪初始化
 * @param  无
 * @return 无
 */
void vision_init(void)
{
    // 视觉跟踪结构体初始化
    vision.error = 0;
    vision.last_error = 0;
    vision.deviation = 0;
    vision.track_found = 0;
    vision.image_ready = 0;
    vision.track.valid_rows = 0;
    
    // 初始化MT9V03X摄像头
    mt9v03x_init();
}

/**
 * @brief  双最长白列巡线算法
 * @param  无
 * @return 无
 * @note   寻找最长白列,识别边界,后续进行中线拟合和优化
 */
void vision_find_track_edge(void)
{
    int i, j;
    int start_column = 20;  // 最长白列的搜索区间
    int end_column = MT9V03X_W - 20;
    uint8 left_border = 0, right_border = 0;
    
    // 初始化
    Longest_White_Column_Left[0] = 0;
    Longest_White_Column_Left[1] = 0;
    Longest_White_Column_Right[0] = 0;
    Longest_White_Column_Right[1] = 0;
    Right_Lost_Time = 0;
    Left_Lost_Time = 0;
    Boundry_Start_Left = 0;
    Boundry_Start_Right = 0;
    Both_Lost_Time = 0;

    // 数据清零
    for (i = 0; i <= MT9V03X_H - 1; i++)
    {
        Right_Lost_Flag[i] = 0;
        Left_Lost_Flag[i] = 0;
        vision.track.left_edge[i] = 0;
        vision.track.right_edge[i] = MT9V03X_W - 1;
    }
    for (i = 0; i <= MT9V03X_W - 1; i++)
    {
        White_Column[i] = 0;
    }

    // 环岛范围限定
    if (Right_Island_Flag == 1)
    {
        if (Island_State == 3)
        {
            start_column = 40;
            end_column = MT9V03X_W - 20;
        }
    }
    else if (Left_Island_Flag == 1)
    {
        if (Island_State == 3)
        {
            start_column = 20;
            end_column = MT9V03X_W - 40;
        }
    }

    // 统计每列白点数量
    for (j = start_column; j <= end_column; j++)
    {
        for (i = MT9V03X_H - 1; i >= 0; i--)
        {
            if (image_data[i][j] == IMG_BLACK)
                break;
            else
                White_Column[j]++;
        }
    }

    // 从左到右找左边最长白列
    Longest_White_Column_Left[0] = 0;
    for (i = start_column; i <= end_column; i++)
    {
        if (Longest_White_Column_Left[0] < White_Column[i])
        {
            Longest_White_Column_Left[0] = White_Column[i];
            Longest_White_Column_Left[1] = i;
        }
    }

    // 从右到左找右边最长白列
    Longest_White_Column_Right[0] = 0;
    for (i = end_column; i >= start_column; i--)
    {
        if (Longest_White_Column_Right[0] < White_Column[i])
        {
            Longest_White_Column_Right[0] = White_Column[i];
            Longest_White_Column_Right[1] = i;
        }
    }

    Search_Stop_Line = Longest_White_Column_Left[0];

    // 常规巡线
    for (i = MT9V03X_H - 1; i >= MT9V03X_H - Search_Stop_Line; i--)
    {
        // 从右边最长白列向右扫描找右边界
        for (j = Longest_White_Column_Right[1]; j <= MT9V03X_W - 1 - 2; j++)
        {
            if (image_data[i][j] == IMG_WHITE &&
                image_data[i][j + 1] == IMG_BLACK &&
                image_data[i][j + 2] == IMG_BLACK)
            {
                right_border = j;
                Right_Lost_Flag[i] = 0;
                break;
            }
            else if (j >= MT9V03X_W - 1 - 2)
            {
                right_border = j;
                Right_Lost_Flag[i] = 1;
                break;
            }
        }

        // 从左边最长白列向左扫描找左边界
        for (j = Longest_White_Column_Left[1]; j >= 0 + 2; j--)
        {
            if (image_data[i][j] == IMG_WHITE &&
                image_data[i][j - 1] == IMG_BLACK &&
                image_data[i][j - 2] == IMG_BLACK)
            {
                left_border = j;
                Left_Lost_Flag[i] = 0;
                break;
            }
            else if (j <= 0 + 2)
            {
                left_border = j;
                Left_Lost_Flag[i] = 1;
                break;
            }
        }

        vision.track.left_edge[i] = left_border;
        vision.track.right_edge[i] = right_border;
    }

    // 赛道数据分析
    for (i = MT9V03X_H - 1; i >= 0; i--)
    {
        if (Left_Lost_Flag[i] == 1)
            Left_Lost_Time++;
        if (Right_Lost_Flag[i] == 1)
            Right_Lost_Time++;
        if (Left_Lost_Flag[i] == 1 && Right_Lost_Flag[i] == 1)
            Both_Lost_Time++;
        if (Boundry_Start_Left == 0 && Left_Lost_Flag[i] != 1)
            Boundry_Start_Left = i;
        if (Boundry_Start_Right == 0 && Right_Lost_Flag[i] != 1)
            Boundry_Start_Right = i;
        vision.track.track_width[i] = vision.track.right_edge[i] - vision.track.left_edge[i];
    }

    // === 优化处理流程 ===
    #if EDGE_SMOOTH_ENABLE
    edge_smooth_filter();  // 边缘平滑
    #endif

    #if LOST_LINE_REPAIR_ENABLE
    lost_line_repair();    // 丢线补线
    #endif

    #if FIT_ENABLE
    centerline_least_square_fit();  // 中线拟合
    #endif

    // 同步数据到vision结构体
    vision.track.valid_rows = 0;
    for (i = 0; i < MT9V03X_H; i++)
    {

        if (vision.track.track_width[i] >= TRACK_WIDTH_MIN &&
            vision.track.track_width[i] <= TRACK_WIDTH_MAX)
        {
            vision.track.center_line[i] = (vision.track.left_edge[i] + vision.track.right_edge[i]) / 2;
            vision.track.valid_rows++;
        }
        else
        {
            vision.track.center_line[i] = IMAGE_WIDTH / 2;
        }
    }

    vision.track_found = (vision.track.valid_rows >= 10) ? 1 : 0;
}


/**
 * @brief  边缘平滑滤波（移动平均）
 * @param  无
 * @return 无
 * @note   对检测到的边缘进行3点移动平均,消除毛刺
 */
void edge_smooth_filter(void)
{
    int i;
    static uint8 left_buffer[IMAGE_HEIGHT];
    static uint8 right_buffer[IMAGE_HEIGHT];

    // 复制原始边缘数据
    for (i = 0; i < IMAGE_HEIGHT; i++)
    {
        left_buffer[i] = vision.track.left_edge[i];
        right_buffer[i] = vision.track.right_edge[i];
    }

    // 3点移动平均滤波
    for (i = 1; i < IMAGE_HEIGHT - 1; i++)
    {
        // 只对非丢线点进行平滑
        if (Left_Lost_Flag[i] == 0 && Left_Lost_Flag[i-1] == 0 && Left_Lost_Flag[i+1] == 0)
        {
            vision.track.left_edge[i] = (left_buffer[i-1] + left_buffer[i] + left_buffer[i+1]) / 3;
        }

        if (Right_Lost_Flag[i] == 0 && Right_Lost_Flag[i-1] == 0 && Right_Lost_Flag[i+1] == 0)
        {
            vision.track.right_edge[i] = (right_buffer[i-1] + right_buffer[i] + right_buffer[i+1]) / 3;
        }
    }
}

/**
 * @brief  丢线补线处理
 * @param  无
 * @return 无
 * @note   使用斜率延拓法补线,防止边线突变
 */
void lost_line_repair(void)
{
    int i;
    int last_valid_left = 0;
    int last_valid_right = MT9V03X_W - 1;
    float left_slope = 0.0f;
    float right_slope = 0.0f;
    int slope_count_left = 0;
    int slope_count_right = 0;

    // 从下往上遍历,计算斜率并补线
    for (i = MT9V03X_H - 1; i >= 0; i--)
    {
        // === 左边线处理 ===
        if (Left_Lost_Flag[i] == 0)
        {
            // 非丢线点,更新斜率
            if (last_valid_left != 0 && i < MT9V03X_H - 1)
            {
                float new_slope = (float)(vision.track.left_edge[i] - last_valid_left);
                // 斜率限制
                if (new_slope > -SLOPE_LIMIT * 10 && new_slope < SLOPE_LIMIT * 10)
                {
                    left_slope = (left_slope * slope_count_left + new_slope) / (slope_count_left + 1);
                    slope_count_left++;
                    if (slope_count_left > 5) slope_count_left = 5;  // 限制历史数量
                }
            }
            last_valid_left = vision.track.left_edge[i];
        }
        else
        {
            // 丢线点,使用斜率延拓
            vision.track.left_edge[i] = last_valid_left + (int)left_slope;
            // 边界限制
            if (vision.track.left_edge[i] < 0) vision.track.left_edge[i] = 0;
            if (vision.track.left_edge[i] >= MT9V03X_W) vision.track.left_edge[i] = MT9V03X_W - 1;
        }

        // === 右边线处理 ===
        if (Right_Lost_Flag[i] == 0)
        {
            // 非丢线点,更新斜率
            if (last_valid_right != MT9V03X_W - 1 && i < MT9V03X_H - 1)
            {
                float new_slope = (float)(vision.track.right_edge[i] - last_valid_right);
                // 斜率限制
                if (new_slope > -SLOPE_LIMIT * 10 && new_slope < SLOPE_LIMIT * 10)
                {
                    right_slope = (right_slope * slope_count_right + new_slope) / (slope_count_right + 1);
                    slope_count_right++;
                    if (slope_count_right > 5) slope_count_right = 5;
                }
            }
            last_valid_right = vision.track.right_edge[i];
        }
        else
        {
            // 丢线点,使用斜率延拓
            vision.track.right_edge[i] = last_valid_right + (int)right_slope;
            // 边界限制
            if (vision.track.right_edge[i] < 0) vision.track.right_edge[i] = 0;
            if (vision.track.right_edge[i] >= MT9V03X_W) vision.track.right_edge[i] = MT9V03X_W - 1;
        }

        // 更新赛道宽度
        vision.track.track_width[i] = vision.track.right_edge[i] - vision.track.left_edge[i];
    }
}

/**
 * @brief  最小二乘法中线拟合
 * @param  无
 * @return 无
 * @note   对指定范围内的中心线进行最小二乘拟合,提高稳定性
 */
void centerline_least_square_fit(void)
{
    int i;
    float sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
    int valid_points = 0;
    float a = 0, b = 0;  // y = a*x + b (x为行号,y为列号)

    // 统计有效点并计算累加值
    for (i = FIT_START_ROW; i >= FIT_END_ROW; i--)
    {
        if (vision.track.track_width[i] >= TRACK_WIDTH_MIN && vision.track.track_width[i] <= TRACK_WIDTH_MAX)
        {
            float center = (vision.track.left_edge[i] + vision.track.right_edge[i]) / 2.0f;
            sum_x += i;
            sum_y += center;
            sum_xy += i * center;
            sum_x2 += i * i;
            valid_points++;
        }
    }

    // 如果有效点足够,进行拟合
    if (valid_points >= FIT_MIN_POINTS)
    {
        float denominator = valid_points * sum_x2 - sum_x * sum_x;
        if (denominator != 0)  // 避免除零
        {
            a = (valid_points * sum_xy - sum_x * sum_y) / denominator;
            b = (sum_y - a * sum_x) / valid_points;

            // 使用拟合结果更新中心线
            for (i = FIT_START_ROW; i >= FIT_END_ROW; i--)
            {
                if (vision.track.track_width[i] >= TRACK_WIDTH_MIN && vision.track.track_width[i] <= TRACK_WIDTH_MAX)
                {
                    float fitted_center = a * i + b;
                    // 限制拟合结果不要偏离原值太远
                    float original_center = (vision.track.left_edge[i] + vision.track.right_edge[i]) / 2.0f;
                    float diff = fitted_center - original_center;
                    if (diff > -20 && diff < 20)  // 允许±20像素偏差
                    {
                        // 使用加权平均:70%拟合值 + 30%原始值
                        vision.track.center_line[i] = (uint8)(fitted_center * 0.7f + original_center * 0.3f);
                    }
                }
            }
        }
    }
}

/**
 * @brief  获取轨道偏差值（使用权重数组优化版）
 * @param  无
 * @return 轨道偏差值
 */
int16 vision_get_deviation(void)
{
    if (!vision.track_found)
    {
        return vision.last_error;
    }
    
    uint8 row;
    uint8 center_col = IMAGE_WIDTH / 2;
    uint32 sum_weighted_center = 0;
    uint32 sum_weight = 0;
    uint16 valid_row_count = 0;
    
    for (row = SCAN_START_ROW; row > SCAN_END_ROW; row -= SCAN_STEP)
    {
        if (vision.track.track_width[row] >= TRACK_WIDTH_MIN && 
            vision.track.track_width[row] <= TRACK_WIDTH_MAX)
        {
            uint8 weight;
            uint16 distance_from_bottom = SCAN_START_ROW - row;
            
            if (distance_from_bottom < 30)
                weight = 3;
            else if (distance_from_bottom < 60)
                weight = 2;
            else
                weight = 1;
            
            if (vision.track.center_line[row] < IMAGE_WIDTH)
            {
                sum_weighted_center += (uint32)vision.track.center_line[row] * weight;
                sum_weight += weight;
                valid_row_count++;
            }
        }
    }
    
    if (sum_weight > 0 && valid_row_count > 0)
    {
        uint32 weighted_center_val = sum_weighted_center / sum_weight;
        
        if (weighted_center_val >= IMAGE_WIDTH) {
            weighted_center_val = IMAGE_WIDTH - 1;
        }
        int16 new_error = (int16)weighted_center_val - (int16)center_col;
        if (new_error < -80) new_error = -80;
        if (new_error > 80) new_error = 80;
        
        vision.error = new_error+1;
        vision.deviation = (float)vision.error / center_col;
        vision.last_error = vision.error;
    }
    else
    {
        // 没有有效数据，保持上次偏差
        vision.error = vision.last_error;
    }
    
    return vision.error;
}

/**
 * @brief  轨道图像处理
 * @param  无
 * @return 无
 * @note   通过调用一系列函数实现轨道检测和偏差计算
 */
void vision_image_process(void)
{
    if (!vision.image_ready)
        return;
    
    // 计算阈值
    //uint8 threshold = otsu_threshold((uint8 *)mt9v03x_image, IMAGE_WIDTH * IMAGE_HEIGHT);
    uint8 threshold = my_adapt_threshold((uint8 *)mt9v03x_image, IMAGE_WIDTH, IMAGE_HEIGHT);
    
    // 使用固定阈值
    //threshold = THRESHOLD_VALUE;
    
    // 图像二值化处理
    image_binarization(threshold);
    
    // 轨道边缘检测
    vision_find_track_edge();
    
    // 获取轨道偏差值
    vision_get_deviation();
    
    vision.image_ready = 0;
}

