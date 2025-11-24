#include "vision_track.h"
#include <math.h>

#ifndef PI
#define PI 3.1415926535f
#endif

vision_track_t vision;

uint8 image_data[IMAGE_HEIGHT][IMAGE_WIDTH];
// 积分图缓冲区，用于快速计算局部均值 (约90KB)
// integral_image[y][x] 存储 (0,0) 到 (x,y) 矩形区域的像素和
static uint32 integral_image[IMAGE_HEIGHT][IMAGE_WIDTH];

//====================================================图像处理====================================================
/**
 * @brief  OTSU阈值计算（优化版，使用整数运算）
 * @param  image  图像数据指针
 * @param  size   图像数据大小
 * @return 阈值
 */
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
 * @brief  计算积分图
 * @param  无 (使用全局 mt9v03x_image)
 * @return 无 (结果存入 integral_image)
 */
static void calculate_integral_image(void)
{
    uint8 r, c;
    uint32 sum_row;

    // 第一行
    sum_row = 0;
    for (c = 0; c < IMAGE_WIDTH; c++)
    {
        sum_row += mt9v03x_image[0][c];
        integral_image[0][c] = sum_row;
    }

    // 后续行
    for (r = 1; r < IMAGE_HEIGHT; r++)
    {
        sum_row = 0;
        for (c = 0; c < IMAGE_WIDTH; c++)
        {
            sum_row += mt9v03x_image[r][c];
            integral_image[r][c] = integral_image[r-1][c] + sum_row;
        }
    }
}

/**
 * @brief  使用积分图快速获取局部区域像素和
 * @param  r1, c1  左上角坐标
 * @param  r2, c2  右下角坐标
 * @return 区域像素和
 */
static uint32 get_region_sum(uint8 r1, uint8 c1, uint8 r2, uint8 c2)
{
    // 边界保护
    if (r2 >= IMAGE_HEIGHT) r2 = IMAGE_HEIGHT - 1;
    if (c2 >= IMAGE_WIDTH) c2 = IMAGE_WIDTH - 1;
    
    uint32 A = (r1 > 0 && c1 > 0) ? integral_image[r1-1][c1-1] : 0;
    uint32 B = (r1 > 0) ? integral_image[r1-1][c2] : 0;
    uint32 C = (c1 > 0) ? integral_image[r2][c1-1] : 0;
    uint32 D = integral_image[r2][c2];

    return D - B - C + A;
}

/**
 * @brief  自适应局部阈值二值化（积分图加速版）
 * @param  无
 * @return 无
 * @note   使用局部均值作为阈值，适应光照变化。时间复杂度 O(W*H)。
 */
static void adaptive_threshold_binarization(void)
{
    uint8 r, c_idx;
    uint8 block_size = ADAPTIVE_BLOCK_SIZE;
    uint8 half_size = block_size / 2;
    
    // 1. 计算积分图
    calculate_integral_image();

    // 2. 遍历图像进行二值化
    for (r = 0; r < IMAGE_HEIGHT; r++)
    {
        for (c_idx = 0; c_idx < IMAGE_WIDTH; c_idx++)
        {
            // 确定局部窗口边界
            uint8 r1 = (r > half_size) ? (r - half_size) : 0;
            uint8 r2 = (r + half_size < IMAGE_HEIGHT) ? (r + half_size) : (IMAGE_HEIGHT - 1);
            uint8 c1 = (c_idx > half_size) ? (c_idx - half_size) : 0;
            uint8 c2 = (c_idx + half_size < IMAGE_WIDTH) ? (c_idx + half_size) : (IMAGE_WIDTH - 1);

            // 快速计算局部均值
            uint32 sum = get_region_sum(r1, c1, r2, c2);
            uint16 count = (r2 - r1 + 1) * (c2 - c1 + 1);
            uint8 mean = sum / count;
            
            // 添加偏移量
            uint8 threshold = (mean > ADAPTIVE_OFFSET) ? (mean - ADAPTIVE_OFFSET) : 0;

            // 二值化
            if (mt9v03x_image[r][c_idx] > threshold)
            {
                image_data[r][c_idx] = 255;
            }
            else
            {
                image_data[r][c_idx] = 0;
            }
        }
    }
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
 * @brief  图像二值化处理（优化版，支持多种模式）
 * @param  threshold  阈值（固定阈值模式使用）
 * @return 无
 */
void image_binarization(uint8 threshold)
{
    uint8 row, col;
    
    #if (BINARIZE_MODE == BINARIZE_FIXED)
        // 模式1：固定阈值二值化
        for (row = 0; row < IMAGE_HEIGHT; row++)
        {
            for (col = 0; col < IMAGE_WIDTH; col++)
            {
                image_data[row][col] = (mt9v03x_image[row][col] > threshold) ? 255 : 0;
            }
        }
        
    #elif (BINARIZE_MODE == BINARIZE_OTSU)
        // 模式2：OTSU全局阈值
        for (row = 0; row < IMAGE_HEIGHT; row++)
        {
            for (col = 0; col < IMAGE_WIDTH; col++)
            {
                image_data[row][col] = (mt9v03x_image[row][col] > threshold) ? 255 : 0;
            }
        }
        
    #elif (BINARIZE_MODE == BINARIZE_ADAPTIVE)
        // 模式3：自适应局部阈值
        adaptive_threshold_binarization();
        
    #elif (BINARIZE_MODE == BINARIZE_OTSU_ADAPTIVE)
        // 模式4：OTSU+自适应混合（推荐）
        // 上半部分使用自适应（远处光照变化大）
        for (row = 0; row < IMAGE_HEIGHT / 2; row++)
        {
            for (col = 0; col < IMAGE_WIDTH; col++)
            {
                uint8 local_threshold = calculate_local_mean(row, col, ADAPTIVE_BLOCK_SIZE);
                if (local_threshold > ADAPTIVE_OFFSET)
                    local_threshold -= ADAPTIVE_OFFSET;
                image_data[row][col] = (mt9v03x_image[row][col] > local_threshold) ? 255 : 0;
            }
        }
        
        // 下半部分使用OTSU阈值（近处更稳定）
        for (row = IMAGE_HEIGHT / 2; row < IMAGE_HEIGHT; row++)
        {
            for (col = 0; col < IMAGE_WIDTH; col++)
            {
                image_data[row][col] = (mt9v03x_image[row][col] > threshold) ? 255 : 0;
            }
        }
    #endif
    
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
 * @brief  计算梯度强度（用于边缘检测）
 * @param  row  行号
 * @param  col  列号
 * @return 梯度强度
 */
static int16 calculate_gradient(uint8 row, uint8 col)
{
    // 边界检查
    if (col < 1 || col >= IMAGE_WIDTH - 1)
        return 0;
    
    // 修复：使用原始灰度图 mt9v03x_image 计算梯度，而不是二值化后的 image_data
    // 这样才能真正利用梯度大小来区分强边缘和噪声
    int16 gradient = (int16)mt9v03x_image[row][col+1] - (int16)mt9v03x_image[row][col-1];
    return gradient > 0 ? gradient : -gradient;  // 返回绝对值
}

/**
 * @brief  视觉跟踪边缘检测（优化版）
 * @param  无
 * @return 无
 * @note   使用梯度法和连续性约束提高检测精度和鲁棒性
 */
void vision_find_track_edge(void)
{
    uint8 row, col;
    uint8 left_found, right_found;
    static uint8 center_col = IMAGE_WIDTH / 2;
    uint8 last_valid_left = 10;
    uint8 last_valid_right = IMAGE_WIDTH - 10;
    uint8 last_valid_center = center_col;
    int16 max_gradient;
    uint8 max_gradient_col;
    
    vision.track.valid_rows = 0;
    
    // 从图像底部向上扫描
    for (row = SCAN_START_ROW; row > SCAN_END_ROW; row -= SCAN_STEP)
    {
        left_found = 0;
        right_found = 0;
        
        // === 左边缘检测：使用梯度法 ===
        max_gradient = 0;
        max_gradient_col = 10;
        
        // 从上一行边缘位置附近开始搜索（连续性约束）
        // 修复：确保搜索范围有效，避免死循环或跳过
        uint8 left_search_start = (last_valid_left > 30) ? (last_valid_left - 30) : 2;
        
        // 向左搜索白到黑的跳变
        for (col = last_valid_left; col > left_search_start; col--)
        {
            // 结合二值化结果进行粗筛选，再用梯度精确定位
            if (image_data[row][col] > 128 && image_data[row][col-1] < 128)
            {
                int16 gradient = calculate_gradient(row, col);
                // 只有梯度足够大且是局部最大值时才认为是边缘
                if (gradient > GRADIENT_THRESHOLD && gradient > max_gradient)
                {
                    max_gradient = gradient;
                    max_gradient_col = col;
                    left_found = 1;
                }
            }
        }
        
        // 如果没找到，扩展到中心区域搜索
        if (!left_found)
        {
            for (col = center_col; col > 10; col--)
            {
                if (image_data[row][col] > 128 && image_data[row][col-1] < 128)
                {
                    int16 gradient = calculate_gradient(row, col);
                    if (gradient > GRADIENT_THRESHOLD)
                    {
                        max_gradient_col = col;
                        left_found = 1;
                        break;
                    }
                }
            }
        }
        if (left_found)
        {
            vision.track.left_edge[row] = max_gradient_col;
            last_valid_left = max_gradient_col;
        }
        else
        {
            // 丢边处理：使用上一行边缘预测
            vision.track.left_edge[row] = last_valid_left;
        }
        
        // === 右边缘检测：使用梯度法 ===
        max_gradient = 0;
        max_gradient_col = IMAGE_WIDTH - 10;
        
        // 从上一行边缘位置附近开始搜索
        uint8 right_search_end = (last_valid_right < IMAGE_WIDTH - 30) ? (last_valid_right + 30) : (IMAGE_WIDTH - 2);
        
        // 向右搜索白到黑的跳变
        for (col = last_valid_right; col < right_search_end; col++)
        {
            if (image_data[row][col] > 128 && image_data[row][col+1] < 128)
            {
                int16 gradient = calculate_gradient(row, col);
                if (gradient > GRADIENT_THRESHOLD && gradient > max_gradient)
                {
                    max_gradient = gradient;
                    max_gradient_col = col;
                    right_found = 1;
                }
            }
        }
        
        // 如果没找到，扩展到中心区域搜索
        if (!right_found)
        {
            for (col = center_col; col < IMAGE_WIDTH - 10; col++)
            {
                if (image_data[row][col] > 128 && image_data[row][col+1] < 128)
                {
                    int16 gradient = calculate_gradient(row, col);
                    if (gradient > 50)
                    {
                        max_gradient_col = col;
                        right_found = 1;
                        break;
                    }
                }
            }
        }
        
        if (right_found)
        {
            vision.track.right_edge[row] = max_gradient_col;
            last_valid_right = max_gradient_col;
        }
        else
        {
            // 丢边处理：使用上一行边缘预测
            vision.track.right_edge[row] = last_valid_right;
        }

        

        // 计算轨道宽度
        vision.track.track_width[row] = vision.track.right_edge[row] - vision.track.left_edge[row];
        
        // 判断轨道宽度是否合理
        if (vision.track.track_width[row] >= TRACK_WIDTH_MIN &&
            vision.track.track_width[row] <= TRACK_WIDTH_MAX)
        {
            // 计算中心线
            vision.track.center_line[row] = (vision.track.left_edge[row] + vision.track.right_edge[row]) / 2;

            // 更新中心列，使用当前行的中心线作为下一行的搜索中心
            center_col = vision.track.center_line[row];

            // 限制中心列，避免过于靠近边缘
            if (center_col < 20) center_col = 20;
            if (center_col > IMAGE_WIDTH - 20) center_col = IMAGE_WIDTH - 20;

            last_valid_center = vision.track.center_line[row];
            vision.track.valid_rows++;
            
            // 计算真实世界坐标
            vision_pixel_to_world(row, vision.track.left_edge[row], &vision.track.left_real_x[row], &vision.track.left_real_y[row]);
            vision_pixel_to_world(row, vision.track.right_edge[row], &vision.track.right_real_x[row], &vision.track.right_real_y[row]);
            vision_pixel_to_world(row, vision.track.center_line[row], &vision.track.center_real_x[row], &vision.track.center_real_y[row]);
        }
        else
        {
            // 轨道宽度不合理时，使用预测中心线
            vision.track.center_line[row] = last_valid_center;
            
            // 计算预测中心线的真实坐标
            vision_pixel_to_world(row, vision.track.center_line[row], &vision.track.center_real_x[row], &vision.track.center_real_y[row]);
            
            // 连续多行丢失，提前退出
            if (vision.track.valid_rows == 0 && row < SCAN_START_ROW - 10)
            {
                break;
            }
        }
    }
    
    // 判断是否成功找到轨道（有效行数不少于50行）
    vision.track_found = (vision.track.valid_rows >= 80) ? 1 : 0;
}


/**
 * @brief  获取轨道偏差值（优化版）
 * @param  无
 * @return 轨道偏差值 (-80 ~ 80)
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
            
            // 修复1：使用更大的数据类型避免溢出
            if (distance_from_bottom < 30)
                weight = 3;
            else if (distance_from_bottom < 60)
                weight = 2;
            else
                weight = 1;
            
            // 修复2：添加边界检查
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
        // 修复3：使用更大的数据类型并添加溢出检查
        uint32 weighted_center_val = sum_weighted_center / sum_weight;
        
        // 修复4：确保在有效范围内
        if (weighted_center_val >= IMAGE_WIDTH) {
            weighted_center_val = IMAGE_WIDTH - 1;
        }
        
        // 修复5：使用有符号运算
        int16 new_error = (int16)weighted_center_val - (int16)center_col;
        
        // 修复6：限制误差范围
        if (new_error < -80) new_error = -80;
        if (new_error > 80) new_error = 80;
        
        vision.error = new_error;
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
    uint8 threshold = otsu_threshold((uint8 *)mt9v03x_image, IMAGE_WIDTH * IMAGE_HEIGHT);
    
    // 使用固定阈值
    threshold = THRESHOLD_VALUE;
    
    // 图像二值化处理
    image_binarization(threshold);
    
    // 轨道边缘检测
    vision_find_track_edge();
    
    // 获取轨道偏差值
    vision_get_deviation();
    
    vision.image_ready = 0;
}

/**
 * @brief  显示轨道图像
 * @param  无
 * @return 无
 */
void vision_show_image(void)
{
    // 该函数用于显示轨道图像，具体实现依赖于硬件平台
    // 具体显示函数见 display_tft180.c
}

/**
 * @brief  像素坐标转世界坐标 (透视变换)
 * @param  row    图像行坐标 (0-119)
 * @param  col    图像列坐标 (0-187)
 * @param  real_x 输出：世界坐标X (cm)
 * @param  real_y 输出：世界坐标Y (cm)
 */
void vision_pixel_to_world(uint8 row, uint8 col, float *real_x, float *real_y)
{
    // 角度换算
    float alpha = CAMERA_VIEW_ANGLE * PI / 180.0f; // 俯仰角弧度
    float beta_v = (CAMERA_FOV_V / 2.0f) * PI / 180.0f; // 垂直半视场角
    float beta_h = (CAMERA_FOV_H / 2.0f) * PI / 180.0f; // 水平半视场角
    
    // 归一化坐标 (-1.0 ~ 1.0)
    // 注意：图像坐标系原点在左上角，row=0对应最上方（远处），row=119对应最下方（近处）
    float y_norm = (2.0f * row / IMAGE_HEIGHT) - 1.0f; 
    float x_norm = (2.0f * col / IMAGE_WIDTH) - 1.0f;
    
    // 垂直方向角度 (相对于光轴)
    // 使用 atan 模型： tan(theta) = y_norm * tan(beta_v)
    float theta_v = atanf(y_norm * tanf(beta_v));
    
    // 总俯仰角 (相对于水平面)
    // row越大，y_norm越大，theta_v越大（向下偏），总角度越大
    float angle_total = alpha + theta_v;
    
    // 计算纵向距离 Y (cm)
    // Y = H / tan(angle_total)
    if (angle_total <= 0.01f) {
        *real_y = 1000.0f; // 限制最大距离
    } else {
        *real_y = CAMERA_HEIGHT / tanf(angle_total);
    }
    
    // 计算横向距离 X (cm)
    // X = Y * tan(theta_h)
    // theta_h = atan(x_norm * tan(beta_h))
    float theta_h = atanf(x_norm * tanf(beta_h));
    *real_x = *real_y * tanf(theta_h);
}
