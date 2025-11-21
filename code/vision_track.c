#include "vision_track.h"


vision_track_t vision;

uint8 image_data[IMAGE_HEIGHT][IMAGE_WIDTH];
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
 * @brief  计算局部区域平均值
 * @param  row    起始行
 * @param  col    起始列
 * @param  size   块大小
 * @return 平均值
 */
static uint8 calculate_local_mean(uint8 row, uint8 col, uint8 size)
{
    uint32 sum = 0;
    uint16 count = 0;
    uint8 half_size = size / 2;
    
    int16 start_row = (int16)row - half_size;
    int16 end_row = (int16)row + half_size;
    int16 start_col = (int16)col - half_size;
    int16 end_col = (int16)col + half_size;
    
    // 边界检查
    if (start_row < 0) start_row = 0;
    if (end_row >= IMAGE_HEIGHT) end_row = IMAGE_HEIGHT - 1;
    if (start_col < 0) start_col = 0;
    if (end_col >= IMAGE_WIDTH) end_col = IMAGE_WIDTH - 1;
    
    // 计算区域平均值
    for (int16 r = start_row; r <= end_row; r++)
    {
        for (int16 c = start_col; c <= end_col; c++)
        {
            sum += mt9v03x_image[r][c];
            count++;
        }
    }
    
    return (count > 0) ? (uint8)(sum / count) : 128;
}

/**
 * @brief  自适应局部阈值二值化
 * @param  无
 * @return 无
 * @note   使用局部均值作为阈值，适应光照变化
 */
static void adaptive_threshold_binarization(void)
{
    uint8 row, col;
    
    for (row = 0; row < IMAGE_HEIGHT; row++)
    {
        for (col = 0; col < IMAGE_WIDTH; col++)
        {
            // 计算局部平均值作为阈值
            uint8 local_threshold = calculate_local_mean(row, col, ADAPTIVE_BLOCK_SIZE);
            
            // 添加偏移量，防止噪声
            if (local_threshold > ADAPTIVE_OFFSET)
                local_threshold -= ADAPTIVE_OFFSET;
            
            // 二值化
            if (mt9v03x_image[row][col] > local_threshold)
                image_data[row][col] = 255;
            else
                image_data[row][col] = 0;
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
    // morphology_erode();   // 腐蚀：去除小噪点
    // morphology_dilate();  // 膨胀：填充小孔洞
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
    
    // 计算水平梯度（Sobel算子简化版）
    int16 gradient = (int16)image_data[row][col+1] - (int16)image_data[row][col-1];
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
        uint8 left_search_start = (last_valid_left > 30) ? (last_valid_left - 30) : 10;
        uint8 left_search_end = (last_valid_left < IMAGE_WIDTH - 30) ? (last_valid_left + 10) : (IMAGE_WIDTH / 2);
        
        // 向左搜索白到黑的跳变
        for (col = last_valid_left; col > left_search_start; col--)
        {
            if (image_data[row][col] > 128 && image_data[row][col-1] < 128)
            {
                int16 gradient = calculate_gradient(row, col);
                if (gradient > 50 && gradient > max_gradient)
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
        
                if (image_data[row][col] > 128 && image_data[row][col-1] < 128)
                {
                    int16 gradient = calculate_gradient(row, col);
                    if (gradient > 50)
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
        uint8 right_search_start = (last_valid_right > 30) ? (last_valid_right - 10) : (IMAGE_WIDTH / 2);
        uint8 right_search_end = (last_valid_right < IMAGE_WIDTH - 30) ? (last_valid_right + 30) : (IMAGE_WIDTH - 10);
        
        // 向右搜索白到黑的跳变
        for (col = last_valid_right; col < right_search_end; col++)
        {
            if (image_data[row][col] > 128 && image_data[row][col+1] < 128)
            {
                int16 gradient = calculate_gradient(row, col);
                if (gradient > 50 && gradient > max_gradient)
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
        }
        else
        {
            // 轨道宽度不合理时，使用预测中心线
            vision.track.center_line[row] = last_valid_center;
            
            // 连续多行丢失，提前退出
            if (vision.track.valid_rows == 0 && row < SCAN_START_ROW - 10)
            {
                break;
            }
        }
    }
    
    // 判断是否成功找到轨道（有效行数不少于50行）
    vision.track_found = (vision.track.valid_rows >= 50) ? 1 : 0;
}


/**
 * @brief  获取轨道偏差值（优化版）
 * @param  无
 * @return 轨道偏差值 (-80 ~ 80)
 */
/**
int16 vision_get_deviation(void)
{
    if (!vision.track_found)
    {
        return vision.last_error;  // 如果未找到轨道，返回上一次的偏差值
    }
    
    uint8 row;
    uint8 center_col = IMAGE_WIDTH / 2;
    uint32 sum_weighted_center = 0;
    uint32 sum_weight = 0;
    
    // 分段加权：近处权重更大，远处权重小
    // 将扫描区域分为3段：近(70%)、中(20%)、远(10%)
    for (row = SCAN_START_ROW; row > SCAN_END_ROW; row -= SCAN_STEP)
    {
        if (vision.track.track_width[row] >= TRACK_WIDTH_MIN && 
            vision.track.track_width[row] <= TRACK_WIDTH_MAX)
        {
            uint8 weight;
            uint8 distance_from_bottom = SCAN_START_ROW - row;
            
            // 近处（底部30行）：权重3
            if (distance_from_bottom < 30)
                weight = 3;
            // 中段（30-60行）：权重2
            else if (distance_from_bottom < 60)
                weight = 2;
            // 远处（60行以上）：权重1
            else
                weight = 1;
            
            sum_weighted_center += vision.track.center_line[row] * weight;
            sum_weight += weight;
        }
    }
    
    // 计算加权平均中心位置
    if (sum_weight > 0)
    {
        uint8 weighted_center = sum_weighted_center / sum_weight;
        vision.error = weighted_center - center_col;
        vision.deviation = (float)vision.error / center_col;  // 归一化偏差值 -1.0 ~ 1.0
        vision.last_error = vision.error;
    }
    else
    {
        // 没有有效数据，保持上次偏差
        vision.error = vision.last_error;
    }
    
    return vision.error;
}

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
