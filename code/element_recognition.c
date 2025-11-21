#include "element_recognition.h"
#include <string.h>
#include <math.h>

//====================================================元素结构体====================================================
element_recognition_t element_recog;

//====================================================轨迹相关函数====================================================
/**
 * @brief  获取轨迹宽度
 * @param  row  行号
 * @return 宽度
 */
static uint16 get_track_width(uint8 row)
{
    if (row >= IMAGE_HEIGHT)
        return 0;
    
    uint8 left = vision.track.left_edge[row];
    uint8 right = vision.track.right_edge[row];
    
    if (left < right)
        return (uint16)(right - left);
    else
        return 0;
}

/**
 * @brief  计算曲率
 * @param  start_row  起始行号
 * @param  end_row    结束行号
 * @return 曲率值
 */
static int16 calculate_curvature(uint8 start_row, uint8 end_row)
{
    if (start_row >= IMAGE_HEIGHT || end_row >= IMAGE_HEIGHT || start_row >= end_row)
        return 0;
    
    int32 sum = 0;
    uint8 count = 0;
    
    for (uint8 row = start_row; row < end_row && row < IMAGE_HEIGHT; row++)
    {
        uint8 center_now = vision.track.center_line[row];
        
        if (row > 0 && row < IMAGE_HEIGHT)
        {
            uint8 center_prev = vision.track.center_line[row - 1];
            int16 diff = (int16)center_now - (int16)center_prev;
            sum += diff;
            count++;
        }
    }
    
    if (count > 0)
        return (int16)(sum / count);
    else
        return 0;
}

//====================================================元素识别相关函数====================================================
/**
 * @brief  元素识别初始化
 * @param  无
 * @return 无
 */
void element_recognition_init(void)
{
    memset(&element_recog, 0, sizeof(element_recognition_t));
    element_recog.current_element.type = ELEMENT_NONE;
    element_recog.current_element.state = ELEMENT_STATE_NONE;
}

/**
 * @brief  元素识别处理
 * @param  无
 * @return 无
 * @note   元素识别处理函数，基于视觉数据进行元素检测和状态更新
 */
void element_recognition_process(void)
{
    // 如果当前有识别到的元素，更新其状态
    if (element_recog.current_element.type != ELEMENT_NONE)
    {
        element_update_state();
        
        // 如果元素状态为已通过，更新最后识别的元素并重置当前元素
        if (element_recog.current_element.state == ELEMENT_STATE_PASSED)
        {
            element_recog.last_element = element_recog.current_element;
            element_reset();
        }
        else
        {
            // 如果元素未通过，继续检测该元素
            element_recog.current_element.detected = 1;
            
            // 根据当前元素类型调用相应的处理函数
            switch (element_recog.current_element.type)
            {
                case ELEMENT_CROSS:
                    element_handle_cross();
                    break;
                case ELEMENT_CIRCLE:
                    element_handle_circle();
                    break;
                case ELEMENT_RAMP:
                    element_handle_ramp();
                    break;
                case ELEMENT_PARKING:
                    element_handle_parking();
                    break;
                case ELEMENT_OBSTACLE:
                    element_handle_obstacle();
                    break;
                default:
                    break;
            }
            return;  // 如果元素未通过，继续检测该元素
        }
    }
    
    // 如果当前没有识别到的元素，按照优先级依次检测元素
    // 优先级顺序：停车 > 障碍物 > 圆环 > 十字 > 坡道 > 其他
    // 根据优先级依次检测元素
    
    if (element_detect_parking())
    {
        if (element_recog.current_element.confidence >= 70)  // 置信度阈值
        {
            element_recog.current_element.type = ELEMENT_PARKING;
            element_recog.current_element.state = ELEMENT_STATE_FOUND;
            element_recog.current_element.detected = 1;
            element_recog.current_element.frame_count = 1;
        }
    }
    else if (element_detect_obstacle())
    {
        if (element_recog.current_element.confidence >= 60)
        {
            element_recog.current_element.type = ELEMENT_OBSTACLE;
            element_recog.current_element.state = ELEMENT_STATE_FOUND;
            element_recog.current_element.detected = 1;
            element_recog.current_element.frame_count = 1;
        }
    }
    else if (element_detect_circle())
    {
        if (element_recog.current_element.confidence >= 70)
        {
            element_recog.current_element.type = ELEMENT_CIRCLE;
            element_recog.current_element.state = ELEMENT_STATE_FOUND;
            element_recog.current_element.detected = 1;
            element_recog.current_element.frame_count = 1;
        }
    }
    else if (element_detect_cross())
    {
        if (element_recog.current_element.confidence >= 65)
        {
            element_recog.current_element.type = ELEMENT_CROSS;
            element_recog.current_element.state = ELEMENT_STATE_FOUND;
            element_recog.current_element.detected = 1;
            element_recog.current_element.frame_count = 1;
        }
    }
    else if (element_detect_ramp())
    {
        if (element_recog.current_element.confidence >= 50)
        {
            element_recog.current_element.type = ELEMENT_RAMP;
            element_recog.current_element.state = ELEMENT_STATE_FOUND;
            element_recog.current_element.detected = 1;
            element_recog.current_element.frame_count = 1;
        }
    }
}

/**
 * @brief  元素识别十字路口
 * @param  无
 * @return 1-识别到十字路口元素 0-未识别到十字路口元素
 */
uint8 element_detect_cross(void)
{
    // 如果未找到赛道或有效行数不足，返回未识别
    if (!vision.track_found || vision.track.valid_rows < 5)
        return 0;
    
    uint8 wide_count = 0;
    uint16 avg_width = 0;
    uint8 check_rows = 0;
    
    // 如果未找到赛道或有效行数不足，返回未识别
    for (uint8 row = 20; row < 60 && row < IMAGE_HEIGHT; row++)
    {
        uint16 width = get_track_width(row);
        if (width > 0)
        {
            avg_width += width;
            check_rows++;
            
            if (width > CROSS_WIDTH_THRESHOLD)
            {
                wide_count++;
            }
        }
    }
    
    if (check_rows > 0)
        avg_width /= check_rows;
    else
        return 0;
    
    // 根据检测到的宽度判断是否为十字路口
    if (wide_count >= CROSS_DETECT_ROWS && avg_width > 80)
    {
        element_recog.cross.left_found = 1;
        element_recog.cross.right_found = 1;
        element_recog.current_element.confidence = (wide_count * 100) / check_rows;
        return 1;
    }
    
    return 0;
}

/**
 * @brief  元素识别圆环
 * @param  无
 * @return 1-识别到圆环元素 0-未识别到圆环元素
 */
uint8 element_detect_circle(void)
{
    // 如果未找到赛道或有效行数不足，返回未识别
    if (!vision.track_found || vision.track.valid_rows < 10)
        return 0;
    
    // 计算曲率
    int16 curvature = calculate_curvature(30, 80);
    element_recog.circle.curvature = curvature;
    
    // 如果曲率超过阈值，认为检测到圆环
    if (abs(curvature) > CIRCLE_CURVATURE_THRESHOLD)
    {
        element_recog.circle.continuous_rows++;
        
        // 确定方向
        if (curvature > 0)
            element_recog.circle.direction = 1;  // 顺时针
        else
            element_recog.circle.direction = 0;  // 逆时针
        
        // 根据连续检测到的行数判断圆环
        if (element_recog.circle.continuous_rows >= CIRCLE_CONTINUOUS_ROWS)
        {
            // 提高置信度
            element_recog.current_element.confidence = 60 + (element_recog.circle.continuous_rows * 2);
            if (element_recog.current_element.confidence > 95)
                element_recog.current_element.confidence = 95;
            return 1;
        }
    }
    else
    {
        // 如果未检测到圆环，连续检测行数递减
        if (element_recog.circle.continuous_rows > 0)
            element_recog.circle.continuous_rows--;
    }
    
    return 0;
}

/**
 * @brief  元素识别坡道
 * @param  无
 * @return 1-识别到坡道元素 0-未识别到坡道元素
 */
uint8 element_detect_ramp(void)
{
    // 如果未找到赛道或有效行数不足，返回未识别
    uint32 top_brightness = 0;
    uint32 bottom_brightness = 0;
    uint16 top_count = 0, bottom_count = 0;
    
    // 计算图像顶部亮度总和
    for (uint8 row = 10; row < 30 && row < IMAGE_HEIGHT; row++)
    {
        for (uint8 col = 40; col < 120 && col < IMAGE_WIDTH; col++)
        {
            top_brightness += image_data[row][col];
            top_count++;
        }
    }
    
    // 计算图像底部亮度总和
    for (uint8 row = 70; row < 90 && row < IMAGE_HEIGHT; row++)
    {
        for (uint8 col = 40; col < 120 && col < IMAGE_WIDTH; col++)
        {
            bottom_brightness += image_data[row][col];
            bottom_count++;
        }
    }
    
    if (top_count > 0 && bottom_count > 0)
    {
        top_brightness /= top_count;
        bottom_brightness /= bottom_count;
        
        int32 brightness_diff = (int32)top_brightness - (int32)bottom_brightness;
        if (brightness_diff < 0) brightness_diff = -brightness_diff;
        
        if (brightness_diff > RAMP_BRIGHTNESS_CHANGE)
        {
            element_recog.ramp.brightness_changed = 1;
            element_recog.current_element.confidence = 55 + (brightness_diff / 5);
            if (element_recog.current_element.confidence > 80)
                element_recog.current_element.confidence = 80;
            return 1;
        }
    }
    
    // 如果有效行数不足且赛道已找到，认为边缘丢失
    if (vision.track.valid_rows < 8 && vision.track_found)
    {
        element_recog.ramp.edge_lost_count++;
        
        if (element_recog.ramp.edge_lost_count > RAMP_EDGE_DISAPPEAR_ROWS)
        {
            element_recog.current_element.confidence = 55;
            return 1;
        }
    }
    else
    {
        element_recog.ramp.edge_lost_count = 0;
    }
    
    return 0;
}

/**
 * @brief  元素识别停车位
 * @param  无
 * @return 1-识别到停车位元素 0-未识别到停车位元素
 */
uint8 element_detect_parking(void)
{
    // 如果未找到赛道或有效行数不足，返回未识别
    // 计算图像顶部亮度总和
    
    uint8 white_line_count = 0;
    
    // 如果未找到赛道或有效行数不足，返回未识别
    for (uint8 row = 60; row < 80 && row < IMAGE_HEIGHT; row++)
    {
        uint16 white_count = 0;
        uint16 check_width = 0;
        
        // 获取赛道左右边缘位置
        uint8 left = vision.track.left_edge[row];
        uint8 right = vision.track.right_edge[row];
        
        if (left < right && right < IMAGE_WIDTH)
        {
            for (uint8 col = left; col < right; col++)
            {
                check_width++;
                if (image_data[row][col] > PARKING_WHITE_THRESHOLD)
                {
                    white_count++;
                }
            }
            
            // 判断白线比例是否超过阈值
            if (check_width > 0 && white_count > (check_width * 70 / 100))
            {
                white_line_count++;
            }
        }
    }
    
    if (white_line_count >= 3)
    {
        element_recog.parking.white_line_detected = 1;
        element_recog.current_element.confidence = 80 + (white_line_count * 2);
        if (element_recog.current_element.confidence > 95)
            element_recog.current_element.confidence = 95;
        element_recog.current_element.distance = 60;  // 识别到停车位距离设为60厘米
        return 1;
    }
    
    return 0;
}

/**
 * @brief  元素识别障碍物
 * @param  无
 * @return 1-识别到障碍物元素 0-未识别到障碍物元素
 */
uint8 element_detect_obstacle(void)
{
    // 如果未找到赛道或有效行数不足，返回未识别
    if (!vision.track_found || vision.track.valid_rows < 5)
        return 0;
    
    uint16 obstacle_area = 0;
    uint8 obstacle_rows = 0;
    
    for (uint8 row = 40; row < 80 && row < IMAGE_HEIGHT; row++)
    {
        uint8 left = vision.track.left_edge[row];
        uint8 right = vision.track.right_edge[row];
        
        if (left < right && right < IMAGE_WIDTH)
        {
            uint8 center = (left + right) / 2;
            uint8 width = right - left;
            
            // 获取检测区域左右边界
            uint8 check_start = center - width / 6;
            uint8 check_end = center + width / 6;
            
            if (check_end > IMAGE_WIDTH)
                check_end = IMAGE_WIDTH;
            
            uint8 dark_count = 0;
            for (uint8 col = check_start; col < check_end; col++)
            {
                // 判断像素是否为暗色
                if (image_data[row][col] < THRESHOLD_VALUE)
                {
                    dark_count++;
                    obstacle_area++;
                }
            }
            
            // 判断暗色像素是否超过一半
            if (dark_count > (check_end - check_start) / 2)
            {
                obstacle_rows++;
            }
        }
    }
    
    element_recog.obstacle.obstacle_area = obstacle_area;
    
    // 如果暗色像素面积超过阈值且暗色行数足够，认为识别到障碍物
    if (obstacle_area > OBSTACLE_BLACK_AREA_MIN && obstacle_rows >= 5)
    {
        element_recog.current_element.confidence = 60 + (obstacle_rows * 3);
        if (element_recog.current_element.confidence > 90)
            element_recog.current_element.confidence = 90;
        
        // 设置障碍物左右边界为0，表示未检测到具体边界
        element_recog.obstacle.obstacle_left = 0;
        element_recog.obstacle.obstacle_right = 0;
        
        return 1;
    }
    
    return 0;
}

/**
 * @brief  元素识别斑马线
 * @param  无
 * @return 1-识别到斑马线元素 0-未识别到斑马线元素
 */
uint8 element_detect_zebra_crossing(void)
{
    // 如果未找到赛道或有效行数不足，返回未识别
    uint8 stripe_count = 0;
    uint8 last_color = 0;  // 0-黑色 1-白色
    
    for (uint8 row = 50; row < 90; row += 2)
    {
        uint8 white_count = 0;
        uint8 total_count = 0;
        
        for (uint8 col = 40; col < 120; col++)
        {
            if (col < MT9V03X_W && row < MT9V03X_H)
            {
                total_count++;
                if (image_data[row][col] == 255)
                {
                    white_count++;
                }
            }
        }
        
        uint8 current_color = (white_count > total_count / 2) ? 1 : 0;
        
        if (row > 50 && current_color != last_color)
        {
            stripe_count++;
        }
        
        last_color = current_color;
    }
    
    // 如果条纹数量达到阈值，认为识别到斑马线
    if (stripe_count >= 4)
    {
        return 1;
    }
    
    return 0;
}

/**
 * @brief  元素识别十字路口处理
 * @param  无
 * @return 无
 */
void element_handle_cross(void)
{
    // 十字路口识别处理逻辑
    
    element_recog.cross.straight_count++;
    
    // 如果赛道宽度变窄，认为进入十字路口
    uint8 narrow_count = 0;
    for (uint8 row = 60; row < 90 && row < IMAGE_HEIGHT; row++)
    {
        uint16 width = get_track_width(row);
        if (width > 0 && width < CROSS_WIDTH_THRESHOLD)
        {
            narrow_count++;
        }
    }
    
    // 如果窄赛道行数超过阈值且直行计数足够，认为通过十字路口
    if (narrow_count > 20 && element_recog.cross.straight_count > 30)
    {
        element_recog.current_element.state = ELEMENT_STATE_PASSED;
    }
    else if (element_recog.cross.straight_count > 80)  // 超时处理
    {
        element_recog.current_element.state = ELEMENT_STATE_PASSED;
    }
}

/**
 * @brief  元素识别圆形环处理
 * @param  无
 * @return 无
 */
void element_handle_circle(void)
{
    // 圆形环识别处理逻辑
    
    element_recog.current_element.frame_count++;
    
    // 计算当前曲率
    int16 current_curvature = calculate_curvature(30, 80);
    element_recog.circle.curvature = current_curvature;
    
    // 连续小曲率计数
    static uint8 small_curvature_count = 0;
    
    if (abs(current_curvature) < CIRCLE_CURVATURE_THRESHOLD / 3)
    {
        small_curvature_count++;
    }
    else
    {
        small_curvature_count = 0;
    }
    
    // 如果连续小曲率计数超过阈值且帧数足够，认为通过圆形环
    if (small_curvature_count > 15 && element_recog.current_element.frame_count > 80)
    {
        element_recog.circle.continuous_rows = 0;
        small_curvature_count = 0;
        element_recog.current_element.state = ELEMENT_STATE_PASSED;
    }
    else if (element_recog.current_element.frame_count > 200)  // 超时处理
    {
        small_curvature_count = 0;
        element_recog.current_element.state = ELEMENT_STATE_PASSED;
    }
}

/**
 * @brief  元素识别坡道处理
 * @param  无
 * @return 无
 */
void element_handle_ramp(void)
{
    //  坡道识别处理逻辑
    
    element_recog.current_element.frame_count++;
    
    // 如果帧数超过阈值，认为通过坡道
    if (element_recog.current_element.frame_count > 80)
    {
        element_recog.current_element.state = ELEMENT_STATE_PASSED;
    }
}

/**
 * @brief  元素识别停车场处理
 * @param  无
 * @return 无
 */
void element_handle_parking(void)
{
    // 停车场识别处理逻辑
    // 如果识别到停车场，进入停车场状态
    
    element_recog.current_element.frame_count++;
    
    // 如果识别到停车场，进入停车场状态
    // 如果识别到停车场，进入停车场状态
    if (element_recog.current_element.state == ELEMENT_STATE_FOUND)
    {
        element_recog.current_element.state = ELEMENT_STATE_ENTERING;
    }
}

/**
 * @brief  元素识别障碍物处理
 * @param  无
 * @return 无
 */
void element_handle_obstacle(void)
{
    // 障碍物识别处理逻辑
    // 如果识别到障碍物，进入障碍物状态
    
    element_recog.current_element.frame_count++;
    
    // 计算当前障碍物面积
    uint16 current_obstacle_area = 0;
    
    for (uint8 row = 50; row < 90 && row < IMAGE_HEIGHT; row++)
    {
        uint8 left = vision.track.left_edge[row];
        uint8 right = vision.track.right_edge[row];
        
        if (left < right && right < IMAGE_WIDTH)
        {
            uint8 center = (left + right) / 2;
            uint8 width = right - left;
            uint8 check_start = center - width / 6;
            uint8 check_end = center + width / 6;
            
            if (check_end > IMAGE_WIDTH)
                check_end = IMAGE_WIDTH;
            
            for (uint8 col = check_start; col < check_end; col++)
            {
                if (image_data[row][col] < THRESHOLD_VALUE)
                {
                    current_obstacle_area++;
                }
            }
        }
    }
    
    // 如果当前障碍物面积小于阈值且帧数足够，认为通过障碍物
    if (current_obstacle_area < OBSTACLE_BLACK_AREA_MIN / 2 && element_recog.current_element.frame_count > 40)
    {
        element_recog.current_element.state = ELEMENT_STATE_PASSED;
    }
    else if (element_recog.current_element.frame_count > 100)  // 超时处理
    {
        element_recog.current_element.state = ELEMENT_STATE_PASSED;
    }
}

/**
 * @brief  元素识别状态更新
 * @param  无
 * @return 无
 */
void element_update_state(void)
{
    element_recog.current_element.frame_count++;
    
    // 元素状态更新
    switch (element_recog.current_element.state)
    {
        case ELEMENT_STATE_FOUND:
            if (element_recog.current_element.frame_count > 5)
            {
                element_recog.current_element.state = ELEMENT_STATE_ENTERING;
            }
            break;
            
        case ELEMENT_STATE_ENTERING:
            if (element_recog.current_element.frame_count > 15)
            {
                element_recog.current_element.state = ELEMENT_STATE_IN_ELEMENT;
            }
            break;
            
        case ELEMENT_STATE_IN_ELEMENT:
            //  元素状态更新逻辑，可能包括从IN_ELEMENT状态转变为LEAVING或PASSED
            break;
            
        case ELEMENT_STATE_LEAVING:
            if (element_recog.current_element.frame_count > 10)
            {
                element_recog.current_element.state = ELEMENT_STATE_PASSED;
            }
            break;
            
        default:
            break;
    }
}

/**
 * @brief  元素是否为当前识别元素
 * @param  type  元素类型
 * @return 1-是 0-否
 */
uint8 element_is_current(element_type_enum type)
{
    return (element_recog.current_element.type == type);
}

/**
 * @brief  元素重置
 * @param  无
 * @return 无
 */
void element_reset(void)
{
    memset(&element_recog.current_element, 0, sizeof(element_info_t));
    element_recog.current_element.type = ELEMENT_NONE;
    element_recog.current_element.state = ELEMENT_STATE_NONE;
    
    // 元素数据重置
    memset(&element_recog.cross, 0, sizeof(element_recog.cross));
    memset(&element_recog.circle, 0, sizeof(element_recog.circle));
    memset(&element_recog.ramp, 0, sizeof(element_recog.ramp));
    memset(&element_recog.parking, 0, sizeof(element_recog.parking));
    memset(&element_recog.obstacle, 0, sizeof(element_recog.obstacle));
}

/**
 * @brief  元素名称获取
 * @param  type  元素类型
 * @return 元素名称字符串
 */
const char* element_get_name(element_type_enum type)
{
    switch (type)
    {
        case ELEMENT_NONE:              return "无";
        case ELEMENT_CROSS:             return "十字路口";
        case ELEMENT_CIRCLE:            return "环岛";
        case ELEMENT_RAMP:              return "坡道";
        case ELEMENT_OBSTACLE:          return "障碍物";
        case ELEMENT_PARKING:           return "停车位";
        case ELEMENT_ZEBRA_CROSSING:    return "斑马线";
        case ELEMENT_SPEED_BUMP:        return "减速带";
        case ELEMENT_FORK:              return "分叉路口";
        default:                        return "未知";
    }
}
