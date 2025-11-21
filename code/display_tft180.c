#include "display_tft180.h"

//====================================================显示相关函数====================================================
/**
 * @brief  显示FT180信息函数
 * @param  无
 * @return 无
 */
void smart_car_display_info_tft180(void)
{
    // 清屏，准备显示信息
    // tft180_clear();
    
    // 设置字体和颜色
    tft180_set_font(TFT180_8X16_FONT);
    tft180_set_color(RGB565_WHITE, RGB565_BLACK);
    
    // 显示标题
    tft180_show_string(0, 0, "Smart Car v1.4");
    
    // 状态显示
    tft180_show_string(0, 20, "State:");
    switch(smart_car.state)
    {
        case CAR_STOP:
            tft180_show_string(60, 20, "STOP ");
            break;
        case CAR_RUNNING:
            tft180_show_string(60, 20, "RUN  ");
            break;
        case CAR_PAUSE:
            tft180_show_string(60, 20, "PAUSE");
            break;
        default:
            tft180_show_string(60, 20, "DEBUG");
            break;
    }
    
    // 左轮速度显示
    tft180_show_string(0, 40, "L_Speed:");
    tft180_show_int(80, 40, car.left_motor.encoder_count, 5);
    
    tft180_show_string(0, 56, "R_Speed:");
    tft180_show_int(80, 56, car.right_motor.encoder_count, 5);
    
    // 转向角度显示
    tft180_show_string(0, 72, "Angle:");
    tft180_show_int(60, 72, car.steering_servo.current_angle, 4);
    
    // 偏差显示
    tft180_show_string(0, 88, "Error:");
    tft180_show_int(60, 88, vision_get_deviation(), 4);
    
    // 位置控制使能显示
    if (smart_car.position_control_enable)
    {
        tft180_show_string(0, 104, "Dist:");
        tft180_show_float(50, 104, position_get_current_distance(), 2, 2);
        tft180_show_string(110, 104, "m");
    }
    

}

/**
 * @brief  显示FT180图像函数
 * @param  无
 * @return 无
 */
void vision_show_image_tft180(void)
{
    // MT9V03X摄像头图像显示
    // TFT180屏幕分辨率160x128，MT9V03X摄像头分辨率188x120
    tft180_displayimage03x((const uint8 *)image_data, 160, 128);
    
    // 设置颜色为红色，准备绘制轨迹线
    tft180_set_color(RGB565_RED, RGB565_BLACK);

    // 显示偏差值
    tft180_show_string(0, 110, "Dev:");
    tft180_show_int(40, 110, vision_get_deviation(), 4);

    // 轨迹状态显示
    if (vision.track_found)
    {
        tft180_set_color(RGB565_GREEN, RGB565_BLACK);
        tft180_show_string(90, 110, "FOUND");
    }
    else
    {
        tft180_set_color(RGB565_RED, RGB565_BLACK);
        tft180_show_string(90, 110, "LOST ");
    }
}

/**
 * @brief  坐标转换辅助函数（图像坐标到屏幕坐标）
 * @param  img_x  图像X坐标 (0-187)
 * @param  img_y  图像Y坐标 (0-119)
 * @param  scr_x  屏幕X坐标指针
 * @param  scr_y  屏幕Y坐标指针
 * @return 无
 */
static void image_to_screen_coord(uint8 img_x, uint8 img_y, uint8 *scr_x, uint8 *scr_y)
{
    // 图像188x120 -> 屏幕160x128
    uint16 temp_x = (img_x * 160) / 188;
    uint16 temp_y = (img_y * 128) / 120;
    
    // 边界限制
    *scr_x = (temp_x > 159) ? 159 : (uint8)temp_x;
    *scr_y = (temp_y > 127) ? 127 : (uint8)temp_y;
}

/**
 * @brief  显示FT180图像函数，带轨迹线显示（优化版）
 * @param  无
 * @return 无
 * @note   优化了绘制性能，添加连线效果，增强可视化调试
 */
void vision_show_image_with_lines_tft180(void)
{
    uint8 row;
    uint8 last_left_x = 0, last_left_y = 0;
    uint8 last_right_x = 0, last_right_y = 0;
    uint8 last_center_x = 0, last_center_y = 0;
    uint8 first_point = 1;
    
    // 显示摄像头图像
    tft180_displayimage03x((const uint8 *)image_data, 160, 128);
    
    // 绘制扫描区域标记线（半透明效果，使用虚线）
    uint8 scan_start_y, scan_end_y;
    image_to_screen_coord(0, SCAN_START_ROW, &scan_start_y, &scan_start_y);
    image_to_screen_coord(0, SCAN_END_ROW, &scan_end_y, &scan_end_y);
    
    // 绘制扫描起始线（黄色虚线）
    for (uint8 x = 0; x < 160; x += 4)
    {
        tft180_draw_point(x, scan_start_y, RGB565_YELLOW);
    }
    
    // 绘制扫描结束线（黄色虚线）
    for (uint8 x = 0; x < 160; x += 4)
    {
        tft180_draw_point(x, scan_end_y, RGB565_YELLOW);
    }
    
    // 轨迹线绘制
    if (vision.track_found && vision.track.valid_rows > 0)
    {
        // 从底部向上遍历扫描行，绘制连续轨迹线
        for (row = SCAN_START_ROW; row > SCAN_END_ROW; row -= SCAN_STEP)
        {
            // 只绘制轨迹宽度合理的行
            if (vision.track.track_width[row] >= TRACK_WIDTH_MIN &&
                vision.track.track_width[row] <= TRACK_WIDTH_MAX)
            {
                uint8 left_x, left_y, right_x, right_y, center_x, center_y;
                
                // 坐标转换
                image_to_screen_coord(vision.track.left_edge[row], row, &left_x, &left_y);
                image_to_screen_coord(vision.track.right_edge[row], row, &right_x, &right_y);
                image_to_screen_coord(vision.track.center_line[row], row, &center_x, &center_y);
                
                if (first_point)
                {
                    // 第一个点，只绘制点
                    tft180_draw_point(left_x, left_y, RGB565_RED);
                    tft180_draw_point(right_x, right_y, RGB565_BLUE);
                    tft180_draw_point(center_x, center_y, RGB565_GREEN);
                    first_point = 0;
                }
                else
                {
                    // 绘制连线（更流畅的轨迹显示）
                    // 左边缘：红色
                    if (left_y != last_left_y || left_x != last_left_x)
                    {
                        tft180_draw_line(last_left_x, last_left_y, left_x, left_y, RGB565_RED);
                    }
                    
                    // 右边缘：蓝色
                    if (right_y != last_right_y || right_x != last_right_x)
                    {
                        tft180_draw_line(last_right_x, last_right_y, right_x, right_y, RGB565_BLUE);
                    }
                    
                    // 中心线：绿色（加粗显示）
                    if (center_y != last_center_y || center_x != last_center_x)
                    {
                        tft180_draw_line(last_center_x, last_center_y, center_x, center_y, RGB565_GREEN);
                        // 加粗中心线
                        if (center_x > 0)
                            tft180_draw_point(center_x - 1, center_y, RGB565_GREEN);
                        if (center_x < 159)
                            tft180_draw_point(center_x + 1, center_y, RGB565_GREEN);
                    }
                }
                
                // 保存当前点坐标
                last_left_x = left_x;
                last_left_y = left_y;
                last_right_x = right_x;
                last_right_y = right_y;
                last_center_x = center_x;
                last_center_y = center_y;
            }
        }
        
        // 绘制图像中心参考线（白色虚线）
        uint8 center_ref_x;
        image_to_screen_coord(IMAGE_WIDTH / 2, 0, &center_ref_x, &scan_start_y);
        for (uint8 y = scan_end_y; y < scan_start_y; y += 3)
        {
            tft180_draw_point(center_ref_x, y, RGB565_WHITE);
        }
    }
    
    // 信息显示区域（使用半透明背景效果）
    // 绘制信息背景条（深色背景便于文字显示）
    for (uint8 y = 85; y < 128; y++)
    {
        for (uint8 x = 0; x < 160; x += 2)
        {
            if ((x + y) % 4 == 0)
                tft180_draw_point(x, y, 0x0000);  // 黑色背景
        }
    }
    
    // 设置颜色为白色，显示文本信息
    tft180_set_color(RGB565_WHITE, RGB565_BLACK);
    
    // 第一行：偏差值和轨道状态
    tft180_show_string(0, 90, "E:");
    tft180_show_int(20, 90, vision.error, 4);
    
    // 显示归一化偏差
    tft180_show_string(65, 90, "D:");
    if (vision.deviation >= 0)
    {
        tft180_show_float(85, 90, vision.deviation, 1, 2);
    }
    else
    {
        tft180_show_float(80, 90, vision.deviation, 1, 2);
    }
    
    // 第二行：有效行数和轨道宽度
    tft180_show_string(0, 104, "R:");
    tft180_show_int(20, 104, vision.track.valid_rows, 3);
    
    // 显示平均轨道宽度
    if (vision.track.valid_rows > 0)
    {
        uint16 avg_width = 0;
        uint8 count = 0;
        for (row = SCAN_START_ROW; row > SCAN_END_ROW; row -= SCAN_STEP)
        {
            if (vision.track.track_width[row] >= TRACK_WIDTH_MIN &&
                vision.track.track_width[row] <= TRACK_WIDTH_MAX)
            {
                avg_width += vision.track.track_width[row];
                count++;
            }
        }
        if (count > 0)
        {
            avg_width /= count;
            tft180_show_string(55, 104, "W:");
            tft180_show_int(75, 104, avg_width, 3);
        }
    }
    
    // 第三行：轨迹状态指示
    tft180_show_string(0, 118, "Track:");
    if (vision.track_found)
    {
        tft180_set_color(RGB565_GREEN, RGB565_BLACK);
        tft180_show_string(50, 118, "OK ");
        
        // 显示轨迹质量指示
        uint8 quality = (vision.track.valid_rows * 100) / (SCAN_START_ROW - SCAN_END_ROW);
        tft180_set_color(RGB565_CYAN, RGB565_BLACK);
        tft180_show_int(75, 118, quality, 3);
        tft180_show_string(100, 118, "%");
    }
    else
    {
        tft180_set_color(RGB565_RED, RGB565_BLACK);
        tft180_show_string(50, 118, "LOST");
    }
}

/**
 * @brief  显示FT180路径规划信息函数
 * @param  无
 * @return 无
 */
void display_path_info_tft180(void)
{
    if (!smart_car.path_planning_enable)
    {
        return;
    }
    
    tft180_set_font(TFT180_8X16_FONT);
    tft180_set_color(RGB565_CYAN, RGB565_BLACK);
    
    // 路径规划标题
    tft180_show_string(0, 0, "Path Planning");
    
    // 路径状态
    tft180_show_string(0, 20, "State:");
    switch(path_planner.state)
    {
        case PATH_STATE_IDLE:
            tft180_show_string(60, 20, "IDLE    ");
            break;
        case PATH_STATE_PLANNING:
            tft180_show_string(60, 20, "PLAN    ");
            break;
        case PATH_STATE_EXECUTING:
            tft180_show_string(60, 20, "EXEC    ");
            break;
        case PATH_STATE_COMPLETED:
            tft180_show_string(60, 20, "COMPLETE");
            break;
        default:
            tft180_show_string(60, 20, "FAILED  ");
            break;
    }
    
    // 显示路径节点数
    tft180_show_string(0, 40, "Nodes:");
    tft180_show_int(60, 40, path_planner.node_count, 2);
    
    tft180_show_string(0, 56, "Current:");
    tft180_show_int(80, 56, path_planner.current_node, 2);
    
    // 显示路径进度
    tft180_show_string(0, 72, "Progress:");
    tft180_show_int(90, 72, path_get_progress(), 3);
    tft180_show_string(120, 72, "%");
    
    // 显示当前路径节点类型
    if (path_planner.current_node < path_planner.node_count)
    {
        path_node_t* node = &path_planner.nodes[path_planner.current_node];
        tft180_show_string(0, 88, "Type:");
        const char* type_name = path_get_type_name(node->type);
        tft180_show_string(50, 88, type_name);
    }
    
    // 轨迹统计信息
    tft180_show_string(0, 104, "Circle:");
    tft180_show_int(70, 104, path_planner.circle_count, 2);
    
    tft180_show_string(0, 120, "Cross:");
    tft180_show_int(60, 120, path_planner.cross_count, 2);
    
    tft180_show_string(90, 120, "Obs:");
    tft180_show_int(130, 120, path_planner.obstacle_count, 2);
}

/**
 * @brief  显示FT180元素识别信息函数
 * @param  无
 * @return 无
 */
void display_element_info_tft180(void)
{
    if (!smart_car.element_recognition_enable)
    {
        return;
    }
    
    tft180_set_font(TFT180_8X16_FONT);
    tft180_set_color(RGB565_YELLOW, RGB565_BLACK);
    
    // 元素识别标题
    tft180_show_string(0, 0, "Element Recog");
    
    // 元素检测状态
    if (element_recog.current_element.detected)
    {
        tft180_show_string(0, 20, "Element:");
        const char* elem_name = element_get_name(element_recog.current_element.type);
        tft180_show_string(80, 20, elem_name);
        
        // 元素状态
        tft180_show_string(0, 40, "State:");
        switch(element_recog.current_element.state)
        {
            case ELEMENT_STATE_NONE:
                tft180_show_string(60, 40, "NONE    ");
                break;
            case ELEMENT_STATE_FOUND:
                tft180_show_string(60, 40, "FOUND   ");
                break;
            case ELEMENT_STATE_ENTERING:
                tft180_show_string(60, 40, "ENTERING");
                break;
            case ELEMENT_STATE_IN_ELEMENT:
                tft180_show_string(60, 40, "IN_ELEM ");
                break;
            case ELEMENT_STATE_LEAVING:
                tft180_show_string(60, 40, "LEAVING ");
                break;
            case ELEMENT_STATE_PASSED:
                tft180_show_string(60, 40, "PASSED  ");
                break;
            default:
                tft180_show_string(60, 40, "UNKNOWN ");
                break;
        }
        
        // 显示置信度
        tft180_show_string(0, 60, "Confidence:");
        tft180_show_int(110, 60, element_recog.current_element.confidence, 3);
    }
    else
    {
        tft180_show_string(0, 20, "No Element Detected");
    }
    
    // 元素统计信息
    tft180_show_string(0, 80, "=== Statistics ===");

    tft180_show_string(0, 96, "Circle:");
    //tft180_show_int(70, 96, element_recog.circle.count, 2);

    tft180_show_string(0, 112, "Cross:");
    //tft180_show_int(60, 112, element_recog.cross.count, 2);

    tft180_show_string(90, 112, "Obs:");
    //tft180_show_int(130, 112, element_recog.obstacle.count, 2);
}
