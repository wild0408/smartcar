#include "path_planning.h"
#include "smart_car.h"
#include <string.h>
#include <math.h>

//====================================================路径规划器====================================================
path_planner_t path_planner;

//====================================================路径规划函数====================================================
/**
 * @brief  路径规划初始化
 * @param  无
 * @return 无
 */
void path_planning_init(void)
{
    memset(&path_planner, 0, sizeof(path_planner_t));
    
    path_planner.state = PATH_STATE_IDLE;
    path_planner.mode = DECISION_MODE_AUTO;
    
    // 距离权重、时间权重、风险权重初始化
    path_planner.distance_weight = 0.4f;
    path_planner.time_weight = 0.3f;
    path_planner.risk_weight = 0.3f;
    
    // 偏好权重初始化
    path_planner.prefer_left = 50;
    path_planner.prefer_right = 50;
}

/**
 * @brief  路径规划重置
 * @param  无
 * @return 无
 */
void path_planning_reset(void)
{
    path_clear_nodes();
    path_planner.state = PATH_STATE_IDLE;
    path_planner.current_node = 0;
}

/**
 * @brief  路径规划处理函数
 * @param  element  元素类型
 * @return 无
 */
void path_plan_for_element(element_type_enum element)
{
    if (path_planner.mode == DECISION_MODE_MANUAL)
    {
        return;  // 手动模式下不进行路径规划
    }
    
    path_clear_nodes();
    path_planner.state = PATH_STATE_PLANNING;
    
    switch (element)
    {
        case ELEMENT_CIRCLE:
            // 圆形元素处理
            if (element_recog.circle.direction == 0)
            {
                // 左转圆形
                if (path_planner.mode == DECISION_MODE_OPTIMAL)
                {
                    // 路径选择优先级设置
                    path_node_t options[2];
                    options[0].type = PATH_LEFT_CIRCLE;
                    options[0].priority = 50 + (100 - path_planner.prefer_left);
                    options[1].type = PATH_RIGHT_CIRCLE;
                    options[1].priority = 50 + (100 - path_planner.prefer_right);
                    
                    path_type_enum selected = path_select_optimal(options, 2);
                    if (selected == PATH_LEFT_CIRCLE)
                        path_preset_left_circle();
                    else
                        path_preset_right_circle();
                }
                else
                {
                    path_preset_left_circle();
                }
            }
            else
            {
                // 右转圆形
                path_preset_right_circle();
            }
            path_planner.circle_count++;
            break;
            
        case ELEMENT_CROSS:
            // 十字路口处理
            if (path_planner.mode == DECISION_MODE_OPTIMAL)
            {
                // 路径选择优先级设置
                path_node_t options[3];
                options[0].type = PATH_CROSS_STRAIGHT;
                options[0].priority = 30;  // 直行优先级
                options[1].type = PATH_CROSS_LEFT;
                options[1].priority = 50 + (100 - path_planner.prefer_left);
                options[2].type = PATH_CROSS_RIGHT;
                options[2].priority = 50 + (100 - path_planner.prefer_right);
                
                path_type_enum selected = path_select_optimal(options, 3);
                
                if (selected == PATH_CROSS_STRAIGHT)
                    path_preset_cross_straight();
                else if (selected == PATH_CROSS_LEFT)
                    path_preset_cross_left();
                else
                    path_preset_cross_right();
            }
            else
            {
                path_preset_cross_straight();  // 直行优先
            }
            path_planner.cross_count++;
            break;
            
        case ELEMENT_OBSTACLE:
            // 障碍物处理
            {
                path_type_enum avoid_type = path_decide_obstacle();
                uint8 side = (avoid_type == PATH_AVOID_LEFT) ? 0 : 1;
                path_preset_avoid_obstacle(side);
            }
            path_planner.obstacle_count++;
            break;
            
        case ELEMENT_RAMP:
            // 坡道元素处理
            {
                path_node_t node;
                node.type = PATH_RAMP;
                node.element = ELEMENT_RAMP;
                node.distance = 3.0f;
                node.duration = 3000;
                node.target_speed = (int16)(BASE_SPEED * 1.3f);
                node.target_angle = 0;
                node.priority = 10;
                path_add_node(node);
            }
            break;
            
        case ELEMENT_PARKING:
            // 停车元素处理
            {
                path_node_t node;
                node.type = PATH_PARKING;
                node.element = ELEMENT_PARKING;
                node.distance = 0.5f;
                node.duration = 0;
                node.target_speed = 0;
                node.target_angle = 0;
                node.priority = 0;  // 停车优先级最高
                path_add_node(node);
            }
            break;
            
        default:
            break;
    }
    
    if (path_planner.node_count > 0)
    {
        path_planner.state = PATH_STATE_EXECUTING;
        path_planner.start_time = system_getval_ms();
        path_planner.start_position = position_get_current_distance();
    }
}

/**
 * @brief  添加路径节点
 * @param  node  路径节点
 * @return 无
 */
void path_add_node(path_node_t node)
{
    if (path_planner.node_count < 20)
    {
        path_planner.nodes[path_planner.node_count++] = node;
    }
}

/**
 * @brief  清除路径节点
 * @param  无
 * @return 无
 */
void path_clear_nodes(void)
{
    path_planner.node_count = 0;
    path_planner.current_node = 0;
}

/**
 * @brief  执行当前路径节点
 * @param  无
 * @return 无
 */
void path_execute_current_node(void)
{
    if (path_planner.current_node >= path_planner.node_count)
    {
        path_planner.state = PATH_STATE_COMPLETED;
        return;
    }
    
    path_node_t* node = &path_planner.nodes[path_planner.current_node];
    
    // 路径类型处理
    switch (node->type)
    {
        case PATH_STRAIGHT:
            car_forward(node->target_speed);
            break;
            
        case PATH_LEFT_TURN:
            car_turn(node->target_speed, node->target_angle);
            break;
            
        case PATH_RIGHT_TURN:
            car_turn(node->target_speed, node->target_angle);
            break;
            
        case PATH_LEFT_CIRCLE:
        case PATH_RIGHT_CIRCLE:
            // 圆形路径处理
            break;
            
        case PATH_AVOID_LEFT:
        case PATH_AVOID_RIGHT:
            car_turn(node->target_speed, node->target_angle);
            break;
            
        case PATH_RAMP:
            car_forward(node->target_speed);
            break;
            
        case PATH_PARKING:
            // 停车元素处理
            position_stop_at(node->distance);
            smart_car_enable_position_control();
            break;
            
        default:
            break;
    }
}

/**
 * @brief  判断路径节点是否完成
 * @param  无
 * @return 1-完成 0-未完成
 */
uint8 path_is_node_completed(void)
{
    if (path_planner.current_node >= path_planner.node_count)
    {
        return 1;
    }
    
    path_node_t* node = &path_planner.nodes[path_planner.current_node];
    
    // 距离判断
    if (node->distance > 0)
    {
        float traveled = position_get_current_distance() - path_planner.start_position;
        if (traveled >= node->distance)
        {
            return 1;
        }
    }
    
    // 时间判断
    if (node->duration > 0)
    {
        uint32 elapsed = system_getval_ms() - path_planner.start_time;
        if (elapsed >= node->duration)
        {
            return 1;
        }
    }
    
    // 元素判断
    if (node->element != ELEMENT_NONE)
    {
        if (element_recog.current_element.type == node->element)
        {
            if (element_recog.current_element.state == ELEMENT_STATE_PASSED)
            {
                return 1;
            }
        }
    }
    
    return 0;
}

/**
 * @brief  切换到下一个路径节点
 * @param  无
 * @return 无
 */
void path_next_node(void)
{
    path_planner.current_node++;
    path_planner.start_time = system_getval_ms();
    path_planner.start_position = position_get_current_distance();
}

/**
 * @brief  决定圆形路径
 * @param  direction  方向 0-左 1-右
 * @return 路径类型
 */
path_type_enum path_decide_circle(uint8 direction)
{
    if (direction == 0)
    {
        // 左转优先判断
        if (path_planner.prefer_left > path_planner.prefer_right)
        {
            return PATH_LEFT_CIRCLE;
        }
        else
        {
            return PATH_RIGHT_CIRCLE;  // 右转优先判断
        }
    }
    else
    {
        return PATH_RIGHT_CIRCLE;
    }
}

/**
 * @brief  决定十字路口路径
 * @param  无
 * @return 路径类型
 */
path_type_enum path_decide_cross(void)
{
    // 路径选择优先级判断
    // 根据当前十字路口数量和偏好值决定路径类型
    
    if (path_planner.cross_count == 0)
    {
        return PATH_CROSS_STRAIGHT;
    }
    else if (path_planner.cross_count == 1)
    {
        // 路径选择优先级判断
        if (path_planner.prefer_left > 70)
            return PATH_CROSS_LEFT;
        else if (path_planner.prefer_right > 70)
            return PATH_CROSS_RIGHT;
        else
            return PATH_CROSS_STRAIGHT;
    }
    else
    {
        return PATH_CROSS_STRAIGHT;
    }
}

/**
 * @brief  决定障碍物路径
 * @param  无
 * @return 路径类型
 */
path_type_enum path_decide_obstacle(void)
{
    // 障碍物路径优先判断
    // 根据当前障碍物数量和偏好值决定路径类型
    
    if (path_planner.prefer_left > path_planner.prefer_right)
    {
        return PATH_AVOID_LEFT;
    }
    else
    {
        return PATH_AVOID_RIGHT;
    }
}

/**
 * @brief  计算路径节点的代价
 * @param  node  路径节点指针
 * @return 路径代价
 */
path_cost_t path_calculate_cost(path_node_t* node)
{
    path_cost_t cost;
    
    // 距离代价
    cost.distance_cost = node->distance * path_planner.distance_weight;
    
    // 时间代价
    if (node->target_speed > 0)
    {
        float time = node->distance / (node->target_speed / 1000.0f);
        cost.time_cost = time * path_planner.time_weight;
    }
    else
    {
        cost.time_cost = node->duration * path_planner.time_weight;
    }
    
    // 风险代价
    switch (node->type)
    {
        case PATH_STRAIGHT:
            cost.risk_cost = 1.0f;
            break;
        case PATH_LEFT_TURN:
        case PATH_RIGHT_TURN:
            cost.risk_cost = 1.5f;
            break;
        case PATH_LEFT_CIRCLE:
        case PATH_RIGHT_CIRCLE:
            cost.risk_cost = 2.0f;
            break;
        case PATH_AVOID_LEFT:
        case PATH_AVOID_RIGHT:
            cost.risk_cost = 2.5f;
            break;
        default:
            cost.risk_cost = 1.0f;
            break;
    }
    cost.risk_cost *= path_planner.risk_weight;
    
    // 总代价
    cost.total_cost = cost.distance_cost + cost.time_cost + cost.risk_cost + (node->priority * 0.01f);
    
    return cost;
}

/**
 * @brief  选择最优路径
 * @param  options  路径选项数组指针
 * @param  count    选项数量
 * @return 路径类型
 */
path_type_enum path_select_optimal(path_node_t* options, uint8 count)
{
    if (count == 0) return PATH_STRAIGHT;
    
    uint8 best_index = 0;
    float min_cost = 9999.9f;
    
    for (uint8 i = 0; i < count; i++)
    {
        path_cost_t cost = path_calculate_cost(&options[i]);
        
        if (cost.total_cost < min_cost)
        {
            min_cost = cost.total_cost;
            best_index = i;
        }
    }
    
    return options[best_index].type;
}

/**
 * @brief  自动路径决策
 * @param  无
 * @return 无
 */
void path_auto_decision(void)
{
    // 当前元素检测状态判断
    if (element_recog.current_element.detected)
    {
        if (element_recog.current_element.state == ELEMENT_STATE_FOUND)
        {
            path_plan_for_element(element_recog.current_element.type);
        }
    }
    
    // 当前路径执行状态判断
    if (path_planner.state == PATH_STATE_EXECUTING)
    {
        path_execute_current_node();
        
        if (path_is_node_completed())
        {
            path_next_node();
        }
    }
}

/**
 * @brief  更新路径偏好
 * @param  无
 * @return 无
 */
void path_update_preference(void)
{
    // 当前路径偏好更新
    // 根据传感器数据和环境信息调整路径偏好值
    
    // 当前路径偏好更新逻辑
    // 根据传感器数据和环境信息调整路径偏好值
}

/**
 * @brief  预设左圆路径
 * @param  无
 * @return 无
 */
void path_preset_left_circle(void)
{
    path_clear_nodes();
    
    // 距离代价
    path_node_t node1;
    node1.type = PATH_STRAIGHT;
    node1.element = ELEMENT_NONE;
    node1.distance = 0.3f;
    node1.target_speed = (int16)(BASE_SPEED * 0.8f);
    node1.target_angle = 0;
    node1.priority = 10;
    path_add_node(node1);
    
    // 风险代价
    path_node_t node2;
    node2.type = PATH_LEFT_CIRCLE;
    node2.element = ELEMENT_CIRCLE;
    node2.distance = 2.0f;
    node2.target_speed = (int16)(BASE_SPEED * 0.7f);
    node2.target_angle = -25;
    node2.priority = 20;
    path_add_node(node2);
    
    // 距离代价
    path_node_t node3;
    node3.type = PATH_STRAIGHT;
    node3.element = ELEMENT_NONE;
    node3.distance = 0.5f;
    node3.target_speed = BASE_SPEED;
    node3.target_angle = 0;
    node3.priority = 30;
    path_add_node(node3);
}

/**
 * @brief  预设右圆路径
 * @param  无
 * @return 无
 */
void path_preset_right_circle(void)
{
    path_clear_nodes();
    
    path_node_t node1;
    node1.type = PATH_STRAIGHT;
    node1.element = ELEMENT_NONE;
    node1.distance = 0.3f;
    node1.target_speed = (int16)(BASE_SPEED * 0.8f);
    node1.target_angle = 0;
    node1.priority = 10;
    path_add_node(node1);
    
    path_node_t node2;
    node2.type = PATH_RIGHT_CIRCLE;
    node2.element = ELEMENT_CIRCLE;
    node2.distance = 2.0f;
    node2.target_speed = (int16)(BASE_SPEED * 0.7f);
    node2.target_angle = 25;
    node2.priority = 20;
    path_add_node(node2);
    
    path_node_t node3;
    node3.type = PATH_STRAIGHT;
    node3.element = ELEMENT_NONE;
    node3.distance = 0.5f;
    node3.target_speed = BASE_SPEED;
    node3.target_angle = 0;
    node3.priority = 30;
    path_add_node(node3);
}

/**
 * @brief  预设十字直行路径
 * @param  无
 * @return 无
 */
void path_preset_cross_straight(void)
{
    path_clear_nodes();
    
    path_node_t node;
    node.type = PATH_CROSS_STRAIGHT;
    node.element = ELEMENT_CROSS;
    node.distance = 1.5f;
    node.target_speed = BASE_SPEED;
    node.target_angle = 0;
    node.priority = 10;
    path_add_node(node);
}

/**
 * @brief  预设十字左转路径 
 * @param  无
 * @return 无
 */
void path_preset_cross_left(void)
{
    path_clear_nodes();
    
    // 预设十字左转路径
    path_node_t node1;
    node1.type = PATH_STRAIGHT;
    node1.element = ELEMENT_NONE;
    node1.distance = 0.5f;
    node1.target_speed = (int16)(BASE_SPEED * 0.8f);
    node1.target_angle = 0;
    node1.priority = 10;
    path_add_node(node1);
    
    // 预设十字左转路径
    path_node_t node2;
    node2.type = PATH_CROSS_LEFT;
    node2.element = ELEMENT_CROSS;
    node2.distance = 0.8f;
    node2.target_speed = (int16)(BASE_SPEED * 0.6f);
    node2.target_angle = -30;
    node2.priority = 20;
    path_add_node(node2);
    
    // 预设十字左转路径
    path_node_t node3;
    node3.type = PATH_STRAIGHT;
    node3.element = ELEMENT_NONE;
    node3.distance = 0.5f;
    node3.target_speed = BASE_SPEED;
    node3.target_angle = 0;
    node3.priority = 30;
    path_add_node(node3);
}

/**
 * @brief  预设十字右转路径
 * @param  无
 * @return 无
 */
void path_preset_cross_right(void)
{
    path_clear_nodes();
    
    path_node_t node1;
    node1.type = PATH_STRAIGHT;
    node1.element = ELEMENT_NONE;
    node1.distance = 0.5f;
    node1.target_speed = (int16)(BASE_SPEED * 0.8f);
    node1.target_angle = 0;
    node1.priority = 10;
    path_add_node(node1);
    
    path_node_t node2;
    node2.type = PATH_CROSS_RIGHT;
    node2.element = ELEMENT_CROSS;
    node2.distance = 0.8f;
    node2.target_speed = (int16)(BASE_SPEED * 0.6f);
    node2.target_angle = 30;
    node2.priority = 20;
    path_add_node(node2);
    
    path_node_t node3;
    node3.type = PATH_STRAIGHT;
    node3.element = ELEMENT_NONE;
    node3.distance = 0.5f;
    node3.target_speed = BASE_SPEED;
    node3.target_angle = 0;
    node3.priority = 30;
    path_add_node(node3);
}

/**
 * @brief  预设避障路径
 * @param  side  侧面 0-左侧 1-右侧
 * @return 无
 */
void path_preset_avoid_obstacle(uint8 side)
{
    path_clear_nodes();
    
    if (side == 0)  // 左侧避障
    {
        // 风险代价
        path_node_t node1;
        node1.type = PATH_AVOID_LEFT;
        node1.element = ELEMENT_OBSTACLE;
        node1.distance = 0.5f;
        node1.target_speed = (int16)(BASE_SPEED * 0.6f);
        node1.target_angle = -20;
        node1.priority = 10;
        path_add_node(node1);
        
        // 距离代价
        path_node_t node2;
        node2.type = PATH_STRAIGHT;
        node2.element = ELEMENT_NONE;
        node2.distance = 0.8f;
        node2.target_speed = (int16)(BASE_SPEED * 0.7f);
        node2.target_angle = 0;
        node2.priority = 20;
        path_add_node(node2);
        
        // 距离代价
        path_node_t node3;
        node3.type = PATH_RIGHT_TURN;
        node3.element = ELEMENT_NONE;
        node3.distance = 0.5f;
        node3.target_speed = (int16)(BASE_SPEED * 0.6f);
        node3.target_angle = 20;
        node3.priority = 30;
        path_add_node(node3);
    }
    else  // 右侧避障
    {
        path_node_t node1;
        node1.type = PATH_AVOID_RIGHT;
        node1.element = ELEMENT_OBSTACLE;
        node1.distance = 0.5f;
        node1.target_speed = (int16)(BASE_SPEED * 0.6f);
        node1.target_angle = 20;
        node1.priority = 10;
        path_add_node(node1);
        
        path_node_t node2;
        node2.type = PATH_STRAIGHT;
        node2.element = ELEMENT_NONE;
        node2.distance = 0.8f;
        node2.target_speed = (int16)(BASE_SPEED * 0.7f);
        node2.target_angle = 0;
        node2.priority = 20;
        path_add_node(node2);
        
        path_node_t node3;
        node3.type = PATH_LEFT_TURN;
        node3.element = ELEMENT_NONE;
        node3.distance = 0.5f;
        node3.target_speed = (int16)(BASE_SPEED * 0.6f);
        node3.target_angle = -20;
        node3.priority = 30;
        path_add_node(node3);
    }
}

/**
 * @brief  预设路径类型名称
 * @param  type  路径类型
 * @return 路径类型名称
 */
const char* path_get_type_name(path_type_enum type)
{
    switch (type)
    {
        case PATH_STRAIGHT:         return "直行";
        case PATH_LEFT_TURN:        return "左转";
        case PATH_RIGHT_TURN:       return "右转";
        case PATH_LEFT_CIRCLE:      return "左环";
        case PATH_RIGHT_CIRCLE:     return "右环";
        case PATH_CROSS_STRAIGHT:   return "十字直行";
        case PATH_CROSS_LEFT:       return "十字左转";
        case PATH_CROSS_RIGHT:      return "十字右转";
        case PATH_AVOID_LEFT:       return "左侧避障";
        case PATH_AVOID_RIGHT:      return "右侧避障";
        case PATH_RAMP:             return "坡道";
        case PATH_PARKING:          return "停车";
        default:                    return "未知";
    }
}

/**
 * @brief  预设路径打印
 * @param  无
 * @return 无
 */
void path_print_plan(void)
{
    printf("\n=== 预设路径计划 ===\n");
    printf("节点数量: %d\n", path_planner.node_count);
    printf("当前节点: %d\n", path_planner.current_node);
    printf("状态: %d\n", path_planner.state);
    
    for (uint8 i = 0; i < path_planner.node_count; i++)
    {
        path_node_t* node = &path_planner.nodes[i];
        printf("节点%d: %s, 距离=%.2fm, 目标速度=%d, 目标角度=%d\n",
               i, path_get_type_name(node->type),
               node->distance, node->target_speed, node->target_angle);
    }
    printf("===============\n\n");
}

/**
 * @brief  获取路径进度
 * @param  无
 * @return 进度百分比，范围0-100
 */
uint8 path_get_progress(void)
{
    if (path_planner.node_count == 0)
    {
        return 0;
    }
    
    return (path_planner.current_node * 100) / path_planner.node_count;
}
