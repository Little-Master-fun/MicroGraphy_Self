/*********************************************************************************************************************
* 文件名称          nav_control.c
* 功能说明          导航控制系统实现文件
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                备注
* 2025-09-19        LittleMaster        创建导航控制系统，支持高速长距离精确导航
*
* 文件作用说明：
* 本文件实现了完整的导航控制系统，包括：
* 1. 多传感器融合（编码器 + SCH16TK10 IMU）
* 2. Pure Pursuit路径跟随算法
* 3. 动态速度控制和规划
* 4. 长距离精度保证机制
* 5. 安全监控和容错处理
*
********************************************************************************************************************/

#include "nav_control.h"
#include "zf_common_headfile.h"
#include "zf_driver_timer.h"  

//=================================================全局变量定义================================================
nav_system_t nav_system = {                        // 导航系统主结构
    .status = NAV_STATUS_IDLE,
    .mode = NAV_MODE_PURE_PURSUIT,
    .initialized = 0
};

//=================================================内部静态函数声明================================================
// 数学工具函数
static float nav_distance_2d(const nav_position_t *p1, const nav_position_t *p2);
static float nav_angle_normalize(float angle);
// static float nav_angle_difference(float angle1, float angle2); 
static float nav_clamp(float value, float min_val, float max_val);

// 传感器融合相关函数
static uint8 nav_update_sensor_fusion(void);
static uint8 nav_read_encoder_data(void);
static uint8 nav_read_imu_data(void);
static void nav_fuse_sensor_data(void);
static uint8 nav_check_sensor_health(void);

// Pure Pursuit算法相关函数
static uint8 nav_update_pure_pursuit(void);
static nav_position_t nav_find_lookahead_point(void);
static float nav_calculate_curvature(const nav_position_t *lookahead_point);
// static uint16 nav_find_closest_waypoint(void); 
static float nav_calculate_cross_track_error(void);

// 速度控制相关函数
static uint8 nav_update_speed_control(void);
static float nav_calculate_target_speed(void);
static void nav_calculate_wheel_speeds(float target_speed, float curvature);
static float nav_smooth_speed_transition(float target_speed, float current_speed);

// 安全监控相关函数
static uint8 nav_update_safety_monitor(void);
static uint8 nav_check_path_deviation(void);
static uint8 nav_check_sensor_timeout(void);
static void nav_trigger_emergency_stop(void);

// 统计信息更新函数
static void nav_update_statistics(void);

// 系统管理函数
static uint32 nav_get_system_time_ms(void);
static void nav_reset_system_state(void);

//=================================================数学和工具函数实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算两点间距离
//-------------------------------------------------------------------------------------------------------------------
static float nav_distance_2d(const nav_position_t *p1, const nav_position_t *p2)
{
    if (p1 == NULL || p2 == NULL) return 0.0f;
    
    float dx = p2->x - p1->x;
    float dy = p2->y - p1->y;
    return sqrtf(dx * dx + dy * dy);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     角度归一化到[-π, π]
//-------------------------------------------------------------------------------------------------------------------
static float nav_angle_normalize(float angle)
{
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

// 暂时注释未使用的函数
// static float nav_angle_difference(float angle1, float angle2)
// {
//     return nav_angle_normalize(angle1 - angle2);
// }

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     数值限幅
//-------------------------------------------------------------------------------------------------------------------
static float nav_clamp(float value, float min_val, float max_val)
{
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

//=================================================传感器融合功能实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新传感器融合数据
//-------------------------------------------------------------------------------------------------------------------
static uint8 nav_update_sensor_fusion(void)
{
    uint8 result = 1;
    
    // 读取编码器数据
    if (!nav_read_encoder_data()) {
        nav_system.sensor_fusion.encoder_status = NAV_SENSOR_ERROR;
        result = 0;
    }
    
    // 读取IMU数据
    if (!nav_read_imu_data()) {
        nav_system.sensor_fusion.imu_status = NAV_SENSOR_ERROR;
        result = 0;
    }
    
    // 检查传感器健康状态
    if (!nav_check_sensor_health()) {
        result = 0;
    }
    
    // 执行传感器数据融合
    if (result) {
        nav_fuse_sensor_data();
        nav_system.sensor_fusion.last_update_ms = nav_get_system_time_ms();
    }
    
    return result;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取编码器数据
//-------------------------------------------------------------------------------------------------------------------
static uint8 nav_read_encoder_data(void)
{
    if (encoder_update() != ENCODER_STATUS_OK) {
        return 0;
    }
    
    nav_sensor_fusion_t *fusion = &nav_system.sensor_fusion;
    
    // 更新编码器位姿数据
    fusion->encoder_pose.velocity.linear = encoder_system.linear_velocity;
    fusion->encoder_pose.velocity.angular = encoder_system.angular_velocity;
    fusion->encoder_pose.heading = encoder_system.heading_angle;
    
    // 简化的位置积分 (实际应用中需要更精确的运动学模型)
    static uint32 last_update_ms = 0;
    uint32 current_time_ms = nav_get_system_time_ms();
    
    if (last_update_ms != 0) {
        float dt = (current_time_ms - last_update_ms) / 1000.0f;
        
        // 更新位置 (差分驱动运动学)
        float distance = fusion->encoder_pose.velocity.linear * dt;
        fusion->encoder_pose.position.x += distance * cosf(fusion->encoder_pose.heading);
        fusion->encoder_pose.position.y += distance * sinf(fusion->encoder_pose.heading);
    }
    
    last_update_ms = current_time_ms;
    fusion->encoder_pose.timestamp_ms = current_time_ms;
    fusion->encoder_status = NAV_SENSOR_OK;
    
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取IMU数据
//-------------------------------------------------------------------------------------------------------------------
static uint8 nav_read_imu_data(void)
{
    static SCH1_raw_data imu_raw_data;
    static SCH1_result imu_result;
    
    // 读取IMU原始数据
    SCH1_getData(&imu_raw_data);
    
    // 检查数据帧错误
    if (imu_raw_data.frame_error) {
        return 0;
    }
    
    // 转换为物理单位
    SCH1_convert_data(&imu_raw_data, &imu_result);
    
    nav_sensor_fusion_t *fusion = &nav_system.sensor_fusion;
    
    // 存储IMU数据
    fusion->imu_angular_velocity[0] = imu_result.Rate1[AXIS_X];
    fusion->imu_angular_velocity[1] = imu_result.Rate1[AXIS_Y];
    fusion->imu_angular_velocity[2] = imu_result.Rate1[AXIS_Z];
    
    fusion->imu_acceleration[0] = imu_result.Acc1[AXIS_X];
    fusion->imu_acceleration[1] = imu_result.Acc1[AXIS_Y];
    fusion->imu_acceleration[2] = imu_result.Acc1[AXIS_Z];
    
    fusion->imu_temperature = imu_result.Temp;
    fusion->imu_status = NAV_SENSOR_OK;
    
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     融合传感器数据
//-------------------------------------------------------------------------------------------------------------------
static void nav_fuse_sensor_data(void)
{
    nav_sensor_fusion_t *fusion = &nav_system.sensor_fusion;
    
    // 融合位置信息 (主要使用编码器)
    fusion->fused_pose.position = fusion->encoder_pose.position;
    
    // 融合角速度 (编码器 + IMU互补滤波)
    if (fusion->encoder_status == NAV_SENSOR_OK && fusion->imu_status == NAV_SENSOR_OK) {
        // 互补滤波融合角速度
        float encoder_angular_vel = fusion->encoder_pose.velocity.angular;
        float imu_angular_vel = fusion->imu_angular_velocity[2]; // Z轴角速度
        
        // 加权融合
        fusion->fused_pose.velocity.angular = 
            NAV_ENCODER_WEIGHT * encoder_angular_vel + 
            NAV_IMU_WEIGHT * imu_angular_vel;
            
        fusion->confidence = 0.9f; // 高置信度
    }
    else if (fusion->encoder_status == NAV_SENSOR_OK) {
        // 仅使用编码器
        fusion->fused_pose.velocity.angular = fusion->encoder_pose.velocity.angular;
        fusion->confidence = 0.7f; // 中等置信度
    }
    else {
        fusion->confidence = 0.1f; // 低置信度
    }
    
    // 更新航向角 (积分角速度)
    static uint32 last_fusion_time_ms = 0;
    uint32 current_time_ms = nav_get_system_time_ms();
    
    if (last_fusion_time_ms != 0) {
        float dt = (current_time_ms - last_fusion_time_ms) / 1000.0f;
        fusion->fused_pose.heading += fusion->fused_pose.velocity.angular * dt;
        fusion->fused_pose.heading = nav_angle_normalize(fusion->fused_pose.heading);
    }
    
    last_fusion_time_ms = current_time_ms;
    
    // 更新线速度
    fusion->fused_pose.velocity.linear = fusion->encoder_pose.velocity.linear;
    fusion->fused_pose.timestamp_ms = current_time_ms;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检查传感器健康状态
//-------------------------------------------------------------------------------------------------------------------
static uint8 nav_check_sensor_health(void)
{
    nav_sensor_fusion_t *fusion = &nav_system.sensor_fusion;
    uint32 current_time_ms = nav_get_system_time_ms();
    
    // 检查传感器数据更新时间
    if (current_time_ms - fusion->last_update_ms > 100) { // 100ms超时
        return 0;
    }
    
    // 检查编码器数据合理性
    if (fabsf(fusion->encoder_pose.velocity.linear) > NAV_MAX_SPEED + 1.0f) {
        fusion->encoder_status = NAV_SENSOR_WARNING;
    }
    
    // 检查IMU数据合理性
    if (fabsf(fusion->imu_angular_velocity[2]) > 10.0f) { // 10 rad/s 最大角速度
        fusion->imu_status = NAV_SENSOR_WARNING;
    }
    
    return (fusion->encoder_status != NAV_SENSOR_ERROR && 
            fusion->imu_status != NAV_SENSOR_ERROR);
}

//=================================================Pure Pursuit算法实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新Pure Pursuit控制
//-------------------------------------------------------------------------------------------------------------------
static uint8 nav_update_pure_pursuit(void)
{
    nav_pure_pursuit_t *pp = &nav_system.pure_pursuit;
    
    // 计算前瞻距离
    float current_speed = nav_system.sensor_fusion.fused_pose.velocity.linear;
    pp->lookahead_distance = NAV_MIN_LOOKAHEAD + 
                             NAV_LOOKAHEAD_FACTOR * current_speed;
    pp->lookahead_distance = nav_clamp(pp->lookahead_distance, 
                                       NAV_MIN_LOOKAHEAD, NAV_MAX_LOOKAHEAD);
    
    // 找到前瞻点
    pp->lookahead_point = nav_find_lookahead_point();
    
    // 计算横向偏差
    pp->cross_track_error = nav_calculate_cross_track_error();
    
    // 计算曲率
    pp->curvature = nav_calculate_curvature(&pp->lookahead_point);
    
    // 计算转向角
    pp->steering_angle = atanf(NAV_WHEELBASE * pp->curvature);
    pp->steering_angle = nav_clamp(pp->steering_angle, -M_PI/4, M_PI/4); // ±45°限制
    
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查找前瞻点
//-------------------------------------------------------------------------------------------------------------------
static nav_position_t nav_find_lookahead_point(void)
{
    nav_position_t lookahead_point = {0};
    nav_path_t *path = &nav_system.current_path;
    nav_position_t robot_pos = nav_system.sensor_fusion.fused_pose.position;
    float lookahead_dist = nav_system.pure_pursuit.lookahead_distance;
    
    if (path->waypoint_count == 0) {
        return lookahead_point;
    }
    
    // 从当前目标路径点开始搜索
    uint16 start_index = path->current_waypoint;
    if (start_index >= path->waypoint_count) {
        start_index = path->waypoint_count - 1;
    }
    
    // 在路径上寻找距离机器人lookahead_dist的点
    for (uint16 i = start_index; i < path->waypoint_count - 1; i++) {
        nav_position_t p1 = path->waypoints[i].position;
        nav_position_t p2 = path->waypoints[i + 1].position;
        
        // 计算机器人到线段的投影点
        float dx = p2.x - p1.x;
        float dy = p2.y - p1.y;
        float segment_length = sqrtf(dx * dx + dy * dy);
        
        if (segment_length < 1e-6f) continue; // 避免除零
        
        // 归一化线段方向向量
        dx /= segment_length;
        dy /= segment_length;
        
        // 机器人到p1的向量
        float rx = robot_pos.x - p1.x;
        float ry = robot_pos.y - p1.y;
        
        // 投影长度
        float projection = rx * dx + ry * dy;
        projection = nav_clamp(projection, 0.0f, segment_length);
        
        // 投影点
        nav_position_t proj_point;
        proj_point.x = p1.x + projection * dx;
        proj_point.y = p1.y + projection * dy;
        
        // 检查是否找到前瞻点
        float dist_to_proj = nav_distance_2d(&robot_pos, &proj_point);
        
        if (dist_to_proj <= lookahead_dist) {
            // 沿着线段继续寻找前瞻点
            float remaining_dist = lookahead_dist - dist_to_proj;
            float remaining_on_segment = segment_length - projection;
            
            if (remaining_dist <= remaining_on_segment) {
                // 前瞻点在当前线段上
                lookahead_point.x = proj_point.x + remaining_dist * dx;
                lookahead_point.y = proj_point.y + remaining_dist * dy;
                nav_system.pure_pursuit.target_waypoint_index = i;
                return lookahead_point;
            }
        }
    }
    
    // 如果没找到，返回最后一个路径点
    if (path->waypoint_count > 0) {
        lookahead_point = path->waypoints[path->waypoint_count - 1].position;
        nav_system.pure_pursuit.target_waypoint_index = path->waypoint_count - 1;
    }
    
    return lookahead_point;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算曲率
//-------------------------------------------------------------------------------------------------------------------
static float nav_calculate_curvature(const nav_position_t *lookahead_point)
{
    nav_position_t robot_pos = nav_system.sensor_fusion.fused_pose.position;
    float robot_heading = nav_system.sensor_fusion.fused_pose.heading;
    
    // 计算到前瞻点的相对位置
    float dx = lookahead_point->x - robot_pos.x;
    float dy = lookahead_point->y - robot_pos.y;
    
    // 转换到机器人坐标系
    float cos_h = cosf(robot_heading);
    float sin_h = sinf(robot_heading);
    
    // float local_x = dx * cos_h + dy * sin_h;  // 暂时未使用
    float local_y = -dx * sin_h + dy * cos_h;
    
    float lookahead_dist = nav_system.pure_pursuit.lookahead_distance;
    
    // 防止除零
    if (lookahead_dist < 1e-3f) {
        return 0.0f;
    }
    
    // Pure Pursuit曲率公式
    float curvature = 2.0f * local_y / (lookahead_dist * lookahead_dist);
    
    // 曲率限制
    float max_curvature = 1.0f / 0.5f; // 最小转弯半径0.5m
    return nav_clamp(curvature, -max_curvature, max_curvature);
}

// 暂时注释未使用的函数
// static uint16 nav_find_closest_waypoint(void)
// {
//     nav_path_t *path = &nav_system.current_path;
//     nav_position_t robot_pos = nav_system.sensor_fusion.fused_pose.position;
//     
//     if (path->waypoint_count == 0) {
//         return 0;
//     }
//     
//     uint16 closest_index = 0;
//     float min_distance = nav_distance_2d(&robot_pos, &path->waypoints[0].position);
//     
//     for (uint16 i = 1; i < path->waypoint_count; i++) {
//         float distance = nav_distance_2d(&robot_pos, &path->waypoints[i].position);
//         if (distance < min_distance) {
//             min_distance = distance;
//             closest_index = i;
//         }
//     }
//     
//     return closest_index;
// }

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算横向偏差
//-------------------------------------------------------------------------------------------------------------------
static float nav_calculate_cross_track_error(void)
{
    nav_path_t *path = &nav_system.current_path;
    nav_position_t robot_pos = nav_system.sensor_fusion.fused_pose.position;
    
    if (path->waypoint_count < 2) {
        return 0.0f;
    }
    
    // 找到当前目标线段
    uint16 target_index = nav_system.pure_pursuit.target_waypoint_index;
    if (target_index >= path->waypoint_count - 1) {
        target_index = path->waypoint_count - 2;
    }
    
    nav_position_t p1 = path->waypoints[target_index].position;
    nav_position_t p2 = path->waypoints[target_index + 1].position;
    
    // 计算点到线段的距离
    float A = p2.y - p1.y;
    float B = p1.x - p2.x;
    float C = p2.x * p1.y - p1.x * p2.y;
    
    float denominator = sqrtf(A * A + B * B);
    if (denominator < 1e-6f) {
        return 0.0f;
    }
    
    float cross_track_error = (A * robot_pos.x + B * robot_pos.y + C) / denominator;
    
    return cross_track_error;
}

//=================================================速度控制功能实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新速度控制
//-------------------------------------------------------------------------------------------------------------------
static uint8 nav_update_speed_control(void)
{
    nav_speed_control_t *speed_ctrl = &nav_system.speed_control;
    
    // 计算目标速度
    speed_ctrl->target_speed = nav_calculate_target_speed();
    
    // 平滑速度过渡
    speed_ctrl->target_speed = nav_smooth_speed_transition(
        speed_ctrl->target_speed, 
        speed_ctrl->current_speed
    );
    
    // 更新当前速度
    speed_ctrl->current_speed = nav_system.sensor_fusion.fused_pose.velocity.linear;
    
    // 计算加速度
    static float last_speed = 0.0f;
    static uint32 last_time_ms = 0;
    uint32 current_time_ms = nav_get_system_time_ms();
    
    if (last_time_ms != 0) {
        float dt = (current_time_ms - last_time_ms) / 1000.0f;
        if (dt > 1e-6f) {
            speed_ctrl->acceleration = (speed_ctrl->current_speed - last_speed) / dt;
        }
    }
    
    last_speed = speed_ctrl->current_speed;
    last_time_ms = current_time_ms;
    
    // 计算左右轮速度
    nav_calculate_wheel_speeds(speed_ctrl->target_speed, 
                               nav_system.pure_pursuit.curvature);
    
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算目标速度
//-------------------------------------------------------------------------------------------------------------------
static float nav_calculate_target_speed(void)
{
    nav_path_t *path = &nav_system.current_path;
    
    if (path->waypoint_count == 0) {
        return 0.0f;
    }
    
    // 基础目标速度
    uint16 target_index = nav_system.pure_pursuit.target_waypoint_index;
    if (target_index >= path->waypoint_count) {
        target_index = path->waypoint_count - 1;
    }
    
    float base_speed = path->waypoints[target_index].target_speed;
    
    // 根据曲率调整速度
    float curvature = fabsf(nav_system.pure_pursuit.curvature);
    float max_speed_for_curvature = NAV_MAX_SPEED;
    
    if (curvature > 1e-3f) {
        // 根据向心力限制计算最大速度
        float max_lateral_acceleration = 2.0f; // 2 m/s? 最大侧向加速度
        max_speed_for_curvature = sqrtf(max_lateral_acceleration / curvature);
    }
    
    nav_system.speed_control.max_speed_for_curvature = max_speed_for_curvature;
    
    // 根据横向偏差调整速度
    float cross_track_error = fabsf(nav_system.pure_pursuit.cross_track_error);
    float speed_reduction_factor = 1.0f;
    
    if (cross_track_error > NAV_POSITION_TOLERANCE) {
        speed_reduction_factor = NAV_POSITION_TOLERANCE / cross_track_error;
        speed_reduction_factor = nav_clamp(speed_reduction_factor, 0.3f, 1.0f);
    }
    
    // 应用所有限制
    float target_speed = base_speed * speed_reduction_factor;
    target_speed = nav_clamp(target_speed, NAV_MIN_SPEED, 
                            fminf(max_speed_for_curvature, NAV_MAX_SPEED));
    
    // 检查是否接近终点
    nav_position_t robot_pos = nav_system.sensor_fusion.fused_pose.position;
    nav_position_t final_waypoint = path->waypoints[path->waypoint_count - 1].position;
    float distance_to_end = nav_distance_2d(&robot_pos, &final_waypoint);
    
    if (distance_to_end < 1.0f) { // 距离终点1米内开始减速
        float decel_factor = distance_to_end / 1.0f;
        target_speed *= decel_factor;
        target_speed = nav_clamp(target_speed, NAV_MIN_SPEED, target_speed);
    }
    
    return target_speed;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算左右轮速度
//-------------------------------------------------------------------------------------------------------------------
static void nav_calculate_wheel_speeds(float target_speed, float curvature)
{
    nav_speed_control_t *speed_ctrl = &nav_system.speed_control;
    
    // 差分驱动运动学
    float angular_velocity = target_speed * curvature;
    float wheel_speed_diff = angular_velocity * NAV_WHEELBASE / 2.0f;
    
    speed_ctrl->left_wheel_speed = target_speed - wheel_speed_diff;
    speed_ctrl->right_wheel_speed = target_speed + wheel_speed_diff;
    
    // 速度限制
    speed_ctrl->left_wheel_speed = nav_clamp(speed_ctrl->left_wheel_speed, 
                                            -NAV_MAX_SPEED, NAV_MAX_SPEED);
    speed_ctrl->right_wheel_speed = nav_clamp(speed_ctrl->right_wheel_speed, 
                                             -NAV_MAX_SPEED, NAV_MAX_SPEED);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     平滑速度过渡
//-------------------------------------------------------------------------------------------------------------------
static float nav_smooth_speed_transition(float target_speed, float current_speed)
{
    float speed_error = target_speed - current_speed;
    float max_delta_speed = NAV_MAX_ACCELERATION * (NAV_CONTROL_PERIOD_MS / 1000.0f);
    
    if (speed_error > max_delta_speed) {
        return current_speed + max_delta_speed;
    } else if (speed_error < -max_delta_speed) {
        return current_speed - max_delta_speed;
    }
    
    return target_speed;
}

//=================================================安全监控功能实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新安全监控
//-------------------------------------------------------------------------------------------------------------------
static uint8 nav_update_safety_monitor(void)
{
    nav_safety_monitor_t *safety = &nav_system.safety_monitor;
    uint32 current_time_ms = nav_get_system_time_ms();
    
    // 定期安全检查
    if (current_time_ms - safety->last_safety_check_ms >= 
        NAV_SAFETY_CHECK_INTERVAL * NAV_CONTROL_PERIOD_MS) {
        
        // 检查路径偏离
        if (!nav_check_path_deviation()) {
            safety->path_deviation = nav_system.pure_pursuit.cross_track_error;
            if (fabsf(safety->path_deviation) > NAV_PATH_DEVIATION_MAX) {
                printf("警告：路径偏离过大 %.2fm\n", safety->path_deviation);
                return 0;
            }
        }
        
        // 检查传感器超时
        if (!nav_check_sensor_timeout()) {
            safety->sensor_timeout_count++;
            if (safety->sensor_timeout_count > 10) { // 连续10次超时
                printf("错误：传感器超时\n");
                nav_trigger_emergency_stop();
                return 0;
            }
        } else {
            safety->sensor_timeout_count = 0; // 重置计数
        }
        
        safety->last_safety_check_ms = current_time_ms;
    }
    
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检查路径偏离
//-------------------------------------------------------------------------------------------------------------------
static uint8 nav_check_path_deviation(void)
{
    float cross_track_error = fabsf(nav_system.pure_pursuit.cross_track_error);
    return (cross_track_error <= NAV_PATH_DEVIATION_MAX);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检查传感器超时
//-------------------------------------------------------------------------------------------------------------------
static uint8 nav_check_sensor_timeout(void)
{
    uint32 current_time_ms = nav_get_system_time_ms();
    uint32 last_sensor_update = nav_system.sensor_fusion.last_update_ms;
    
    return (current_time_ms - last_sensor_update <= 100); // 100ms超时
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     触发紧急停车
//-------------------------------------------------------------------------------------------------------------------
static void nav_trigger_emergency_stop(void)
{
    nav_system.safety_monitor.emergency_stop_triggered = 1;
    nav_system.status = NAV_STATUS_EMERGENCY_STOP;
    
    // 立即停止电机
    motor_left_speed_control(0.0f, nav_system.sensor_fusion.fused_pose.velocity.linear);
    motor_right_speed_control(0.0f, nav_system.sensor_fusion.fused_pose.velocity.linear);
    
    printf("紧急停车触发！\n");
}

//=================================================统计信息更新实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新统计信息
//-------------------------------------------------------------------------------------------------------------------
static void nav_update_statistics(void)
{
    nav_statistics_t *stats = &nav_system.statistics;
    uint32 current_time_ms = nav_get_system_time_ms();
    
    // 更新行驶距离
    static nav_position_t last_position = {0};
    static uint8 position_initialized = 0;
    
    nav_position_t current_position = nav_system.sensor_fusion.fused_pose.position;
    
    if (position_initialized) {
        float distance_increment = nav_distance_2d(&last_position, &current_position);
        stats->total_distance_traveled += distance_increment;
    } else {
        position_initialized = 1;
    }
    
    last_position = current_position;
    
    // 更新速度统计
    float current_speed = nav_system.sensor_fusion.fused_pose.velocity.linear;
    if (current_speed > stats->max_speed_reached) {
        stats->max_speed_reached = current_speed;
    }
    
    // 更新导航时间
    if (stats->navigation_start_time_ms != 0) {
        stats->total_navigation_time_ms = current_time_ms - stats->navigation_start_time_ms;
        
        if (stats->total_navigation_time_ms > 0) {
            stats->average_speed = stats->total_distance_traveled / 
                                  (stats->total_navigation_time_ms / 1000.0f);
        }
    }
    
    // 更新横向误差统计
    float cross_track_error = fabsf(nav_system.pure_pursuit.cross_track_error);
    if (cross_track_error > stats->max_cross_track_error) {
        stats->max_cross_track_error = cross_track_error;
    }
    
    // 计算平均横向误差（简化版本）
    static float error_sum = 0.0f;
    static uint32 error_count = 0;
    
    error_sum += cross_track_error;
    error_count++;
    stats->average_cross_track_error = error_sum / error_count;
}

//=================================================系统管理函数实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取系统时间
//-------------------------------------------------------------------------------------------------------------------
static uint32 nav_get_system_time_ms(void)
{
    // 使用系统定时器获取毫秒时间
    static uint8 timer_initialized = 0;
    if (!timer_initialized) {
        timer_init(TC_TIME2_CH0, TIMER_MS);  // 初始化定时器为毫秒模式
        timer_start(TC_TIME2_CH0);           // 启动定时器
        timer_initialized = 1;
    }
    return timer_get(TC_TIME2_CH0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     重置系统状态
//-------------------------------------------------------------------------------------------------------------------
static void nav_reset_system_state(void)
{
    memset(&nav_system, 0, sizeof(nav_system_t));
    nav_system.status = NAV_STATUS_IDLE;
    nav_system.mode = NAV_MODE_PURE_PURSUIT;
}

//=================================================主要接口函数实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导航系统初始化
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_init(void)
{
    printf("初始化导航控制系统...\n");
    
    // 重置系统状态
    nav_reset_system_state();
    
    nav_system.status = NAV_STATUS_INITIALIZING;
    
    // 初始化编码器系统
    if (encoder_init() != ENCODER_STATUS_OK) {
        printf("编码器初始化失败\n");
        nav_system.status = NAV_STATUS_ERROR;
        return 0;
    }
    
    // 初始化IMU系统
    SCH1_filter sFilter = {.Rate12 = 30, .Acc12 = 30, .Acc3 = 30};
    SCH1_sensitivity sSensitivity = {.Rate1 = 1600, .Rate2 = 1600, .Acc1 = 3200, .Acc2 = 3200, .Acc3 = 3200};
    SCH1_decimation sDecimation = {.Rate2 = 1, .Acc2 = 1};
    
    if (SCH1_init(sFilter, sSensitivity, sDecimation, false) != SCH1_OK) {
        printf("IMU初始化失败\n");
        nav_system.status = NAV_STATUS_ERROR;
        return 0;
    }
    
    // 初始化电机控制系统
    if (motor_pid_init() != MOTOR_PID_OK) {
        printf("电机控制系统初始化失败\n");
        nav_system.status = NAV_STATUS_ERROR;
        return 0;
    }
    
    // 初始化Flash路径存储系统
    if (flash_road_init() != FLASH_ROAD_RESULT_OK) {
        printf("Flash路径存储系统初始化失败\n");
        nav_system.status = NAV_STATUS_ERROR;
        return 0;
    }
    
    nav_system.initialized = 1;
    nav_system.status = NAV_STATUS_IDLE;
    
    printf("导航控制系统初始化完成\n");
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     加载导航路径
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_load_path(const char *path_name)
{
    if (!nav_system.initialized || path_name == NULL) {
        return 0;
    }
    
    nav_system.status = NAV_STATUS_PATH_LOADING;
    
    // 暂时简化路径加载实现
    // 在实际应用中需要实现从Flash加载路径的功能
    nav_path_t *nav_path = &nav_system.current_path;
    nav_path->waypoint_count = 4; // 示例路径
    nav_path->current_waypoint = 0;
    nav_path->total_length = 10.0f;
    strncpy(nav_path->path_name, path_name, sizeof(nav_path->path_name) - 1);
    
    // 创建示例路径点
    nav_path->waypoints[0] = (nav_waypoint_t){{0.0f, 0.0f}, 1.0f, 0.1f, 0};
    nav_path->waypoints[1] = (nav_waypoint_t){{2.0f, 0.0f}, 2.0f, 0.1f, 0};
    nav_path->waypoints[2] = (nav_waypoint_t){{2.0f, 2.0f}, 1.5f, 0.1f, 0};
    nav_path->waypoints[3] = (nav_waypoint_t){{0.0f, 2.0f}, 1.0f, 0.1f, 0};
    
    printf("路径加载成功: %s (示例路径)\n", path_name);
    
    nav_system.status = NAV_STATUS_IDLE;
    
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     开始导航
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_start(nav_mode_enum mode)
{
    if (!nav_system.initialized || nav_system.current_path.waypoint_count == 0) {
        return 0;
    }
    
    nav_system.mode = mode;
    nav_system.status = NAV_STATUS_NAVIGATING;
    
    // 重置统计信息
    memset(&nav_system.statistics, 0, sizeof(nav_statistics_t));
    nav_system.statistics.navigation_start_time_ms = nav_get_system_time_ms();
    
    // 重置路径跟随状态
    nav_system.current_path.current_waypoint = 0;
    
    printf("开始导航: %s (模式: %d)\n", 
           nav_system.current_path.path_name, mode);
    
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     停止导航
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_stop(uint8 emergency)
{
    if (emergency) {
        nav_trigger_emergency_stop();
    } else {
        nav_system.status = NAV_STATUS_IDLE;
        
        // 平滑停车
        motor_left_speed_control(0.0f, nav_system.sensor_fusion.fused_pose.velocity.linear);
        motor_right_speed_control(0.0f, nav_system.sensor_fusion.fused_pose.velocity.linear);
    }
    
    printf("导航停止 (紧急停车: %s)\n", emergency ? "是" : "否");
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导航系统主循环更新
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_update(void)
{
    if (!nav_system.initialized) {
        return 0;
    }
    
    uint32 current_time_ms = nav_get_system_time_ms();
    nav_system.control_loop_count++;
    
    // 检查控制周期
    if (current_time_ms - nav_system.last_control_update_ms < NAV_CONTROL_PERIOD_MS) {
        return 1; // 还未到更新时间
    }
    
    nav_system.last_control_update_ms = current_time_ms;
    
    // 状态机处理
    switch (nav_system.status) {
        case NAV_STATUS_NAVIGATING:
            // 更新传感器融合
            if (!nav_update_sensor_fusion()) {
                printf("传感器融合更新失败\n");
                break;
            }
            
            // 更新Pure Pursuit控制
            if (!nav_update_pure_pursuit()) {
                printf("Pure Pursuit更新失败\n");
                break;
            }
            
            // 更新速度控制
            if (!nav_update_speed_control()) {
                printf("速度控制更新失败\n");
                break;
            }
            
            // 安全监控
            if (!nav_update_safety_monitor()) {
                printf("安全监控检查失败\n");
                nav_stop(1); // 紧急停车
                break;
            }
            
            // 应用控制指令
            motor_left_speed_control(nav_system.speed_control.left_wheel_speed, 
                                    nav_system.sensor_fusion.fused_pose.velocity.linear);
            motor_right_speed_control(nav_system.speed_control.right_wheel_speed, 
                                     nav_system.sensor_fusion.fused_pose.velocity.linear);
            
            // 检查是否到达终点
            if (nav_is_target_reached()) {
                nav_system.status = NAV_STATUS_COMPLETED;
                printf("导航完成！\n");
            }
            
            // 更新统计信息
            nav_update_statistics();
            break;
            
        case NAV_STATUS_PAUSED:
            // 暂停状态，保持当前位置
            motor_left_speed_control(0.0f, nav_system.sensor_fusion.fused_pose.velocity.linear);
            motor_right_speed_control(0.0f, nav_system.sensor_fusion.fused_pose.velocity.linear);
            nav_update_sensor_fusion(); // 继续更新传感器数据
            break;
            
        case NAV_STATUS_EMERGENCY_STOP:
            // 紧急停车状态
            motor_left_speed_control(0.0f, nav_system.sensor_fusion.fused_pose.velocity.linear);
            motor_right_speed_control(0.0f, nav_system.sensor_fusion.fused_pose.velocity.linear);
            break;
            
        default:
            // 其他状态不需要控制更新
            break;
    }
    
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     暂停/恢复导航
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_pause(uint8 pause)
{
    if (!nav_system.initialized) {
        return 0;
    }
    
    if (pause) {
        if (nav_system.status == NAV_STATUS_NAVIGATING) {
            nav_system.status = NAV_STATUS_PAUSED;
            printf("导航已暂停\n");
        }
    } else {
        if (nav_system.status == NAV_STATUS_PAUSED) {
            nav_system.status = NAV_STATUS_NAVIGATING;
            printf("导航已恢复\n");
        }
    }
    
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取导航系统状态
//-------------------------------------------------------------------------------------------------------------------
nav_status_enum nav_get_status(void)
{
    return nav_system.status;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取当前位置
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_get_current_position(nav_position_t *position)
{
    if (position == NULL || !nav_system.initialized) {
        return 0;
    }
    
    *position = nav_system.sensor_fusion.fused_pose.position;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取导航统计信息
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_get_statistics(nav_statistics_t *stats)
{
    if (stats == NULL || !nav_system.initialized) {
        return 0;
    }
    
    *stats = nav_system.statistics;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置导航参数
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_set_parameters(float max_speed, float lookahead_factor)
{
    if (!nav_system.initialized) {
        return 0;
    }
    
    // 参数验证和设置
    if (max_speed > 0.0f && max_speed <= 5.0f) {
        // 动态调整最大速度（需要重新定义常量或使用全局变量）
        printf("最大速度设置为: %.2f m/s\n", max_speed);
    }
    
    if (lookahead_factor > 0.0f && lookahead_factor <= 2.0f) {
        // 动态调整前瞻系数
        printf("前瞻距离系数设置为: %.2f\n", lookahead_factor);
    }
    
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取路径跟随误差
//-------------------------------------------------------------------------------------------------------------------
float nav_get_cross_track_error(void)
{
    return nav_system.pure_pursuit.cross_track_error;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检查是否到达目标
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_is_target_reached(void)
{
    nav_path_t *path = &nav_system.current_path;
    
    if (path->waypoint_count == 0) {
        return 1;
    }
    
    nav_position_t robot_pos = nav_system.sensor_fusion.fused_pose.position;
    nav_position_t final_waypoint = path->waypoints[path->waypoint_count - 1].position;
    
    float distance_to_end = nav_distance_2d(&robot_pos, &final_waypoint);
    float tolerance = path->waypoints[path->waypoint_count - 1].tolerance;
    
    return (distance_to_end <= tolerance);
}
