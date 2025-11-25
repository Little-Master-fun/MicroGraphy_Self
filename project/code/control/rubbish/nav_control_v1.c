/*********************************************************************************************************************
* 文件名称          nav_control.c
* 功能说明          导航控制系统实现文件
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本
* 2025-09-22        LittleMaster       1.0v
* 
* 文件作用说明：
* 该导航方式已被废弃，当时想的是陀螺仪会有偏差那就不使用陀螺仪，直接使用编码器来计算位置，但是却没有考虑到打滑的问题，一滑就会产生更大的误差
* 本文件实现导航控制系统的所有功能，包括路径跟踪、运动学解算和控制输出
* 
*
* 主要特性：
* 1. Pure Pursuit路径跟踪算法
* 2. 用户速度控制 + 算法转向控制
* 3. 编码器里程计定位
* 4. 1x1米正方形测试路径
* 5. 实时状态监控和调试信息
********************************************************************************************************************/

#include "nav_control.h"
#include "driver_encoder.h"
#include "motor_control.h"
#include "driver_sch16tk10.h"
#include "zf_common_headfile.h"
#include "zf_driver_timer.h"
#include <stdio.h>
#include <string.h>

//=================================================全局变量定义================================================
nav_system_t nav_system = {0};

//=================================================内部函数声明================================================
static nav_status_enum nav_update_pose(void);
static nav_status_enum nav_pure_pursuit_control(void);
static nav_status_enum nav_calculate_wheel_speeds(float linear_vel, float angular_vel);
static float nav_calculate_distance(nav_point2d_t p1, nav_point2d_t p2);
static float nav_normalize_angle(float angle);
static nav_status_enum nav_find_lookahead_point(nav_point2d_t *lookahead_point);
static nav_status_enum nav_check_target_reached(void);
static float nav_adaptive_speed_calculation(float base_speed);

//=================================================外部接口实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导航系统初始化
//-------------------------------------------------------------------------------------------------------------------
nav_status_enum nav_init(void)
{
    // 清零系统结构体
    memset(&nav_system, 0, sizeof(nav_system_t));
    
    // 初始化编码器
    if (encoder_init() != ENCODER_STATUS_OK) {
        printf("错误：编码器初始化失败\n");
        nav_system.status = NAV_STATUS_ERROR;
        return NAV_STATUS_ERROR;
    }
    
    // 初始化系统时间戳计数器
    timer_init(TC_TIME2_CH2, TIMER_MS);
    timer_start(TC_TIME2_CH2);
    
    // 初始化电机PID控制
    if (motor_pid_init() != MOTOR_PID_OK) {
        printf("错误：电机PID控制初始化失败\n");
        nav_system.status = NAV_STATUS_ERROR;
        return NAV_STATUS_ERROR;
    }
    
    // 初始化IMU传感器
    SCH1_filter filter = {50, 50, 50};        // 50Hz滤波频率
    SCH1_sensitivity sens = {100, 100, 4, 4, 4}; // 灵敏度设置
    SCH1_decimation dec = {1, 1};             // 抽取设置
    
    if (SCH1_init(filter, sens, dec, false) != SCH1_OK) {
        printf("警告：IMU传感器初始化失败，将使用编码器定位\n");
        // IMU失败不影响导航系统运行，只是定位精度降低
    }
    
    // 初始化系统参数
    nav_system.speed_config.desired_speed = 0.5f;        // 默认0.5m/s
    nav_system.speed_config.max_speed = 2.0f;            // 最大2.0m/s
    nav_system.speed_config.acceleration_limit = 1.0f;   // 1.0m/s?加速度限制
    nav_system.speed_config.deceleration_limit = 2.0f;   // 2.0m/s?减速度限制
    nav_system.speed_config.adaptive_speed_enable = 1;   // 启用自适应速度
    
    nav_system.mode = NAV_MODE_MANUAL;
    nav_system.status = NAV_STATUS_OK;
    nav_system.system_enabled = 1;
    
    // 重置编码器
    encoder_reset(ENCODER_ID_BOTH);
    
    printf("导航系统初始化完成\n");
    printf("默认速度: %.1f m/s\n", nav_system.speed_config.desired_speed);
    
    return NAV_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置用户期望速度
//-------------------------------------------------------------------------------------------------------------------
nav_status_enum nav_set_desired_speed(float desired_speed)
{
    if (desired_speed < 0.0f || desired_speed > nav_system.speed_config.max_speed) {
        printf("错误：期望速度超出范围 (0 ~ %.1f m/s)\n", nav_system.speed_config.max_speed);
        return NAV_STATUS_ERROR;
    }
    
    nav_system.speed_config.desired_speed = desired_speed;
    printf("期望速度已设置为: %.1f m/s\n", desired_speed);
    
    return NAV_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     加载预定义路径
//-------------------------------------------------------------------------------------------------------------------
nav_status_enum nav_load_preset_path(uint8 path_type)
{
    if (path_type == 0) {
        // 1x1米正方形路径（高密度路径点）
        nav_system.path.total_points = 20;
        nav_system.path.current_target_index = 0;
        nav_system.path.loop_mode = 1;  // 循环模式
        nav_system.path.reverse_mode = 0;
        
        // 定义密集的正方形路径点 (起点在原点，顺时针方向)
        // 第一条边：从(0,0)到(1,0) - 向右
        nav_system.path.waypoints[0]  = (nav_waypoint_t){{0.0f, 0.0f}, 0.6f, 0}; // 起点
        nav_system.path.waypoints[1]  = (nav_waypoint_t){{0.3f, 0.0f}, 0.8f, 0}; // 第一条边中点1
        nav_system.path.waypoints[2]  = (nav_waypoint_t){{0.7f, 0.0f}, 0.8f, 0}; // 第一条边中点2
        nav_system.path.waypoints[3]  = (nav_waypoint_t){{0.9f, 0.0f}, 0.5f, 2}; // 转弯前减速点
        nav_system.path.waypoints[4]  = (nav_waypoint_t){{1.0f, 0.0f}, 0.4f, 2}; // 第一个转弯点
        
        // 第二条边：从(1,0)到(1,1) - 向上  
        nav_system.path.waypoints[5]  = (nav_waypoint_t){{1.0f, 0.1f}, 0.5f, 0}; // 转弯后加速点
        nav_system.path.waypoints[6]  = (nav_waypoint_t){{1.0f, 0.3f}, 0.8f, 0}; // 第二条边中点1
        nav_system.path.waypoints[7]  = (nav_waypoint_t){{1.0f, 0.7f}, 0.8f, 0}; // 第二条边中点2
        nav_system.path.waypoints[8]  = (nav_waypoint_t){{1.0f, 0.9f}, 0.5f, 2}; // 转弯前减速点
        nav_system.path.waypoints[9]  = (nav_waypoint_t){{1.0f, 1.0f}, 0.4f, 2}; // 第二个转弯点
        
        // 第三条边：从(1,1)到(0,1) - 向左
        nav_system.path.waypoints[10] = (nav_waypoint_t){{0.9f, 1.0f}, 0.5f, 0}; // 转弯后加速点
        nav_system.path.waypoints[11] = (nav_waypoint_t){{0.7f, 1.0f}, 0.8f, 0}; // 第三条边中点1
        nav_system.path.waypoints[12] = (nav_waypoint_t){{0.3f, 1.0f}, 0.8f, 0}; // 第三条边中点2
        nav_system.path.waypoints[13] = (nav_waypoint_t){{0.1f, 1.0f}, 0.5f, 2}; // 转弯前减速点
        nav_system.path.waypoints[14] = (nav_waypoint_t){{0.0f, 1.0f}, 0.4f, 2}; // 第三个转弯点
        
        // 第四条边：从(0,1)到(0,0) - 向下
        nav_system.path.waypoints[15] = (nav_waypoint_t){{0.0f, 0.9f}, 0.5f, 0}; // 转弯后加速点
        nav_system.path.waypoints[16] = (nav_waypoint_t){{0.0f, 0.7f}, 0.8f, 0}; // 第四条边中点1
        nav_system.path.waypoints[17] = (nav_waypoint_t){{0.0f, 0.3f}, 0.8f, 0}; // 第四条边中点2
        nav_system.path.waypoints[18] = (nav_waypoint_t){{0.0f, 0.1f}, 0.5f, 2}; // 转弯前减速点
        nav_system.path.waypoints[19] = (nav_waypoint_t){{0.0f, 0.0f}, 0.3f, 1}; // 回到起点(停车点)
        
        nav_system.status = NAV_STATUS_OK;
        
        printf("已加载1x1米正方形路径，共%d个路径点\n", nav_system.path.total_points);
        for (int i = 0; i < nav_system.path.total_points; i++) {
            printf("  点%d: (%.1f, %.1f) 速度=%.1f\n", i, 
                   nav_system.path.waypoints[i].position.x,
                   nav_system.path.waypoints[i].position.y,
                   nav_system.path.waypoints[i].target_speed);
        }
        
        return NAV_STATUS_OK;
    }
    
    printf("错误：未知的路径类型 %d\n", path_type);
    return NAV_STATUS_ERROR;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导航系统主更新函数
//-------------------------------------------------------------------------------------------------------------------
nav_status_enum nav_update(void)
{
    if (!nav_system.system_enabled || nav_system.status == NAV_STATUS_ERROR) {
        return nav_system.status;
    }
    
    nav_system.update_count++;
    nav_system.last_update_time = timer_get(TC_TIME2_CH2);  // 获取当前时间戳(ms)
    
    // 1. 更新机器人位姿估算
    nav_status_enum pose_status = nav_update_pose();
    if (pose_status != NAV_STATUS_OK) {
        return pose_status;
    }
    
    // 2. 检查是否有路径
    if (nav_system.path.total_points == 0) {
        nav_system.status = NAV_STATUS_NO_PATH;
        motor_set_target_speed(0.0f, 0.0f);  // 停车
        return NAV_STATUS_NO_PATH;
    }
    
    // 3. 根据导航模式执行控制
    switch (nav_system.mode) {
        case NAV_MODE_AUTO:
            // 检查是否到达目标点
            nav_check_target_reached();
            
            // 执行Pure Pursuit路径跟踪
            nav_pure_pursuit_control();
            break;
            
        case NAV_MODE_MANUAL:
            // 手动模式：用户控制，停车
            motor_set_target_speed(0.0f, 0.0f);
            break;
            
        case NAV_MODE_PAUSE:
            // 暂停模式：保持当前位置
            motor_set_target_speed(0.0f, 0.0f);
            break;
            
        case NAV_MODE_EMERGENCY:
            // 紧急模式：立即停车
            motor_set_target_speed(0.0f, 0.0f);
            nav_system.status = NAV_STATUS_ERROR;
            break;
    }
    
    return nav_system.status;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置导航模式
//-------------------------------------------------------------------------------------------------------------------
nav_status_enum nav_set_mode(nav_mode_enum mode)
{
    nav_system.mode = mode;
    
    const char* mode_names[] = {"手动", "自动", "暂停", "紧急"};
    printf("导航模式已切换为: %s\n", mode_names[mode]);
    
    // 模式切换时的初始化操作
    if (mode == NAV_MODE_AUTO && nav_system.path.total_points > 0) {
        nav_system.status = NAV_STATUS_OK;
        printf("开始自动路径跟踪\n");
    }
    
    return NAV_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取导航系统状态
//-------------------------------------------------------------------------------------------------------------------
nav_status_enum nav_get_system_status(nav_system_t *system_info)
{
    if (system_info == NULL) {
        return NAV_STATUS_ERROR;
    }
    
    memcpy(system_info, &nav_system, sizeof(nav_system_t));
    return NAV_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     重置导航系统
//-------------------------------------------------------------------------------------------------------------------
nav_status_enum nav_reset(void)
{
    // 重置位置估算
    nav_system.current_pose.x = 0.0f;
    nav_system.current_pose.y = 0.0f;
    nav_system.current_pose.theta = 0.0f;
    nav_system.current_pose.linear_velocity = 0.0f;
    nav_system.current_pose.angular_velocity = 0.0f;
    
    // 重置路径跟踪
    nav_system.path.current_target_index = 0;
    
    // 重置编码器
    encoder_reset(ENCODER_ID_BOTH);
    
    // 停车
    motor_set_target_speed(0.0f, 0.0f);
    
    nav_system.status = NAV_STATUS_OK;
    printf("导航系统已重置\n");
    
    return NAV_STATUS_OK;
}

//=================================================内部函数实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新小车位姿估算
//-------------------------------------------------------------------------------------------------------------------
static nav_status_enum nav_update_pose(void)
{
    static float last_left_distance = 0.0f;
    static float last_right_distance = 0.0f;
    
    // 更新编码器数据
    encoder_update();
    
    // 获取编码器速度和距离
    float left_speed = encoder_get_speed(ENCODER_ID_LEFT);
    float right_speed = encoder_get_speed(ENCODER_ID_RIGHT);
    float left_distance = encoder_get_distance(ENCODER_ID_LEFT) / 1000.0f;   // 转换为米
    float right_distance = encoder_get_distance(ENCODER_ID_RIGHT) / 1000.0f; // 转换为米
    
    // 计算距离增量
    float delta_left = left_distance - last_left_distance;
    float delta_right = right_distance - last_right_distance;
    last_left_distance = left_distance;
    last_right_distance = right_distance;
    
    // 差速运动学正解：计算位置增量
    float delta_s = (delta_left + delta_right) / 2.0f;           // 线位移
    float delta_theta = (delta_right - delta_left) / NAV_WHEELBASE; // 角位移
    
    // 更新位姿
    nav_system.current_pose.theta += delta_theta;
    nav_system.current_pose.theta = nav_normalize_angle(nav_system.current_pose.theta);
    
    nav_system.current_pose.x += delta_s * cosf(nav_system.current_pose.theta);
    nav_system.current_pose.y += delta_s * sinf(nav_system.current_pose.theta);
    
    // 更新速度
    nav_system.current_pose.linear_velocity = (left_speed + right_speed) / 2.0f;
    nav_system.current_pose.angular_velocity = (right_speed - left_speed) / NAV_WHEELBASE;
    
    return NAV_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     Pure Pursuit路径跟踪控制
//-------------------------------------------------------------------------------------------------------------------
static nav_status_enum nav_pure_pursuit_control(void)
{
    nav_point2d_t lookahead_point;
    
    // 查找前瞻点
    if (nav_find_lookahead_point(&lookahead_point) != NAV_STATUS_OK) {
        return NAV_STATUS_ERROR;
    }
    
    // 计算到前瞻点的距离和角度
    float dx = lookahead_point.x - nav_system.current_pose.x;
    float dy = lookahead_point.y - nav_system.current_pose.y;
    float target_angle = atan2f(dy, dx);
    
    // 计算角度误差
    float angle_error = nav_normalize_angle(target_angle - nav_system.current_pose.theta);
    
    // Pure Pursuit算法计算曲率
    float lookahead_distance = sqrtf(dx*dx + dy*dy);
    float curvature = 2.0f * sinf(angle_error) / lookahead_distance;
    
    // 计算目标速度（基于用户设定 + 自适应调整）
    float target_linear_speed = nav_adaptive_speed_calculation(nav_system.speed_config.desired_speed);
    
    // 计算角速度
    float target_angular_speed = curvature * target_linear_speed;
    
    // 角速度限制
    if (target_angular_speed > NAV_MAX_ANGULAR_VEL) {
        target_angular_speed = NAV_MAX_ANGULAR_VEL;
    } else if (target_angular_speed < -NAV_MAX_ANGULAR_VEL) {
        target_angular_speed = -NAV_MAX_ANGULAR_VEL;
    }
    
    // 更新控制输出
    nav_system.output.linear_velocity = target_linear_speed;
    nav_system.output.angular_velocity = target_angular_speed;
    nav_system.output.cross_track_error = lookahead_distance * sinf(angle_error);
    nav_point2d_t current_pos = {nav_system.current_pose.x, nav_system.current_pose.y};
    nav_system.output.distance_to_target = nav_calculate_distance(
        current_pos, 
        nav_system.path.waypoints[nav_system.path.current_target_index].position
    );
    
    // 运动学逆解：计算左右轮速度
    nav_calculate_wheel_speeds(target_linear_speed, target_angular_speed);
    
    return NAV_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     运动学逆解：计算左右轮速度
//-------------------------------------------------------------------------------------------------------------------
static nav_status_enum nav_calculate_wheel_speeds(float linear_vel, float angular_vel)
{
    // 差速运动学逆解
    float left_speed = linear_vel - angular_vel * NAV_WHEELBASE / 2.0f;
    float right_speed = linear_vel + angular_vel * NAV_WHEELBASE / 2.0f;
    
    // 更新输出结构体
    nav_system.output.left_wheel_speed = left_speed;
    nav_system.output.right_wheel_speed = right_speed;
    
    // 发送到电机控制
    motor_set_target_speed(left_speed, right_speed);
    
    return NAV_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查找前瞻点
//-------------------------------------------------------------------------------------------------------------------
static nav_status_enum nav_find_lookahead_point(nav_point2d_t *lookahead_point)
{
    if (nav_system.path.current_target_index >= nav_system.path.total_points) {
        return NAV_STATUS_ERROR;
    }
    
    // 简化版本：直接使用当前目标点作为前瞻点
    nav_waypoint_t *current_target = &nav_system.path.waypoints[nav_system.path.current_target_index];
    lookahead_point->x = current_target->position.x;
    lookahead_point->y = current_target->position.y;
    
    return NAV_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检查目标点是否到达
//-------------------------------------------------------------------------------------------------------------------
static nav_status_enum nav_check_target_reached(void)
{
    if (nav_system.path.current_target_index >= nav_system.path.total_points) {
        return NAV_STATUS_PATH_COMPLETE;
    }
    
    nav_waypoint_t *current_target = &nav_system.path.waypoints[nav_system.path.current_target_index];
    
    nav_point2d_t current_pos = {nav_system.current_pose.x, nav_system.current_pose.y};
    float distance = nav_calculate_distance(current_pos, current_target->position);
    
    if (distance < NAV_TARGET_TOLERANCE) {
        printf("到达路径点 %d: (%.2f, %.2f)\n", 
               nav_system.path.current_target_index,
               current_target->position.x, 
               current_target->position.y);
        
        nav_system.path.current_target_index++;
        
        // 检查是否完成路径
        if (nav_system.path.current_target_index >= nav_system.path.total_points) {
            if (nav_system.path.loop_mode) {
                nav_system.path.current_target_index = 0;  // 循环模式重新开始
                printf("路径循环，重新开始\n");
            } else {
                nav_system.status = NAV_STATUS_PATH_COMPLETE;
                motor_set_target_speed(0.0f, 0.0f);  // 停车
                printf("路径跟踪完成！\n");
            }
        }
    }
    
    return NAV_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     自适应速度计算
//-------------------------------------------------------------------------------------------------------------------
static float nav_adaptive_speed_calculation(float base_speed)
{
    if (!nav_system.speed_config.adaptive_speed_enable) {
        return base_speed;
    }
    
    float adaptive_speed = base_speed;
    
    // 根据横向误差调整速度
    float lateral_error = fabsf(nav_system.output.cross_track_error);
    if (lateral_error > 0.1f) {
        adaptive_speed *= 0.8f;  // 偏离路径时减速
    }
    
    // 根据角速度调整速度
    float angular_vel = fabsf(nav_system.output.angular_velocity);
    if (angular_vel > 1.0f) {
        adaptive_speed *= 0.7f;  // 转弯时减速
    }
    
    // 接近目标点时减速
    if (nav_system.output.distance_to_target < 0.3f) {
        adaptive_speed *= 0.6f;
    }
    
    return adaptive_speed;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算两点间距离
//-------------------------------------------------------------------------------------------------------------------
static float nav_calculate_distance(nav_point2d_t p1, nav_point2d_t p2)
{
    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    return sqrtf(dx*dx + dy*dy);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     角度归一化到(-π, π]
//-------------------------------------------------------------------------------------------------------------------
static float nav_normalize_angle(float angle)
{
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle <= -M_PI) angle += 2.0f * M_PI;
    return angle;
}
