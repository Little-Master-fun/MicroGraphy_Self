/*********************************************************************************************************************
* 文件名称          test_nav_ahrs.c
* 功能说明          AHRS导航系统测试文件
* 作者              AI Assistant
* 版本信息          v1.2
* 修改记录
* 日期              作者                版本
* 2025-01-XX        AI Assistant       1.0v
* 2025-01-XX        AI Assistant       1.1v - 简化为一键启动
* 2025-01-XX        AI Assistant       1.2v - 添加路径生成函数
* 
* 文件作用说明：
* 本文件提供AHRS导航系统的测试路径生成和启动函数
********************************************************************************************************************/

#include "test_nav_ahrs.h"
#include "nav_control_ahrs.h"
#include "imu_ahrs_complementary.h"
#include "driver_encoder.h"
#include "motor_control.h"
#include "zf_common_headfile.h"
#include <string.h>
#include <math.h>

//=================================================路径生成函数================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     生成正方形测试路径
// 参数说明     size            正方形边长 (m)
// 返回参数     状态
//-------------------------------------------------------------------------------------------------------------------
nav_ahrs_status_enum test_nav_ahrs_generate_square_path(float size)
{
    if (!nav_ahrs.initialized) {
        return NAV_AHRS_STATUS_NOT_INIT;
    }
    
    // 计算参数
    float perimeter = 4.0f * size;  // 周长 (m)
    float total_distance = perimeter * 1000.0f;  // 总距离 (mm)
    
    // 每条边的距离 (mm)
    float distance_per_side = total_distance / 4.0f;
    
    // 生成路径点
    uint16 point_idx = 0;
    float current_distance = 0.0f;
    
    // 第一条边：0° (向右)
    float yaw = 0.0f;
    while (current_distance < distance_per_side && point_idx < NAV_AHRS_MAX_POINTS) {
        nav_ahrs.path.points[point_idx].yaw = yaw;
        nav_ahrs.path.points[point_idx].distance = current_distance;
        point_idx++;
        current_distance += NAV_AHRS_DISTANCE_PER_POINT;
    }
    
    // 第二条边：90° (向上)
    yaw = 90.0f;
    float side2_end = 2.0f * distance_per_side;
    while (current_distance < side2_end && point_idx < NAV_AHRS_MAX_POINTS) {
        nav_ahrs.path.points[point_idx].yaw = yaw;
        nav_ahrs.path.points[point_idx].distance = current_distance;
        point_idx++;
        current_distance += NAV_AHRS_DISTANCE_PER_POINT;
    }
    
    // 第三条边：180° (向左)
    yaw = 180.0f;
    float side3_end = 3.0f * distance_per_side;
    while (current_distance < side3_end && point_idx < NAV_AHRS_MAX_POINTS) {
        nav_ahrs.path.points[point_idx].yaw = yaw;
        nav_ahrs.path.points[point_idx].distance = current_distance;
        point_idx++;
        current_distance += NAV_AHRS_DISTANCE_PER_POINT;
    }
    
    // 第四条边：270° (向下)
    yaw = 270.0f;
    while (current_distance < total_distance && point_idx < NAV_AHRS_MAX_POINTS) {
        nav_ahrs.path.points[point_idx].yaw = yaw;
        nav_ahrs.path.points[point_idx].distance = current_distance;
        point_idx++;
        current_distance += NAV_AHRS_DISTANCE_PER_POINT;
    }
    
    // 添加转弯过渡点（平滑转角）
    // 在每个转角前后添加过渡角度
    uint16 corner_indices[] = {
        (uint16)(distance_per_side / NAV_AHRS_DISTANCE_PER_POINT),           // 第一个转角
        (uint16)(2 * distance_per_side / NAV_AHRS_DISTANCE_PER_POINT),       // 第二个转角
        (uint16)(3 * distance_per_side / NAV_AHRS_DISTANCE_PER_POINT)        // 第三个转角
    };
    
    // 平滑转角（添加过渡段）
    for (int i = 0; i < 3; i++) {
        uint16 corner_idx = corner_indices[i];
        if (corner_idx > 5 && corner_idx < point_idx - 5) {
            float angle_start = nav_ahrs.path.points[corner_idx - 5].yaw;
            float angle_end = nav_ahrs.path.points[corner_idx + 5].yaw;
            
            // 处理角度跨越问题
            if (fabs(angle_end - angle_start) > 180.0f) {
                if (angle_end > angle_start) {
                    angle_start += 360.0f;
                } else {
                    angle_end += 360.0f;
                }
            }
            
            // 线性插值过渡段
            for (int j = -4; j <= 4; j++) {
                float ratio = (j + 4) / 8.0f;
                float smooth_yaw = angle_start + (angle_end - angle_start) * ratio;
                // 归一化角度
                while (smooth_yaw > 180.0f) smooth_yaw -= 360.0f;
                while (smooth_yaw <= -180.0f) smooth_yaw += 360.0f;
                nav_ahrs.path.points[corner_idx + j].yaw = smooth_yaw;
            }
        }
    }
    
    nav_ahrs.path.total_points = point_idx;
    nav_ahrs.path.current_index = 0;
    nav_ahrs.path.loop_mode = 1;  // 循环模式
    nav_ahrs.path_loaded = 1;
    
    return NAV_AHRS_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     生成直线测试路径
// 参数说明     length          直线长度 (m)
// 参数说明     direction       前进方向角度 (度), 0=向右, 90=向上, 180=向左, 270=向下
// 返回参数     状态
//-------------------------------------------------------------------------------------------------------------------
nav_ahrs_status_enum test_nav_ahrs_generate_straight_path(float length, float direction)
{
    if (!nav_ahrs.initialized) {
        return NAV_AHRS_STATUS_NOT_INIT;
    }
    
    // 计算参数
    float total_distance = length * 1000.0f;  // 总距离 (mm)
    
    // 生成路径点
    uint16 point_idx = 0;
    float current_distance = 0.0f;
    
    // 沿直线方向生成点
    while (current_distance <= total_distance && point_idx < NAV_AHRS_MAX_POINTS) {
        nav_ahrs.path.points[point_idx].yaw = direction;
        nav_ahrs.path.points[point_idx].distance = current_distance;
        point_idx++;
        current_distance += NAV_AHRS_DISTANCE_PER_POINT;
    }
    
    nav_ahrs.path.total_points = point_idx;
    nav_ahrs.path.current_index = 0;
    nav_ahrs.path.loop_mode = 0;  // 非循环模式
    nav_ahrs.path_loaded = 1;
    
    return NAV_AHRS_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     生成长方形测试路径
// 参数说明     length          长方形长边 (m)
// 参数说明     width           长方形短边 (m)
// 返回参数     状态
//-------------------------------------------------------------------------------------------------------------------
nav_ahrs_status_enum test_nav_ahrs_generate_rectangle_path(float length, float width)
{
    if (!nav_ahrs.initialized) {
        return NAV_AHRS_STATUS_NOT_INIT;
    }
    
    // 计算参数
    float perimeter = 2.0f * (length + width);  // 周长 (m)
    float total_distance = perimeter * 1000.0f;  // 总距离 (mm)
    
    // 每条边的距离 (mm)
    float distance_long = length * 1000.0f;   // 长边距离
    float distance_short = width * 1000.0f;   // 短边距离
    
    // 生成路径点
    uint16 point_idx = 0;
    float current_distance = 0.0f;
    
    // 第一条边：0° (向右, 长边)
    float yaw = 0.0f;
    float side1_end = distance_long;
    while (current_distance < side1_end && point_idx < NAV_AHRS_MAX_POINTS) {
        nav_ahrs.path.points[point_idx].yaw = yaw;
        nav_ahrs.path.points[point_idx].distance = current_distance;
        point_idx++;
        current_distance += NAV_AHRS_DISTANCE_PER_POINT;
    }
    
    // 第二条边：90° (向上, 短边)
    yaw = 90.0f;
    float side2_end = distance_long + distance_short;
    while (current_distance < side2_end && point_idx < NAV_AHRS_MAX_POINTS) {
        nav_ahrs.path.points[point_idx].yaw = yaw;
        nav_ahrs.path.points[point_idx].distance = current_distance;
        point_idx++;
        current_distance += NAV_AHRS_DISTANCE_PER_POINT;
    }
    
    // 第三条边：180° (向左, 长边)
    yaw = 180.0f;
    float side3_end = 2.0f * distance_long + distance_short;
    while (current_distance < side3_end && point_idx < NAV_AHRS_MAX_POINTS) {
        nav_ahrs.path.points[point_idx].yaw = yaw;
        nav_ahrs.path.points[point_idx].distance = current_distance;
        point_idx++;
        current_distance += NAV_AHRS_DISTANCE_PER_POINT;
    }
    
    // 第四条边：270° (向下, 短边)
    yaw = 270.0f;
    while (current_distance < total_distance && point_idx < NAV_AHRS_MAX_POINTS) {
        nav_ahrs.path.points[point_idx].yaw = yaw;
        nav_ahrs.path.points[point_idx].distance = current_distance;
        point_idx++;
        current_distance += NAV_AHRS_DISTANCE_PER_POINT;
    }
    
    // 添加转弯过渡点（平滑转角）
    uint16 corner_indices[] = {
        (uint16)(distance_long / NAV_AHRS_DISTANCE_PER_POINT),                           // 第一个转角
        (uint16)((distance_long + distance_short) / NAV_AHRS_DISTANCE_PER_POINT),        // 第二个转角
        (uint16)((2.0f * distance_long + distance_short) / NAV_AHRS_DISTANCE_PER_POINT)  // 第三个转角
    };
    
    // 平滑转角（添加过渡段）
    for (int i = 0; i < 3; i++) {
        uint16 corner_idx = corner_indices[i];
        if (corner_idx > 5 && corner_idx < point_idx - 5) {
            float angle_start = nav_ahrs.path.points[corner_idx - 5].yaw;
            float angle_end = nav_ahrs.path.points[corner_idx + 5].yaw;
            
            // 处理角度跨越问题
            if (fabs(angle_end - angle_start) > 180.0f) {
                if (angle_end > angle_start) {
                    angle_start += 360.0f;
                } else {
                    angle_end += 360.0f;
                }
            }
            
            // 线性插值过渡段
            for (int j = -4; j <= 4; j++) {
                float ratio = (j + 4) / 8.0f;
                float smooth_yaw = angle_start + (angle_end - angle_start) * ratio;
                // 归一化角度
                while (smooth_yaw > 180.0f) smooth_yaw -= 360.0f;
                while (smooth_yaw <= -180.0f) smooth_yaw += 360.0f;
                nav_ahrs.path.points[corner_idx + j].yaw = smooth_yaw;
            }
        }
    }
    
    nav_ahrs.path.total_points = point_idx;
    nav_ahrs.path.current_index = 0;
    nav_ahrs.path.loop_mode = 1;  // 循环模式
    nav_ahrs.path_loaded = 1;
    
    return NAV_AHRS_STATUS_OK;
}

//=================================================核心函数================================================

/**
 * @brief  一键启动AHRS导航系统
 * @note   调用此函数完成所有初始化并开始导航
 * @return 是否成功
 */
uint8 test_nav_ahrs_quick_start(void)
{
    // 1. 初始化AHRS姿态解算系统
    if (ahrs_complementary_init() != AHRS_STATUS_OK) {
        return 0;
    }
    
    // 2. 初始化编码器
    if (encoder_init() != ENCODER_STATUS_OK) {
        return 0;
    }
    
    // 3. 初始化电机控制
    if (motor_pid_init() != MOTOR_PID_OK) {
        return 0;
    }
    
    // 4. 初始化AHRS导航系统
    if (nav_ahrs_init() != NAV_AHRS_STATUS_OK) {
        return 0;
    }
    
    // 5. 生成1m×1m正方形路径
    if (test_nav_ahrs_generate_square_path(1.0f) != NAV_AHRS_STATUS_OK) {
        return 0;
    }
    
    // 6. 启动导航
    nav_ahrs_reset();                      // 重置状态
    nav_ahrs_set_speed(2.5f);              // 设置基础速度 2.5 m/s
    nav_ahrs_set_mode(NAV_AHRS_MODE_REPLAY);  // 启动回放模式
    
    return 1;
}

/**
 * @brief  一键启动AHRS导航系统（直线路径）
 * @param  length       直线长度 (m)
 * @param  direction    方向角度 (度)
 * @note   调用此函数完成所有初始化并开始直线导航
 * @return 是否成功
 */
uint8 test_nav_ahrs_quick_start_straight(float length, float direction)
{
    // 1. 初始化AHRS姿态解算系统
    if (ahrs_complementary_init() != AHRS_STATUS_OK) {
        return 0;
    }
    
    // 2. 初始化编码器
    if (encoder_init() != ENCODER_STATUS_OK) {
        return 0;
    }
    
    // 3. 初始化电机控制
    if (motor_pid_init() != MOTOR_PID_OK) {
        return 0;
    }
    
    // 4. 初始化AHRS导航系统
    if (nav_ahrs_init() != NAV_AHRS_STATUS_OK) {
        return 0;
    }
    
    // 5. 生成直线路径
    if (test_nav_ahrs_generate_straight_path(length, direction) != NAV_AHRS_STATUS_OK) {
        return 0;
    }
    
    // 6. 启动导航
    nav_ahrs_reset();                      // 重置状态
    nav_ahrs_set_speed(2.5f);              // 设置基础速度 2.5 m/s
    nav_ahrs_set_mode(NAV_AHRS_MODE_REPLAY);  // 启动回放模式
    
    return 1;
}

/**
 * @brief  导航系统主循环（在定时器中调用）
 * @note   建议5-10ms周期调用
 * @param  dt: 更新周期（秒），例如0.01表示10ms
 */
void test_nav_ahrs_loop(float dt)
{
    // 1. 更新导航系统
    nav_ahrs_update(dt);
    
    // 2. 执行电机控制（内部自动调用motor_set_target_speed）
    nav_ahrs_get_motor_output(NULL, NULL);
}

