/*********************************************************************************************************************
* 文件名称          nav_control.h
* 功能说明          导航控制系统头文件
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                备注
* 2025-09-19        LittleMaster        创建导航控制系统，支持高速长距离精确导航
*
* 文件作用说明：
* 本文件定义了完整的导航控制系统，包括：
* 1. 多传感器融合（编码器 + SCH16TK10 IMU）
* 2. Pure Pursuit路径跟随算法
* 3. 动态速度控制和规划
* 4. 长距离精度保证机制
* 5. 安全监控和容错处理
*
********************************************************************************************************************/

#ifndef _NAV_CONTROL_H_
#define _NAV_CONTROL_H_

#include "zf_common_typedef.h"
#include "driver_encoder.h"
#include "driver_sch16tk10.h"
#include "driver_flash_road.h"
#include "motor_control.h"
#include <math.h>

//=================================================配置参数定义================================================
// 导航系统基础参数
#define NAV_MAX_SPEED               3.8f        // 最大速度 m/s
#define NAV_MIN_SPEED               3.5f        // 最小速度 m/s
#define NAV_DEFAULT_SPEED           2.0f        // 默认速度 m/s

// Pure Pursuit算法参数
#define NAV_MIN_LOOKAHEAD           0.5f        // 最小前瞻距离 m
#define NAV_MAX_LOOKAHEAD           2.0f        // 最大前瞻距离 m
#define NAV_LOOKAHEAD_FACTOR        0.6f        // 速度-前瞻系数
#define NAV_WHEELBASE               0.26f       // 轮距 m

// 精度控制参数
#define NAV_WAYPOINT_TOLERANCE      0.15f       // 路径点容差 m
#define NAV_HEADING_TOLERANCE       0.087f      // 航向容差 rad (5°)
#define NAV_POSITION_TOLERANCE      0.1f        // 位置容差 m
#define NAV_PATH_DEVIATION_MAX      0.5f        // 最大路径偏离距离 m

// 速度控制参数
#define NAV_MAX_ACCELERATION        2.0f        // 最大加速度 
#define NAV_MAX_DECELERATION        3.0f        // 最大减速度 
#define NAV_MAX_JERK                5.0f        // 最大加加速度 

// 传感器融合参数
#define NAV_ENCODER_WEIGHT          0.7f        // 编码器权重
#define NAV_IMU_WEIGHT              0.3f        // IMU权重
#define NAV_FUSION_RATE             100         // 融合频率 Hz
#define NAV_CORRECTION_RATE         10          // 校正频率 Hz

// 系统控制参数
#define NAV_CONTROL_PERIOD_MS       10          // 控制周期 ms (100Hz)
#define NAV_MAX_WAYPOINTS           50          // 最大路径点数量
#define NAV_SAFETY_CHECK_INTERVAL   5           // 安全检查间隔 (控制周期倍数)

//=================================================枚举类型定义================================================
// 导航系统状态枚举
typedef enum {
    NAV_STATUS_IDLE             = 0,        // 空闲状态
    NAV_STATUS_INITIALIZING     = 1,        // 初始化中
    NAV_STATUS_PATH_LOADING     = 2,        // 路径加载中
    NAV_STATUS_NAVIGATING       = 3,        // 导航中
    NAV_STATUS_PAUSED           = 4,        // 暂停
    NAV_STATUS_COMPLETED        = 5,        // 完成
    NAV_STATUS_ERROR            = 6,        // 错误状态
    NAV_STATUS_EMERGENCY_STOP   = 7         // 紧急停车
} nav_status_enum;

// 导航模式枚举
typedef enum {
    NAV_MODE_PURE_PURSUIT       = 0,        // Pure Pursuit模式
    NAV_MODE_PRECISE            = 1,        // 精确模式
    NAV_MODE_HIGH_SPEED         = 2,        // 高速模式
    NAV_MODE_SAFE               = 3         // 安全模式
} nav_mode_enum;

// 传感器状态枚举
typedef enum {
    NAV_SENSOR_OK               = 0,        // 传感器正常
    NAV_SENSOR_WARNING          = 1,        // 传感器警告
    NAV_SENSOR_ERROR            = 2,        // 传感器错误
    NAV_SENSOR_OFFLINE          = 3         // 传感器离线
} nav_sensor_status_enum;

//=================================================数据结构定义================================================
// 2D位置结构
typedef struct {
    float x;                                // X坐标 m
    float y;                                // Y坐标 m
} nav_position_t;

// 2D速度结构
typedef struct {
    float vx;                               // X方向速度 m/s
    float vy;                               // Y方向速度 m/s
    float linear;                           // 线速度 m/s
    float angular;                          // 角速度 rad/s
} nav_velocity_t;

// 机器人姿态结构
typedef struct {
    nav_position_t position;                // 位置
    float heading;                          // 航向角 rad
    nav_velocity_t velocity;                // 速度
    uint32 timestamp_ms;                    // 时间戳 ms
} nav_pose_t;

// 路径点结构
typedef struct {
    nav_position_t position;                // 位置
    float target_speed;                     // 目标速度 m/s
    float tolerance;                        // 到达容差 m
    uint8 point_type;                       // 路径点类型
} nav_waypoint_t;

// 路径结构
typedef struct {
    nav_waypoint_t waypoints[NAV_MAX_WAYPOINTS];  // 路径点数组
    uint16 waypoint_count;                  // 路径点数量
    uint16 current_waypoint;                // 当前目标路径点索引
    float total_length;                     // 路径总长度 m
    char path_name[32];                     // 路径名称
} nav_path_t;

// 传感器融合状态结构
typedef struct {
    // 编码器数据
    nav_pose_t encoder_pose;                // 编码器位姿
    nav_sensor_status_enum encoder_status;  // 编码器状态
    
    // IMU数据
    float imu_angular_velocity[3];          // IMU角速度 [x,y,z] rad/s
    float imu_acceleration[3];              // IMU加速度 [x,y,z] m/s?
    float imu_temperature;                  // IMU温度 °C
    nav_sensor_status_enum imu_status;      // IMU状态
    
    // 融合结果
    nav_pose_t fused_pose;                  // 融合后位姿
    float confidence;                       // 融合置信度 0-1
    uint32 last_update_ms;                  // 最后更新时间
} nav_sensor_fusion_t;

// Pure Pursuit控制结构
typedef struct {
    nav_position_t lookahead_point;         // 前瞻点
    float lookahead_distance;               // 前瞻距离 m
    float curvature;                        // 曲率 1/m
    float steering_angle;                   // 转向角 rad
    float cross_track_error;                // 横向偏差 m
    uint16 target_waypoint_index;           // 目标路径点索引
} nav_pure_pursuit_t;

// 速度控制结构
typedef struct {
    float target_speed;                     // 目标速度 m/s
    float current_speed;                    // 当前速度 m/s
    float left_wheel_speed;                 // 左轮速度 m/s
    float right_wheel_speed;                // 右轮速度 m/s
    float acceleration;                     // 当前加速度 m/s?
    float max_speed_for_curvature;          // 曲率限制的最大速度 m/s
} nav_speed_control_t;

// 安全监控结构
typedef struct {
    uint8 emergency_stop_triggered;         // 紧急停车标志
    float path_deviation;                   // 路径偏离距离 m
    uint32 sensor_timeout_count;            // 传感器超时计数
    uint32 control_loop_overrun_count;      // 控制循环超时计数
    uint32 last_safety_check_ms;            // 最后安全检查时间
    uint8 sensor_failure_flags;             // 传感器故障标志位
} nav_safety_monitor_t;

// 导航系统统计信息
typedef struct {
    float total_distance_traveled;          // 总行驶距离 m
    float average_speed;                    // 平均速度 m/s
    float max_speed_reached;                // 达到的最大速度 m/s
    uint32 navigation_start_time_ms;        // 导航开始时间
    uint32 total_navigation_time_ms;        // 总导航时间
    uint16 waypoints_reached;               // 已到达路径点数
    float max_cross_track_error;            // 最大横向误差 m
    float average_cross_track_error;        // 平均横向误差 m
} nav_statistics_t;

// 导航系统主结构
typedef struct {
    // 系统状态
    nav_status_enum status;                 // 导航状态
    nav_mode_enum mode;                     // 导航模式
    uint8 initialized;                      // 初始化标志
    
    // 路径信息
    nav_path_t current_path;                // 当前路径
    
    // 传感器融合
    nav_sensor_fusion_t sensor_fusion;      // 传感器融合
    
    // 控制算法
    nav_pure_pursuit_t pure_pursuit;        // Pure Pursuit控制
    nav_speed_control_t speed_control;      // 速度控制
    
    // 安全监控
    nav_safety_monitor_t safety_monitor;    // 安全监控
    
    // 统计信息
    nav_statistics_t statistics;            // 统计信息
    
    // 时间管理
    uint32 last_control_update_ms;          // 最后控制更新时间
    uint32 control_loop_count;              // 控制循环计数
    
} nav_system_t;

//=================================================全局变量声明================================================
extern nav_system_t nav_system;

//=================================================函数声明================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导航系统初始化
// 参数说明     void
// 返回参数     uint8               初始化结果 (1=成功, 0=失败)
// 使用示例     nav_init();
// 备注信息     初始化导航系统，配置传感器和控制参数
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_init(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     加载导航路径
// 参数说明     path_name           路径名称
// 返回参数     uint8               加载结果 (1=成功, 0=失败)
// 使用示例     nav_load_path("主路径");
// 备注信息     从Flash加载指定路径到导航系统
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_load_path(const char *path_name);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     开始导航
// 参数说明     mode                导航模式
// 返回参数     uint8               启动结果 (1=成功, 0=失败)
// 使用示例     nav_start(NAV_MODE_HIGH_SPEED);
// 备注信息     启动导航系统，开始路径跟随
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_start(nav_mode_enum mode);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     停止导航
// 参数说明     emergency           是否紧急停车 (1=是, 0=否)
// 返回参数     uint8               停止结果 (1=成功, 0=失败)
// 使用示例     nav_stop(0);
// 备注信息     停止导航系统，可选择紧急停车模式
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_stop(uint8 emergency);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导航系统主循环更新
// 参数说明     void
// 返回参数     uint8               更新结果 (1=成功, 0=失败)
// 使用示例     nav_update(); // 在主循环中以100Hz频率调用
// 备注信息     导航系统的核心更新函数，需要定期调用
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_update(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     暂停/恢复导航
// 参数说明     pause               暂停标志 (1=暂停, 0=恢复)
// 返回参数     uint8               操作结果 (1=成功, 0=失败)
// 使用示例     nav_pause(1);
// 备注信息     暂停或恢复导航，暂停时保持当前位置
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_pause(uint8 pause);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取导航系统状态
// 参数说明     void
// 返回参数     nav_status_enum     当前导航状态
// 使用示例     status = nav_get_status();
// 备注信息     获取当前导航系统的运行状态
//-------------------------------------------------------------------------------------------------------------------
nav_status_enum nav_get_status(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取当前位置
// 参数说明     position            位置输出指针
// 返回参数     uint8               获取结果 (1=成功, 0=失败)
// 使用示例     nav_get_current_position(&pos);
// 备注信息     获取当前小车的融合位置信息
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_get_current_position(nav_position_t *position);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取导航统计信息
// 参数说明     stats               统计信息输出指针
// 返回参数     uint8               获取结果 (1=成功, 0=失败)
// 使用示例     nav_get_statistics(&stats);
// 备注信息     获取导航过程的统计信息
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_get_statistics(nav_statistics_t *stats);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置导航参数
// 参数说明     max_speed           最大速度 m/s
// 参数说明     lookahead_factor    前瞻距离系数
// 返回参数     uint8               设置结果 (1=成功, 0=失败)
// 使用示例     nav_set_parameters(3.5f, 0.6f);
// 备注信息     动态调整导航系统参数
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_set_parameters(float max_speed, float lookahead_factor);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取路径跟随误差
// 参数说明     void
// 返回参数     float               横向偏差 m
// 使用示例     error = nav_get_cross_track_error();
// 备注信息     获取当前的路径跟随横向误差
//-------------------------------------------------------------------------------------------------------------------
float nav_get_cross_track_error(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检查是否到达目标
// 参数说明     void
// 返回参数     uint8               到达状态 (1=已到达, 0=未到达)
// 使用示例     arrived = nav_is_target_reached();
// 备注信息     检查是否已到达路径终点
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_is_target_reached(void);

#endif // _NAV_CONTROL_H_

