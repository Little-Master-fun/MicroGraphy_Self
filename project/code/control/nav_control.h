/*********************************************************************************************************************
* 文件名称          nav_control.h
* 功能说明          导航控制系统头文件
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本
* 2025-09-22        LittleMaster       1.0v
* 
* 文件作用说明：
* 本文件为导航控制系统的头文件，提供路径跟踪和运动控制功能
* 
* 主要功能：
* 1. 路径点管理和路径跟踪
* 2. Pure Pursuit路径跟踪算法
* 3. 差速运动学解算
* 4. 多传感器数据融合定位
* 5. 用户速度控制接口
********************************************************************************************************************/

#ifndef _NAV_CONTROL_H_
#define _NAV_CONTROL_H_

#include "zf_common_typedef.h"
#include <math.h>

//=================================================配置参数定义================================================
#define NAV_MAX_WAYPOINTS       (50)        // 最大路径点数量
#define NAV_LOOKAHEAD_DISTANCE  (0.3f)      // Pure Pursuit前瞻距离 (m)
#define NAV_WHEELBASE          (0.2f)       // 轮间距 (m)
#define NAV_MIN_TURN_RADIUS    (0.15f)      // 最小转弯半径 (m)
#define NAV_PATH_TOLERANCE     (0.05f)      // 路径跟踪容差 (m)
#define NAV_TARGET_TOLERANCE   (0.1f)       // 目标点到达容差 (m)
#define NAV_MAX_ANGULAR_VEL    (2.0f)       // 最大角速度 (rad/s)
#define NAV_MAX_ACCELERATION   (1.0f)       // 最大加速度 (m/s?)

//=================================================枚举类型定义================================================
// 导航系统状态枚举
typedef enum
{
    NAV_STATUS_OK           = 0,    // 正常运行
    NAV_STATUS_ERROR        = 1,    // 系统错误
    NAV_STATUS_NOT_INIT     = 2,    // 未初始化
    NAV_STATUS_NO_PATH      = 3,    // 无路径
    NAV_STATUS_PATH_COMPLETE = 4,   // 路径完成
    NAV_STATUS_OFF_TRACK    = 5,    // 偏离路径
} nav_status_enum;

// 导航模式枚举
typedef enum
{
    NAV_MODE_MANUAL         = 0,    // 手动模式
    NAV_MODE_AUTO           = 1,    // 自动跟踪模式
    NAV_MODE_PAUSE          = 2,    // 暂停模式
    NAV_MODE_EMERGENCY      = 3,    // 紧急停车模式
} nav_mode_enum;

//=================================================数据结构定义================================================
// 2D位置结构体
typedef struct
{
    float x;                        // X坐标 (m)
    float y;                        // Y坐标 (m)
} nav_point2d_t;

// 机器人姿态结构体
typedef struct
{
    float x;                        // X坐标 (m)
    float y;                        // Y坐标 (m)
    float theta;                    // 航向角 (rad)
    float linear_velocity;          // 线速度 (m/s)
    float angular_velocity;         // 角速度 (rad/s)
} nav_pose_t;

// 路径点结构体
typedef struct
{
    nav_point2d_t position;         // 位置坐标
    float target_speed;             // 目标速度 (m/s)
    uint8 point_type;               // 点类型: 0=普通点, 1=停车点, 2=减速点
} nav_waypoint_t;

// 路径管理结构体
typedef struct
{
    nav_waypoint_t waypoints[NAV_MAX_WAYPOINTS]; // 路径点数组
    uint16 total_points;            // 总路径点数
    uint16 current_target_index;    // 当前目标点索引
    uint8 loop_mode;                // 循环模式: 0=单次, 1=循环
    uint8 reverse_mode;             // 反向模式: 0=正向, 1=反向
} nav_path_t;

// 控制输出结构体
typedef struct
{
    float left_wheel_speed;         // 左轮目标速度 (m/s)
    float right_wheel_speed;        // 右轮目标速度 (m/s)
    float linear_velocity;          // 计算得到的线速度 (m/s)
    float angular_velocity;         // 计算得到的角速度 (rad/s)
    float cross_track_error;        // 横向跟踪误差 (m)
    float distance_to_target;       // 到目标点距离 (m)
} nav_control_output_t;

// 速度配置结构体
typedef struct
{
    float desired_speed;            // 用户期望速度 (m/s)
    float max_speed;                // 最大速度限制 (m/s)
    float acceleration_limit;       // 加速度限制 (m/s?)
    float deceleration_limit;       // 减速度限制 (m/s?)
    uint8 adaptive_speed_enable;    // 自适应速度使能
} nav_speed_config_t;

// 导航系统主结构体
typedef struct
{
    nav_path_t path;                // 路径管理
    nav_pose_t current_pose;        // 当前位置姿态
    nav_speed_config_t speed_config; // 速度配置
    nav_control_output_t output;    // 控制输出
    nav_status_enum status;         // 系统状态
    nav_mode_enum mode;             // 导航模式
    uint32 update_count;            // 更新计数
    uint32 last_update_time;        // 上次更新时间
    uint8 system_enabled;           // 系统使能标志
} nav_system_t;

//=================================================全局变量声明================================================
extern nav_system_t nav_system;

//=================================================函数声明================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导航系统初始化
// 参数说明     void
// 返回参数     nav_status_enum     初始化状态
// 使用示例     nav_init();
// 备注信息     初始化导航系统和相关传感器
//-------------------------------------------------------------------------------------------------------------------
nav_status_enum nav_init(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置用户期望速度
// 参数说明     desired_speed       期望速度 (m/s)
// 返回参数     nav_status_enum     设置状态
// 使用示例     nav_set_desired_speed(1.0f);
// 备注信息     设置用户控制的基础巡航速度
//-------------------------------------------------------------------------------------------------------------------
nav_status_enum nav_set_desired_speed(float desired_speed);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     加载预定义路径
// 参数说明     path_type           路径类型 (0=1x1正方形)
// 返回参数     nav_status_enum     加载状态
// 使用示例     nav_load_preset_path(0);
// 备注信息     加载预定义的测试路径
//-------------------------------------------------------------------------------------------------------------------
nav_status_enum nav_load_preset_path(uint8 path_type);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导航系统主更新函数
// 参数说明     void
// 返回参数     nav_status_enum     更新状态
// 使用示例     nav_update();
// 备注信息     周期性调用此函数进行路径跟踪控制，建议50ms调用一次
//-------------------------------------------------------------------------------------------------------------------
nav_status_enum nav_update(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置导航模式
// 参数说明     mode                导航模式
// 返回参数     nav_status_enum     设置状态
// 使用示例     nav_set_mode(NAV_MODE_AUTO);
// 备注信息     切换导航系统工作模式
//-------------------------------------------------------------------------------------------------------------------
nav_status_enum nav_set_mode(nav_mode_enum mode);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取导航系统状态
// 参数说明     system_info         系统状态输出指针
// 返回参数     nav_status_enum     获取状态
// 使用示例     nav_get_system_status(&nav_info);
// 备注信息     获取完整的导航系统状态信息
//-------------------------------------------------------------------------------------------------------------------
nav_status_enum nav_get_system_status(nav_system_t *system_info);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     重置导航系统
// 参数说明     void
// 返回参数     nav_status_enum     重置状态
// 使用示例     nav_reset();
// 备注信息     重置路径跟踪和位置估算
//-------------------------------------------------------------------------------------------------------------------
nav_status_enum nav_reset(void);

#endif // _NAV_CONTROL_H_