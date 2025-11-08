/*********************************************************************************************************************
* 文件名称          nav_control_ahrs.h
* 功能说明          基于航向角跟踪的导航控制系统头文件
* 作者              AI Assistant
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本
* 2025-01-XX        AI Assistant       1.0v
* 
* 文件作用说明：
* 本文件实现类似dog项目的航向角跟踪导航系统
* 核心思路：记录轨迹上每个点的航向角，回放时跟踪目标航向角
* 
* 主要特性：
* 1. 预生成正方形路径（1m×1m）
* 2. 基于里程的前瞻点查找
* 3. 动态曲率计算
* 4. 双PID控制策略（直线/转弯）
********************************************************************************************************************/

#ifndef _NAV_CONTROL_AHRS_H_
#define _NAV_CONTROL_AHRS_H_

#include "zf_common_typedef.h"

// 避免M_PI重复定义
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

//=================================================配置参数================================================
#define NAV_AHRS_MAX_POINTS         2000        // 最大路径点数
#define NAV_AHRS_DISTANCE_PER_POINT 4.0f        // 每个点的距离间隔 (mm)
#define NAV_AHRS_WHEELBASE          0.134f      // 轮距 (m)
#define NAV_AHRS_WHEEL_DIAMETER     0.065f      // 轮径 (m)

// 启动延时配置（假设更新周期为10ms）
#define NAV_AHRS_STABLE_WAIT_TIME_MS   2000     // 陀螺仪校准后的稳定等待时间 (ms)
#define NAV_AHRS_UPDATE_PERIOD_MS      10       // 导航系统更新周期 (ms)

//=================================================枚举定义================================================
// 导航状态枚举
typedef enum {
    NAV_AHRS_STATUS_OK = 0,
    NAV_AHRS_STATUS_ERROR,
    NAV_AHRS_STATUS_NOT_INIT,
    NAV_AHRS_STATUS_PATH_END,
    NAV_AHRS_STATUS_WAITING_CALIB,      // 等待陀螺仪校准
    NAV_AHRS_STATUS_WAITING_STABLE      // 等待系统稳定
} nav_ahrs_status_enum;

// 导航模式枚举
typedef enum {
    NAV_AHRS_MODE_IDLE = 0,     // 空闲
    NAV_AHRS_MODE_REPLAY,       // 回放模式
    NAV_AHRS_MODE_PAUSE         // 暂停
} nav_ahrs_mode_enum;

//=================================================数据结构定义================================================
// 路径点结构体
typedef struct {
    float yaw;              // 该点的航向角 (度)
    float distance;         // 该点的累积距离 (mm)
} nav_ahrs_waypoint_t;

// 路径数据结构
typedef struct {
    nav_ahrs_waypoint_t points[NAV_AHRS_MAX_POINTS];   // 路径点数组
    uint16 total_points;                                 // 总路径点数
    uint16 current_index;                                // 当前跟踪点索引
    uint8 loop_mode;                                     // 循环模式标志
} nav_ahrs_path_t;

// PID控制器结构体
typedef struct {
    float kp;               // 比例系数
    float ki;               // 积分系数
    float kd;               // 微分系数
    float error;            // 当前误差
    float error_last;       // 上次误差
    float integral;         // 积分累积
    float output;           // 输出值
    float integral_limit;   // 积分限幅
} nav_ahrs_pid_t;

// 导航控制器结构体
typedef struct {
    // 路径数据
    nav_ahrs_path_t path;
    
    // 当前状态
    float current_yaw;          // 当前航向角 (度)
    float current_distance;     // 当前累积距离 (mm)
    float left_distance;        // 左轮累积距离 (mm)
    float right_distance;       // 右轮累积距离 (mm)
    
    // 目标状态
    float target_yaw;           // 目标航向角 (度)
    float angle_error;          // 角度误差 (度)
    
    // 前瞻控制
    uint16 lookahead_index;     // 前瞻点索引
    float lookahead_distance;   // 前瞻距离 (mm)
    float curvature;            // 当前路径曲率
    
    // PID控制器
    nav_ahrs_pid_t pid_turn;    // 转弯PID
    nav_ahrs_pid_t pid_straight;// 直线PID
    
    // 速度控制
    float base_speed;           // 基础速度 (m/s)
    float left_speed;           // 左轮目标速度 (m/s)
    float right_speed;          // 右轮目标速度 (m/s)
    
    // 系统状态
    nav_ahrs_mode_enum mode;
    nav_ahrs_status_enum status;
    uint8 initialized;
    uint8 path_loaded;
    
    // 启动延时控制
    uint8 ahrs_calibration_done;        // AHRS校准完成标志
    uint32 stable_wait_counter;         // 稳定等待计数器
    uint32 stable_wait_required;        // 需要的稳定等待次数
    uint8 ready_to_start;               // 准备好启动标志
    
} nav_ahrs_controller_t;

//=================================================全局变量声明================================================
extern nav_ahrs_controller_t nav_ahrs;

//=================================================函数声明================================================

/**
 * @brief  初始化AHRS导航系统
 * @retval 状态
 */
nav_ahrs_status_enum nav_ahrs_init(void);

/**
 * @brief  导航系统更新（周期调用）
 * @param  dt       时间间隔 (s)
 * @retval 状态
 * @note   建议在5-10ms定时器中调用
 */
nav_ahrs_status_enum nav_ahrs_update(float dt);

/**
 * @brief  获取电机控制输出
 * @param  left_pwm     左轮PWM输出
 * @param  right_pwm    右轮PWM输出
 * @retval 状态
 */
nav_ahrs_status_enum nav_ahrs_get_motor_output(int32 *left_pwm, int32 *right_pwm);

/**
 * @brief  设置导航模式
 * @param  mode     导航模式
 * @retval 状态
 */
nav_ahrs_status_enum nav_ahrs_set_mode(nav_ahrs_mode_enum mode);

/**
 * @brief  设置基础速度
 * @param  speed    速度 (m/s)
 * @retval 状态
 */
nav_ahrs_status_enum nav_ahrs_set_speed(float speed);

/**
 * @brief  重置导航系统
 * @retval 状态
 */
nav_ahrs_status_enum nav_ahrs_reset(void);

/**
 * @brief  获取导航状态信息
 * @param  yaw          当前航向角
 * @param  target_yaw   目标航向角
 * @param  error        角度误差
 * @param  curvature    路径曲率
 * @retval 状态
 */
nav_ahrs_status_enum nav_ahrs_get_status(float *yaw, float *target_yaw, 
                                          float *error, float *curvature);

/**
 * @brief  设置PID参数
 * @param  is_turn      1-转弯PID，0-直线PID
 * @param  kp           比例系数
 * @param  ki           积分系数
 * @param  kd           微分系数
 * @retval 状态
 */
nav_ahrs_status_enum nav_ahrs_set_pid(uint8 is_turn, float kp, float ki, float kd);

#endif // _NAV_CONTROL_AHRS_H_

