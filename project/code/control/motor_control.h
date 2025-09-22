/*********************************************************************************************************************
* 文件名称          motor_control.h
* 功能说明          电机PID闭环控制系统头文件
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本              备注
* 2025-09-19        LittleMaster        v1.0              创建电机PID控制系统
*
* 文件作用说明：
* 本文件为电机PID闭环控制系统的头文件，提供双电机的精确速度控制
* 
* 主要功能：
* 1. 双电机独立PID控制器
* 2. 差速控制支持
* 3. 参数动态调整
* 4. 积分防饱和保护
* 5. 实时状态监控
*
* PID参数配置：
* - 比例系数 Kp: 45.0
* - 积分系数 Ki: 3.0
* - 微分系数 Kd: 0.0
********************************************************************************************************************/

#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_

#include "zf_common_typedef.h"

//=================================================配置参数定义================================================
#define MOTOR_PID_KP_DEFAULT        (45.0f)     // 默认比例系数
#define MOTOR_PID_KI_DEFAULT        (3.0f)      // 默认积分系数
#define MOTOR_PID_KD_DEFAULT        (0.0f)      // 默认微分系数
#define MOTOR_PID_INTEGRAL_MAX      (2000.0f)   // 积分限幅
#define MOTOR_PID_OUTPUT_MAX        (9999.0f)   // 输出限幅
#define MOTOR_PID_MIN_DT            (0.001f)    // 最小时间间隔 (s)

//=================================================枚举类型定义================================================
// 电机ID枚举
typedef enum
{
    MOTOR_PID_LEFT      = 0,                    // 左电机
    MOTOR_PID_RIGHT     = 1,                    // 右电机
    MOTOR_PID_BOTH      = 2,                    // 双电机
} motor_pid_id_enum;

// PID控制器状态枚举
typedef enum
{
    MOTOR_PID_OK        = 0,                    // 正常
    MOTOR_PID_ERROR     = 1,                    // 错误
    MOTOR_PID_NOT_INIT  = 2,                    // 未初始化
} motor_pid_status_enum;

//=================================================数据结构定义================================================
// PID控制器数据结构
typedef struct {
    float kp, ki, kd;          // PID参数
    float integral;            // 积分累积
    float last_error;          // 上次误差
    float integral_max;        // 积分限幅
    float output_max;          // 输出限幅
    uint8 initialized;         // 初始化标志
    
    // 统计信息
    uint32 update_count;       // 更新次数
    float max_error;           // 最大误差记录
    float average_error;       // 平均误差
} motor_pid_controller_t;

// 电机控制系统状态结构
typedef struct {
    motor_pid_controller_t left_pid;            // 左电机PID控制器
    motor_pid_controller_t right_pid;           // 右电机PID控制器
    
    // 系统状态
    uint8 system_enabled;                       // 系统使能状态
    uint32 total_update_count;                  // 总更新次数
    float control_frequency;                    // 控制频率 (Hz)
    
    // 当前控制值
    float left_target_speed;                    // 左电机目标速度
    float right_target_speed;                   // 右电机目标速度
    float left_current_speed;                   // 左电机当前速度
    float right_current_speed;                  // 右电机当前速度
    float left_output;                          // 左电机输出
    float right_output;                         // 右电机输出
} motor_control_system_t;

//=================================================全局变量声明================================================
extern motor_control_system_t motor_control_system;

//=================================================函数声明================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化电机PID控制器
// 参数说明     void
// 返回参数     motor_pid_status_enum   初始化状态
// 使用示例     motor_pid_init();
// 备注信息     使用默认PID参数初始化左右电机控制器 (Kp=45, Ki=3, Kd=0)
//-------------------------------------------------------------------------------------------------------------------
motor_pid_status_enum motor_pid_init(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置电机目标速度
// 参数说明     left_speed          左电机目标速度 (m/s)
// 参数说明     right_speed         右电机目标速度 (m/s)
// 返回参数     motor_pid_status_enum   执行状态
// 使用示例     motor_set_target_speed(1.0f, 1.0f);
// 备注信息     集成PID控制和电机输出的完整控制接口
//-------------------------------------------------------------------------------------------------------------------
motor_pid_status_enum motor_set_target_speed(float left_speed, float right_speed);

#endif // _MOTOR_CONTROL_H_
