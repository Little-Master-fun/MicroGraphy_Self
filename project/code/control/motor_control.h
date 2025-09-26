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
// 前馈控制参数 - 基于两点直线拟合 (2000,1.1), (3000,1.7)
#define MOTOR_FEEDFORWARD_KFF       (1666.7f)   // 前馈系数 Kff = 1/b
#define MOTOR_FEEDFORWARD_A         (-0.1f)     // 截距 a = v - b*PWM
#define MOTOR_FEEDFORWARD_B         (0.0006f)   // 斜率 b = (1.7-1.1)/(3000-2000)

// 左电机PI控制参数(20ms响应级)
#define MOTOR_LEFT_PI_KP_DEFAULT    (2500.0f)    // 左电机比例系数
#define MOTOR_LEFT_PI_KI_DEFAULT    (525.0f)     // 左电机积分系数
#define MOTOR_LEFT_PI_KD_DEFAULT    (0.0f)       // 左电机微分系数

// 右电机PI控制参数  
#define MOTOR_RIGHT_PI_KP_DEFAULT   (2500.0f)    // 右电机比例系数
#define MOTOR_RIGHT_PI_KI_DEFAULT   (525.0f)     // 右电机积分系数
#define MOTOR_RIGHT_PI_KD_DEFAULT   (0.0f)       // 右电机微分系数

// 通用PI控制参数
#define MOTOR_PI_INTEGRAL_MAX       (7.0f)       // 积分限幅 (m) - 防止积分饱和
#define MOTOR_PI_OUTPUT_MAX         (9999.0f)    // 输出限幅 (PWM)
#define MOTOR_PI_CONTROL_PERIOD     (0.002f)     // 控制周期 (s) - 2ms

// PWM数据记录配置
#define MOTOR_PWM_RECORD_SIZE       (100)       // 记录PWM值的数组大小
#define MOTOR_PWM_RECORD_THRESHOLD  (10.0f)     // PWM记录阈值（绝对值大于此值才记录）

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
// 增量式PI控制器数据结构
typedef struct {
    // PI控制参数
    float kp, ki, kd;          // PI参数 (kd不使用)
    float integral;            // 积分累积 (单位: m*s) - 增量式PI中不使用
    float last_error;          // 上次误差
    float last_output;         // 上次输出值 (增量式PI所需)
    float integral_max;        // 积分限幅 (m*s)
    float output_max;          // 输出限幅 (PWM)
    
    // 前馈控制参数 (增量式PI中不使用)
    float kff;                 // 前馈系数 (PWM/(m/s))
    float feedforward_a;       // 线性拟合截距 (m/s)
    float feedforward_b;       // 线性拟合斜率 (m/s per PWM)
    
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
    
    // PWM数据记录（用于分析响应曲线）
    float left_pwm_record[MOTOR_PWM_RECORD_SIZE];   // 左电机PWM记录数组
    float right_pwm_record[MOTOR_PWM_RECORD_SIZE];  // 右电机PWM记录数组
    uint16 left_pwm_record_count;               // 左电机PWM记录计数
    uint16 right_pwm_record_count;              // 右电机PWM记录计数
    uint8 pwm_record_enabled;                   // PWM记录使能标志
} motor_control_system_t;

//=================================================全局变量声明================================================
extern motor_control_system_t motor_control_system;

//=================================================函数声明================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化电机PID控制器
// 参数说明     void
// 返回参数     motor_pid_status_enum   初始化状态
// 使用示例     motor_pid_init();
// 备注信息     使用默认增量式PI参数初始化左右电机控制器 (Kp=250, Ki=420, Kd=0)
//-------------------------------------------------------------------------------------------------------------------
motor_pid_status_enum motor_pid_init(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置电机目标速度（仅设置目标，不进行控制计算）
// 参数说明     left_speed          左电机目标速度 (m/s)
// 参数说明     right_speed         右电机目标速度 (m/s)
// 返回参数     motor_pid_status_enum   执行状态
// 使用示例     motor_set_target_speed(1.0f, 1.0f);
// 备注信息     只负责设置目标速度值，控制计算在motor_control_update中完成
//-------------------------------------------------------------------------------------------------------------------
motor_pid_status_enum motor_set_target_speed(float left_speed, float right_speed);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机控制更新函数（在定时中断中调用）
// 参数说明     void
// 返回参数     motor_pid_status_enum   执行状态
// 使用示例     motor_control_update();  // 在10ms定时中断中调用
// 备注信息     执行编码器读取、PID计算、PWM输出的完整控制循环，需在固定周期中断中调用
//-------------------------------------------------------------------------------------------------------------------
motor_pid_status_enum motor_control_update(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     启动PWM数据记录
// 参数说明     void
// 返回参数     void
// 使用示例     motor_start_pwm_record();
// 备注信息     清除之前的记录数据并开始记录新的PWM值，用于分析响应曲线
//-------------------------------------------------------------------------------------------------------------------
void motor_start_pwm_record(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     停止PWM数据记录
// 参数说明     void
// 返回参数     void
// 使用示例     motor_stop_pwm_record();
// 备注信息     停止记录PWM值，保留已记录的数据供分析
//-------------------------------------------------------------------------------------------------------------------
void motor_stop_pwm_record(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取PWM记录数据
// 参数说明     motor_id            电机ID (MOTOR_PID_LEFT/MOTOR_PID_RIGHT)
// 参数说明     data_buffer         数据缓冲区指针
// 参数说明     buffer_size         缓冲区大小
// 返回参数     uint16              实际复制的数据个数
// 使用示例     count = motor_get_pwm_record(MOTOR_PID_LEFT, buffer, 100);
// 备注信息     将记录的PWM数据复制到指定缓冲区，返回实际复制的数据个数
//-------------------------------------------------------------------------------------------------------------------
uint16 motor_get_pwm_record(motor_pid_id_enum motor_id, float *data_buffer, uint16 buffer_size);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置电机PI参数
// 参数说明     motor_id            电机ID (MOTOR_PID_LEFT/MOTOR_PID_RIGHT/MOTOR_PID_BOTH)
// 参数说明     kp                  比例系数
// 参数说明     ki                  积分系数  
// 参数说明     kd                  微分系数
// 返回参数     motor_pid_status_enum   设置状态
// 使用示例     motor_set_pi_params(MOTOR_PID_LEFT, 2000.0f, 400.0f, 0.0f);
// 备注信息     运行时动态调整指定电机的PI参数，支持左右电机独立设置
//-------------------------------------------------------------------------------------------------------------------
motor_pid_status_enum motor_set_pi_params(motor_pid_id_enum motor_id, float kp, float ki, float kd);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取电机PI参数
// 参数说明     motor_id            电机ID (MOTOR_PID_LEFT/MOTOR_PID_RIGHT)
// 参数说明     kp                  比例系数指针
// 参数说明     ki                  积分系数指针
// 参数说明     kd                  微分系数指针
// 返回参数     motor_pid_status_enum   获取状态
// 使用示例     motor_get_pi_params(MOTOR_PID_LEFT, &kp, &ki, &kd);
// 备注信息     获取指定电机当前的PI参数值
//-------------------------------------------------------------------------------------------------------------------
motor_pid_status_enum motor_get_pi_params(motor_pid_id_enum motor_id, float *kp, float *ki, float *kd);

#endif // _MOTOR_CONTROL_H_
