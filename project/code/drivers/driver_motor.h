/*********************************************************************************************************************
* 文件名称          driver_motor.h
* 功能说明          双电机驱动程序头文件（TB67H420FTG驱动器）
* 作者              LittleMaster
* 版本信息          v2.0
* 修改记录
* 日期              作者                版本              备注
* 2025-09-17        LittleMaster       1.0v              创建基础功能
* 2025-09-22        LittleMaster       2.0v              修复TB67H420FTG控制逻辑
* 
* 文件作用说明：
* 本文件为双电机驱动系统的头文件，使用TB67H420FTG双H桥驱动器
* 
* TB67H420FTG控制逻辑：
* - 停止：IN1=L, IN2=L, PWM=0
* - 正转：IN1=H, IN2=L, PWM=速度值
* - 反转：IN1=L, IN2=H, PWM=速度值
* - 制动：IN1=H, IN2=H, PWM=0
********************************************************************************************************************/

#ifndef _DRIVER_MOTOR_H_
#define _DRIVER_MOTOR_H_

#include "zf_common_typedef.h"

//=================================================硬件引脚定义================================================
// 左电机控制引脚定义
#define MOTOR_LEFT_IN1          (P03_0)     // 左电机方向控制引脚1
#define MOTOR_LEFT_IN2          (P05_1)     // 左电机方向控制引脚2
#define MOTOR_LEFT_PWM1         (TCPWM_CH00_P03_1)  // 左电机PWM通道1
#define MOTOR_LEFT_PWM2         (TCPWM_CH01_P03_0)  // 左电机PWM通道2

// 右电机控制引脚定义  
#define MOTOR_RIGHT_IN1         (P02_4)     // 右电机方向控制引脚1
#define MOTOR_RIGHT_IN2         (P06_3)     // 右电机方向控制引脚2
#define MOTOR_RIGHT_PWM1        (TCPWM_CH04_P02_3)  // 右电机PWM通道1
#define MOTOR_RIGHT_PWM2        (TCPWM_CH03_P02_4)  // 右电机PWM通道2

// 开关输入引脚（可选）
#define SWITCH1                 (P21_5)
#define SWITCH2                 (P21_6)

//=================================================配置参数定义================================================
#define MOTOR_PWM_FREQUENCY     (17000)     // PWM频率 17KHz
#define MOTOR_PWM_MAX           (9999)      // PWM最大占空比
#define MOTOR_PWM_MIN           (10)        // PWM最小有效占空比
#define MOTOR_PWM_INIT_DUTY     (0)         // PWM初始占空比

//=================================================枚举类型定义================================================
// 电机ID枚举
typedef enum
{
    MOTOR_LEFT  = 0,                        // 左电机
    MOTOR_RIGHT = 1,                        // 右电机
    MOTOR_BOTH  = 2,                        // 双电机
} motor_id_enum;

// 电机方向枚举
typedef enum
{
    MOTOR_STOP      = 0,                    // 停止
    MOTOR_FORWARD   = 1,                    // 正转
    MOTOR_BACKWARD  = 2,                    // 反转
} motor_direction_enum;

// 电机状态枚举
typedef enum
{
    MOTOR_STATUS_OK     = 0,                // 正常
    MOTOR_STATUS_ERROR  = 1,                // 错误
} motor_status_enum;

//=================================================函数声明================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机系统初始化
// 参数说明     void
// 返回参数     motor_status_enum   初始化状态
// 使用示例     motor_init();
// 备注信息     初始化PWM通道和GPIO引脚
//-------------------------------------------------------------------------------------------------------------------
motor_status_enum motor_init(void);



//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置电机PWM
// 参数说明     motor_id            电机ID (MOTOR_LEFT/MOTOR_RIGHT/MOTOR_BOTH)
// 参数说明     pwm_value           电机PWM值 (-MOTOR_PWM_MAX ~ MOTOR_PWM_MAX)
// 返回参数     motor_status_enum   执行状态
// 使用示例     motor_set_pwm_single(MOTOR_LEFT, 5000);
// 备注信息     设置电机的PWM值，正值正转，负值反转，支持双电机同时控制
//-------------------------------------------------------------------------------------------------------------------
motor_status_enum motor_set_pwm(motor_id_enum motor_id, int16 pwm_value);


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PWM值保护函数
// 参数说明     pwm_input           输入PWM值
// 返回参数     int16               保护后的PWM值
// 使用示例     protected_pwm = motor_pwm_protect(input_pwm);
// 备注信息     限制PWM值在有效范围内，防止电机损坏
//-------------------------------------------------------------------------------------------------------------------
int16 motor_pwm_protect(int16 pwm_input);


#endif