/*********************************************************************************************************************
* 文件名称          driver_motor.h
* 功能说明          双电机驱动程序头文件（TB67H420FTG驱动芯片）
* 作者              LittleMaster
* 版本信息          v4.0
* 修改记录
* 日期              作者                版本              备注
* 2025-09-17        LittleMaster       1.0v              创建基础功能
* 2025-09-22        LittleMaster       2.0v              修复TB67H420FTG控制逻辑
* 2025-09-23        LittleMaster       3.0v              改用双PWM控制，移除方向GPIO
* 2025-11-05        LittleMaster       4.0v              修改为TB67H420FTG标准控制方式
* 
* 文件作用说明：
* 本文件为双电机驱动系统的头文件，使用TB67H420FTG双H桥驱动芯片
* 
* TB67H420FTG控制逻辑（每个电机有A/B两个通道）：
* - 通道A: INA1/INA2控制方向，PWMA控制速度
* - 通道B: INB1/INB2控制方向，PWMB控制速度
* 
* 方向控制真值表（已反转以匹配编码器方向）：
* - IN1=0, IN2=1: 正转 (CW) - 已反转
* - IN1=1, IN2=0: 反转 (CCW) - 已反转
* - IN1=0, IN2=0: 短路刹车
* - IN1=1, IN2=1: 停止（高阻态）
********************************************************************************************************************/

#ifndef _DRIVER_MOTOR_H_
#define _DRIVER_MOTOR_H_

#include "zf_common_typedef.h"

//=================================================硬件引脚定义================================================
// 左电机驱动引脚定义（使用通道A）- INA1/INA2已交换以反转方向
#define MOTOR_RIGHT_PWMA         (TCPWM_CH13_P05_4)  // 左电机PWM通道A P5.4 (引脚38)
#define MOTOR_RIGHT_INA1         (P05_0)             // 左电机方向控制1 P5.0 (引脚42) 
#define MOTOR_RIGHT_INA2         (P06_3)             // 左电机方向控制2 P6.3 

// 右电机驱动引脚定义（使用通道A）- INA1/INA2已交换以反转方向
#define MOTOR_LEFT_PWMA        (TCPWM_CH12_P05_3)  // 右电机PWM通道A P5.3 (引脚39)
#define MOTOR_LEFT_INA1        (P05_2)             // 右电机方向控制1 P5.2 (引脚40) 
#define MOTOR_LEFT_INA2        (P05_1)             // 右电机方向控制2 P5.1 (引脚41) 

// 开关输入引脚
#define SWITCH1                 (P21_5)
#define SWITCH2                 (P21_6)

//=================================================配置参数定义================================================
#define MOTOR_PWM_FREQUENCY     (17000)     // PWM频率 17KHz
#define MOTOR_PWM_MAX           (9000)     // PWM最大占空比
#define MOTOR_PWM_MIN           (10)        // PWM最小有效占空比
#define MOTOR_PWM_INIT_DUTY     (0)         // PWM初始占空比

// TB67H420FTG方向控制定义（已反转以匹配编码器）
#define MOTOR_DIR_CW            0           // 顺时针（正转）: IN1=0, IN2=1 (已反转)
#define MOTOR_DIR_CCW           1           // 逆时针（反转）: IN1=1, IN2=0 (已反转)
#define MOTOR_DIR_BRAKE         2           // 短路刹车: IN1=0, IN2=0
#define MOTOR_DIR_STOP          3           // 停止（高阻）: IN1=1, IN2=1

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
// 函数简介     设置电机速度（有符号PWM值）
// 参数说明     motor_id            电机ID (MOTOR_LEFT/MOTOR_RIGHT/MOTOR_BOTH)
// 参数说明     pwm_value           电机PWM值 (-MOTOR_PWM_MAX ~ MOTOR_PWM_MAX)
// 返回参数     motor_status_enum   执行状态
// 使用示例     motor_set_pwm(MOTOR_LEFT, 5000);
// 备注信息     正值正转，负值反转，自动控制方向引脚和PWM速度
//-------------------------------------------------------------------------------------------------------------------
motor_status_enum motor_set_pwm(motor_id_enum motor_id, int16 pwm_value);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     直接设置电机方向和速度
// 参数说明     motor_id            电机ID (MOTOR_LEFT/MOTOR_RIGHT)
// 参数说明     direction           方向 (MOTOR_DIR_CW/CCW/BRAKE/STOP)
// 参数说明     speed               速度 (0 ~ MOTOR_PWM_MAX)
// 返回参数     motor_status_enum   执行状态
// 使用示例     motor_set_direction_speed(MOTOR_LEFT, MOTOR_DIR_CW, 5000);
// 备注信息     直接控制方向和速度，用于高级应用
//-------------------------------------------------------------------------------------------------------------------
motor_status_enum motor_set_direction_speed(motor_id_enum motor_id, uint8 direction, uint16 speed);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机紧急停止（刹车模式）
// 参数说明     motor_id            电机ID (MOTOR_LEFT/MOTOR_RIGHT/MOTOR_BOTH)
// 返回参数     motor_status_enum   执行状态
// 使用示例     motor_brake(MOTOR_BOTH);
// 备注信息     使用短路刹车模式快速停止电机
//-------------------------------------------------------------------------------------------------------------------
motor_status_enum motor_brake(motor_id_enum motor_id);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置双电机速度（兼容接口）
// 参数说明     left_speed          左电机速度 (-MOTOR_PWM_MAX ~ MOTOR_PWM_MAX)
// 参数说明     right_speed         右电机速度 (-MOTOR_PWM_MAX ~ MOTOR_PWM_MAX)
// 返回参数     motor_status_enum   执行状态
// 使用示例     motor_set_speed(1000, 1000);
// 备注信息     用于差速控制，正值正转，负值反转
//-------------------------------------------------------------------------------------------------------------------
motor_status_enum motor_set_speed(int16 left_speed, int16 right_speed);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PWM值保护函数
// 参数说明     pwm_input           输入PWM值
// 返回参数     int16               保护后的PWM值
// 使用示例     protected_pwm = motor_pwm_protect(input_pwm);
// 备注信息     限制PWM值在有效范围内，防止电机损坏
//-------------------------------------------------------------------------------------------------------------------
int16 motor_pwm_protect(int16 pwm_input);


#endif