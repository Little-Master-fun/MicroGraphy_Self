/*********************************************************************************************************************
* 文件名称          motor_control.c
* 功能说明          电机PID闭环控制系统实现文件
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本              备注
* 2025-09-19        LittleMaster        v1.0              创建电机PID控制系统实现
*
* 文件作用说明：
* 本文件实现电机PID闭环控制系统的所有功能，提供双电机的精确速度控制
* 支持差速控制、参数动态调整、积分防饱和、实时状态监控等功能
*
* 主要特性：
* 1. 高精度PID控制算法
* 2. 双电机独立控制
* 3. 差速运动支持
* 4. 实时性能监控
* 5. 参数在线调整
********************************************************************************************************************/

#include "motor_control.h"
#include "zf_common_headfile.h"
#include "driver_encoder.h"
#include "driver_motor.h"

//=================================================全局变量定义================================================
motor_control_system_t motor_control_system = {0};

//=================================================内部函数声明================================================
static float motor_pid_calculate(motor_pid_controller_t *pid, float target, float current);
static motor_pid_status_enum motor_pid_init_single(motor_pid_controller_t *pid, const char* name);
static void motor_pid_update_statistics(motor_pid_controller_t *pid, float error);

//=================================================主要接口函数================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化电机PID控制器
//-------------------------------------------------------------------------------------------------------------------
motor_pid_status_enum motor_pid_init(void)
{
    // 清零系统结构体
    memset(&motor_control_system, 0, sizeof(motor_control_system_t));
    
    // 初始化左电机PID控制器
    if (motor_pid_init_single(&motor_control_system.left_pid, "左电机") != MOTOR_PID_OK) {
        printf("错误：左电机PID初始化失败\n");
        return MOTOR_PID_ERROR;
    }
    
    // 初始化右电机PID控制器
    if (motor_pid_init_single(&motor_control_system.right_pid, "右电机") != MOTOR_PID_OK) {
        printf("错误：右电机PID初始化失败\n");
        return MOTOR_PID_ERROR;
    }
    
    // 设置系统参数
    motor_control_system.system_enabled = 1;
    motor_control_system.control_frequency = 100.0f;  // 默认100Hz
    motor_control_system.total_update_count = 0;
    
    printf("电机PID闭环控制系统初始化完成\n");
    printf("参数配置: Kp=%.1f Ki=%.1f Kd=%.1f\n", 
           MOTOR_PID_KP_DEFAULT, MOTOR_PID_KI_DEFAULT, MOTOR_PID_KD_DEFAULT);
    
    return MOTOR_PID_OK;
}



//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置电机目标速度（仅设置目标，不进行控制计算）
//-------------------------------------------------------------------------------------------------------------------
motor_pid_status_enum motor_set_target_speed(float left_speed, float right_speed)
{
    if (!motor_control_system.system_enabled) {
        return MOTOR_PID_NOT_INIT;
    }
    
    // 只负责设置目标速度
    motor_control_system.left_target_speed = left_speed;
    motor_control_system.right_target_speed = right_speed;
    
    return MOTOR_PID_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机控制更新函数（在定时中断中调用，如10ms中断）
//-------------------------------------------------------------------------------------------------------------------
motor_pid_status_enum motor_control_update(void)
{
    if (!motor_control_system.system_enabled) {
        return MOTOR_PID_NOT_INIT;
    }
    
    // 更新编码器数据
    encoder_update();
    
    // 获取当前速度
    float left_current = encoder_get_speed(ENCODER_ID_LEFT);
    float right_current = encoder_get_speed(ENCODER_ID_RIGHT);
    
    // 更新系统状态
    motor_control_system.left_current_speed = left_current;
    motor_control_system.right_current_speed = right_current;
    
    // 计算PID输出
    float left_output = motor_pid_calculate(&motor_control_system.left_pid, 
                                          motor_control_system.left_target_speed, 
                                          left_current);
    float right_output = motor_pid_calculate(&motor_control_system.right_pid, 
                                           motor_control_system.right_target_speed, 
                                           right_current);
    
    // 更新输出值
    motor_control_system.left_output = left_output;
    motor_control_system.right_output = right_output;
    
    // 输出PWM到电机
    motor_set_pwm(MOTOR_LEFT, (int16)left_output);
    motor_set_pwm(MOTOR_RIGHT, (int16)right_output);
    
    // 统计计数
    motor_control_system.total_update_count++;
    
    return MOTOR_PID_OK;
}







//=================================================内部函数实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PID控制器计算（内部函数）
//-------------------------------------------------------------------------------------------------------------------
static float motor_pid_calculate(motor_pid_controller_t *pid, float target, float current)
{
    if (pid == NULL || !pid->initialized) {
        return 0.0f;
    }
    
    // 使用固定的控制周期（嵌入式系统标准做法）
    const float dt = 0.01f;  // 固定10ms控制周期，假设以100Hz频率调用
    
    // 计算误差
    float error = target - current;
    
    // 比例项
    float proportional = pid->kp * error;
    
    // 积分项
    pid->integral += error * dt;
    
    // 积分限幅（防止积分饱和）
    if (pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if (pid->integral < -pid->integral_max) {
        pid->integral = -pid->integral_max;
    }
    
    float integral = pid->ki * pid->integral;
    
    // 微分项
    float derivative = pid->kd * (error - pid->last_error) / dt;
    
    // PID输出
    float output = proportional + integral + derivative;
    
    // 输出限幅
    if (output > pid->output_max) {
        output = pid->output_max;
    } else if (output < -pid->output_max) {
        output = -pid->output_max;
    }
    
    // 更新历史值
    pid->last_error = error;
    
    // 更新统计信息
    motor_pid_update_statistics(pid, error);
    
    return output;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化单个PID控制器（内部函数）
//-------------------------------------------------------------------------------------------------------------------
static motor_pid_status_enum motor_pid_init_single(motor_pid_controller_t *pid, const char* name)
{
    if (pid == NULL) {
        return MOTOR_PID_ERROR;
    }
    
    // 设置PID参数
    pid->kp = MOTOR_PID_KP_DEFAULT;
    pid->ki = MOTOR_PID_KI_DEFAULT;
    pid->kd = MOTOR_PID_KD_DEFAULT;
    
    // 清零状态变量
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->integral_max = MOTOR_PID_INTEGRAL_MAX;
    pid->output_max = MOTOR_PID_OUTPUT_MAX;
    
    // 清零统计信息
    pid->update_count = 0;
    pid->max_error = 0.0f;
    pid->average_error = 0.0f;
    
    // 标记为已初始化
    pid->initialized = 1;
    
    printf("%sPID控制器初始化成功\n", name);
    
    return MOTOR_PID_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新PID统计信息（内部函数）
//-------------------------------------------------------------------------------------------------------------------
static void motor_pid_update_statistics(motor_pid_controller_t *pid, float error)
{
    if (pid == NULL) {
        return;
    }
    
    // 更新计数
    pid->update_count++;
    
    // 更新最大误差
    float abs_error = fabs(error);
    if (abs_error > pid->max_error) {
        pid->max_error = abs_error;
    }
    
    // 更新平均误差（滑动平均）
    if (pid->update_count == 1) {
        pid->average_error = abs_error;
    } else {
        pid->average_error = 0.95f * pid->average_error + 0.05f * abs_error;
    }
}
