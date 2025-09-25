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
    
    // 初始化PWM记录相关变量
    motor_control_system.left_pwm_record_count = 0;
    motor_control_system.right_pwm_record_count = 0;
    motor_control_system.pwm_record_enabled = 0;
    memset(motor_control_system.left_pwm_record, 0, sizeof(motor_control_system.left_pwm_record));
    memset(motor_control_system.right_pwm_record, 0, sizeof(motor_control_system.right_pwm_record));
    
    printf("电机增量式PI闭环控制系统初始化完成\n");
    printf("增量式PI参数: Kp=%.1f Ki=%.1f 输出限幅=%.1f\n", 
           MOTOR_PI_KP_DEFAULT, MOTOR_PI_KI_DEFAULT, MOTOR_PI_OUTPUT_MAX);
    
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
    //encoder_update();
    
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
    motor_set_pwm(MOTOR_RIGHT, (int16)left_output);
    motor_set_pwm(MOTOR_LEFT, (int16)right_output);
    
    // 记录PWM值用于分析响应曲线
    if (motor_control_system.pwm_record_enabled) {
        // 记录左电机PWM值（实际输出给右电机）
        if (fabs(left_output) >= MOTOR_PWM_RECORD_THRESHOLD && 
            motor_control_system.left_pwm_record_count < MOTOR_PWM_RECORD_SIZE) {
            motor_control_system.left_pwm_record[motor_control_system.left_pwm_record_count] = left_output;
            motor_control_system.left_pwm_record_count++;
        }
        
        // 记录右电机PWM值（实际输出给左电机）
        if (fabs(right_output) >= MOTOR_PWM_RECORD_THRESHOLD && 
            motor_control_system.right_pwm_record_count < MOTOR_PWM_RECORD_SIZE) {
            motor_control_system.right_pwm_record[motor_control_system.right_pwm_record_count] = right_output;
            motor_control_system.right_pwm_record_count++;
        }
        
        // 如果两个电机都记录满了，自动停止记录
        if (motor_control_system.left_pwm_record_count >= MOTOR_PWM_RECORD_SIZE &&
            motor_control_system.right_pwm_record_count >= MOTOR_PWM_RECORD_SIZE) {
            motor_control_system.pwm_record_enabled = 0;
            printf("PWM记录已满，自动停止记录\n");
        }
    }
    
    // 统计计数
    motor_control_system.total_update_count++;
    
    return MOTOR_PID_OK;
}







//=================================================内部函数实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     增量式PI控制器计算（内部函数）
//-------------------------------------------------------------------------------------------------------------------
static float motor_pid_calculate(motor_pid_controller_t *pid, float target, float current)
{
    if (pid == NULL || !pid->initialized) {
        return 0.0f;
    }
    
    // 计算当前误差
    float error = -target + current;
    
    // ========== 增量式PI控制算法 ==========
    // 增量式PI控制公式: Δu(k) = Kp * [e(k) - e(k-1)] + Ki * e(k)
    // 其中: e(k)为当前误差, e(k-1)为上次误差
    
    // 比例增量项: Kp * [e(k) - e(k-1)]
    float proportional_increment = pid->kp * (error - pid->last_error);
    
    // 积分增量项: Ki * e(k)  
    float integral_increment = pid->ki * error;
    
    // 计算控制量增量: Δu(k) = Kp*[e(k)-e(k-1)] + Ki*e(k)
    float output_increment = proportional_increment + integral_increment;
    
    // 计算当前输出: u(k) = u(k-1) + Δu(k)
    float output = pid->last_output + output_increment;
    
    // 输出限幅，确保PWM在有效范围内
    if (output > pid->output_max) {
        output = pid->output_max;
    } else if (output < -pid->output_max) {
        output = -pid->output_max;
    }
    
    // 更新历史值（为下次计算做准备）
    pid->last_error = error;      // 保存当前误差作为下次的上次误差
    pid->last_output = output;    // 保存当前输出作为下次的上次输出
    
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
    
    // 设置PI控制参数  
    pid->kp = MOTOR_PI_KP_DEFAULT;
    pid->ki = MOTOR_PI_KI_DEFAULT;
    pid->kd = MOTOR_PI_KD_DEFAULT;
    
    // 设置前馈控制参数
    pid->kff = MOTOR_FEEDFORWARD_KFF;
    pid->feedforward_a = MOTOR_FEEDFORWARD_A;
    pid->feedforward_b = MOTOR_FEEDFORWARD_B;
    
    // 清零状态变量
    pid->integral = 0.0f;              // 增量式PI中不使用，但保留
    pid->last_error = 0.0f;            // 上次误差初始化为0
    pid->last_output = 0.0f;           // 上次输出初始化为0（增量式PI重要参数）
    pid->integral_max = MOTOR_PI_INTEGRAL_MAX;
    pid->output_max = MOTOR_PI_OUTPUT_MAX;
    
    // 清零统计信息
    pid->update_count = 0;
    pid->max_error = 0.0f;
    pid->average_error = 0.0f;
    
    // 标记为已初始化
    pid->initialized = 1;
    
    printf("%s增量式PI控制器初始化成功\n", name);
    
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

//=================================================PWM数据记录功能================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     启动PWM数据记录
//-------------------------------------------------------------------------------------------------------------------
void motor_start_pwm_record(void)
{
    if (!motor_control_system.system_enabled) {
        printf("错误：电机控制系统未初始化，无法启动PWM记录\n");
        return;
    }
    
    // 清除之前的记录数据
    motor_control_system.left_pwm_record_count = 0;
    motor_control_system.right_pwm_record_count = 0;
    memset(motor_control_system.left_pwm_record, 0, sizeof(motor_control_system.left_pwm_record));
    memset(motor_control_system.right_pwm_record, 0, sizeof(motor_control_system.right_pwm_record));
    
    // 启动记录
    motor_control_system.pwm_record_enabled = 1;
    
    printf("PWM数据记录已启动，将记录前%d个不为零的PWM值\n", MOTOR_PWM_RECORD_SIZE);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     停止PWM数据记录
//-------------------------------------------------------------------------------------------------------------------
void motor_stop_pwm_record(void)
{
    motor_control_system.pwm_record_enabled = 0;
    
    printf("PWM数据记录已停止\n");
    printf("左电机记录了%d个PWM值，右电机记录了%d个PWM值\n", 
           motor_control_system.left_pwm_record_count, 
           motor_control_system.right_pwm_record_count);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取PWM记录数据
//-------------------------------------------------------------------------------------------------------------------
uint16 motor_get_pwm_record(motor_pid_id_enum motor_id, float *data_buffer, uint16 buffer_size)
{
    if (data_buffer == NULL || buffer_size == 0) {
        return 0;
    }
    
    uint16 copy_count = 0;
    
    switch (motor_id) {
        case MOTOR_PID_LEFT:
            copy_count = (motor_control_system.left_pwm_record_count < buffer_size) ? 
                         motor_control_system.left_pwm_record_count : buffer_size;
            memcpy(data_buffer, motor_control_system.left_pwm_record, copy_count * sizeof(float));
            break;
            
        case MOTOR_PID_RIGHT:
            copy_count = (motor_control_system.right_pwm_record_count < buffer_size) ? 
                         motor_control_system.right_pwm_record_count : buffer_size;
            memcpy(data_buffer, motor_control_system.right_pwm_record, copy_count * sizeof(float));
            break;
            
        default:
            printf("错误：无效的电机ID\n");
            return 0;
    }
    
    return copy_count;
}
