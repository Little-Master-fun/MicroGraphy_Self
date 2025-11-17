/*********************************************************************************************************************
* 文件名称          line_tracking_control.c
* 功能说明          基于光电管的巡线控制系统实现文件
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本
* 2025-11-05        LittleMaster       1.0v
* 
* 文件作用说明：
* 本文件实现类似dog项目的光电管巡线控制系统
* 通过18路光电管检测黑线位置，计算偏差并进行PID控制
********************************************************************************************************************/

#include "line_tracking_control.h"
#include "driver_motor.h"
#include "zf_common_headfile.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

//=================================================全局变量定义================================================
line_track_controller_t line_track = {0};

//=================================================内部函数声明================================================
static void line_track_read_sensors(void);
static void line_track_calculate_error(void);
static float line_track_pid_update(line_track_pid_t *pid, float error, float dt);
static void line_track_speed_control(void);
static void line_track_lost_detect(void);

//=================================================外部接口实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化巡线控制系统
// 返回参数     状态码
//-------------------------------------------------------------------------------------------------------------------
uint8 line_track_init(void)
{
    // 清零系统结构体
    memset(&line_track, 0, sizeof(line_track_controller_t));
    
    // 初始化光电管传感器
    photoelectric_init();
    
    // 初始化直线PID参数（小偏差，稳定精确）
    line_track.pid_line.kp = 2.5f;
    line_track.pid_line.ki = 0.01f;
    line_track.pid_line.kd = 5.0f;
    line_track.pid_line.integral_limit = 1000.0f;
    line_track.pid_line.output_limit = 5000.0f;
    
    // 初始化转弯PID参数（大偏差，快速响应）
    line_track.pid_turn.kp = 4.0f;
    line_track.pid_turn.ki = 0.005f;
    line_track.pid_turn.kd = 8.0f;
    line_track.pid_turn.integral_limit = 800.0f;
    line_track.pid_turn.output_limit = 6000.0f;
    
    // 初始化速度参数
    line_track.target_speed = LINE_TRACK_BASE_SPEED;
    
    // 系统状态
    line_track.mode = LINE_TRACK_MODE_IDLE;
    line_track.status = LINE_TRACK_STATUS_OK;
    line_track.initialized = 1;
    
    printf("[LINE_TRACK] 巡线系统初始化完成\n");
    printf("[LINE_TRACK] 直线PID: Kp=%.2f, Ki=%.3f, Kd=%.2f\n", 
           line_track.pid_line.kp, line_track.pid_line.ki, line_track.pid_line.kd);
    printf("[LINE_TRACK] 转弯PID: Kp=%.2f, Ki=%.3f, Kd=%.2f\n", 
           line_track.pid_turn.kp, line_track.pid_turn.ki, line_track.pid_turn.kd);
    
    return LINE_TRACK_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     巡线控制主循环更新（需周期调用）
// 参数说明     dt          时间间隔 (秒)
// 返回参数     状态码
//-------------------------------------------------------------------------------------------------------------------
uint8 line_track_update(float dt)
{
    if (!line_track.initialized) {
        return LINE_TRACK_STATUS_ERROR;
    }
    
    // 更新循环计数
    line_track.loop_count++;
    
    // 如果是空闲模式，停车
    if (line_track.mode == LINE_TRACK_MODE_IDLE) {
        line_track.left_speed = 0;
        line_track.right_speed = 0;
        motor_set_speed(0, 0);
        return LINE_TRACK_STATUS_OK;
    }
    
    // 1. 读取光电管传感器
    line_track_read_sensors();
    
    // 2. 计算偏差
    line_track_calculate_error();
    
    // 3. 丢线检测
    line_track_lost_detect();
    
    // 4. PID控制计算
    line_track_pid_t *active_pid;
    
    // 根据偏差大小选择PID控制器
    if (fabs(line_track.error) > 0.5f) {
        // 大偏差，使用转弯PID
        active_pid = &line_track.pid_turn;
    } else {
        // 小偏差，使用直线PID
        active_pid = &line_track.pid_line;
    }
    
    // 计算PID输出
    line_track.control_output = (int16)line_track_pid_update(active_pid, line_track.error, dt);
    
    // 5. 速度控制
    line_track_speed_control();
    
    // 6. 输出到电机
    motor_set_speed(line_track.left_speed, line_track.right_speed);
    
    return LINE_TRACK_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     启动巡线
// 返回参数     状态码
//-------------------------------------------------------------------------------------------------------------------
uint8 line_track_start(void)
{
    if (!line_track.initialized) {
        return LINE_TRACK_STATUS_ERROR;
    }
    
    // 清零PID积分项
    line_track.pid_line.integral = 0.0f;
    line_track.pid_turn.integral = 0.0f;
    
    // 清零丢线计数
    line_track.lost_counter = 0;
    line_track.recover_counter = 0;
    
    // 设置为运行模式
    line_track.mode = LINE_TRACK_MODE_RUNNING;
    line_track.status = LINE_TRACK_STATUS_OK;
    
    printf("[LINE_TRACK] 启动巡线\n");
    return LINE_TRACK_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     停止巡线
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void line_track_stop(void)
{
    line_track.mode = LINE_TRACK_MODE_IDLE;
    line_track.left_speed = 0;
    line_track.right_speed = 0;
    motor_set_speed(0, 0);
    
    printf("[LINE_TRACK] 停止巡线\n");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置目标速度
// 参数说明     speed       目标速度 (PWM值)
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void line_track_set_speed(int16 speed)
{
    // 限幅
    if (speed > LINE_TRACK_MAX_SPEED) {
        speed = LINE_TRACK_MAX_SPEED;
    } else if (speed < LINE_TRACK_MIN_SPEED) {
        speed = LINE_TRACK_MIN_SPEED;
    }
    
    line_track.target_speed = speed;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置PID参数（直线）
// 参数说明     kp/ki/kd    PID系数
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void line_track_set_pid_straight(float kp, float ki, float kd)
{
    line_track.pid_line.kp = kp;
    line_track.pid_line.ki = ki;
    line_track.pid_line.kd = kd;
    
    printf("[LINE_TRACK] 直线PID更新: Kp=%.2f, Ki=%.3f, Kd=%.2f\n", kp, ki, kd);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置PID参数（转弯）
// 参数说明     kp/ki/kd    PID系数
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void line_track_set_pid_turn(float kp, float ki, float kd)
{
    line_track.pid_turn.kp = kp;
    line_track.pid_turn.ki = ki;
    line_track.pid_turn.kd = kd;
    
    printf("[LINE_TRACK] 转弯PID更新: Kp=%.2f, Ki=%.3f, Kd=%.2f\n", kp, ki, kd);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取当前线位置
// 返回参数     线位置 (-90 ~ 90)
//-------------------------------------------------------------------------------------------------------------------
int16 line_track_get_position(void)
{
    return line_track.line_position;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取当前偏差
// 返回参数     偏差值 (-1.0 ~ 1.0)
//-------------------------------------------------------------------------------------------------------------------
float line_track_get_error(void)
{
    return line_track.error;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     打印调试信息
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void line_track_print_debug(void)
{
    printf("[LINE_TRACK] Pos=%d, Err=%.2f, Ctrl=%d, L=%d, R=%d, Lost=%d\n",
           line_track.line_position,
           line_track.error,
           line_track.control_output,
           line_track.left_speed,
           line_track.right_speed,
           line_track.lost_counter);
}

//=================================================内部函数实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取光电管传感器
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
static void line_track_read_sensors(void)
{
    // 读取光电管数据
    photoelectric_read(&line_track.sensor_data);
    
    // 获取线位置（加权平均）
    line_track.line_position = photoelectric_get_line_position(&line_track.sensor_data);
    
    // 获取线宽
    line_track.line_width = photoelectric_get_line_width(&line_track.sensor_data);
    
    // 判断是否检测到线
    line_track.is_line_detected = (line_track.line_width > 0) ? 1 : 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算偏差
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
static void line_track_calculate_error(void)
{
    // 保存上次偏差
    line_track.last_error = line_track.error;
    
    if (line_track.is_line_detected) {
        // 检测到线，归一化偏差 (-1.0 ~ 1.0)
        // 线位置范围：-90 ~ 90
        line_track.error = line_track.line_position / 90.0f;
        
        // 更新最后转向方向
        if (line_track.error > 0.1f) {
            line_track.last_direction = 1;  // 右
        } else if (line_track.error < -0.1f) {
            line_track.last_direction = -1; // 左
        } else {
            line_track.last_direction = 0;  // 直
        }
    } else {
        // 丢线，保持上次偏差方向
        if (line_track.last_direction > 0) {
            line_track.error = 1.0f;  // 继续向右找
        } else if (line_track.last_direction < 0) {
            line_track.error = -1.0f; // 继续向左找
        } else {
            line_track.error = 0.0f;  // 停止
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PID控制器更新
// 参数说明     pid         PID结构体指针
//              error       当前误差
//              dt          时间间隔
// 返回参数     控制输出
//-------------------------------------------------------------------------------------------------------------------
static float line_track_pid_update(line_track_pid_t *pid, float error, float dt)
{
    float output;
    
    // P项
    float p_term = pid->kp * error;
    
    // I项（带限幅）
    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
    float i_term = pid->ki * pid->integral;
    
    // D项
    float derivative = (error - pid->last_error) / dt;
    float d_term = pid->kd * derivative;
    
    // 更新上次误差
    pid->last_error = error;
    
    // 总输出
    output = p_term + i_term + d_term;
    
    // 输出限幅
    if (output > pid->output_limit) {
        output = pid->output_limit;
    } else if (output < -pid->output_limit) {
        output = -pid->output_limit;
    }
    
    return output;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     速度控制（差速控制）
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
static void line_track_speed_control(void)
{
    // 基础速度
    int16 base_speed = line_track.target_speed;
    
    // 根据偏差调整速度（大偏差时减速）
    float speed_factor = 1.0f - fabs(line_track.error) * 0.3f;
    if (speed_factor < 0.5f) {
        speed_factor = 0.5f;  // 最低速度为50%
    }
    
    base_speed = (int16)(base_speed * speed_factor);
    
    // 差速控制
    // control_output为正时右转，左轮快右轮慢
    // control_output为负时左转，右轮快左轮慢
    line_track.left_speed = base_speed + line_track.control_output;
    line_track.right_speed = base_speed - line_track.control_output;
    
    // 限幅保护
    if (line_track.left_speed > LINE_TRACK_MAX_SPEED) {
        line_track.left_speed = LINE_TRACK_MAX_SPEED;
    } else if (line_track.left_speed < -LINE_TRACK_MAX_SPEED) {
        line_track.left_speed = -LINE_TRACK_MAX_SPEED;
    }
    
    if (line_track.right_speed > LINE_TRACK_MAX_SPEED) {
        line_track.right_speed = LINE_TRACK_MAX_SPEED;
    } else if (line_track.right_speed < -LINE_TRACK_MAX_SPEED) {
        line_track.right_speed = -LINE_TRACK_MAX_SPEED;
    }
    
    // 如果丢线严重，停车
    if (line_track.mode == LINE_TRACK_MODE_LOST) {
        line_track.left_speed = 0;
        line_track.right_speed = 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     丢线检测与处理
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
static void line_track_lost_detect(void)
{
    if (!line_track.is_line_detected) {
        // 丢线
        line_track.lost_counter++;
        line_track.recover_counter = 0;
        
        if (line_track.lost_counter >= LINE_TRACK_LOST_THRESHOLD) {
            // 连续丢线，进入丢线模式
            if (line_track.mode != LINE_TRACK_MODE_LOST) {
                line_track.mode = LINE_TRACK_MODE_LOST;
                line_track.status = LINE_TRACK_STATUS_LOST;
                line_track.lost_count++;
                printf("[LINE_TRACK] 丢线！累计次数: %d\n", line_track.lost_count);
            }
        }
    } else {
        // 检测到线
        line_track.recover_counter++;
        
        if (line_track.recover_counter >= LINE_TRACK_RECOVER_THRESHOLD) {
            // 恢复
            line_track.lost_counter = 0;
            if (line_track.mode == LINE_TRACK_MODE_LOST) {
                line_track.mode = LINE_TRACK_MODE_RUNNING;
                line_track.status = LINE_TRACK_STATUS_OK;
                printf("[LINE_TRACK] 恢复巡线\n");
            }
        }
    }
}

