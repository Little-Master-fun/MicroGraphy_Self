/*********************************************************************************************************************
* 文件名称          line_tracking_control.h
* 功能说明          基于光电管的巡线控制系统头文件
* 作者              AI Assistant
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本
* 2025-11-05        AI Assistant       1.0v
* 
* 文件作用说明：
* 本文件实现类似dog项目的光电管巡线控制系统
* 通过18路光电管检测黑线位置，计算偏差并进行PID控制
********************************************************************************************************************/

#ifndef __LINE_TRACKING_CONTROL_H__
#define __LINE_TRACKING_CONTROL_H__

#include "zf_common_headfile.h"
#include "driver_photoelectric.h"

//=================================================宏定义================================================

// 系统状态
#define LINE_TRACK_MODE_IDLE        0       // 空闲模式
#define LINE_TRACK_MODE_RUNNING     1       // 运行模式
#define LINE_TRACK_MODE_LOST        2       // 丢线模式

// 状态码
#define LINE_TRACK_STATUS_OK        0
#define LINE_TRACK_STATUS_ERROR     1
#define LINE_TRACK_STATUS_LOST      2

// 控制参数
#define LINE_TRACK_BASE_SPEED       150     // 基础速度 (PWM值 0-10000)
#define LINE_TRACK_MAX_SPEED        800     // 最大速度
#define LINE_TRACK_MIN_SPEED        100     // 最小速度

// 丢线判断
#define LINE_TRACK_LOST_THRESHOLD   3       // 连续丢线次数阈值
#define LINE_TRACK_RECOVER_THRESHOLD 2      // 恢复线路次数阈值

//=================================================结构体定义================================================

// PID控制器结构体
typedef struct {
    float kp;                   // 比例系数
    float ki;                   // 积分系数
    float kd;                   // 微分系数
    float integral;             // 积分累加值
    float last_error;           // 上次误差
    float integral_limit;       // 积分限幅
    float output_limit;         // 输出限幅
} line_track_pid_t;

// 巡线控制器结构体
typedef struct {
    // 系统状态
    uint8 mode;                 // 运行模式
    uint8 status;               // 系统状态
    uint8 initialized;          // 初始化标志
    
    // 光电管数据
    photoelectric_data_t sensor_data;   // 传感器原始数据
    int16 line_position;        // 当前线位置 (-90 ~ 90，0为中心)
    uint8 line_width;           // 线宽（检测到的传感器数量）
    uint8 is_line_detected;     // 是否检测到线
    
    // 控制参数
    float error;                // 当前偏差（归一化 -1.0 ~ 1.0）
    float last_error;           // 上次偏差
    int16 control_output;       // 控制输出（差速值）
    
    // 速度控制
    int16 target_speed;         // 目标速度
    int16 left_speed;           // 左轮速度
    int16 right_speed;          // 右轮速度
    
    // PID控制器
    line_track_pid_t pid_line;  // 直线PID
    line_track_pid_t pid_turn;  // 转弯PID
    
    // 丢线处理
    uint8 lost_counter;         // 连续丢线计数
    uint8 recover_counter;      // 恢复计数
    int16 last_direction;       // 最后转向方向 (-1左/0直/1右)
    
    // 统计信息
    uint32 loop_count;          // 控制循环计数
    uint32 lost_count;          // 累计丢线次数
    
} line_track_controller_t;

//=================================================外部变量声明================================================
extern line_track_controller_t line_track;

//=================================================函数声明================================================

/**
 * @brief   初始化巡线控制系统
 * @return  状态码
 */
uint8 line_track_init(void);

/**
 * @brief   巡线控制主循环更新（需周期调用）
 * @param   dt  时间间隔 (秒)
 * @return  状态码
 */
uint8 line_track_update(float dt);

/**
 * @brief   启动巡线
 * @return  状态码
 */
uint8 line_track_start(void);

/**
 * @brief   停止巡线
 * @return  void
 */
void line_track_stop(void);

/**
 * @brief   设置目标速度
 * @param   speed   目标速度 (PWM值)
 * @return  void
 */
void line_track_set_speed(int16 speed);

/**
 * @brief   设置PID参数（直线）
 * @param   kp  比例系数
 * @param   ki  积分系数
 * @param   kd  微分系数
 * @return  void
 */
void line_track_set_pid_straight(float kp, float ki, float kd);

/**
 * @brief   设置PID参数（转弯）
 * @param   kp  比例系数
 * @param   ki  积分系数
 * @param   kd  微分系数
 * @return  void
 */
void line_track_set_pid_turn(float kp, float ki, float kd);

/**
 * @brief   获取当前线位置
 * @return  线位置 (-90 ~ 90)
 */
int16 line_track_get_position(void);

/**
 * @brief   获取当前偏差
 * @return  偏差值 (-1.0 ~ 1.0)
 */
float line_track_get_error(void);

/**
 * @brief   打印调试信息
 * @return  void
 */
void line_track_print_debug(void);

#endif // __LINE_TRACKING_CONTROL_H__

