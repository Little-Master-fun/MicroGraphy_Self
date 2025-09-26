/*********************************************************************************************************************
* 文件名称          driver_encoder.h
* 功能说明          双编码器驱动程序头文件
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本
* 2025-09-17        LittleMaster       1.0v
* 
* 文件作用说明：
* 本文件为双编码器驱动系统的头文件，支持左右两个独立编码器的数据采集
* 适用于轮式机器人的里程计测量和速度反馈控制
* 
* 主要功能：
* 1. 编码器硬件初始化和配置
* 2. 编码器脉冲计数读取
* 3. 速度计算和滤波
* 4. 里程统计和方向判断
********************************************************************************************************************/

#ifndef _DRIVER_ENCODER_H_
#define _DRIVER_ENCODER_H_

#include "zf_common_typedef.h"

 #define M_PI 3.14159265358979323846
// M_PI定义在config_navigation.h中，避免重复定义
//=================================================硬件引脚定义================================================
// 左编码器（编码器1）引脚定义
#define ENCODER_LEFT                    (TC_CH07_ENCODER)                   // 左编码器接口
#define ENCODER_LEFT_A                  (TC_CH07_ENCODER_CH1_P07_6)         // 左编码器A相引脚
#define ENCODER_LEFT_B                  (TC_CH07_ENCODER_CH2_P07_7)         // 左编码器B相引脚

// 右编码器（编码器2）引脚定义
#define ENCODER_RIGHT                   (TC_CH20_ENCODER)                   // 右编码器接口
#define ENCODER_RIGHT_A                 (TC_CH20_ENCODER_CH1_P08_1)         // 右编码器A相引脚
#define ENCODER_RIGHT_B                 (TC_CH20_ENCODER_CH2_P08_2)         // 右编码器B相引脚

//=================================================配置参数定义================================================
#define ENCODER_SAMPLE_TIME             (2)        // 采样时间间隔 (ms)
#define ENCODER_PPR                     (2900)      // 编码器每转脉冲数 (Pulse Per Revolution)
#define ENCODER_WHEEL_DIAMETER          (20.0f)     // 车轮直径 (mm) 
#define ENCODER_GEAR_RATIO              (1.0f)      // 齿轮减速比
#define ENCODER_SPEED_FILTER_FACTOR     (0.8f)      // 速度滤波系数 (0-1)

//=================================================枚举类型定义================================================
// 编码器ID枚举
typedef enum
{
    ENCODER_ID_LEFT     = 0,                        // 左编码器
    ENCODER_ID_RIGHT    = 1,                        // 右编码器
    ENCODER_ID_BOTH     = 2,                        // 双编码器
} encoder_id_enum;

// 编码器方向枚举
typedef enum
{
    ENCODER_DIR_FORWARD     = 1,                    // 正向
    ENCODER_DIR_BACKWARD    = -1,                   // 反向
    ENCODER_DIR_STOP        = 0,                    // 停止
} encoder_direction_enum;

// 编码器状态枚举
typedef enum
{
    ENCODER_STATUS_OK       = 0,                    // 正常
    ENCODER_STATUS_ERROR    = 1,                    // 错误
} encoder_status_enum;

//=================================================数据结构定义================================================
// 编码器数据结构
typedef struct
{
    int32 pulse_count;                              // 脉冲计数值（带方向）
    int32 pulse_count_abs;                          // 脉冲计数值（绝对值）
    int32 total_pulse;                              // 总脉冲计数
    float speed_rpm;                                // 转速 (RPM)
    float speed_mps;                                // 线速度 (m/s)
    float distance_mm;                              // 累计距离 (mm)
    encoder_direction_enum direction;               // 转动方向
} encoder_data_struct;

// 编码器系统信息结构
typedef struct
{
    encoder_data_struct left;                       // 左编码器数据
    encoder_data_struct right;                      // 右编码器数据
    float linear_velocity;                          // 线速度 (m/s)
    float angular_velocity;                         // 角速度 (rad/s)
    float total_distance;                           // 总行驶距离 (mm)
    float heading_angle;                            // 航向角 (rad)
} encoder_system_struct;

//=================================================全局变量声明================================================
extern encoder_system_struct encoder_system;

//=================================================函数声明================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     编码器系统初始化
// 参数说明     void
// 返回参数     encoder_status_enum     初始化状态
// 使用示例     encoder_init();
// 备注信息     初始化双编码器硬件和数据结构
//-------------------------------------------------------------------------------------------------------------------
encoder_status_enum encoder_init(void);


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取编码器速度
// 参数说明     encoder_id              编码器ID
// 返回参数     float                   速度值 (m/s)
// 使用示例     speed = encoder_get_speed(ENCODER_ID_LEFT);
// 备注信息     获取指定编码器的线速度
//-------------------------------------------------------------------------------------------------------------------
float encoder_get_speed(encoder_id_enum encoder_id);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取编码器距离
// 参数说明     encoder_id              编码器ID
// 返回参数     float                   距离值 (mm)
// 使用示例     distance = encoder_get_distance(ENCODER_ID_LEFT);
// 备注信息     获取指定编码器的累计行驶距离
//-------------------------------------------------------------------------------------------------------------------
float encoder_get_distance(encoder_id_enum encoder_id);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     重置编码器数据
// 参数说明     encoder_id              编码器ID
// 返回参数     encoder_status_enum     重置状态
// 使用示例     encoder_reset(ENCODER_ID_BOTH);
// 备注信息     重置指定编码器的累计数据
//-------------------------------------------------------------------------------------------------------------------
encoder_status_enum encoder_reset(encoder_id_enum encoder_id);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取编码器脉冲计数
// 参数说明     encoder_id              编码器ID
// 返回参数     int32                   脉冲计数值（带符号）
// 使用示例     pulse = encoder_get_pulse(ENCODER_ID_LEFT);
// 备注信息     获取指定编码器的实时脉冲计数，正值表示正转，负值表示反转
//-------------------------------------------------------------------------------------------------------------------
int32 encoder_get_pulse(encoder_id_enum encoder_id);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     编码器数据更新（周期调用）
// 参数说明     void
// 返回参数     encoder_status_enum     更新状态
// 使用示例     encoder_update();
// 备注信息     周期性调用此函数更新编码器数据，建议10ms调用一次
//-------------------------------------------------------------------------------------------------------------------
encoder_status_enum encoder_update(void);


#endif // _DRIVER_ENCODER_H_