/*********************************************************************************************************************
* 文件名称          driver_encoder.c
* 功能说明          双编码器驱动程序实现文件
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本
* 2025-09-17        LittleMaster       1.0v
* 
* 文件作用说明：
* 本文件是双编码器驱动系统的实现文件，提供完整的编码器数据采集和处理功能
* 支持轮式机器人的里程计测量、速度反馈和运动控制
********************************************************************************************************************/

#include "driver_encoder.h"
#include "zf_common_headfile.h"
#include <math.h>

//=================================================全局变量定义================================================
encoder_system_struct encoder_system = {0};

//=================================================内部函数声明================================================
static encoder_status_enum encoder_read_single(encoder_id_enum encoder_id);
static encoder_status_enum encoder_calculate_single_speed(encoder_id_enum encoder_id);
static void encoder_reset_single(encoder_id_enum encoder_id);
static float encoder_pulse_to_distance(int32 pulse_count);

//=================================================外部接口实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     编码器系统初始化
//-------------------------------------------------------------------------------------------------------------------
encoder_status_enum encoder_init(void)
{
    // 初始化左编码器硬件
    encoder_dir_init(ENCODER_LEFT, ENCODER_LEFT_A, ENCODER_LEFT_B);
    
    // 初始化右编码器硬件
    encoder_dir_init(ENCODER_RIGHT, ENCODER_RIGHT_A, ENCODER_RIGHT_B);
    
    // 初始化数据结构
    memset(&encoder_system, 0, sizeof(encoder_system_struct));
    
    return ENCODER_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取编码器数据
//-------------------------------------------------------------------------------------------------------------------
encoder_status_enum encoder_read_data(encoder_id_enum encoder_id)
{
    encoder_status_enum status = ENCODER_STATUS_OK;
    
    if (encoder_id == ENCODER_ID_LEFT || encoder_id == ENCODER_ID_BOTH)
    {
        status |= encoder_read_single(ENCODER_ID_LEFT);
    }
    
    if (encoder_id == ENCODER_ID_RIGHT || encoder_id == ENCODER_ID_BOTH)
    {
        status |= encoder_read_single(ENCODER_ID_RIGHT);
    }
    
    return status;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取编码器脉冲计数
//-------------------------------------------------------------------------------------------------------------------
int32 encoder_get_pulse_count(encoder_id_enum encoder_id)
{
    switch (encoder_id)
    {
        case ENCODER_ID_LEFT:
            return encoder_system.left.pulse_count;
        case ENCODER_ID_RIGHT:
            return encoder_system.right.pulse_count;
        default:
            return 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     编码器数据更新（周期调用）
//-------------------------------------------------------------------------------------------------------------------
encoder_status_enum encoder_update(void)
{
    encoder_status_enum status = ENCODER_STATUS_OK;
    
    // 读取编码器数据
    status |= encoder_read_data(ENCODER_ID_BOTH);
    
    // 计算运动参数
    encoder_system.linear_velocity = (encoder_system.left.speed_mps + encoder_system.right.speed_mps) / 2.0f;
    encoder_system.angular_velocity = (encoder_system.right.speed_mps - encoder_system.left.speed_mps) / (ENCODER_WHEEL_DIAMETER / 1000.0f);
    
    return status;
}

//=================================================内部函数实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取单个编码器数据（内部函数）
//-------------------------------------------------------------------------------------------------------------------
static encoder_status_enum encoder_read_single(encoder_id_enum encoder_id)
{
    encoder_data_struct *encoder_data;
    int32 raw_count;
    
    // 获取对应的编码器数据指针和硬件句柄
    if (encoder_id == ENCODER_ID_LEFT)
    {
        encoder_data = &encoder_system.left;
        raw_count = encoder_get_count(ENCODER_LEFT);
        encoder_clear_count(ENCODER_LEFT);
    }
    else if (encoder_id == ENCODER_ID_RIGHT)
    {
        encoder_data = &encoder_system.right;
        raw_count = encoder_get_count(ENCODER_RIGHT);
        encoder_clear_count(ENCODER_RIGHT);
    }
    else
    {
        return ENCODER_STATUS_ERROR;
    }
    
    // 更新脉冲计数
    encoder_data->pulse_count = raw_count;
    encoder_data->pulse_count_abs = (raw_count < 0) ? -raw_count : raw_count;
    
    // 更新总脉冲计数
    encoder_data->total_pulse += encoder_data->pulse_count_abs;
    
    // 判断方向
    if (raw_count > 0)
    {
        encoder_data->direction = ENCODER_DIR_FORWARD;
    }
    else if (raw_count < 0)
    {
        encoder_data->direction = ENCODER_DIR_BACKWARD;
    }
    else
    {
        encoder_data->direction = ENCODER_DIR_STOP;
    }
    
    // 计算速度 (简化版本)
    float wheel_circumference = M_PI * (ENCODER_WHEEL_DIAMETER / 1000.0f);  // 转换为米
    encoder_data->speed_mps = ((float)encoder_data->pulse_count_abs / ENCODER_PPR) * 
                             wheel_circumference * (1000.0f / ENCODER_SAMPLE_TIME);
    
    // 考虑方向
    if (encoder_data->direction == ENCODER_DIR_BACKWARD)
    {
        encoder_data->speed_mps = -encoder_data->speed_mps;
    }
    
    return ENCODER_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     脉冲转换为距离（内部函数）
//-------------------------------------------------------------------------------------------------------------------
static float encoder_pulse_to_distance(int32 pulse_count)
{
    float wheel_circumference = M_PI * ENCODER_WHEEL_DIAMETER;  // mm
    float distance_per_pulse = wheel_circumference / ENCODER_PPR;
    
    return (float)pulse_count * distance_per_pulse;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     脉冲转换为RPM（内部函数）
//-------------------------------------------------------------------------------------------------------------------
// static float encoder_pulse_to_rpm(int32 pulse_count, uint32 time_interval_ms)
// {
//     float revolutions = (float)pulse_count / ENCODER_PPR;
//     float time_minutes = (float)time_interval_ms / 60000.0f;
    
//     return revolutions / time_minutes;
// }

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算单个编码器速度（内部函数）
//-------------------------------------------------------------------------------------------------------------------
static encoder_status_enum encoder_calculate_single_speed(encoder_id_enum encoder_id)
{
    encoder_data_struct *encoder_data;
    
    if (encoder_id == ENCODER_ID_LEFT)
    {
        encoder_data = &encoder_system.left;
    }
    else if (encoder_id == ENCODER_ID_RIGHT)
    {
        encoder_data = &encoder_system.right;
    }
    else
    {
        return ENCODER_STATUS_ERROR;
    }
    
    // 计算转速 (RPM)
    encoder_data->speed_rpm = ((float)encoder_data->pulse_count_abs / ENCODER_PPR) * 
                             (60000.0f / ENCODER_SAMPLE_TIME);
    
    // 计算线速度 (m/s)
    float wheel_circumference = M_PI * (ENCODER_WHEEL_DIAMETER / 1000.0f);
    encoder_data->speed_mps = ((float)encoder_data->pulse_count_abs / ENCODER_PPR) * 
                             wheel_circumference * (1000.0f / ENCODER_SAMPLE_TIME);
    
    // 计算距离 (mm)
    encoder_data->distance_mm = encoder_pulse_to_distance(encoder_data->total_pulse);
    
    // 考虑方向
    if (encoder_data->direction == ENCODER_DIR_BACKWARD)
    {
        encoder_data->speed_mps = -encoder_data->speed_mps;
        encoder_data->speed_rpm = -encoder_data->speed_rpm;
    }
    
    return ENCODER_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     重置单个编码器（内部函数）
//-------------------------------------------------------------------------------------------------------------------
static void encoder_reset_single(encoder_id_enum encoder_id)
{
    encoder_data_struct *encoder_data;
    
    if (encoder_id == ENCODER_ID_LEFT)
    {
        encoder_data = &encoder_system.left;
        encoder_clear_count(ENCODER_LEFT);
    }
    else if (encoder_id == ENCODER_ID_RIGHT)
    {
        encoder_data = &encoder_system.right;
        encoder_clear_count(ENCODER_RIGHT);
    }
    else
    {
        return;
    }
    
    // 重置所有数据
    encoder_data->pulse_count = 0;
    encoder_data->pulse_count_abs = 0;
    encoder_data->total_pulse = 0;
    encoder_data->speed_rpm = 0.0f;
    encoder_data->speed_mps = 0.0f;
    encoder_data->distance_mm = 0.0f;
    encoder_data->direction = ENCODER_DIR_STOP;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取编码器速度
//-------------------------------------------------------------------------------------------------------------------
float encoder_get_speed(encoder_id_enum encoder_id)
{
    switch (encoder_id)
    {
        case ENCODER_ID_LEFT:
            return encoder_system.left.speed_mps;
        case ENCODER_ID_RIGHT:
            return encoder_system.right.speed_mps;
        default:
            return 0.0f;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取编码器距离
//-------------------------------------------------------------------------------------------------------------------
float encoder_get_distance(encoder_id_enum encoder_id)
{
    switch (encoder_id)
    {
        case ENCODER_ID_LEFT:
            return encoder_system.left.distance_mm;
        case ENCODER_ID_RIGHT:
            return encoder_system.right.distance_mm;
        default:
            return 0.0f;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算编码器速度
//-------------------------------------------------------------------------------------------------------------------
encoder_status_enum encoder_calculate_speed(encoder_id_enum encoder_id)
{
    encoder_status_enum status = ENCODER_STATUS_OK;
    
    if (encoder_id == ENCODER_ID_LEFT || encoder_id == ENCODER_ID_BOTH)
    {
        status |= encoder_calculate_single_speed(ENCODER_ID_LEFT);
    }
    
    if (encoder_id == ENCODER_ID_RIGHT || encoder_id == ENCODER_ID_BOTH)
    {
        status |= encoder_calculate_single_speed(ENCODER_ID_RIGHT);
    }
    
    return status;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     重置编码器数据
//-------------------------------------------------------------------------------------------------------------------
encoder_status_enum encoder_reset(encoder_id_enum encoder_id)
{
    if (encoder_id == ENCODER_ID_LEFT || encoder_id == ENCODER_ID_BOTH)
    {
        encoder_reset_single(ENCODER_ID_LEFT);
    }
    
    if (encoder_id == ENCODER_ID_RIGHT || encoder_id == ENCODER_ID_BOTH)
    {
        encoder_reset_single(ENCODER_ID_RIGHT);
    }
    
    return ENCODER_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取机器人速度信息
//-------------------------------------------------------------------------------------------------------------------
encoder_status_enum encoder_get_robot_velocity(float *linear_velocity, float *angular_velocity)
{
    if (linear_velocity != NULL)
    {
        *linear_velocity = encoder_system.linear_velocity;
    }
    
    if (angular_velocity != NULL)
    {
        *angular_velocity = encoder_system.angular_velocity;
    }
    
    return ENCODER_STATUS_OK;
}