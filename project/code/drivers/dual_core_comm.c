/*********************************************************************************************************************
* 文件名称          dual_core_comm.c
* 功能说明          双核通信实现（基于共享内存+DCache同步）
* 作者              LittleMaster
* 版本信息          v2.0             改成共享内存模式，提高通信效率
* 
* 说明：
* 1. 使用#pragma location指定共享内存地址（0x28001000开始）
* 2. 读取前调用SCB_CleanInvalidateDCache_by_Addr同步DCache和RAM
* 3. 写入后调用SCB_CleanInvalidateDCache_by_Addr同步DCache和RAM
* 4. 不使用IPC通知，直接轮询读取共享内存
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "dual_core_comm.h"
#include "imu_ahrs_complementary.h"
#include "driver_encoder.h"
#include "motor_control.h"
#include "nav_control_ahrs.h"
#include <string.h>

//=================================================共享内存变量定义================================================

// CM7_0写入，CM7_1读取的传感器数据
#pragma location = 0x28001000
__no_init core0_to_core1_data_t shared_core0_data;

// CM7_1写入，CM7_0读取的控制数据
#pragma location = 0x28001100
__no_init core1_to_core0_data_t shared_core1_data;

//=================================================本地变量================================================

static uint32 core0_timestamp = 0;
static uint32 core1_timestamp = 0;

//=================================================CM7_0核心函数================================================

/**
 * @brief  CM7_0核心初始化
 */
void dual_core_comm_init_core0(void)
{
    // 清空共享数据
    memset((void*)&shared_core0_data, 0, sizeof(core0_to_core1_data_t));
    memset((void*)&shared_core1_data, 0, sizeof(core1_to_core0_data_t));
    
    core0_timestamp = 0;
    
    printf("[CORE0] 双核通信初始化完成\n");
}

/**
 * @brief  更新传感器数据到共享内存（CM7_0调用）
 */
void dual_core_update_sensor_data(void)
{
    core0_timestamp++;
    
    // 读取1D Yaw积分器的航向角（用于平面导航）
    float yaw_gyro_deg;
    ahrs_get_yaw_gyro(&yaw_gyro_deg);
    shared_core0_data.yaw = yaw_gyro_deg;
    
    // 读取AHRS姿态数据（四元数欧拉角，可用于调试或俯仰补偿）
    ahrs_euler_angles_t euler;
    ahrs_get_euler_angles(&euler);
    
    shared_core0_data.pitch = euler.pitch;
    shared_core0_data.roll = euler.roll;
    
    // 读取陀螺仪数据（角速度）
    //shared_core0_data.gyro_z = 0.0f;  // 没啥必要，暂时没做从IMU获取Z轴角速度
    
    // 读取编码器数据（encoder_update()已计算好距离）
    extern encoder_system_struct encoder_system;
    shared_core0_data.encoder_left = encoder_system.left.total_pulse;
    shared_core0_data.encoder_right = encoder_system.right.total_pulse;
    
    // 读取编码器累积距离（已自动计算，单位mm）
    shared_core0_data.distance = encoder_system.total_distance;
    // shared_core0_data.position_x = 0.0f;  
    // shared_core0_data.position_y = 0.0f;
    
    // 读取速度（m/s）
    shared_core0_data.velocity = encoder_system.linear_velocity;
    
    // 更新AHRS系统状态
    shared_core0_data.ahrs_ready = ahrs_is_ready();
    shared_core0_data.gyro_calibrated = ahrs_is_ready();  // 当AHRS就绪时，陀螺仪已校准
    
    shared_core0_data.data_valid = 1;
    shared_core0_data.timestamp = core0_timestamp;
    
    // 同步DCache到RAM，确保其他核心能读取到最新数据
    SCB_CleanInvalidateDCache_by_Addr((uint32_t*)&shared_core0_data, sizeof(core0_to_core1_data_t));
}

/**
 * @brief  从共享内存读取控制数据（CM7_0调用）
 */
void dual_core_read_control_data(void)
{
    // 读取前同步DCache，确保读取到RAM中的最新数据
    SCB_CleanInvalidateDCache_by_Addr((uint32_t*)&shared_core1_data, sizeof(core1_to_core0_data_t));
    
    if (shared_core1_data.data_valid) {
        // 读取目标速度
        float left_speed = shared_core1_data.target_speed_left;
        float right_speed = shared_core1_data.target_speed_right;
        
        // 应用到电机控制
        motor_set_target_speed(left_speed, right_speed);
    }
}

//=================================================CM7_1核心函数================================================

/**
 * @brief  CM7_1核心初始化
 */
void dual_core_comm_init_core1(void)
{
    core1_timestamp = 0;
    
    printf("[CORE1] 双核通信初始化完成\n");
}

/**
 * @brief  更新控制数据到共享内存（CM7_1调用）
 */
void dual_core_update_control_data(void)
{
    core1_timestamp++;
    
    // 从导航系统获取目标速度（直接读取nav_ahrs结构体）
    shared_core1_data.target_speed_left = nav_ahrs.left_speed;
    shared_core1_data.target_speed_right = nav_ahrs.right_speed;
    
    // 更新状态信息
    shared_core1_data.control_mode = 2;  // 自动导航模式
    shared_core1_data.nav_running = (nav_ahrs.mode == NAV_AHRS_MODE_REPLAY);
    shared_core1_data.current_path_index = nav_ahrs.path.current_index;
    
    shared_core1_data.data_valid = 1;
    shared_core1_data.timestamp = core1_timestamp;
    
    // 同步DCache到RAM，确保其他核心能读取到最新数据
    SCB_CleanInvalidateDCache_by_Addr((uint32_t*)&shared_core1_data, sizeof(core1_to_core0_data_t));
}

/**
 * @brief  读取AHRS就绪状态（CM7_1调用）
 * @return AHRS就绪标志 (0=校准中, 1=就绪)
 */
uint8 dual_core_read_ahrs_ready(void)
{
    // 读取前同步DCache，确保读取到RAM中的最新数据
    SCB_CleanInvalidateDCache_by_Addr((uint32_t*)&shared_core0_data, sizeof(core0_to_core1_data_t));
    
    return shared_core0_data.ahrs_ready;
}

