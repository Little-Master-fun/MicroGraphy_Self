/*********************************************************************************************************************
* 文件名称          dual_core_comm.c
* 功能说明          双核通信实现
* 作者              AI Assistant
* 版本信息          v1.0
********************************************************************************************************************/

#include "dual_core_comm.h"
#include "zf_driver_ipc.h"
#include "imu_ahrs_complementary.h"
#include "driver_encoder.h"
#include "motor_control.h"
#include "nav_control_ahrs.h"
#include <string.h>

//=================================================共享内存变量================================================

// 使用volatile确保编译器不会优化掉对这些变量的访问
SHARED_DATA_SECTION volatile core0_to_core1_data_t shared_core0_data = {0};
SHARED_DATA_SECTION volatile core1_to_core0_data_t shared_core1_data = {0};
SHARED_DATA_SECTION volatile uint8 core0_data_ready = 0;
SHARED_DATA_SECTION volatile uint8 core1_data_ready = 0;

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
    
    core0_data_ready = 0;
    core1_data_ready = 0;
    core0_timestamp = 0;
    
    // 初始化IPC通信（CM7_0使用IPC_PORT_1）
    ipc_communicate_init(IPC_PORT_1, dual_core_ipc_callback_core0);
    
    printf("[CORE0] 双核通信初始化完成\n");
}

/**
 * @brief  更新传感器数据到共享内存（CM7_0调用）
 */
void dual_core_update_sensor_data(void)
{
    core0_timestamp++;
    
    // 读取AHRS姿态数据
    ahrs_euler_angles_t euler;
    ahrs_get_euler_angles(&euler);
    
    shared_core0_data.yaw = euler.yaw;
    shared_core0_data.pitch = euler.pitch;
    shared_core0_data.roll = euler.roll;
    
    // 读取陀螺仪数据（角速度）
    // 注意：gyro_z需要从IMU原始数据中获取，这里暂时设为0，实际使用时需要完善
    shared_core0_data.gyro_z = 0.0f;  // TODO: 从IMU获取Z轴角速度
    
    // 读取编码器数据
    shared_core0_data.encoder_left = encoder_get_pulse(ENCODER_ID_LEFT);
    shared_core0_data.encoder_right = encoder_get_pulse(ENCODER_ID_RIGHT);
    
    // 读取导航位置数据（如果有）
    if (nav_ahrs.initialized) {
        shared_core0_data.distance = nav_ahrs.current_distance;
        // 位置坐标需要通过里程计计算，这里使用累积距离
        shared_core0_data.position_x = 0.0f;  // TODO: 添加位置计算
        shared_core0_data.position_y = 0.0f;
        shared_core0_data.velocity = (nav_ahrs.left_speed + nav_ahrs.right_speed) / 2.0f;
    }
    
    shared_core0_data.data_valid = 1;
    shared_core0_data.timestamp = core0_timestamp;
    
    // 标记数据就绪
    core0_data_ready = 1;
    
    // 通过IPC通知CM7_1
    ipc_send_data(IPC_MSG_SENSOR_DATA_READY);
}

/**
 * @brief  从共享内存读取控制数据（CM7_0调用）
 */
void dual_core_read_control_data(void)
{
    if (core1_data_ready && shared_core1_data.data_valid) {
        // 读取目标速度
        float left_speed = shared_core1_data.target_speed_left;
        float right_speed = shared_core1_data.target_speed_right;
        
        // 应用到电机控制
        motor_set_target_speed(left_speed, right_speed);
        
        // 清除就绪标志
        core1_data_ready = 0;
    }
}

/**
 * @brief  CM7_0 IPC回调函数
 */
void dual_core_ipc_callback_core0(uint32 ipc_data)
{
    switch(ipc_data) {
        case IPC_MSG_CONTROL_DATA_READY:
            // CM7_1发送了新的控制数据
            core1_data_ready = 1;
            break;
            
        case IPC_MSG_CORE1_INIT_DONE:
            printf("[CORE0] 收到CM7_1初始化完成消息\n");
            break;
            
        default:
            break;
    }
}

//=================================================CM7_1核心函数================================================

/**
 * @brief  CM7_1核心初始化
 */
void dual_core_comm_init_core1(void)
{
    core1_timestamp = 0;
    
    // 初始化IPC通信（CM7_1使用IPC_PORT_2）
    ipc_communicate_init(IPC_PORT_2, dual_core_ipc_callback_core1);
    
    printf("[CORE1] 双核通信初始化完成\n");
    
    // 通知CM7_0初始化完成
    ipc_send_data(IPC_MSG_CORE1_INIT_DONE);
}

/**
 * @brief  从共享内存读取传感器数据（CM7_1调用）
 */
void dual_core_read_sensor_data(void)
{
    if (core0_data_ready && shared_core0_data.data_valid) {
        // 数据已经在共享内存中，可以直接访问
        // 这里不需要额外操作，导航算法会直接读取shared_core0_data
        
        // 清除就绪标志
        core0_data_ready = 0;
    }
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
    
    // 标记数据就绪
    core1_data_ready = 1;
    
    // 通过IPC通知CM7_0
    ipc_send_data(IPC_MSG_CONTROL_DATA_READY);
}

/**
 * @brief  CM7_1 IPC回调函数
 */
void dual_core_ipc_callback_core1(uint32 ipc_data)
{
    switch(ipc_data) {
        case IPC_MSG_SENSOR_DATA_READY:
            // CM7_0发送了新的传感器数据
            core0_data_ready = 1;
            break;
            
        case IPC_MSG_NAV_START:
            printf("[CORE1] 收到开始导航消息\n");
            break;
            
        case IPC_MSG_NAV_STOP:
            printf("[CORE1] 收到停止导航消息\n");
            break;
            
        default:
            break;
    }
}

//=================================================辅助函数================================================

/**
 * @brief  获取传感器数据（供CM7_1导航算法使用）
 */
void dual_core_get_sensor_data_for_nav(float* yaw, float* gyro_z, float* velocity)
{
    if (shared_core0_data.data_valid) {
        if (yaw) *yaw = shared_core0_data.yaw;
        if (gyro_z) *gyro_z = shared_core0_data.gyro_z;
        if (velocity) *velocity = shared_core0_data.velocity;
    }
}


