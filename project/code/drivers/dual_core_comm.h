/*********************************************************************************************************************
* 文件名称          dual_core_comm.h
* 功能说明          双核通信数据结构定义
* 作者              AI Assistant
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本
* 2025-01-XX        AI Assistant       1.0v
* 
* 文件作用说明：
* 本文件定义CM7_0和CM7_1核心之间的共享数据结构
* CM7_0负责数据采集和底层控制
* CM7_1负责导航算法和决策
********************************************************************************************************************/

#ifndef _DUAL_CORE_COMM_H_
#define _DUAL_CORE_COMM_H_

#include "zf_common_typedef.h"

//=================================================共享数据结构================================================

// 从CM7_0发送到CM7_1的数据（传感器数据）
typedef struct
{
    // 姿态数据（来自AHRS）
    float yaw;              // 偏航角 (度)
    float pitch;            // 俯仰角 (度)
    float roll;             // 横滚角 (度)
    
    // 角速度（来自陀螺仪）
    float gyro_z;           // Z轴角速度 (度/秒)
    
    // 位置数据（来自编码器）
    float position_x;       // X位置 (mm)
    float position_y;       // Y位置 (mm)
    float distance;         // 累计距离 (mm)
    
    // 速度数据
    float velocity;         // 当前速度 (m/s)
    
    // 编码器数据
    int32 encoder_left;     // 左编码器计数
    int32 encoder_right;    // 右编码器计数
    
    // 数据有效标志
    uint8 data_valid;       // 数据有效性标志
    uint32 timestamp;       // 时间戳
    
} core0_to_core1_data_t;

// 从CM7_1发送到CM7_0的数据（控制指令）
typedef struct
{
    // 目标速度（来自导航算法）
    float target_speed_left;    // 左轮目标速度 (m/s)
    float target_speed_right;   // 右轮目标速度 (m/s)
    
    // 控制模式
    uint8 control_mode;         // 0=停止, 1=手动, 2=自动导航
    
    // 导航状态
    uint8 nav_running;          // 导航是否运行中
    uint16 current_path_index;  // 当前路径点索引
    
    // 数据有效标志
    uint8 data_valid;           // 数据有效性标志
    uint32 timestamp;           // 时间戳
    
} core1_to_core0_data_t;

//=================================================共享内存区================================================

// 使用属性将数据放置在共享内存区域
// CYT4BB的两个CM7核心可以访问相同的RAM区域

#if defined(__ICCARM__)
    // IAR编译器
    #define SHARED_DATA_SECTION __attribute__((section(".shared_data")))
#elif defined(__GNUC__)
    // GCC编译器
    #define SHARED_DATA_SECTION __attribute__((section(".shared_data")))
#else
    #define SHARED_DATA_SECTION
#endif

// 共享内存变量（两个核心都可以访问）
extern volatile core0_to_core1_data_t shared_core0_data;  // CM7_0写入，CM7_1读取
extern volatile core1_to_core0_data_t shared_core1_data;  // CM7_1写入，CM7_0读取

// 数据同步标志
extern volatile uint8 core0_data_ready;  // CM7_0数据就绪标志
extern volatile uint8 core1_data_ready;  // CM7_1数据就绪标志

//=================================================接口函数================================================

// CM7_0核心使用的函数
void dual_core_comm_init_core0(void);                           // CM7_0初始化
void dual_core_update_sensor_data(void);                        // 更新传感器数据到共享内存
void dual_core_read_control_data(void);                         // 从共享内存读取控制数据
void dual_core_ipc_callback_core0(uint32 ipc_data);            // CM7_0 IPC回调函数

// CM7_1核心使用的函数
void dual_core_comm_init_core1(void);                           // CM7_1初始化
void dual_core_read_sensor_data(void);                          // 从共享内存读取传感器数据
void dual_core_update_control_data(void);                       // 更新控制数据到共享内存
void dual_core_ipc_callback_core1(uint32 ipc_data);            // CM7_1 IPC回调函数

//=================================================IPC消息定义================================================

// IPC消息类型
#define IPC_MSG_SENSOR_DATA_READY       0x0001  // 传感器数据就绪
#define IPC_MSG_CONTROL_DATA_READY      0x0002  // 控制数据就绪
#define IPC_MSG_CORE1_INIT_DONE         0x0010  // CM7_1初始化完成
#define IPC_MSG_NAV_START               0x0020  // 开始导航
#define IPC_MSG_NAV_STOP                0x0030  // 停止导航

#endif // _DUAL_CORE_COMM_H_


