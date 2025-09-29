/*********************************************************************************************************************
* 文件名称          test_imu.h
* 功能说明          IMU姿态计算系统测试程序头文件
* 作者              LittleMaster  
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本
* 2025-09-28        LittleMaster       1.0v
* 
* 文件作用说明：
* 本文件为IMU姿态计算系统的测试程序头文件，提供IMU测试功能声明
* 
* 主要功能：
* 1. IMU系统测试和验证  
* 2. 实时角度信息显示
* 3. 使用SCH16TK10真实传感器数据
* 4. 分离数据采样和姿态解算
********************************************************************************************************************/

#ifndef _test_imu_h_
#define _test_imu_h_

#include "zf_common_typedef.h"
#include "imu_attitude.h"

//=================================================配置参数定义================================================
#define TEST_IMU_DISPLAY_REFRESH_MS     (100)           // 显示刷新间隔 (ms) - 10Hz
#define TEST_IMU_ATTITUDE_UPDATE_MS     (100)           // 姿态解算间隔 (ms) - 10Hz (IMU采样在1ms中断)
#define TEST_IMU_CALIBRATION_TIME_MS    (10000)         // 校准时间 (ms)

//=================================================枚举类型定义================================================
// IMU测试模式枚举
typedef enum
{
    IMU_TEST_MODE_REALTIME      = 0,    // 实时显示模式
    IMU_TEST_MODE_CALIBRATION   = 1,    // 校准模式
    IMU_TEST_MODE_ALGORITHM     = 2,    // 算法对比模式
    IMU_TEST_MODE_DEBUG         = 3,    // 调试模式
} imu_test_mode_enum;

// IMU测试状态枚举
typedef enum
{
    IMU_TEST_STATUS_IDLE        = 0,    // 空闲状态
    IMU_TEST_STATUS_RUNNING     = 1,    // 运行中
    IMU_TEST_STATUS_CALIBRATING = 2,    // 校准中
    IMU_TEST_STATUS_ERROR       = 3,    // 错误状态
} imu_test_status_enum;

//=================================================数据结构定义================================================
// IMU测试配置结构体
typedef struct
{
    imu_test_mode_enum test_mode;       // 测试模式
    uint8 auto_calibration;             // 自动校准使能
    uint16 display_refresh_ms;          // 显示刷新间隔
    uint16 attitude_update_ms;          // 姿态解算间隔 (原data_update_ms)
} imu_test_config_t;

// IMU测试统计信息结构体
typedef struct
{
    uint32 update_count;                // 更新计数
    uint32 error_count;                 // 错误计数
    float max_pitch;                    // 最大俯仰角
    float min_pitch;                    // 最小俯仰角
    float max_roll;                     // 最大滚转角
    float min_roll;                     // 最小滚转角
    float max_yaw;                      // 最大偏航角
    float min_yaw;                      // 最小偏航角
    uint32 start_time;                  // 测试开始时间
} imu_test_statistics_t;

// IMU测试系统主结构体
typedef struct
{
    imu_test_config_t config;           // 测试配置
    imu_test_statistics_t stats;        // 统计信息
    imu_test_status_enum status;        // 测试状态
    uint32 last_display_time;           // 上次显示更新时间
    uint32 last_attitude_time;          // 上次姿态解算时间 (原last_data_time)
    uint8 system_initialized;           // 系统初始化标志
} imu_test_system_t;

//=================================================全局变量声明================================================
extern imu_test_system_t imu_test_system;

//=================================================函数声明================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU测试系统主函数
// 参数说明     void
// 返回参数     void
// 使用示例     test_imu_system();
// 备注信息     IMU姿态计算系统的完整测试程序，包含实时显示功能
//-------------------------------------------------------------------------------------------------------------------
void test_imu_system(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化IMU测试系统
// 参数说明     test_mode           测试模式
// 返回参数     uint8               初始化状态 (1=成功, 0=失败)
// 使用示例     test_imu_init(IMU_TEST_MODE_REALTIME);
// 备注信息     初始化IMU测试系统和显示界面
//-------------------------------------------------------------------------------------------------------------------
uint8 test_imu_init(imu_test_mode_enum test_mode);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取测试系统时间戳
// 参数说明     void
// 返回参数     uint32              当前时间戳 (ms)
// 使用示例     uint32 time = test_imu_get_timestamp();
// 备注信息     获取测试系统运行时间，用于时间测量
//-------------------------------------------------------------------------------------------------------------------
uint32 test_imu_get_timestamp(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     1ms中断中的IMU数据采样处理 (需在中断中调用)
// 参数说明     void
// 返回参数     void  
// 使用示例     在1ms定时器中断中调用 imu_1ms_interrupt_handler();
// 备注信息     高频采样IMU数据，建议在1ms中断中调用以保证采样精度
//-------------------------------------------------------------------------------------------------------------------
void imu_1ms_interrupt_handler(void);

#endif // _test_imu_h_