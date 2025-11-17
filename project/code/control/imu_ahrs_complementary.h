/*********************************************************************************************************************
* 文件名称          imu_ahrs_complementary.h
* 功能说明          基于四元数的AHRS互补滤波器头文件
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本
* 2025-01-XX        LittleMaster       1.0v
********************************************************************************************************************/

#ifndef __IMU_AHRS_COMPLEMENTARY_H_
#define __IMU_AHRS_COMPLEMENTARY_H_

#include "zf_common_typedef.h"
#include "driver_sch16tk10.h"

//=================================================枚举定义================================================
// AHRS系统状态枚举
typedef enum
{
    AHRS_STATUS_OK = 0,             // 正常运行
    AHRS_STATUS_ERROR,              // 错误
    AHRS_STATUS_NOT_INIT,           // 未初始化
    AHRS_STATUS_CALIBRATING,        // 正在校准
} ahrs_status_enum;

//=================================================结构体定义================================================

// 四元数结构体
typedef struct
{
    float q0;           // 四元数实部 (w)
    float q1;           // 四元数虚部 (x)
    float q2;           // 四元数虚部 (y)
    float q3;           // 四元数虚部 (z)
} ahrs_quaternion_t;

// 欧拉角结构体
typedef struct
{
    float pitch;                // 俯仰角 (度)
    float roll;                 // 滚转角 (度)
    float yaw;                  // 偏航角 (度) [-180, 180]
    float yaw_accumulated;      // 累积偏航角 (度)，无限制
} ahrs_euler_angles_t;

// 陀螺仪零偏结构体
typedef struct
{
    float x;            // X轴零偏 (rad/s)
    float y;            // Y轴零偏 (rad/s)
    float z;            // Z轴零偏 (rad/s)
} ahrs_gyro_offset_t;

// IMU数据结构体
typedef struct
{
    float gyro_x;       // 陀螺仪X轴 (rad/s)
    float gyro_y;       // 陀螺仪Y轴 
    float gyro_z;       // 陀螺仪Z轴 
    float acc_x;        // 加速度计X轴 (g)
    float acc_y;        // 加速度计Y轴 
    float acc_z;        // 加速度计Z轴 
} ahrs_imu_data_t;

// PI控制器结构体
typedef struct
{
    float integral_ex;  // X轴积分误差
    float integral_ey;  // Y轴积分误差
    float integral_ez;  // Z轴积分误差
} ahrs_pi_controller_t;

// AHRS系统结构体
typedef struct
{
    uint8 initialized;          // 初始化标志
    uint8 ready;                // 系统准备就绪标志
    uint32 update_count;        // 更新次数计数
} ahrs_system_t;

//=================================================外部函数声明================================================

/**
 * @brief  初始化AHRS互补滤波器系统
 * @return ahrs_status_enum  初始化状态
 * @note   该函数会自动开始陀螺仪零偏校准过程，需保持设备静止约2秒
 */
ahrs_status_enum ahrs_complementary_init(void);

/**
 * @brief  AHRS系统更新（在定时中断中调用）
 * @param  raw_data  SCH16TK10传感器原始数据指针
 * @return ahrs_status_enum  更新状态
 * @note   建议在1ms周期的定时中断中调用此函数
 * @note   函数内部会累加AVG_FACTOR次原始数据后进行平均，以降低噪声
 */
ahrs_status_enum ahrs_complementary_update(SCH1_raw_data *raw_data);

/**
 * @brief  获取当前欧拉角
 * @param  euler  欧拉角输出指针
 * @return ahrs_status_enum  状态
 */
ahrs_status_enum ahrs_get_euler_angles(ahrs_euler_angles_t *euler);

/**
 * @brief  获取当前四元数
 * @param  quat  四元数输出指针
 * @return ahrs_status_enum  状态
 */
ahrs_status_enum ahrs_get_quaternion(ahrs_quaternion_t *quat);

/**
 * @brief  获取陀螺仪零偏
 * @param  offset  零偏输出指针
 * @return ahrs_status_enum  状态
 */
ahrs_status_enum ahrs_get_gyro_offset(ahrs_gyro_offset_t *offset);

/**
 * @brief  重置航向角零点
 * @return ahrs_status_enum  状态
 * @note   调用此函数后，当前航向角会被设置为零点
 */
ahrs_status_enum ahrs_reset_yaw(void);

/**
 * @brief  设置PI控制器参数
 * @param  kp  比例增益
 * @param  ki  积分增益
 * @return ahrs_status_enum  状态
 * @note   默认值为0，可以根据实际情况调整以补偿陀螺仪漂移
 */
ahrs_status_enum ahrs_set_pi_params(float kp, float ki);

/**
 * @brief  检查AHRS系统是否准备就绪
 * @return uint8  1-就绪（校准完成） 0-未就绪（正在校准或未初始化）
 */
uint8 ahrs_is_ready(void);

#endif /* __IMU_AHRS_COMPLEMENTARY_H_ */

