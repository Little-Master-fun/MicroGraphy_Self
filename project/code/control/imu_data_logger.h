/*********************************************************************************************************************
* 文件名称          imu_data_logger.h
* 功能说明          IMU数据采集与Flash存储系统头文件
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本
* 2025-11-25        LittleMaster       1.0v
* 
* 功能说明：
* 采集SCH16TK10和IMU963RA的运行数据，存储到Flash中用于后续神经网络训练
* 采集数据包括：温度、通电时间、加速度、角速度、轮速等
********************************************************************************************************************/

#ifndef _IMU_DATA_LOGGER_H_
#define _IMU_DATA_LOGGER_H_

#include "zf_common_typedef.h"

//=================================================配置参数================================================
#define IMU_LOG_BUFFER_SIZE         500      // 缓冲区大小（500条记录约30KB）
#define IMU_LOG_SAMPLE_RATE_MS      20       // 采样率：20ms采样一次（50Hz）
#define IMU_LOG_FLASH_SECTOR        0xAF080000  // DFLASH起始地址
#define IMU_LOG_MAX_FLASH_RECORDS   1000     // Flash最多存储1000条记录

//=================================================枚举定义================================================
// 数据记录器状态
typedef enum {
    IMU_LOG_STATUS_OK = 0,
    IMU_LOG_STATUS_ERROR,
    IMU_LOG_STATUS_NOT_INIT,
    IMU_LOG_STATUS_BUFFER_FULL,
    IMU_LOG_STATUS_FLASH_FULL,
} imu_log_status_enum;

//=================================================数据结构定义================================================
// 训练数据结构体（用于神经网络训练，以SCH16TK10为标准）
typedef struct {
    // ========== 时间戳 ==========
    uint32 timestamp_ms;             // 系统运行时间(ms)
    uint32 power_on_time_s;          // 通电时间(s)
    
    // ========== 温度信息（来自SCH16TK10）==========
    float temperature_deg;           // SCH16TK10内部温度(°C)
    float temp_rate_degps;           // 温度变化率(°C/s)
    
    // ========== 运动状态（轮速）==========
    float left_wheel_speed;          // 左轮速度(m/s)
    float right_wheel_speed;         // 右轮速度(m/s)
    float linear_velocity;           // 线速度(m/s)
    uint8 is_stationary;             // 是否静止(0/1)
    
    // ========== IMU963RA 加速度 ==========
    float imu963ra_acc_x;            // X轴加速度(g)
    float imu963ra_acc_y;            // Y轴加速度(g)
    float imu963ra_acc_z;            // Z轴加速度(g)
    
    // ========== IMU963RA 角速度（原始，未零偏补偿）==========
    int16 imu963ra_gyro_z_raw;       // Z轴原始LSB值
    float imu963ra_gyro_z_dps;       // Z轴角速度(°/s)
    
    // ========== SCH16TK10 角速度（零偏补偿后，作为标签）==========
    int32 sch16tk10_gyro_z_raw;      // Z轴原始LSB值
    float sch16tk10_gyro_z_dps;      // Z轴角速度(°/s) - 训练目标 ?
    
    // ========== 双陀螺仪对比 ==========
    float gyro_diff_dps;             // 两陀螺仪角速度差(°/s)
    
    // ========== 数据质量标志 ==========
    uint8 data_valid;                // 数据是否有效(0/1)
    
} imu_training_data_t;

// 数据记录器统计信息
typedef struct {
    uint32 total_records;            // 总记录数
    uint32 buffer_records;           // 缓冲区记录数
    uint32 flash_records;            // Flash已存储记录数
    uint8 is_logging;                // 是否正在记录
    uint8 buffer_full;               // 缓冲区是否已满
} imu_log_stats_t;

//=================================================外部接口声明================================================

/**
 * @brief  初始化数据记录器
 * @return imu_log_status_enum  初始化状态
 */
imu_log_status_enum imu_logger_init(void);

/**
 * @brief  开始记录数据
 * @return imu_log_status_enum  状态
 */
imu_log_status_enum imu_logger_start(void);

/**
 * @brief  停止记录数据
 * @return imu_log_status_enum  状态
 */
imu_log_status_enum imu_logger_stop(void);

/**
 * @brief  更新数据记录（在定时器中调用，建议1ms周期）
 * @param  left_speed   左轮速度(m/s)
 * @param  right_speed  右轮速度(m/s)
 * @return imu_log_status_enum  状态
 */
imu_log_status_enum imu_logger_update(float left_speed, float right_speed);

/**
 * @brief  保存缓冲区数据到Flash
 * @return imu_log_status_enum  状态
 */
imu_log_status_enum imu_logger_save_to_flash(void);

/**
 * @brief  从Flash读取数据
 * @param  buffer       数据缓冲区指针
 * @param  start_index  起始索引
 * @param  count        读取数量
 * @return uint32       实际读取的数量
 */
uint32 imu_logger_read_from_flash(imu_training_data_t *buffer, uint32 start_index, uint32 count);

/**
 * @brief  导出CSV格式数据（通过串口）
 * @param  start_index  起始索引
 * @param  count        导出数量（0表示全部）
 * @return imu_log_status_enum  状态
 */
imu_log_status_enum imu_logger_export_csv(uint32 start_index, uint32 count);

/**
 * @brief  清空Flash中的数据
 * @return imu_log_status_enum  状态
 */
imu_log_status_enum imu_logger_clear_flash(void);

/**
 * @brief  获取记录器统计信息
 * @param  stats  统计信息输出指针
 * @return imu_log_status_enum  状态
 */
imu_log_status_enum imu_logger_get_stats(imu_log_stats_t *stats);

/**
 * @brief  获取当前缓冲区记录数
 * @return uint32  记录数
 */
uint32 imu_logger_get_buffer_count(void);

/**
 * @brief  获取Flash中的记录数
 * @return uint32  记录数
 */
uint32 imu_logger_get_flash_count(void);

#endif // _IMU_DATA_LOGGER_H_
