/*********************************************************************************************************************
* 文件名称          imu_data_logger.c
* 功能说明          IMU数据采集与Flash存储系统实现
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本
* 2025-11-25        LittleMaster       1.0v
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "imu_data_logger.h"
#include "imu_ahrs_complementary.h"
#include "driver_sch16tk10.h"
#include "zf_device_imu963ra.h"
#include "zf_driver_flash.h"
#include "driver_wireless_uart.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

//=================================================全局变量================================================
static imu_training_data_t data_buffer[IMU_LOG_BUFFER_SIZE];
static uint32 buffer_index = 0;
static uint32 total_records = 0;
static uint32 flash_record_count = 0;

// 温度跟踪
static float last_temperature = 25.0f;
static uint8 temperature_initialized = 0;

// 采样控制
static uint32 sample_timer = 0;
static uint8 logger_enabled = 0;
static uint8 logger_initialized = 0;

// 系统时间（需要外部提供，或使用定时器累加）
static uint32 system_time_ms = 0;

// 无线传输控制
static uint8 wireless_tx_enabled = 0;    // 无线传输使能
static uint32 wireless_tx_count = 0;     // 已传输记录数

// Flash存储控制
static uint8 flash_storage_enabled = 1;  // Flash存储使能（默认开启）

//=================================================内部函数声明================================================
static void update_system_time(void);
static float calculate_temp_rate(float current_temp);

//=================================================函数实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化数据记录器
// 返回参数     初始化状态
//-------------------------------------------------------------------------------------------------------------------
imu_log_status_enum imu_logger_init(void)
{
    // 清空缓冲区
    memset(data_buffer, 0, sizeof(data_buffer));
    buffer_index = 0;
    total_records = 0;
    flash_record_count = 0;
    sample_timer = 0;
    logger_enabled = 0;
    system_time_ms = 0;
    temperature_initialized = 0;
    
    // 初始化Flash
    flash_init();
    
    // 尝试读取Flash中已有的记录数
    // 这里简化处理，假设Flash第一个uint32存储记录数
    uint32 flash_header[2];
    flash_read_page(0, 0, flash_header, 2);  // sector 0, page 0
    
    if(flash_header[0] == 0x12345678)  // 魔术字，表示Flash已初始化
    {
        flash_record_count = flash_header[1];
        if(flash_record_count > IMU_LOG_MAX_FLASH_RECORDS)
            flash_record_count = 0;  // 数据异常，重置
    }
    else
    {
        flash_record_count = 0;
    }
    
    logger_initialized = 1;
    
    
    return IMU_LOG_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     开始记录数据
// 返回参数     状态
//-------------------------------------------------------------------------------------------------------------------
imu_log_status_enum imu_logger_start(void)
{
    if(!logger_initialized)
        return IMU_LOG_STATUS_NOT_INIT;
    
    if(!ahrs_is_ready())
    {
        printf("AHRS系统未就绪，无法开始记录\r\n");
        return IMU_LOG_STATUS_ERROR;
    }
    
    logger_enabled = 1;
    sample_timer = 0;
    system_time_ms = 0;
    
    printf("开始记录IMU数据...\r\n");
    return IMU_LOG_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     停止记录数据
// 返回参数     状态
//-------------------------------------------------------------------------------------------------------------------
imu_log_status_enum imu_logger_stop(void)
{
    if(!logger_initialized)
        return IMU_LOG_STATUS_NOT_INIT;
    
    logger_enabled = 0;
    
    printf("停止记录数据，缓冲区共 %d 条记录\r\n", buffer_index);
    return IMU_LOG_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新数据记录（在1ms定时器中调用）
// 参数说明     left_speed   左轮速度(m/s)
// 参数说明     right_speed  右轮速度(m/s)
// 返回参数     状态
//-------------------------------------------------------------------------------------------------------------------
imu_log_status_enum imu_logger_update(float left_speed, float right_speed)
{
    if(!logger_initialized || !logger_enabled)
        return IMU_LOG_STATUS_NOT_INIT;
    
    if(!ahrs_is_ready())
        return IMU_LOG_STATUS_ERROR;
    
    // 更新系统时间
    update_system_time();
    
    // 采样率控制
    sample_timer++;
    if(sample_timer < IMU_LOG_SAMPLE_RATE_MS)
        return IMU_LOG_STATUS_OK;
    sample_timer = 0;
    
    // 临时数据结构（用于收集数据）
    imu_training_data_t temp_data;
    imu_training_data_t *data = &temp_data;
    
    // ========== 时间戳 ==========
    data->timestamp_ms = system_time_ms;
    data->power_on_time_s = system_time_ms / 1000;
    
    // ========== 获取SCH16TK10数据 ==========
    
    // 温度信息
    data->temperature_deg = g_ahrs_debug.sch_temp_deg;  // 需要在AHRS中添加温度字段
    data->temp_rate_degps = calculate_temp_rate(data->temperature_deg);
    
    // SCH16TK10角速度（零偏补偿后，作为训练标签）
    data->sch16tk10_gyro_z_raw = g_ahrs_debug.raw_gyro_z;
    data->sch16tk10_gyro_z_dps = g_ahrs_debug.gyro_z_dps;
    
    // ========== 获取IMU963RA数据 ==========
    // 读取陀螺仪
    imu963ra_get_gyro();
    data->imu963ra_gyro_z_raw = imu963ra_gyro_z;
    data->imu963ra_gyro_z_dps = imu963ra_gyro_transition(imu963ra_gyro_z);
    
    // 读取加速度
    imu963ra_get_acc();
    data->imu963ra_acc_x = imu963ra_acc_transition(imu963ra_acc_x);
    data->imu963ra_acc_y = imu963ra_acc_transition(imu963ra_acc_y);
    data->imu963ra_acc_z = imu963ra_acc_transition(imu963ra_acc_z);
    
    // ========== 轮速数据 ==========
    data->left_wheel_speed = left_speed;
    data->right_wheel_speed = right_speed;
    data->linear_velocity = (left_speed + right_speed) / 2.0f;
    
    // 判断是否静止（速度小于0.01m/s）
    data->is_stationary = (fabsf(data->linear_velocity) < 0.01f) ? 1 : 0;
    
    // ========== 双陀螺仪对比 ==========
    data->gyro_diff_dps = data->imu963ra_gyro_z_dps - data->sch16tk10_gyro_z_dps;
    
    // ========== 数据质量标志 ==========
    data->data_valid = 1;
    
    // ========== 无线串口实时传输 ==========
    if(wireless_tx_enabled)
    {
        imu_logger_wireless_send_record(data);
    }
    
    // ========== Flash缓冲区存储 ==========
    if(flash_storage_enabled)
    {
        // 检查缓冲区是否已满
        if(buffer_index >= IMU_LOG_BUFFER_SIZE)
        {
            printf("数据缓冲区已满，停止记录\r\n");
            logger_enabled = 0;
            return IMU_LOG_STATUS_BUFFER_FULL;
        }
        
        // 存入缓冲区
        memcpy(&data_buffer[buffer_index], data, sizeof(imu_training_data_t));
        buffer_index++;
    }
    
    // 更新总记录数
    total_records++;
    
    // 每100条记录打印一次进度
    if(total_records % 100 == 0)
    {
        printf("[LOG] 记录 %d 条 | 温度=%.2f°C | 陀螺差=%.3f°/s | 速度=%.3fm/s\r\n", 
               total_records, data->temperature_deg, data->gyro_diff_dps, data->linear_velocity);
    }
    
    return IMU_LOG_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     保存缓冲区数据到Flash
// 返回参数     状态
//-------------------------------------------------------------------------------------------------------------------
imu_log_status_enum imu_logger_save_to_flash(void)
{
    if(!logger_initialized)
        return IMU_LOG_STATUS_NOT_INIT;
    
    if(buffer_index == 0)
    {
        printf("没有数据可保存\r\n");
        return IMU_LOG_STATUS_OK;
    }
    
    // 检查Flash容量
    if(flash_record_count + buffer_index > IMU_LOG_MAX_FLASH_RECORDS)
    {
        printf("Flash空间不足，无法保存\r\n");
        return IMU_LOG_STATUS_FLASH_FULL;
    }
    
    printf("正在保存 %d 条记录到Flash...\r\n", buffer_index);
    
    // 计算写入位置（使用页号）
    // 头部占用2个字，每条记录约15个字（60字节）
    uint32 header_words = 2;
    uint32 record_size_words = (sizeof(imu_training_data_t) + 3) / 4;
    uint32 start_word = header_words + (flash_record_count * record_size_words);
    uint32 start_page = start_word / FLASH_PAGE_LENGTH;
    
    // 分页写入数据（简化处理，假设不跨页）
    uint32 total_bytes = buffer_index * sizeof(imu_training_data_t);
    uint32 total_words = (total_bytes + 3) / 4;
    
    flash_write_page(0, start_page, (const uint32*)data_buffer, total_words);
    
    // 更新Flash记录数
    flash_record_count += buffer_index;
    
    // 更新Flash头部
    uint32 flash_header[2];
    flash_header[0] = 0x12345678;  // 魔术字
    flash_header[1] = flash_record_count;
    flash_write_page(0, 0, (const uint32*)flash_header, 2);
    
    printf("数据保存完成，Flash共 %d 条记录\r\n", flash_record_count);
    
    // 清空缓冲区
    buffer_index = 0;
    memset(data_buffer, 0, sizeof(data_buffer));
    
    return IMU_LOG_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     从Flash读取数据
// 参数说明     buffer       数据缓冲区指针
// 参数说明     start_index  起始索引
// 参数说明     count        读取数量
// 返回参数     实际读取的数量
//-------------------------------------------------------------------------------------------------------------------
uint32 imu_logger_read_from_flash(imu_training_data_t *buffer, uint32 start_index, uint32 count)
{
    if(!logger_initialized || buffer == NULL)
        return 0;
    
    if(start_index >= flash_record_count)
        return 0;
    
    // 限制读取数量
    uint32 actual_count = count;
    if(start_index + count > flash_record_count)
        actual_count = flash_record_count - start_index;
    
    // 计算读取位置（使用页号）
    uint32 header_words = 2;
    uint32 record_size_words = (sizeof(imu_training_data_t) + 3) / 4;
    uint32 start_word = header_words + (start_index * record_size_words);
    uint32 start_page = start_word / FLASH_PAGE_LENGTH;
    
    // 读取数据（简化处理，假设不跨页）
    uint32 total_bytes = actual_count * sizeof(imu_training_data_t);
    uint32 total_words = (total_bytes + 3) / 4;
    
    flash_read_page(0, start_page, (uint32*)buffer, total_words);
    
    return actual_count;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导出CSV格式数据（通过串口）
// 参数说明     start_index  起始索引
// 参数说明     count        导出数量（0表示全部）
// 返回参数     状态
//-------------------------------------------------------------------------------------------------------------------
imu_log_status_enum imu_logger_export_csv(uint32 start_index, uint32 count)
{
    if(!logger_initialized)
        return IMU_LOG_STATUS_NOT_INIT;
    
    uint32 export_count = count;
    if(count == 0 || start_index + count > flash_record_count)
        export_count = flash_record_count - start_index;
    
    if(export_count == 0)
    {
        printf("没有数据可导出\r\n");
        return IMU_LOG_STATUS_OK;
    }
    
    printf("\r\n========== CSV数据开始 ==========\r\n");
    
    // CSV表头
    printf("timestamp_ms,power_on_time_s,temperature_deg,temp_rate_degps,");
    printf("left_speed,right_speed,linear_vel,is_stationary,");
    printf("imu963ra_acc_x,imu963ra_acc_y,imu963ra_acc_z,");
    printf("imu963ra_gyro_z_raw,imu963ra_gyro_z_dps,");
    printf("sch16tk10_gyro_z_raw,sch16tk10_gyro_z_dps,");
    printf("gyro_diff_dps\r\n");
    
    // 分批读取并导出
    imu_training_data_t temp_buffer[50];  // 临时缓冲区
    uint32 exported = 0;
    
    while(exported < export_count)
    {
        uint32 batch_size = (export_count - exported) > 50 ? 50 : (export_count - exported);
        uint32 read_count = imu_logger_read_from_flash(temp_buffer, start_index + exported, batch_size);
        
        for(uint32 i = 0; i < read_count; i++)
        {
            imu_training_data_t *d = &temp_buffer[i];
            
            printf("%u,%u,%.3f,%.6f,",
                   d->timestamp_ms, d->power_on_time_s, 
                   d->temperature_deg, d->temp_rate_degps);
            
            printf("%.4f,%.4f,%.4f,%d,",
                   d->left_wheel_speed, d->right_wheel_speed, 
                   d->linear_velocity, d->is_stationary);
            
            printf("%.6f,%.6f,%.6f,",
                   d->imu963ra_acc_x, d->imu963ra_acc_y, d->imu963ra_acc_z);
            
            printf("%d,%.6f,",
                   d->imu963ra_gyro_z_raw, d->imu963ra_gyro_z_dps);
            
            printf("%d,%.6f,",
                   d->sch16tk10_gyro_z_raw, d->sch16tk10_gyro_z_dps);
            
            printf("%.6f\r\n", d->gyro_diff_dps);
        }
        
        exported += read_count;
    }
    
    printf("========== CSV数据结束 ==========\r\n");
    printf("共导出 %d 条记录\r\n\r\n", exported);
    
    return IMU_LOG_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清空Flash中的数据
// 返回参数     状态
//-------------------------------------------------------------------------------------------------------------------
imu_log_status_enum imu_logger_clear_flash(void)
{
    if(!logger_initialized)
        return IMU_LOG_STATUS_NOT_INIT;
    
    printf("正在清空Flash数据...\r\n");
    
    // 擦除所有使用的页（擦除前10页，足够存储1000条记录）
    for(uint32 page = 0; page < 10; page++)
    {
        flash_erase_page(0, page);
    }
    
    // 重置计数
    flash_record_count = 0;
    
    printf("Flash数据已清空\r\n");
    
    return IMU_LOG_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取记录器统计信息
// 参数说明     stats  统计信息输出指针
// 返回参数     状态
//-------------------------------------------------------------------------------------------------------------------
imu_log_status_enum imu_logger_get_stats(imu_log_stats_t *stats)
{
    if(!logger_initialized || stats == NULL)
        return IMU_LOG_STATUS_ERROR;
    
    stats->total_records = total_records;
    stats->buffer_records = buffer_index;
    stats->flash_records = flash_record_count;
    stats->is_logging = logger_enabled;
    stats->buffer_full = (buffer_index >= IMU_LOG_BUFFER_SIZE) ? 1 : 0;
    
    return IMU_LOG_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取当前缓冲区记录数
// 返回参数     记录数
//-------------------------------------------------------------------------------------------------------------------
uint32 imu_logger_get_buffer_count(void)
{
    return buffer_index;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取Flash中的记录数
// 返回参数     记录数
//-------------------------------------------------------------------------------------------------------------------
uint32 imu_logger_get_flash_count(void)
{
    return flash_record_count;
}

//=================================================内部函数实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新系统时间（每次调用增加1ms）
//-------------------------------------------------------------------------------------------------------------------
static void update_system_time(void)
{
    system_time_ms++;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算温度变化率
// 参数说明     current_temp  当前温度
// 返回参数     温度变化率(°C/s)
//-------------------------------------------------------------------------------------------------------------------
static float calculate_temp_rate(float current_temp)
{
    if(!temperature_initialized)
    {
        last_temperature = current_temp;
        temperature_initialized = 1;
        return 0.0f;
    }
    
    float temp_rate = (current_temp - last_temperature) / (IMU_LOG_SAMPLE_RATE_MS / 1000.0f);
    last_temperature = current_temp;
    
    return temp_rate;
}

//=================================================无线串口传输函数================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     使能/禁用Flash存储
// 参数说明     enable  1-使能，0-禁用
//-------------------------------------------------------------------------------------------------------------------
void imu_logger_flash_enable(uint8 enable)
{
    flash_storage_enabled = enable;
    
    if(enable)
    {
        printf("Flash缓冲区存储已使能\r\n");
    }
    else
    {
        printf("Flash缓冲区存储已禁用（仅无线发送）\r\n");
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取Flash存储状态
// 返回参数     1-已使能，0-已禁用
//-------------------------------------------------------------------------------------------------------------------
uint8 imu_logger_flash_is_enabled(void)
{
    return flash_storage_enabled;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置数据记录模式
// 参数说明     mode  0-仅Flash, 1-仅无线, 2-同时使用
//-------------------------------------------------------------------------------------------------------------------
void imu_logger_set_mode(uint8 mode)
{
    switch(mode)
    {
        case 0:  // 仅Flash存储
            flash_storage_enabled = 1;
            wireless_tx_enabled = 0;
            printf("模式: 仅Flash存储\r\n");
            break;
            
        case 1:  // 仅无线发送
            flash_storage_enabled = 0;
            wireless_tx_enabled = 1;
            printf("模式: 仅无线串口实时发送\r\n");
            // 发送CSV表头
            imu_logger_wireless_send_header();
            break;
            
        case 2:  // 同时使用
            flash_storage_enabled = 1;
            wireless_tx_enabled = 1;
            printf("模式: Flash存储 + 无线发送\r\n");
            // 发送CSV表头
            imu_logger_wireless_send_header();
            break;
            
        default:
            printf("无效模式，使用默认设置（仅Flash）\r\n");
            flash_storage_enabled = 1;
            wireless_tx_enabled = 0;
            break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     使能/禁用无线串口实时传输
// 参数说明     enable  1-使能，0-禁用
//-------------------------------------------------------------------------------------------------------------------
void imu_logger_wireless_enable(uint8 enable)
{
    wireless_tx_enabled = enable;
    wireless_tx_count = 0;
    
    if(enable)
    {
        printf("无线串口实时传输已使能\r\n");
        // 发送CSV表头
        imu_logger_wireless_send_header();
    }
    else
    {
        printf("无线串口实时传输已禁用\r\n");
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     通过无线串口发送CSV表头
//-------------------------------------------------------------------------------------------------------------------
void imu_logger_wireless_send_header(void)
{
    wireless_uart_send_string("timestamp_ms,power_on_time_s,temperature_deg,temp_rate_degps,");
    wireless_uart_send_string("left_speed,right_speed,linear_vel,is_stationary,");
    wireless_uart_send_string("imu963ra_acc_x,imu963ra_acc_y,imu963ra_acc_z,");
    wireless_uart_send_string("imu963ra_gyro_z_raw,imu963ra_gyro_z_dps,");
    wireless_uart_send_string("sch16tk10_gyro_z_raw,sch16tk10_gyro_z_dps,");
    wireless_uart_send_string("gyro_diff_dps\r\n");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     通过无线串口发送单条数据记录
// 参数说明     data  数据记录指针
//-------------------------------------------------------------------------------------------------------------------
void imu_logger_wireless_send_record(const imu_training_data_t *data)
{
    if(!wireless_tx_enabled || data == NULL)
        return;
    
    // 使用缓冲区构建CSV行
    char buffer[256];
    
    // 格式化数据为CSV格式
    sprintf(buffer, "%u,%u,%.3f,%.6f,",
            data->timestamp_ms, data->power_on_time_s, 
            data->temperature_deg, data->temp_rate_degps);
    wireless_uart_send_string(buffer);
    
    sprintf(buffer, "%.4f,%.4f,%.4f,%d,",
            data->left_wheel_speed, data->right_wheel_speed, 
            data->linear_velocity, data->is_stationary);
    wireless_uart_send_string(buffer);
    
    sprintf(buffer, "%.6f,%.6f,%.6f,",
            data->imu963ra_acc_x, data->imu963ra_acc_y, data->imu963ra_acc_z);
    wireless_uart_send_string(buffer);
    
    sprintf(buffer, "%d,%.6f,",
            data->imu963ra_gyro_z_raw, data->imu963ra_gyro_z_dps);
    wireless_uart_send_string(buffer);
    
    sprintf(buffer, "%d,%.6f,",
            data->sch16tk10_gyro_z_raw, data->sch16tk10_gyro_z_dps);
    wireless_uart_send_string(buffer);
    
    sprintf(buffer, "%.6f\r\n", data->gyro_diff_dps);
    wireless_uart_send_string(buffer);
    
    wireless_tx_count++;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     通过无线串口批量发送缓冲区数据
// 参数说明     start_index  起始索引
// 参数说明     count        发送数量（0表示全部）
// 返回参数     实际发送的数量
//-------------------------------------------------------------------------------------------------------------------
uint32 imu_logger_wireless_send_buffer(uint32 start_index, uint32 count)
{
    uint32 send_count = count;
    
    if(start_index >= buffer_index)
        return 0;
    
    // 限制发送数量
    if(count == 0 || start_index + count > buffer_index)
        send_count = buffer_index - start_index;
    
    printf("开始通过无线串口发送数据...\r\n");
    
    // 发送CSV表头
    imu_logger_wireless_send_header();
    
    // 发送数据记录
    for(uint32 i = start_index; i < start_index + send_count; i++)
    {
        imu_logger_wireless_send_record(&data_buffer[i]);
        
        // 每100条显示进度
        if((i - start_index + 1) % 100 == 0)
        {
            printf("已发送 %u 条记录...\r\n", i - start_index + 1);
        }
        
        // 适当延时，避免缓冲区溢出
        system_delay_ms(5);
    }
    
    printf("无线串口发送完成，共 %u 条记录\r\n", send_count);
    
    return send_count;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     通过无线串口批量发送Flash数据
// 参数说明     start_index  起始索引
// 参数说明     count        发送数量（0表示全部）
// 返回参数     实际发送的数量
//-------------------------------------------------------------------------------------------------------------------
uint32 imu_logger_wireless_send_flash(uint32 start_index, uint32 count)
{
    if(!logger_initialized)
        return 0;
    
    uint32 send_count = count;
    if(count == 0 || start_index + count > flash_record_count)
        send_count = flash_record_count - start_index;
    
    if(send_count == 0)
    {
        printf("没有数据可发送\r\n");
        return 0;
    }
    
    printf("开始通过无线串口发送Flash数据...\r\n");
    
    // 发送CSV表头
    imu_logger_wireless_send_header();
    
    // 分批读取并发送（每次50条）
    imu_training_data_t temp_buffer[50];
    uint32 sent = 0;
    
    while(sent < send_count)
    {
        uint32 batch_size = (send_count - sent) > 50 ? 50 : (send_count - sent);
        uint32 read_count = imu_logger_read_from_flash(temp_buffer, start_index + sent, batch_size);
        
        for(uint32 i = 0; i < read_count; i++)
        {
            imu_logger_wireless_send_record(&temp_buffer[i]);
            
            // 适当延时
            system_delay_ms(5);
        }
        
        sent += read_count;
        
        printf("已发送 %u/%u 条记录...\r\n", sent, send_count);
    }
    
    printf("无线串口发送完成，共 %u 条记录\r\n", sent);
    
    return sent;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取无线传输状态
// 返回参数     1-传输使能，0-传输禁用
//-------------------------------------------------------------------------------------------------------------------
uint8 imu_logger_wireless_is_enabled(void)
{
    return wireless_tx_enabled;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取无线传输计数
// 返回参数     已传输的记录数
//-------------------------------------------------------------------------------------------------------------------
uint32 imu_logger_wireless_get_count(void)
{
    return wireless_tx_count;
}
