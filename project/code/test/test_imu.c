/*********************************************************************************************************************
* 文件名称          test_imu.c
* 功能说明          IMU姿态计算系统测试程序实现文件 - 使用SCH16TK10真实传感器
* 作者              LittleMaster  
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本
* 2025-09-28        LittleMaster       1.0v
* 
* IPS114显示安全区域 (240×135像素，8×16字体)：
* - 标题区域：y=0-16     - "IMU Test System"
* - 角度区域：y=32-80    - 实时角度显示（俯仰、滚转、偏航）
* - 状态区域：y=80-112   - 系统状态和统计信息
* 
* 架构设计：
* 1. 1ms中断采样IMU数据 - 1kHz高频数据采集 (在imu_attitude.c中实现)
* 2. 主循环进行姿态解算 - 10Hz合理的处理频率  
* 3. 显示更新 - 10Hz满足用户观察需求
* 4. 使用SCH16TK10高精度IMU传感器
********************************************************************************************************************/

#include "test_imu.h"
#include "imu_attitude.h"
#include "zf_device_ips114.h"
#include "zf_common_headfile.h"
#include "zf_driver_timer.h"
#include "zf_driver_delay.h"
#include <stdio.h>
#include <math.h>

//=================================================全局变量定义================================================
imu_test_system_t imu_test_system;

// 用于时间测量的定时器
static uint8 test_timer_initialized = 0;

//=================================================内部函数声明================================================
static void display_realtime_angles(void);
static void display_system_status(void);
static void update_statistics(imu_euler_t *euler);
static uint32 test_get_time_ms(void);
static void test_init_timer(void);
static void draw_filled_rectangle(uint16 x, uint16 y, uint16 width, uint16 height, uint16 color);

//=================================================外部接口实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IMU测试系统主函数 - 使用SCH16TK10传感器
//-------------------------------------------------------------------------------------------------------------------
void test_imu_system(void)
{
    printf("=== IMU姿态计算系统测试开始 (SCH16TK10) ===\r\n");
    
    // 初始化测试系统
    if(!test_imu_init(IMU_TEST_MODE_REALTIME))
    {
        printf("IMU测试系统初始化失败！\r\n");
        return;
    }
    
    printf("IMU测试系统初始化成功\r\n");
    printf("开始实时角度显示...\r\n");
    printf("数据采样频率: 1kHz (1ms中断，在控制模块中实现)\r\n");
    printf("姿态解算频率: 10Hz\r\n");
    printf("显示更新频率: 10Hz\r\n");
    printf("注意：请在1ms中断中调用 imu_data_collection_handler() 函数\r\n");
    
    imu_test_system.status = IMU_TEST_STATUS_RUNNING;
    
    // 主循环 - 只处理姿态解算和显示更新
    while(1)
    {
        uint32 current_time = test_get_time_ms();
        
        // 姿态解算更新 - 10Hz (IMU数据采样由1ms中断处理)
        if(current_time - imu_test_system.last_attitude_time >= imu_test_system.config.attitude_update_ms)
        {
            imu_test_system.last_attitude_time = current_time;
            
            // 进行姿态解算 (数据已在中断中更新到imu_attitude模块)
            imu_attitude_update();
            
            // 更新统计信息
            imu_euler_t euler;
            if(imu_get_euler_angles(&euler) == IMU_STATUS_OK)
            {
                update_statistics(&euler);
            }
        }
        
        // 显示更新 - 10Hz
        if(current_time - imu_test_system.last_display_time >= imu_test_system.config.display_refresh_ms)
        {
            imu_test_system.last_display_time = current_time;
            display_realtime_angles();
            display_system_status();
        }
        
        system_delay_ms(10); // 100Hz主循环，确保响应及时
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     1ms中断中的IMU数据采样处理 (向后兼容接口)
// 备注信息     该函数已迁移到 imu_attitude.c，此处保留向后兼容性
//-------------------------------------------------------------------------------------------------------------------
void imu_1ms_interrupt_handler(void)
{
    // 调用控制模块中的数据采集函数
    imu_data_collection_handler();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化IMU测试系统
//-------------------------------------------------------------------------------------------------------------------
uint8 test_imu_init(imu_test_mode_enum test_mode)
{
    // 清空测试系统结构体
    memset(&imu_test_system, 0, sizeof(imu_test_system_t));
    
    // 设置配置参数
    imu_test_system.config.test_mode = test_mode;
    imu_test_system.config.auto_calibration = 1;
    imu_test_system.config.display_refresh_ms = TEST_IMU_DISPLAY_REFRESH_MS;
    imu_test_system.config.attitude_update_ms = TEST_IMU_ATTITUDE_UPDATE_MS;
    
    // 初始化统计信息
    imu_test_system.stats.max_pitch = -180.0f;
    imu_test_system.stats.min_pitch = 180.0f;
    imu_test_system.stats.max_roll = -180.0f;
    imu_test_system.stats.min_roll = 180.0f;
    imu_test_system.stats.max_yaw = -180.0f;
    imu_test_system.stats.min_yaw = 180.0f;
    imu_test_system.stats.start_time = test_get_time_ms();
    
    // 初始化IMU姿态计算系统 (包含传感器初始化)
    if(imu_attitude_init(IMU_MODE_COMPLEMENTARY) != IMU_STATUS_OK)
    {
        printf("IMU姿态系统初始化失败\r\n");
        return 0;
    }
    
    // 初始化显示屏
    ips114_init();
    ips114_clear();
    ips114_set_color(RGB565_WHITE, RGB565_BLACK);
    ips114_show_string(5, 0, "IMU Test - SCH16TK10");
    
    imu_test_system.status = IMU_TEST_STATUS_IDLE;
    imu_test_system.system_initialized = 1;
    
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取测试系统时间戳
//-------------------------------------------------------------------------------------------------------------------
uint32 test_imu_get_timestamp(void)
{
    return test_get_time_ms();
}

//=================================================内部函数实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示实时角度信息
//-------------------------------------------------------------------------------------------------------------------
static void display_realtime_angles(void)
{
    imu_euler_t euler;
    char angle_str[32];
    
    if(imu_get_euler_angles(&euler) == IMU_STATUS_OK)
    {
        // // 清除角度显示区域
        // draw_filled_rectangle(0, 32, 240, 48, RGB565_BLACK);
        // ips114_set_color(RGB565_WHITE, RGB565_BLACK);
        
        // 显示俯仰角
        sprintf(angle_str, "Pitch: %6.2f deg", euler.pitch_deg);
        ips114_show_string(5, 32, angle_str);
        
        // 显示滚转角
        sprintf(angle_str, "Roll:  %6.2f deg", euler.roll_deg);
        ips114_show_string(5, 48, angle_str);
        
        // 显示偏航角
        sprintf(angle_str, "Yaw:   %6.2f deg", euler.yaw_deg);
        ips114_show_string(5, 64, angle_str);
        
        // 显示角度变化的简单图形指示
        // 俯仰角指示条
        int pitch_bar = (int)((euler.pitch_deg + 90.0f) * 100.0f / 180.0f);
        if(pitch_bar < 0) pitch_bar = 0;
        if(pitch_bar > 100) pitch_bar = 100;
        
        draw_filled_rectangle(200, 32, pitch_bar/5, 4, RGB565_GREEN);
        draw_filled_rectangle(200 + pitch_bar/5, 32, 240 - (200 + pitch_bar/5), 4, RGB565_BLACK);
        
        // 滚转角指示条
        int roll_bar = (int)((euler.roll_deg + 90.0f) * 100.0f / 180.0f);
        if(roll_bar < 0) roll_bar = 0;
        if(roll_bar > 100) roll_bar = 100;
        
        draw_filled_rectangle(200, 48, roll_bar/5, 4, RGB565_BLUE);
        draw_filled_rectangle(200 + roll_bar/5, 48, 240 - (200 + roll_bar/5), 4, RGB565_BLACK);
        
        // 偏航角指示条
        int yaw_bar = (int)((euler.yaw_deg + 180.0f) * 100.0f / 360.0f);
        if(yaw_bar < 0) yaw_bar = 0;
        if(yaw_bar > 100) yaw_bar = 100;
        
        draw_filled_rectangle(200, 64, yaw_bar/5, 4, RGB565_RED);
        draw_filled_rectangle(200 + yaw_bar/5, 64, 240 - (200 + yaw_bar/5), 4, RGB565_BLACK);
        
        ips114_set_color(RGB565_WHITE, RGB565_BLACK);
    }
    else
    {
        ips114_show_string(5, 32, "IMU Data Error!");
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示系统状态
//-------------------------------------------------------------------------------------------------------------------
static void display_system_status(void)
{
    char status_str[32];
    
    // 运行时间
    uint32 runtime = test_get_time_ms() - imu_test_system.stats.start_time;
    sprintf(status_str, "Runtime: %d.%ds", (int)(runtime/1000), (int)((runtime%1000)/100));
    ips114_show_string(5, 80, status_str);
    
    // 更新次数
    sprintf(status_str, "Updates: %d", (int)imu_test_system.stats.update_count);
    ips114_show_string(5, 96, status_str);
    
    // 错误次数
    sprintf(status_str, "Errors: %d", (int)imu_test_system.stats.error_count);
    ips114_show_string(5, 112, status_str);
    
    // IMU系统状态
    imu_attitude_system_t imu_system;
    if(imu_get_system_status(&imu_system) == IMU_STATUS_OK)
    {
        const char* imu_status_text = "OK";
        switch(imu_system.status)
        {
            case IMU_STATUS_OK: imu_status_text = "OK"; break;
            case IMU_STATUS_CALIBRATING: imu_status_text = "CAL"; break;
            case IMU_STATUS_ERROR: imu_status_text = "ERR"; break;
            case IMU_STATUS_NOT_INIT: imu_status_text = "INIT"; break;
            default: imu_status_text = "UNK"; break;
        }
        
        sprintf(status_str, "IMU: %s", imu_status_text);
        ips114_show_string(120, 96, status_str);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新统计信息
//-------------------------------------------------------------------------------------------------------------------
static void update_statistics(imu_euler_t *euler)
{
    imu_test_system.stats.update_count++;
    
    // 更新角度范围
    if(euler->pitch_deg > imu_test_system.stats.max_pitch)
        imu_test_system.stats.max_pitch = euler->pitch_deg;
    if(euler->pitch_deg < imu_test_system.stats.min_pitch)
        imu_test_system.stats.min_pitch = euler->pitch_deg;
    
    if(euler->roll_deg > imu_test_system.stats.max_roll)
        imu_test_system.stats.max_roll = euler->roll_deg;
    if(euler->roll_deg < imu_test_system.stats.min_roll)
        imu_test_system.stats.min_roll = euler->roll_deg;
    
    if(euler->yaw_deg > imu_test_system.stats.max_yaw)
        imu_test_system.stats.max_yaw = euler->yaw_deg;
    if(euler->yaw_deg < imu_test_system.stats.min_yaw)
        imu_test_system.stats.min_yaw = euler->yaw_deg;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取当前时间（毫秒）
//-------------------------------------------------------------------------------------------------------------------
static uint32 test_get_time_ms(void)
{
    if(!test_timer_initialized)
    {
        test_init_timer();
    }
    return timer_get(TC_TIME2_CH1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化测试时间测量定时器
//-------------------------------------------------------------------------------------------------------------------
static void test_init_timer(void)
{
    if(!test_timer_initialized)
    {
        timer_init(TC_TIME2_CH1, TIMER_MS);
        timer_clear(TC_TIME2_CH1);
        timer_start(TC_TIME2_CH1);
        test_timer_initialized = 1;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     绘制填充矩形（替代不存在的ips114_show_rectangle函数）
//-------------------------------------------------------------------------------------------------------------------
static void draw_filled_rectangle(uint16 x, uint16 y, uint16 width, uint16 height, uint16 color)
{
    // 边界检查防止超出屏幕范围 (240x135)
    if(x >= 240 || y >= 135) return;
    
    // 限制宽度和高度在屏幕范围内
    if(x + width > 240) width = 240 - x;
    if(y + height > 135) height = 135 - y;
    
    // 使用多条线绘制矩形
    for(uint16 i = 0; i < height; i++)
    {
        ips114_draw_line(x, y + i, x + width - 1, y + i, color);
    }
}