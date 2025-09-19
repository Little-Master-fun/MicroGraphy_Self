/*********************************************************************************************************************
* 文件名称          test_encoder.c
* 功能说明          双编码器驱动测试程序实现文件
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本
* 2025-09-17        LittleMaster       1.0v
* 
* 文件作用说明：
* 本文件为双编码器驱动测试程序的实现，用于验证编码器驱动的各项功能
* 
* 测试内容：
* 1. 编码器基本功能测试（初始化、脉冲计数、方向判断）
* 2. 编码器速度计算测试
* 3. 编码器里程统计测试
* 4. 综合功能测试（实时数据显示）
* 
* 使用方式：
* 在主函数中调用相应的测试函数即可开始测试
* 
* 注意事项：
* 1. 确保编码器硬件连接正确
* 2. 手动转动车轮进行测试
* 3. 观察屏幕显示的数据是否合理
********************************************************************************************************************/

#include "test_encoder.h"
#include "driver_encoder.h"
#include "zf_device_ips114.h"
#include "zf_common_headfile.h"
#include <stdio.h>

//=================================================全局变量定义================================================
static uint8 line_count = 0;           // 显示行计数器

//=================================================内部函数声明================================================
static void display_test_info(const char* info);

//=================================================测试函数实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     编码器基本功能测试
//-------------------------------------------------------------------------------------------------------------------
void test_encoder_basic(void)
{
    ips114_clear();
    ips114_show_string(0, 0, "Encoder Basic Test");
    ips114_show_string(0, 16, "Turn wheels manually");
    
    // 初始化编码器
    if (encoder_init() != ENCODER_STATUS_OK)
    {
        ips114_show_string(0, 32, "Init Failed!");
        return;
    }
    
    display_test_info("Encoder Init OK");
    
    uint32 test_duration = 0;
    while (test_duration < 10000)  // 测试10秒
    {
        // 读取编码器数据
        encoder_read_data(ENCODER_ID_BOTH);
        
        // 获取脉冲计数
        int32 left_pulse = encoder_get_pulse_count(ENCODER_ID_LEFT);
        int32 right_pulse = encoder_get_pulse_count(ENCODER_ID_RIGHT);
        
        // 显示结果
        char info[50];
        sprintf(info, "L:%ld R:%ld", left_pulse, right_pulse);
        display_test_info(info);
        
        system_delay_ms(100);
        test_duration += 100;
    }
    
    display_test_info("Basic Test Complete");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     编码器速度计算测试
//-------------------------------------------------------------------------------------------------------------------
void test_encoder_speed(void)
{
    ips114_clear();
    ips114_show_string(0, 0, "Speed Test");
    ips114_show_string(0, 16, "Rotate wheels fast");
    
    uint32 test_duration = 0;
    while (test_duration < 15000)  // 测试15秒
    {
        // 更新编码器数据
        encoder_update();
        
        // 计算速度
        encoder_calculate_speed(ENCODER_ID_BOTH);
        
        // 获取速度值
        float left_speed = encoder_get_speed(ENCODER_ID_LEFT);
        float right_speed = encoder_get_speed(ENCODER_ID_RIGHT);
        
        // 显示结果
        char info[50];
        sprintf(info, "L:%.3f R:%.3f m/s", left_speed, right_speed);
        display_test_info(info);
        
        system_delay_ms(200);
        test_duration += 200;
    }
    
    display_test_info("Speed Test Complete");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     编码器距离测试
//-------------------------------------------------------------------------------------------------------------------
void test_encoder_distance(void)
{
    ips114_clear();
    ips114_show_string(0, 0, "Distance Test");
    ips114_show_string(0, 16, "Move robot forward");
    
    // 重置编码器数据
    encoder_reset(ENCODER_ID_BOTH);
    display_test_info("Distance Reset");
    
    uint32 test_duration = 0;
    while (test_duration < 20000)  // 测试20秒
    {
        // 更新编码器数据
        encoder_update();
        encoder_calculate_speed(ENCODER_ID_BOTH);
        
        // 获取距离值
        float left_distance = encoder_get_distance(ENCODER_ID_LEFT);
        float right_distance = encoder_get_distance(ENCODER_ID_RIGHT);
        
        // 显示结果
        char info[50];
        sprintf(info, "L:%.1f R:%.1f mm", left_distance, right_distance);
        display_test_info(info);
        
        system_delay_ms(300);
        test_duration += 300;
    }
    
    display_test_info("Distance Test Complete");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     编码器综合测试
//-------------------------------------------------------------------------------------------------------------------
void test_encoder(void)
{
    // 初始化屏幕
    ips114_set_dir(IPS114_PORTAIT);
    ips114_set_color(RGB565_WHITE, RGB565_BLACK);
    ips114_init();
    ips114_clear();
    
    ips114_show_string(0, 0, "Encoder Full Test");
    system_delay_ms(2000);
    
    // 运行各项测试
    test_encoder_basic();
    ips114_show_string(0, 112, "Basic Test OK");
    
    system_delay_ms(1000);
    
    test_encoder_speed();
    ips114_show_string(0, 112, "Speed Test OK");
    
    system_delay_ms(1000);
    
    test_encoder_distance();
    ips114_show_string(0, 112, "Distance Test OK");
    
    // 实时监控模式
    ips114_clear();
    ips114_show_string(0, 0, "Real-time Monitor");
    ips114_show_string(0, 16, "Press key to exit");
    
    while(1)
    {
        // 更新数据
        encoder_update();
        encoder_calculate_speed(ENCODER_ID_BOTH);
        
        // 获取所有数据
        int32 left_pulse = encoder_get_pulse_count(ENCODER_ID_LEFT);
        int32 right_pulse = encoder_get_pulse_count(ENCODER_ID_RIGHT);
        float left_speed = encoder_get_speed(ENCODER_ID_LEFT);
        float right_speed = encoder_get_speed(ENCODER_ID_RIGHT);
        float left_distance = encoder_get_distance(ENCODER_ID_LEFT);
        float right_distance = encoder_get_distance(ENCODER_ID_RIGHT);
        
        float linear_vel, angular_vel;
        encoder_get_robot_velocity(&linear_vel, &angular_vel);
        
        // 显示数据
        char info[50];
        sprintf(info, "Pulse L:%ld R:%ld", left_pulse, right_pulse);
        ips114_show_string(0, 32, info);
        
        sprintf(info, "Speed L:%.3f R:%.3f", left_speed, right_speed);
        ips114_show_string(0, 48, info);
        
        sprintf(info, "Dist L:%.1f R:%.1f", left_distance, right_distance);
        ips114_show_string(0, 64, info);
        
        sprintf(info, "Robot V:%.3f W:%.3f", linear_vel, angular_vel);
        ips114_show_string(0, 80, info);
        
        system_delay_ms(100);
        
        // 检查退出条件（可以根据按键等实现）
        // if (exit_condition) break;
    }
}

//=================================================内部函数实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示测试信息（内部函数）
//-------------------------------------------------------------------------------------------------------------------
static void display_test_info(const char* info)
{
    // 显示信息
    ips114_show_string(0, line_count * 16, info);
    
    // 更新行计数，超过屏幕高度时清屏重新开始
    line_count++;
    if (line_count > 7)  // IPS114屏幕大约可显示8行
    {
        line_count = 0;
        system_delay_ms(2000);  // 暂停2秒让用户看清信息
        ips114_clear();
    }
}