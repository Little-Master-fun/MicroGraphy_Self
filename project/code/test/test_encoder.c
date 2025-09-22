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
    // 初始化屏幕
    ips114_set_dir(IPS114_PORTAIT);
    ips114_set_color(RGB565_WHITE, RGB565_BLACK);
    ips114_init();
    ips114_clear();
    
    ips114_show_string(0, 0, "Encoder Basic Test");
    ips114_show_string(0, 16, "Turn wheels manually");
    ips114_show_string(0, 32, "Data:");
    
    // 初始化编码器
    if (encoder_init() != ENCODER_STATUS_OK)
    {
        ips114_show_string(0, 48, "ERROR: Init Failed!");
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
        sprintf(info, "L:%d R:%d", (int)left_pulse, (int)right_pulse);
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
    ips114_set_dir(IPS114_PORTAIT);
    ips114_set_color(RGB565_WHITE, RGB565_BLACK);
    ips114_init();
    ips114_clear();
    ips114_show_string(0, 0, "Speed Test");
    ips114_show_string(0, 16, "Rotate wheels fast");
    ips114_show_string(0, 32, "Data:");
    
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
    ips114_set_dir(IPS114_PORTAIT);
    ips114_set_color(RGB565_WHITE, RGB565_BLACK);
    ips114_init();
    ips114_clear();
    ips114_show_string(0, 0, "Distance Test");
    ips114_show_string(0, 16, "Move robot forward");
    ips114_show_string(0, 32, "Data:");
    
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
        sprintf(info, "Pulse L:%d R:%d", (int)left_pulse, (int)right_pulse);
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

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     编码器累计脉冲显示测试
//-------------------------------------------------------------------------------------------------------------------
void test_encoder_pulse_display(void)
{
    // 初始化屏幕
    ips114_set_dir(IPS114_PORTAIT);
    ips114_set_color(RGB565_WHITE, RGB565_BLACK);
    ips114_init();
    ips114_clear();
    
    // 显示标题
    ips114_show_string(0, 0, "Encoder Monitor");
    ips114_show_string(0, 16, "Turn wheels to test");
    
    // 初始化编码器
    if (encoder_init() != ENCODER_STATUS_OK)
    {
        ips114_show_string(0, 32, "ERROR: Init Failed!");
        return;
    }
    
    ips114_show_string(0, 32, "Init Success");
    system_delay_ms(1000);
    
    // 重置编码器累计脉冲
    encoder_reset(ENCODER_ID_BOTH);
    
    uint32_t update_counter = 0;
    int32_t last_left_total = 0;
    int32_t last_right_total = 0;
    
    while(1)  // 无限循环显示
    {
        // 更新编码器数据
        encoder_update();
        
        // 获取当前累计脉冲数
        int32_t left_pulse = encoder_get_pulse_count(ENCODER_ID_LEFT);
        int32_t right_pulse = encoder_get_pulse_count(ENCODER_ID_RIGHT);
        
        // 获取累计总脉冲数（带符号）
        int32_t left_total = encoder_system.left.total_pulse;
        int32_t right_total = encoder_system.right.total_pulse;
        
        // 显示实时数据
        char display_buffer[64];
        
        // 当前周期脉冲（紧凑显示）
        sprintf(display_buffer, "Now L:%6d R:%6d", (int)left_pulse, (int)right_pulse);
        ips114_show_string(0, 48, display_buffer);
        
        // 累计脉冲总数（带变化指示，支持负数）
        sprintf(display_buffer, "Tot L:%8d%s", (int)left_total, 
                (left_total != last_left_total) ? "*" : " ");
        ips114_show_string(0, 64, display_buffer);
        
        sprintf(display_buffer, "Tot R:%8d%s", (int)right_total,
                (right_total != last_right_total) ? "*" : " ");
        ips114_show_string(0, 80, display_buffer);
        
        // 显示编码器方向和计数（合并显示）
        const char* left_dir = (encoder_system.left.direction == ENCODER_DIR_FORWARD) ? "F" :
                              (encoder_system.left.direction == ENCODER_DIR_BACKWARD) ? "B" : "S";
        const char* right_dir = (encoder_system.right.direction == ENCODER_DIR_FORWARD) ? "F" :
                               (encoder_system.right.direction == ENCODER_DIR_BACKWARD) ? "B" : "S";
        
        sprintf(display_buffer, "Dir L:%s R:%s Cnt:%u", left_dir, right_dir, 
                (unsigned int)update_counter % 1000);
        ips114_show_string(0, 96, display_buffer);
        
        // 每100次更新显示一次统计信息
        if (update_counter % 100 == 0) {
            if (right_total != 0) {
                sprintf(display_buffer, "L/R Ratio: %.2f", (float)left_total / right_total);
            } else {
                sprintf(display_buffer, "L/R Ratio: N/A");
            }
            ips114_show_string(0, 112, display_buffer);
        }
        
        // 保存上次的值用于变化检测
        last_left_total = left_total;
        last_right_total = right_total;
        
        // 更新计数器
        update_counter++;
        
        // 延时100ms，实现10Hz的显示刷新率
        system_delay_ms(100);
        
        // 可以添加退出条件，比如按键检测
        // if (gpio_get_level(EXIT_KEY_PIN) == 0) {
        //     break;
        // }
    }
}

//=================================================内部函数实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示测试信息（内部函数）
//-------------------------------------------------------------------------------------------------------------------
static void display_test_info(const char* info)
{
    // 固定在第3行显示数据，覆盖刷新
    char display_buffer[64];
    sprintf(display_buffer, "%-30s", info); // 左对齐，30个字符宽度，用空格填充
    ips114_show_string(0, 48, display_buffer);  // 固定在Y=48位置显示
}