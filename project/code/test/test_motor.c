/*********************************************************************************************************************
* 文件名称          test_motor.c
* 功能说明          双电机驱动测试程序实现文件
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本
* 2025-09-17        LittleMaster       1.0v
* 
* 文件作用说明：
* 本文件为双电机驱动测试程序的实现，用于验证电机驱动的各项功能
* 
* 测试内容：
* 1. 电机基本功能测试（初始化、方向控制、停止）
* 2. 电机速度控制测试（不同速度档位）
* 3. 差速运动测试（直行、转弯、原地转向）
* 4. 综合功能测试
* 
* 使用方式：
* 在主函数中调用相应的测试函数即可开始测试
* 
* 注意事项：
* 1. 确保电机硬件连接正确
* 2. 测试前检查电源供应是否充足
* 3. 注意观察电机运行状态，异常时立即停止
********************************************************************************************************************/

#include "test_motor.h"
#include "driver_motor.h"
#include "zf_device_ips114.h"
#include "zf_common_headfile.h"
#include <stdio.h>

//=================================================测试参数定义================================================
#define TEST_SPEED_LOW          (2000)      // 低速测试值
#define TEST_SPEED_MEDIUM       (5000)      // 中速测试值
#define TEST_SPEED_HIGH         (8000)      // 高速测试值
#define TEST_DELAY_SHORT        (1000)      // 短延时 1秒
#define TEST_DELAY_MEDIUM       (2000)      // 中延时 2秒
#define TEST_DELAY_LONG         (3000)      // 长延时 3秒

//=================================================内部函数声明================================================
static void test_display_info(const char* test_name, const char* status);
static void test_init_display(void);

//=================================================外部接口实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机基本功能测试
// 参数说明     void
// 返回参数     void
// 使用示例     test_motor_basic();
// 备注信息     测试电机的基本功能：初始化、正转、反转、停止
//-------------------------------------------------------------------------------------------------------------------
void test_motor_basic(void)
{
    test_init_display();
    test_display_info("Motor Basic Test", "Starting...");
    
    // 初始化电机
    if (motor_init() == MOTOR_STATUS_OK)
    {
        test_display_info("Motor Init", "OK");
    }
    else
    {
        test_display_info("Motor Init", "FAILED");
        return;
    }
    
    system_delay_ms(TEST_DELAY_SHORT);
    
    // 测试左电机正转
    test_display_info("Left Motor", "Forward");
    motor_set_direction(MOTOR_LEFT, MOTOR_FORWARD);
    motor_set_speed(MOTOR_LEFT, TEST_SPEED_MEDIUM);
    system_delay_ms(TEST_DELAY_MEDIUM);
    
    // 测试左电机反转
    test_display_info("Left Motor", "Backward");
    motor_set_direction(MOTOR_LEFT, MOTOR_BACKWARD);
    system_delay_ms(TEST_DELAY_MEDIUM);
    
    // 停止左电机
    test_display_info("Left Motor", "Stop");
    motor_stop(MOTOR_LEFT);
    system_delay_ms(TEST_DELAY_SHORT);
    
    // 测试右电机正转
    test_display_info("Right Motor", "Forward");
    motor_set_direction(MOTOR_RIGHT, MOTOR_FORWARD);
    motor_set_speed(MOTOR_RIGHT, TEST_SPEED_MEDIUM);
    system_delay_ms(TEST_DELAY_MEDIUM);
    
    // 测试右电机反转
    test_display_info("Right Motor", "Backward");
    motor_set_direction(MOTOR_RIGHT, MOTOR_BACKWARD);
    system_delay_ms(TEST_DELAY_MEDIUM);
    
    // 停止右电机
    test_display_info("Right Motor", "Stop");
    motor_stop(MOTOR_RIGHT);
    system_delay_ms(TEST_DELAY_SHORT);
    
    // 测试双电机同步
    test_display_info("Both Motors", "Forward");
    motor_set_speed(MOTOR_BOTH, TEST_SPEED_MEDIUM);
    system_delay_ms(TEST_DELAY_MEDIUM);
    
    test_display_info("Both Motors", "Stop");
    motor_stop(MOTOR_BOTH);
    
    test_display_info("Basic Test", "Complete");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机速度测试
// 参数说明     void
// 返回参数     void
// 使用示例     test_motor_speed();
// 备注信息     测试电机的不同速度档位
//-------------------------------------------------------------------------------------------------------------------
void test_motor_speed(void)
{
    test_init_display();
    test_display_info("Motor Speed Test", "Starting...");
    
    // 初始化电机
    motor_init();
    system_delay_ms(TEST_DELAY_SHORT);
    
    // 低速测试
    test_display_info("Speed Test", "Low Speed");
    motor_set_speed(MOTOR_BOTH, TEST_SPEED_LOW);
    system_delay_ms(TEST_DELAY_MEDIUM);
    
    // 中速测试
    test_display_info("Speed Test", "Medium Speed");
    motor_set_speed(MOTOR_BOTH, TEST_SPEED_MEDIUM);
    system_delay_ms(TEST_DELAY_MEDIUM);
    
    // 高速测试
    test_display_info("Speed Test", "High Speed");
    motor_set_speed(MOTOR_BOTH, TEST_SPEED_HIGH);
    system_delay_ms(TEST_DELAY_MEDIUM);
    
    // 反向低速测试
    test_display_info("Speed Test", "Reverse Low");
    motor_set_speed(MOTOR_BOTH, -TEST_SPEED_LOW);
    system_delay_ms(TEST_DELAY_MEDIUM);
    
    // 反向高速测试
    test_display_info("Speed Test", "Reverse High");
    motor_set_speed(MOTOR_BOTH, -TEST_SPEED_HIGH);
    system_delay_ms(TEST_DELAY_MEDIUM);
    
    // 停止
    test_display_info("Speed Test", "Stop");
    motor_stop(MOTOR_BOTH);
    
    test_display_info("Speed Test", "Complete");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     差速运动测试
// 参数说明     void
// 返回参数     void
// 使用示例     test_motor_differential();
// 备注信息     测试差速运动功能：直行、转弯、原地转向
//-------------------------------------------------------------------------------------------------------------------
void test_motor_differential(void)
{
    test_init_display();
    test_display_info("Differential Test", "Starting...");
    
    // 初始化电机
    motor_init();
    system_delay_ms(TEST_DELAY_SHORT);
    
    // 直行测试
    test_display_info("Differential", "Forward");
    motor_differential_drive(TEST_SPEED_MEDIUM, 0);
    system_delay_ms(TEST_DELAY_MEDIUM);
    
    // 后退测试
    test_display_info("Differential", "Backward");
    motor_differential_drive(-TEST_SPEED_MEDIUM, 0);
    system_delay_ms(TEST_DELAY_MEDIUM);
    
    // 左转测试
    test_display_info("Differential", "Turn Left");
    motor_differential_drive(TEST_SPEED_LOW, TEST_SPEED_LOW);
    system_delay_ms(TEST_DELAY_MEDIUM);
    
    // 右转测试
    test_display_info("Differential", "Turn Right");
    motor_differential_drive(TEST_SPEED_LOW, -TEST_SPEED_LOW);
    system_delay_ms(TEST_DELAY_MEDIUM);
    
    // 原地左转
    test_display_info("Differential", "Spin Left");
    motor_differential_drive(0, TEST_SPEED_MEDIUM);
    system_delay_ms(TEST_DELAY_MEDIUM);
    
    // 原地右转
    test_display_info("Differential", "Spin Right");
    motor_differential_drive(0, -TEST_SPEED_MEDIUM);
    system_delay_ms(TEST_DELAY_MEDIUM);
    
    // 停止
    test_display_info("Differential", "Stop");
    motor_stop(MOTOR_BOTH);
    
    test_display_info("Differential", "Complete");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机综合测试
// 参数说明     void
// 返回参数     void
// 使用示例     test_motor();
// 备注信息     综合测试所有电机功能，包括基本功能、速度控制和差速运动
//-------------------------------------------------------------------------------------------------------------------
void test_motor(void)
{
    test_init_display();
    test_display_info("Motor Full Test", "Starting...");
    
    // 基本功能测试
    test_display_info("Phase 1", "Basic Test");
    test_motor_basic();
    system_delay_ms(TEST_DELAY_LONG);
    
    // 速度控制测试
    test_display_info("Phase 2", "Speed Test");
    test_motor_speed();
    system_delay_ms(TEST_DELAY_LONG);
    
    // 差速运动测试
    test_display_info("Phase 3", "Diff Test");
    test_motor_differential();
    system_delay_ms(TEST_DELAY_LONG);
    
    test_display_info("Full Test", "Complete!");
    
    // 测试完成后保持显示
    while(1)
    {
        system_delay_ms(1000);
    }
}

//=================================================内部函数实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     测试信息显示（内部函数）
// 参数说明     test_name           测试项目名称
// 参数说明     status              测试状态
// 返回参数     void
// 备注信息     在屏幕上显示测试信息
//-------------------------------------------------------------------------------------------------------------------
static void test_display_info(const char* test_name, const char* status)
{
    char display_buffer[50];
    static uint8 line_count = 0;
    
    // 格式化显示字符串
    sprintf(display_buffer, "%s: %s", test_name, status);
    
    // 显示在屏幕上
    ips114_show_string(0, line_count * 16, display_buffer);
    
    // 更新行计数，超过屏幕高度时清屏重新开始
    line_count++;
    if (line_count > 7)  // IPS114屏幕大约可显示8行
    {
        line_count = 0;
        system_delay_ms(2000);  // 暂停2秒让用户看清信息
        ips114_clear();
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     测试显示初始化（内部函数）
// 参数说明     void
// 返回参数     void
// 备注信息     初始化测试显示屏幕
//-------------------------------------------------------------------------------------------------------------------
static void test_init_display(void)
{
    // 初始化屏幕
    ips114_set_dir(IPS114_PORTAIT);
    ips114_set_color(RGB565_WHITE, RGB565_BLACK);
    ips114_init();
    ips114_clear();
}
