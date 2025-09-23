/*********************************************************************************************************************
* 文件名称          test_motor.c
* 功能说明          电机驱动测试程序（简化版）
* 作者              LittleMaster
* 版本信息          v3.0
* 修改记录
* 日期              作者                版本              备注
* 2025-09-17        LittleMaster       1.0v              创建基础电机测试功能
* 2025-09-22        LittleMaster       2.0v              简化为按键控制测试系统
* 2025-09-22        LittleMaster       3.0v              修复IPS114显示断言错误
* 
* 文件作用说明：
* 本文件为电机驱动的简化测试程序，提供按键控制的电机测试功能
* 
* 测试功能：
* - KEY1(P00_3): 左右电机以1m/s速度转动
* - KEY2(P00_2): 左右电机以3.5m/s速度转动  
* - KEY3(P01_0): 以1m/s前进一米后停下
* - KEY4(P01_1): 以3.5m/s前进一米后停下
* 
* IPS114显示安全区域 (240×135像素，8×16字体)：
* - 标题区域：y=0-16    - "Motor Test"
* - 菜单区域：y=32-80   - 按键说明  
* - 状态区域：y=96-112  - 实时状态显示
* - 最大安全y坐标：119 (135-16=119，避免断言错误)
* 
* 使用的优化驱动：
* - driver_motor.c (TB67H420FTG驱动器)
* - driver_encoder.c (简化版) 
* - motor_control.c (简化版PID控制)
********************************************************************************************************************/

#include "test_motor.h"
#include "driver_motor.h"
#include "driver_encoder.h" 
#include "motor_control.h"
#include "zf_device_ips114.h"
#include "zf_common_headfile.h"
#include <stdio.h>
#include <math.h>

//=================================================按键定义================================================
#define KEY1                    (P00_3)
#define KEY2                    (P00_2)
#define KEY3                    (P01_0)
#define KEY4                    (P01_1)

//=================================================测试参数================================================
#define TEST_SPEED_LOW          (1.0f)      // 低速：1 m/s
#define TEST_SPEED_HIGH         (3.5f)      // 高速：3.5 m/s
#define TEST_DISTANCE           (1000.0f)   // 测试距离：1米 (1000mm)

//=================================================内部函数声明================================================
static void test_init_keys(void);
static void test_display_menu(void);
static void test_display_status(const char* action, float speed);
static uint8 test_read_keys(void);
static void test_move_distance(float speed, float distance);

//=================================================外部接口实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机按键控制测试
// 参数说明     void
// 返回参数     void
// 使用示例     test_motor_key_control();
// 备注信息     简化的按键控制测试，支持不同速度和距离控制
//-------------------------------------------------------------------------------------------------------------------
void test_motor_key_control(void)
{
    uint8 key_pressed = 0;
    
    // 初始化硬件
    printf("正在初始化硬件...\n");
    
    // 初始化按键
    test_init_keys();
    
    // 初始化显示屏
    ips114_init();
    ips114_clear();
    
    // 初始化电机驱动
    if (motor_init() != MOTOR_STATUS_OK) {
        printf("错误：电机初始化失败\n");
        return;
    }
    
    // 初始化编码器
    if (encoder_init() != ENCODER_STATUS_OK) {
        printf("错误：编码器初始化失败\n");
        return;
    }
    
    // 初始化PID控制系统
    if (motor_pid_init() != MOTOR_PID_OK) {
        printf("错误：PID控制系统初始化失败\n");
        return;
    }
    
    printf("硬件初始化完成\n");
    
    // 显示主菜单
    test_display_menu();
    
    // 主控制循环
    while(1)
    {
        // 读取按键
        key_pressed = test_read_keys();
        
        switch(key_pressed)
        {
            case 1:  // KEY1 - 1m/s连续转动
                test_display_status("Move", TEST_SPEED_LOW);
                motor_set_target_speed(TEST_SPEED_LOW, TEST_SPEED_LOW);
                break;
                
            case 2:  // KEY2 - 3.5m/s连续转动
                test_display_status("Move", TEST_SPEED_HIGH);
                motor_set_target_speed(TEST_SPEED_HIGH, TEST_SPEED_HIGH);
                break;
                
            case 3:  // KEY3 - 1m/s前进一米
                test_display_status("Move1m", TEST_SPEED_LOW);
                test_move_distance(1.0f, TEST_DISTANCE);
                break;
                
            case 4:  // KEY4 - 3.5m/s前进一米
                test_display_status("Move1m", TEST_SPEED_HIGH);
                test_move_distance(TEST_SPEED_HIGH, TEST_DISTANCE);
                break;
                
            default:
                // 无按键按下时停止电机
                motor_set_target_speed(0.0f, 0.0f);
                break;
        }
        
        system_delay_ms(50);  // 50ms控制周期
    }
}

//=================================================内部函数实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化按键GPIO
//-------------------------------------------------------------------------------------------------------------------
static void test_init_keys(void)
{
    // 初始化按键为输入模式，上拉电阻
    gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示主菜单
//-------------------------------------------------------------------------------------------------------------------
static void test_display_menu(void)
{
    ips114_clear();
    
    // 设置显示颜色为白色 (黑色背景白色字)
    ips114_set_color(RGB565_WHITE, RGB565_BLACK);
    
    // 标题 - 使用英文
    ips114_show_string(5, 0, "Motor Test");
    ips114_show_string(5, 16, "==========");
    
    // 按键说明 - 英文简写，坐标在安全范围内
    ips114_show_string(5, 32, "KEY1: 1.0m/s Cont");
    ips114_show_string(5, 48, "KEY2: 3.5m/s Cont");
    ips114_show_string(5, 64, "KEY3: 1.0m/s 1m");
    ips114_show_string(5, 80, "KEY4: 3.5m/s 1m");
    
    // 状态区域 - 在安全坐标范围内 (y < 135)
    ips114_show_string(5, 96, "Status: Wait Key");
    ips114_show_string(5, 112, "Speed: 0.0 m/s");
    
    printf("Menu display complete\n");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示当前状态
//-------------------------------------------------------------------------------------------------------------------
static void test_display_status(const char* action, float speed)
{
    char status_str[20];
    char speed_str[20];
    
    // 设置显示颜色 (黑色背景白色字)
    ips114_set_color(RGB565_WHITE, RGB565_BLACK);
    
    // 清空状态区域 - 使用安全坐标
    ips114_show_string(5, 96, "               ");
    ips114_show_string(5, 112, "               ");
    
    // 显示新状态 - 英文简写
    sprintf(status_str, "St: %s", action);
    sprintf(speed_str, "Spd: %.1f m/s", speed);
    
    ips114_show_string(5, 96, status_str);
    ips114_show_string(5, 112, speed_str);
    
    printf("Status: %s - Speed: %.1f m/s\n", action, speed);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取按键状态
// 返回参数     uint8   按键编号(1-4)，0表示无按键
//-------------------------------------------------------------------------------------------------------------------
static uint8 test_read_keys(void)
{
    static uint8 key_last_state[4] = {1, 1, 1, 1};  // 上次按键状态
    uint8 key_current_state[4];
    uint8 i;
    
    // 读取当前按键状态（低电平有效）
    key_current_state[0] = gpio_get_level(KEY1);
    key_current_state[1] = gpio_get_level(KEY2);
    key_current_state[2] = gpio_get_level(KEY3);
    key_current_state[3] = gpio_get_level(KEY4);
    
    // 检测按键按下（下降沿触发）
    for(i = 0; i < 4; i++)
    {
        if(key_last_state[i] == 1 && key_current_state[i] == 0)
        {
            // 防抖延时
            system_delay_ms(20);
            
            // 再次确认按键状态
            switch(i)
            {
                case 0: if(gpio_get_level(KEY1) == 0) { key_last_state[i] = 0; return 1; } break;
                case 1: if(gpio_get_level(KEY2) == 0) { key_last_state[i] = 0; return 2; } break;
                case 2: if(gpio_get_level(KEY3) == 0) { key_last_state[i] = 0; return 3; } break;
                case 3: if(gpio_get_level(KEY4) == 0) { key_last_state[i] = 0; return 4; } break;
            }
        }
        key_last_state[i] = key_current_state[i];
    }
    
    return 0;  // 无按键按下
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按指定速度前进指定距离
// 参数说明     speed       目标速度 (m/s)
// 参数说明     distance    目标距离 (mm)
//-------------------------------------------------------------------------------------------------------------------
static void test_move_distance(float speed, float distance)
{
    float left_distance, right_distance;
    float avg_distance;
    char distance_str[32];
    
    // 重置编码器距离
    encoder_reset(ENCODER_ID_BOTH);
    
    printf("开始前进 %.1f米，速度 %.1f m/s\n", distance/1000.0f, speed);
    
    // 开始移动
    motor_set_target_speed(speed, speed);
    
    // 监控距离直到达到目标
    while(1)
    {
        // 获取当前行驶距离
        left_distance = encoder_get_distance(ENCODER_ID_LEFT);
        right_distance = encoder_get_distance(ENCODER_ID_RIGHT);
        avg_distance = (fabs(left_distance) + fabs(right_distance)) / 2.0f;
        
        // 显示当前距离 - 使用安全坐标和英文 (y=119安全范围)
        sprintf(distance_str, "D:%.0f/%.0fmm", avg_distance, distance);
        ips114_show_string(5, 112, "               "); // 重用speed显示行
        ips114_show_string(5, 112, distance_str);
        
        // 检查是否达到目标距离
        if(avg_distance >= distance)
        {
            break;
        }
        
        system_delay_ms(50);
    }
    
    // 停止电机
    motor_set_target_speed(0.0f, 0.0f);
    
    // 显示完成状态 - 英文
    ips114_show_string(5, 96, "St: Done!     ");
    ips114_show_string(5, 112, "Spd: 0.0 m/s  ");
    
    printf("前进完成！实际距离: %.1fmm\n", avg_distance);
    
    // 等待一段时间显示结果
    system_delay_ms(2000);
    
    // 清除距离显示 - 恢复speed行
    ips114_show_string(5, 112, "Spd: 0.0 m/s  ");
    
    // 恢复菜单显示 - 英文
    ips114_show_string(5, 96, "Status: Wait Key");
}
