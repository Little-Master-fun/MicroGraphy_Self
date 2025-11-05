/*********************************************************************************************************************
* 文件名称          test_line_tracking.c
* 功能说明          光电管巡线控制测试实现文件
* 作者              AI Assistant
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本
* 2025-11-05        AI Assistant       1.0v
********************************************************************************************************************/

#include "test_line_tracking.h"
#include "line_tracking_control.h"
#include "driver_photoelectric.h"
#include "driver_motor.h"
#include "zf_common_headfile.h"
#include <stdio.h>

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     测试光电管巡线功能
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void test_line_tracking(void)
{
    printf("\n======== 光电管巡线测试 ========\n");
    
    // 初始化巡线系统
    if (line_track_init() != LINE_TRACK_STATUS_OK) {
        printf("巡线系统初始化失败！\n");
        return;
    }
    
    // 初始化电机
    motor_init();
    
    printf("巡线系统初始化完成\n");
    printf("按任意键启动巡线...\n");
    getchar();
    
    // 启动巡线
    line_track_start();
    
    printf("巡线已启动！按 'q' 键停止\n");
    printf("调试信息每秒输出一次\n\n");
    
    uint32 debug_timer = 0;
    
    // 主循环（实际使用时应在定时器中断中调用）
    while (1) {
        // 更新巡线控制（假设10ms周期）
        line_track_update(0.01f);
        
        // 每100次循环（约1秒）打印调试信息
        if (++debug_timer >= 100) {
            debug_timer = 0;
            line_track_print_debug();
        }
        
        // 模拟10ms延时
        system_delay_ms(10);
        
        // 检查是否按下 'q' 键
        // 注意：这里需要根据实际平台实现非阻塞键盘输入
        // 示例代码仅作演示
    }
    
    // 停止巡线
    line_track_stop();
    printf("巡线已停止\n");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     测试光电管传感器读取（不进行控制）
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void test_line_tracking_sensor_only(void)
{
    photoelectric_data_t sensor_data;
    
    printf("\n======== 光电管传感器测试 ========\n");
    printf("初始化光电管...\n");
    
    photoelectric_init();
    
    printf("初始化完成！开始读取数据\n");
    printf("按 Ctrl+C 停止\n\n");
    
    while (1) {
        // 读取传感器数据
        photoelectric_read(&sensor_data);
        
        // 打印原始值
        printf("Raw: ");
        for (int i = 0; i < PHOTOELECTRIC_CHANNEL_COUNT; i++) {
            printf("%4d ", sensor_data.raw_values[i]);
        }
        printf("\n");
        
        // 打印二值化结果
        printf("Bin: ");
        for (int i = 0; i < PHOTOELECTRIC_CHANNEL_COUNT; i++) {
            printf("%c", (sensor_data.bitmap & (1 << i)) ? '1' : '0');
        }
        printf("\n");
        
        // 打印线位置
        int16 line_pos = photoelectric_get_line_position(&sensor_data);
        uint8 line_width = photoelectric_get_line_width(&sensor_data);
        printf("Line Position: %d, Width: %d\n", line_pos, line_width);
        
        printf("\n");
        
        // 延时
        system_delay_ms(500);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PID参数调试模式
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
void test_line_tracking_pid_tuning(void)
{
    printf("\n======== PID参数调试模式 ========\n");
    
    // 初始化巡线系统
    if (line_track_init() != LINE_TRACK_STATUS_OK) {
        printf("巡线系统初始化失败！\n");
        return;
    }
    
    // 初始化电机
    motor_init();
    
    float kp_line = 2.5f, ki_line = 0.01f, kd_line = 5.0f;
    float kp_turn = 4.0f, ki_turn = 0.005f, kd_turn = 8.0f;
    int16 speed = LINE_TRACK_BASE_SPEED;
    
    printf("当前参数：\n");
    printf("直线PID: Kp=%.2f, Ki=%.3f, Kd=%.2f\n", kp_line, ki_line, kd_line);
    printf("转弯PID: Kp=%.2f, Ki=%.3f, Kd=%.2f\n", kp_turn, ki_turn, kd_turn);
    printf("速度: %d\n\n", speed);
    
    printf("命令列表：\n");
    printf("  1 - 调整直线Kp\n");
    printf("  2 - 调整直线Ki\n");
    printf("  3 - 调整直线Kd\n");
    printf("  4 - 调整转弯Kp\n");
    printf("  5 - 调整转弯Ki\n");
    printf("  6 - 调整转弯Kd\n");
    printf("  7 - 调整速度\n");
    printf("  s - 启动巡线\n");
    printf("  p - 停止巡线\n");
    printf("  d - 打印调试信息\n");
    printf("  q - 退出\n\n");
    
    char cmd;
    while (1) {
        printf("请输入命令: ");
        scanf(" %c", &cmd);
        
        switch (cmd) {
            case '1':
                printf("输入新的直线Kp值: ");
                scanf("%f", &kp_line);
                line_track_set_pid_straight(kp_line, ki_line, kd_line);
                break;
                
            case '2':
                printf("输入新的直线Ki值: ");
                scanf("%f", &ki_line);
                line_track_set_pid_straight(kp_line, ki_line, kd_line);
                break;
                
            case '3':
                printf("输入新的直线Kd值: ");
                scanf("%f", &kd_line);
                line_track_set_pid_straight(kp_line, ki_line, kd_line);
                break;
                
            case '4':
                printf("输入新的转弯Kp值: ");
                scanf("%f", &kp_turn);
                line_track_set_pid_turn(kp_turn, ki_turn, kd_turn);
                break;
                
            case '5':
                printf("输入新的转弯Ki值: ");
                scanf("%f", &ki_turn);
                line_track_set_pid_turn(kp_turn, ki_turn, kd_turn);
                break;
                
            case '6':
                printf("输入新的转弯Kd值: ");
                scanf("%f", &kd_turn);
                line_track_set_pid_turn(kp_turn, ki_turn, kd_turn);
                break;
                
            case '7':
                printf("输入新的速度值 (100-800): ");
                scanf("%hd", &speed);
                line_track_set_speed(speed);
                printf("速度已设置为: %d\n", speed);
                break;
                
            case 's':
                line_track_start();
                printf("巡线已启动\n");
                break;
                
            case 'p':
                line_track_stop();
                printf("巡线已停止\n");
                break;
                
            case 'd':
                line_track_print_debug();
                break;
                
            case 'q':
                printf("退出调试模式\n");
                line_track_stop();
                return;
                
            default:
                printf("无效命令\n");
                break;
        }
    }
}

