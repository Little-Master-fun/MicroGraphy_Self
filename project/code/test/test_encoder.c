/*********************************************************************************************************************
* 文件名称          test_encoder.c
* 功能说明          编码器简化测试程序
* 作者              LittleMaster
* 版本信息          v2.0
* 修改记录
* 日期              作者                版本              备注
* 2025-09-17        LittleMaster       1.0v              创建基础编码器测试功能
* 2025-09-22        LittleMaster       2.0v              简化为实时数据显示
* 
* 文件作用说明：
* 本文件为编码器的简化测试程序，只显示关键数据
* 
* 显示内容：
* - 左右编码器速度 (m/s)
* - 左右编码器累计距离 (mm)  
* - 实时脉冲计数变化
* 
* IPS114显示安全区域 (240×135像素，8×16字体)：
* - 标题区域：y=0-16     - "Encoder Test"
* - 表头区域：y=32-48    - 数据格式说明
* - 数据区域：y=64-96    - 实时数据显示
* - 最大安全y坐标：112   - 避免断言错误
* 
* 使用方式：
* 调用test_encoder_simple()函数开始测试
* 手动转动车轮观察数据变化
********************************************************************************************************************/

#include "test_encoder.h"
#include "driver_encoder.h"
#include "zf_device_ips114.h"
#include "zf_common_headfile.h"
#include <stdio.h>

//=================================================简化测试函数实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     编码器简化测试 - 只显示速度、距离、脉冲
//-------------------------------------------------------------------------------------------------------------------
void test_encoder_simple(void)
{
    char buffer[20];      // 缩短缓冲区防止溢出
    uint32 loop_count = 0;
    
    printf("=== 编码器简化测试开始 ===\n");
    
    // 初始化编码器
    encoder_status_enum status = encoder_init();
    if (status != ENCODER_STATUS_OK)
    {
        printf("编码器初始化失败！\n");
        return;
    }
    
    // 初始化显示
    ips114_init();
    ips114_set_color(RGB565_WHITE, RGB565_BLACK);
    ips114_clear();
    
    // 显示标题 - 使用英文简写
    ips114_show_string(5, 0, "Encoder Test");
    ips114_show_string(5, 16, "============");
    
    // 显示表头 - 格式：速度 距离 脉冲
    ips114_show_string(5, 32, "L: Spd Dist Pls");
    ips114_show_string(5, 48, "R: Spd Dist Pls");
    
    printf("编码器初始化成功！请手动转动车轮...\n");
    
    while(1)
    {
        // 更新编码器数据
        //encoder_update();
        
        // 获取数据
        float left_speed = encoder_get_speed(ENCODER_ID_LEFT);
        float right_speed = encoder_get_speed(ENCODER_ID_RIGHT);
        float left_distance = encoder_get_distance(ENCODER_ID_LEFT);
        float right_distance = encoder_get_distance(ENCODER_ID_RIGHT);
        int32 left_pulse = encoder_get_pulse(ENCODER_ID_LEFT);
        int32 right_pulse = encoder_get_pulse(ENCODER_ID_RIGHT);
        
        // 每20次循环更新一次显示（200ms）
        if (loop_count % 20 == 0)
        {
            // 左编码器数据 - 在安全坐标范围内 (y=64 < 112)
            sprintf(buffer, "%.1f %4.0f %3ld", left_speed, left_distance, left_pulse);
            ips114_show_string(5, 64, "              "); // 清除旧数据
            ips114_show_string(5, 64, buffer);
            
            // 右编码器数据 - 在安全坐标范围内 (y=80 < 112)
            sprintf(buffer, "%.1f %4.0f %3ld", right_speed, right_distance, right_pulse);
            ips114_show_string(5, 80, "              "); // 清除旧数据
            ips114_show_string(5, 80, buffer);
            
            // 状态显示 - 在安全坐标范围内 (y=96 < 112)
            sprintf(buffer, "Loop: %lu", loop_count);
            ips114_show_string(5, 96, "            "); // 清除旧数据
            ips114_show_string(5, 96, buffer);
        }
        
        // 每100次循环输出一次到串口（1秒）
        if (loop_count % 100 == 0)
        {
            printf("L: %.2fm/s %.0fmm %ldpls | R: %.2fm/s %.0fmm %ldpls\n", 
                   left_speed, left_distance, left_pulse,
                   right_speed, right_distance, right_pulse);
        }
        
        loop_count++;
        system_delay_ms(10); // 10ms更新周期
    }
}