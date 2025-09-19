/*********************************************************************************************************************
* 文件名称          test_sch16tk10.c
* 功能说明          SCH16TK10 IMU传感器测试程序实现文件
* 作者              LittleMaster 
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本
* 2025-09-17        LittleMaster       1.0v
* 
* 文件作用说明：
* 本文件为SCH16TK10 IMU传感器的测试程序实现，主要用于验证IMU传感器的功能
* 
********************************************************************************************************************/

#include "test_sch16tk10.h"
#include "driver_sch16tk10.h"
#include "zf_device_ips114.h"
#include "zf_common_headfile.h"
#include <stdio.h>

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     SCH16TK10 IMU传感器测试主函数
// 参数说明     void
// 返回参数     void
// 使用示例     test_imu();
// 备注信息     该函数完成IMU传感器和屏幕的初始化，然后进入无限循环读取并显示传感器数据
//              函数不会返回，需要通过复位或断电来停止程序
//-------------------------------------------------------------------------------------------------------------------
void test_imu(void)
{
    // 初始化屏幕
    ips114_set_dir(IPS114_PORTAIT);
    ips114_set_color(RGB565_WHITE, RGB565_BLACK);
    ips114_init();
    ips114_clear();
    ips114_show_string(0, 0, "SCH16TK10 Test...");

    // 初始化IMU
    SCH1_filter sFilter;
    SCH1_sensitivity sSensitivity;
    SCH1_decimation sDecimation;

    // 设置默认参数
    sFilter.Rate12 = 30.0f; 
    sFilter.Acc12 = 30.0f;
    sFilter.Acc3 = 30.0f;

    sSensitivity.Rate1 = 6400.0f;
    sSensitivity.Rate2 = 6400.0f;
    sSensitivity.Acc1 = 25600.0f;
    sSensitivity.Acc2 = 25600.0f;
    sSensitivity.Acc3 = 25600.0f;

    sDecimation.Rate2 = 1;
    sDecimation.Acc2 = 1;

    SCH1_status status;
    int init_result = SCH1_init(sFilter, sSensitivity, sDecimation, false);
    
    if (init_result != SCH1_OK)
    {
        char error_msg[50];
        sprintf(error_msg, "Init Error: %d", init_result);
        ips114_show_string(0, 16, error_msg);
        
        // 读取状态寄存器进行诊断
        SCH1_getStatus(&status);
        sprintf(error_msg, "Summary: 0x%04X", status.Summary);
        ips114_show_string(0, 32, error_msg);
        sprintf(error_msg, "Common: 0x%04X", status.Common);
        ips114_show_string(0, 48, error_msg);
        
        while (1);
    }
    
    if (SCH1_init(sFilter, sSensitivity, sDecimation, false) != SCH1_OK)
    {
       ips114_show_string(0, 16, "IMU Init Failed!");
       while (1);
    }
    else
    {
       ips114_show_string(0, 16, "IMU Init Success!");
    }
    
    system_delay_ms(1000);

    SCH1_raw_data raw_data;
    SCH1_result result_data;
    char text_buffer[50];

    while(1)
    {
        SCH1_getData(&raw_data);
        SCH1_convert_data(&raw_data, &result_data);
        
        ips114_clear();

        sprintf(text_buffer, "AccX: %.2f", result_data.Acc1[AXIS_X]);
        ips114_show_string(0, 0, text_buffer);

        sprintf(text_buffer, "AccY: %.2f", result_data.Acc1[AXIS_Y]);
        ips114_show_string(0, 16, text_buffer);

        sprintf(text_buffer, "AccZ: %.2f", result_data.Acc1[AXIS_Z]);
        ips114_show_string(0, 32, text_buffer);

        sprintf(text_buffer, "GyrX: %.2f", result_data.Rate1[AXIS_X]);
        ips114_show_string(0, 48, text_buffer);

        sprintf(text_buffer, "GyrY: %.2f", result_data.Rate1[AXIS_Y]);
        ips114_show_string(0, 64, text_buffer);

        sprintf(text_buffer, "GyrZ: %.2f", result_data.Rate1[AXIS_Z]);
        ips114_show_string(0, 80, text_buffer);

        sprintf(text_buffer, "Temp: %.2f", result_data.Temp);
        ips114_show_string(0, 96, text_buffer);

        system_delay_ms(100);
    }
}
