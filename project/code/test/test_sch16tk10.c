/*********************************************************************************************************************
* �ļ�����          test_sch16tk10.c
* ����˵��          SCH16TK10 IMU���������Գ���ʵ���ļ�
* ����              LittleMaster 
* �汾��Ϣ          v1.0
* �޸ļ�¼
* ����              ����                �汾
* 2025-09-17        LittleMaster       1.0v
* 
* �ļ�����˵����
* ���ļ�ΪSCH16TK10 IMU�������Ĳ��Գ���ʵ�֣���Ҫ������֤IMU�������Ĺ���
* 
********************************************************************************************************************/

#include "test_sch16tk10.h"
#include "driver_sch16tk10.h"
#include "zf_device_ips114.h"
#include "zf_common_headfile.h"
#include <stdio.h>

//-------------------------------------------------------------------------------------------------------------------
// �������     SCH16TK10 IMU����������������
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     test_imu();
// ��ע��Ϣ     �ú������IMU����������Ļ�ĳ�ʼ����Ȼ���������ѭ����ȡ����ʾ����������
//              �������᷵�أ���Ҫͨ����λ��ϵ���ֹͣ����
//-------------------------------------------------------------------------------------------------------------------
void test_imu(void)
{
    // ��ʼ����Ļ
    ips114_set_dir(IPS114_PORTAIT);
    ips114_set_color(RGB565_WHITE, RGB565_BLACK);
    ips114_init();
    ips114_clear();
    ips114_show_string(0, 0, "SCH16TK10 Test...");

    // ��ʼ��IMU
    SCH1_filter sFilter;
    SCH1_sensitivity sSensitivity;
    SCH1_decimation sDecimation;

    // ����Ĭ�ϲ���
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
        
        // ��ȡ״̬�Ĵ����������
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
