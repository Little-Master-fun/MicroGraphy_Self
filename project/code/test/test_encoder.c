/*********************************************************************************************************************
* �ļ�����          test_encoder.c
* ����˵��          ˫�������������Գ���ʵ���ļ�
* ����              LittleMaster
* �汾��Ϣ          v1.0
* �޸ļ�¼
* ����              ����                �汾
* 2025-09-17        LittleMaster       1.0v
* 
* �ļ�����˵����
* ���ļ�Ϊ˫�������������Գ����ʵ�֣�������֤�����������ĸ����
* 
* �������ݣ�
* 1. �������������ܲ��ԣ���ʼ������������������жϣ�
* 2. �������ٶȼ������
* 3. ���������ͳ�Ʋ���
* 4. �ۺϹ��ܲ��ԣ�ʵʱ������ʾ��
* 
* ʹ�÷�ʽ��
* ���������е�����Ӧ�Ĳ��Ժ������ɿ�ʼ����
* 
* ע�����
* 1. ȷ��������Ӳ��������ȷ
* 2. �ֶ�ת�����ֽ��в���
* 3. �۲���Ļ��ʾ�������Ƿ����
********************************************************************************************************************/

#include "test_encoder.h"
#include "driver_encoder.h"
#include "zf_device_ips114.h"
#include "zf_common_headfile.h"
#include <stdio.h>

//=================================================ȫ�ֱ�������================================================
static uint8 line_count = 0;           // ��ʾ�м�����

//=================================================�ڲ���������================================================
static void display_test_info(const char* info);

//=================================================���Ժ���ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     �������������ܲ���
//-------------------------------------------------------------------------------------------------------------------
void test_encoder_basic(void)
{
    ips114_clear();
    ips114_show_string(0, 0, "Encoder Basic Test");
    ips114_show_string(0, 16, "Turn wheels manually");
    
    // ��ʼ��������
    if (encoder_init() != ENCODER_STATUS_OK)
    {
        ips114_show_string(0, 32, "Init Failed!");
        return;
    }
    
    display_test_info("Encoder Init OK");
    
    uint32 test_duration = 0;
    while (test_duration < 10000)  // ����10��
    {
        // ��ȡ����������
        encoder_read_data(ENCODER_ID_BOTH);
        
        // ��ȡ�������
        int32 left_pulse = encoder_get_pulse_count(ENCODER_ID_LEFT);
        int32 right_pulse = encoder_get_pulse_count(ENCODER_ID_RIGHT);
        
        // ��ʾ���
        char info[50];
        sprintf(info, "L:%ld R:%ld", left_pulse, right_pulse);
        display_test_info(info);
        
        system_delay_ms(100);
        test_duration += 100;
    }
    
    display_test_info("Basic Test Complete");
}

//-------------------------------------------------------------------------------------------------------------------
// �������     �������ٶȼ������
//-------------------------------------------------------------------------------------------------------------------
void test_encoder_speed(void)
{
    ips114_clear();
    ips114_show_string(0, 0, "Speed Test");
    ips114_show_string(0, 16, "Rotate wheels fast");
    
    uint32 test_duration = 0;
    while (test_duration < 15000)  // ����15��
    {
        // ���±���������
        encoder_update();
        
        // �����ٶ�
        encoder_calculate_speed(ENCODER_ID_BOTH);
        
        // ��ȡ�ٶ�ֵ
        float left_speed = encoder_get_speed(ENCODER_ID_LEFT);
        float right_speed = encoder_get_speed(ENCODER_ID_RIGHT);
        
        // ��ʾ���
        char info[50];
        sprintf(info, "L:%.3f R:%.3f m/s", left_speed, right_speed);
        display_test_info(info);
        
        system_delay_ms(200);
        test_duration += 200;
    }
    
    display_test_info("Speed Test Complete");
}

//-------------------------------------------------------------------------------------------------------------------
// �������     �������������
//-------------------------------------------------------------------------------------------------------------------
void test_encoder_distance(void)
{
    ips114_clear();
    ips114_show_string(0, 0, "Distance Test");
    ips114_show_string(0, 16, "Move robot forward");
    
    // ���ñ���������
    encoder_reset(ENCODER_ID_BOTH);
    display_test_info("Distance Reset");
    
    uint32 test_duration = 0;
    while (test_duration < 20000)  // ����20��
    {
        // ���±���������
        encoder_update();
        encoder_calculate_speed(ENCODER_ID_BOTH);
        
        // ��ȡ����ֵ
        float left_distance = encoder_get_distance(ENCODER_ID_LEFT);
        float right_distance = encoder_get_distance(ENCODER_ID_RIGHT);
        
        // ��ʾ���
        char info[50];
        sprintf(info, "L:%.1f R:%.1f mm", left_distance, right_distance);
        display_test_info(info);
        
        system_delay_ms(300);
        test_duration += 300;
    }
    
    display_test_info("Distance Test Complete");
}

//-------------------------------------------------------------------------------------------------------------------
// �������     �������ۺϲ���
//-------------------------------------------------------------------------------------------------------------------
void test_encoder(void)
{
    // ��ʼ����Ļ
    ips114_set_dir(IPS114_PORTAIT);
    ips114_set_color(RGB565_WHITE, RGB565_BLACK);
    ips114_init();
    ips114_clear();
    
    ips114_show_string(0, 0, "Encoder Full Test");
    system_delay_ms(2000);
    
    // ���и������
    test_encoder_basic();
    ips114_show_string(0, 112, "Basic Test OK");
    
    system_delay_ms(1000);
    
    test_encoder_speed();
    ips114_show_string(0, 112, "Speed Test OK");
    
    system_delay_ms(1000);
    
    test_encoder_distance();
    ips114_show_string(0, 112, "Distance Test OK");
    
    // ʵʱ���ģʽ
    ips114_clear();
    ips114_show_string(0, 0, "Real-time Monitor");
    ips114_show_string(0, 16, "Press key to exit");
    
    while(1)
    {
        // ��������
        encoder_update();
        encoder_calculate_speed(ENCODER_ID_BOTH);
        
        // ��ȡ��������
        int32 left_pulse = encoder_get_pulse_count(ENCODER_ID_LEFT);
        int32 right_pulse = encoder_get_pulse_count(ENCODER_ID_RIGHT);
        float left_speed = encoder_get_speed(ENCODER_ID_LEFT);
        float right_speed = encoder_get_speed(ENCODER_ID_RIGHT);
        float left_distance = encoder_get_distance(ENCODER_ID_LEFT);
        float right_distance = encoder_get_distance(ENCODER_ID_RIGHT);
        
        float linear_vel, angular_vel;
        encoder_get_robot_velocity(&linear_vel, &angular_vel);
        
        // ��ʾ����
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
        
        // ����˳����������Ը��ݰ�����ʵ�֣�
        // if (exit_condition) break;
    }
}

//=================================================�ڲ�����ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ʾ������Ϣ���ڲ�������
//-------------------------------------------------------------------------------------------------------------------
static void display_test_info(const char* info)
{
    // ��ʾ��Ϣ
    ips114_show_string(0, line_count * 16, info);
    
    // �����м�����������Ļ�߶�ʱ�������¿�ʼ
    line_count++;
    if (line_count > 7)  // IPS114��Ļ��Լ����ʾ8��
    {
        line_count = 0;
        system_delay_ms(2000);  // ��ͣ2�����û�������Ϣ
        ips114_clear();
    }
}