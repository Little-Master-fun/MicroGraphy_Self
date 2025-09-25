/*********************************************************************************************************************
* �ļ�����          test_encoder.c
* ����˵��          �������򻯲��Գ���
* ����              LittleMaster
* �汾��Ϣ          v2.0
* �޸ļ�¼
* ����              ����                �汾              ��ע
* 2025-09-17        LittleMaster       1.0v              �����������������Թ���
* 2025-09-22        LittleMaster       2.0v              ��Ϊʵʱ������ʾ
* 
* �ļ�����˵����
* ���ļ�Ϊ�������ļ򻯲��Գ���ֻ��ʾ�ؼ�����
* 
* ��ʾ���ݣ�
* - ���ұ������ٶ� (m/s)
* - ���ұ������ۼƾ��� (mm)  
* - ʵʱ��������仯
* 
* IPS114��ʾ��ȫ���� (240��135���أ�8��16����)��
* - ��������y=0-16     - "Encoder Test"
* - ��ͷ����y=32-48    - ���ݸ�ʽ˵��
* - ��������y=64-96    - ʵʱ������ʾ
* - ���ȫy���꣺112   - ������Դ���
* 
* ʹ�÷�ʽ��
* ����test_encoder_simple()������ʼ����
* �ֶ�ת�����ֹ۲����ݱ仯
********************************************************************************************************************/

#include "test_encoder.h"
#include "driver_encoder.h"
#include "zf_device_ips114.h"
#include "zf_common_headfile.h"
#include <stdio.h>

//=================================================�򻯲��Ժ���ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     �������򻯲��� - ֻ��ʾ�ٶȡ����롢����
//-------------------------------------------------------------------------------------------------------------------
void test_encoder_simple(void)
{
    char buffer[20];      // ���̻�������ֹ���
    uint32 loop_count = 0;
    
    printf("=== �������򻯲��Կ�ʼ ===\n");
    
    // ��ʼ��������
    encoder_status_enum status = encoder_init();
    if (status != ENCODER_STATUS_OK)
    {
        printf("��������ʼ��ʧ�ܣ�\n");
        return;
    }
    
    // ��ʼ����ʾ
    ips114_init();
    ips114_set_color(RGB565_WHITE, RGB565_BLACK);
    ips114_clear();
    
    // ��ʾ���� - ʹ��Ӣ�ļ�д
    ips114_show_string(5, 0, "Encoder Test");
    ips114_show_string(5, 16, "============");
    
    // ��ʾ��ͷ - ��ʽ���ٶ� ���� ����
    ips114_show_string(5, 32, "L: Spd Dist Pls");
    ips114_show_string(5, 48, "R: Spd Dist Pls");
    
    printf("��������ʼ���ɹ������ֶ�ת������...\n");
    
    while(1)
    {
        // ���±���������
        //encoder_update();
        
        // ��ȡ����
        float left_speed = encoder_get_speed(ENCODER_ID_LEFT);
        float right_speed = encoder_get_speed(ENCODER_ID_RIGHT);
        float left_distance = encoder_get_distance(ENCODER_ID_LEFT);
        float right_distance = encoder_get_distance(ENCODER_ID_RIGHT);
        int32 left_pulse = encoder_get_pulse(ENCODER_ID_LEFT);
        int32 right_pulse = encoder_get_pulse(ENCODER_ID_RIGHT);
        
        // ÿ20��ѭ������һ����ʾ��200ms��
        if (loop_count % 20 == 0)
        {
            // ����������� - �ڰ�ȫ���귶Χ�� (y=64 < 112)
            sprintf(buffer, "%.1f %4.0f %3ld", left_speed, left_distance, left_pulse);
            ips114_show_string(5, 64, "              "); // ���������
            ips114_show_string(5, 64, buffer);
            
            // �ұ��������� - �ڰ�ȫ���귶Χ�� (y=80 < 112)
            sprintf(buffer, "%.1f %4.0f %3ld", right_speed, right_distance, right_pulse);
            ips114_show_string(5, 80, "              "); // ���������
            ips114_show_string(5, 80, buffer);
            
            // ״̬��ʾ - �ڰ�ȫ���귶Χ�� (y=96 < 112)
            sprintf(buffer, "Loop: %lu", loop_count);
            ips114_show_string(5, 96, "            "); // ���������
            ips114_show_string(5, 96, buffer);
        }
        
        // ÿ100��ѭ�����һ�ε����ڣ�1�룩
        if (loop_count % 100 == 0)
        {
            printf("L: %.2fm/s %.0fmm %ldpls | R: %.2fm/s %.0fmm %ldpls\n", 
                   left_speed, left_distance, left_pulse,
                   right_speed, right_distance, right_pulse);
        }
        
        loop_count++;
        system_delay_ms(10); // 10ms��������
    }
}