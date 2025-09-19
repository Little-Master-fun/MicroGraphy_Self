/*********************************************************************************************************************
* �ļ�����          test_motor.c
* ����˵��          ˫����������Գ���ʵ���ļ�
* ����              LittleMaster
* �汾��Ϣ          v1.0
* �޸ļ�¼
* ����              ����                �汾
* 2025-09-17        LittleMaster       1.0v
* 
* �ļ�����˵����
* ���ļ�Ϊ˫����������Գ����ʵ�֣�������֤��������ĸ����
* 
* �������ݣ�
* 1. ����������ܲ��ԣ���ʼ����������ơ�ֹͣ��
* 2. ����ٶȿ��Ʋ��ԣ���ͬ�ٶȵ�λ��
* 3. �����˶����ԣ�ֱ�С�ת�䡢ԭ��ת��
* 4. �ۺϹ��ܲ���
* 
* ʹ�÷�ʽ��
* ���������е�����Ӧ�Ĳ��Ժ������ɿ�ʼ����
* 
* ע�����
* 1. ȷ�����Ӳ��������ȷ
* 2. ����ǰ����Դ��Ӧ�Ƿ����
* 3. ע��۲�������״̬���쳣ʱ����ֹͣ
********************************************************************************************************************/

#include "test_motor.h"
#include "driver_motor.h"
#include "zf_device_ips114.h"
#include "zf_common_headfile.h"
#include <stdio.h>

//=================================================���Բ�������================================================
#define TEST_SPEED_LOW          (2000)      // ���ٲ���ֵ
#define TEST_SPEED_MEDIUM       (5000)      // ���ٲ���ֵ
#define TEST_SPEED_HIGH         (8000)      // ���ٲ���ֵ
#define TEST_DELAY_SHORT        (1000)      // ����ʱ 1��
#define TEST_DELAY_MEDIUM       (2000)      // ����ʱ 2��
#define TEST_DELAY_LONG         (3000)      // ����ʱ 3��

//=================================================�ڲ���������================================================
static void test_display_info(const char* test_name, const char* status);
static void test_init_display(void);

//=================================================�ⲿ�ӿ�ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ����������ܲ���
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     test_motor_basic();
// ��ע��Ϣ     ���Ե���Ļ������ܣ���ʼ������ת����ת��ֹͣ
//-------------------------------------------------------------------------------------------------------------------
void test_motor_basic(void)
{
    test_init_display();
    test_display_info("Motor Basic Test", "Starting...");
    
    // ��ʼ�����
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
    
    // ����������ת
    test_display_info("Left Motor", "Forward");
    motor_set_direction(MOTOR_LEFT, MOTOR_FORWARD);
    motor_set_speed(MOTOR_LEFT, TEST_SPEED_MEDIUM);
    system_delay_ms(TEST_DELAY_MEDIUM);
    
    // ����������ת
    test_display_info("Left Motor", "Backward");
    motor_set_direction(MOTOR_LEFT, MOTOR_BACKWARD);
    system_delay_ms(TEST_DELAY_MEDIUM);
    
    // ֹͣ����
    test_display_info("Left Motor", "Stop");
    motor_stop(MOTOR_LEFT);
    system_delay_ms(TEST_DELAY_SHORT);
    
    // �����ҵ����ת
    test_display_info("Right Motor", "Forward");
    motor_set_direction(MOTOR_RIGHT, MOTOR_FORWARD);
    motor_set_speed(MOTOR_RIGHT, TEST_SPEED_MEDIUM);
    system_delay_ms(TEST_DELAY_MEDIUM);
    
    // �����ҵ����ת
    test_display_info("Right Motor", "Backward");
    motor_set_direction(MOTOR_RIGHT, MOTOR_BACKWARD);
    system_delay_ms(TEST_DELAY_MEDIUM);
    
    // ֹͣ�ҵ��
    test_display_info("Right Motor", "Stop");
    motor_stop(MOTOR_RIGHT);
    system_delay_ms(TEST_DELAY_SHORT);
    
    // ����˫���ͬ��
    test_display_info("Both Motors", "Forward");
    motor_set_speed(MOTOR_BOTH, TEST_SPEED_MEDIUM);
    system_delay_ms(TEST_DELAY_MEDIUM);
    
    test_display_info("Both Motors", "Stop");
    motor_stop(MOTOR_BOTH);
    
    test_display_info("Basic Test", "Complete");
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����ٶȲ���
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     test_motor_speed();
// ��ע��Ϣ     ���Ե���Ĳ�ͬ�ٶȵ�λ
//-------------------------------------------------------------------------------------------------------------------
void test_motor_speed(void)
{
    test_init_display();
    test_display_info("Motor Speed Test", "Starting...");
    
    // ��ʼ�����
    motor_init();
    system_delay_ms(TEST_DELAY_SHORT);
    
    // ���ٲ���
    test_display_info("Speed Test", "Low Speed");
    motor_set_speed(MOTOR_BOTH, TEST_SPEED_LOW);
    system_delay_ms(TEST_DELAY_MEDIUM);
    
    // ���ٲ���
    test_display_info("Speed Test", "Medium Speed");
    motor_set_speed(MOTOR_BOTH, TEST_SPEED_MEDIUM);
    system_delay_ms(TEST_DELAY_MEDIUM);
    
    // ���ٲ���
    test_display_info("Speed Test", "High Speed");
    motor_set_speed(MOTOR_BOTH, TEST_SPEED_HIGH);
    system_delay_ms(TEST_DELAY_MEDIUM);
    
    // ������ٲ���
    test_display_info("Speed Test", "Reverse Low");
    motor_set_speed(MOTOR_BOTH, -TEST_SPEED_LOW);
    system_delay_ms(TEST_DELAY_MEDIUM);
    
    // ������ٲ���
    test_display_info("Speed Test", "Reverse High");
    motor_set_speed(MOTOR_BOTH, -TEST_SPEED_HIGH);
    system_delay_ms(TEST_DELAY_MEDIUM);
    
    // ֹͣ
    test_display_info("Speed Test", "Stop");
    motor_stop(MOTOR_BOTH);
    
    test_display_info("Speed Test", "Complete");
}

//-------------------------------------------------------------------------------------------------------------------
// �������     �����˶�����
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     test_motor_differential();
// ��ע��Ϣ     ���Բ����˶����ܣ�ֱ�С�ת�䡢ԭ��ת��
//-------------------------------------------------------------------------------------------------------------------
void test_motor_differential(void)
{
    test_init_display();
    test_display_info("Differential Test", "Starting...");
    
    // ��ʼ�����
    motor_init();
    system_delay_ms(TEST_DELAY_SHORT);
    
    // ֱ�в���
    test_display_info("Differential", "Forward");
    motor_differential_drive(TEST_SPEED_MEDIUM, 0);
    system_delay_ms(TEST_DELAY_MEDIUM);
    
    // ���˲���
    test_display_info("Differential", "Backward");
    motor_differential_drive(-TEST_SPEED_MEDIUM, 0);
    system_delay_ms(TEST_DELAY_MEDIUM);
    
    // ��ת����
    test_display_info("Differential", "Turn Left");
    motor_differential_drive(TEST_SPEED_LOW, TEST_SPEED_LOW);
    system_delay_ms(TEST_DELAY_MEDIUM);
    
    // ��ת����
    test_display_info("Differential", "Turn Right");
    motor_differential_drive(TEST_SPEED_LOW, -TEST_SPEED_LOW);
    system_delay_ms(TEST_DELAY_MEDIUM);
    
    // ԭ����ת
    test_display_info("Differential", "Spin Left");
    motor_differential_drive(0, TEST_SPEED_MEDIUM);
    system_delay_ms(TEST_DELAY_MEDIUM);
    
    // ԭ����ת
    test_display_info("Differential", "Spin Right");
    motor_differential_drive(0, -TEST_SPEED_MEDIUM);
    system_delay_ms(TEST_DELAY_MEDIUM);
    
    // ֹͣ
    test_display_info("Differential", "Stop");
    motor_stop(MOTOR_BOTH);
    
    test_display_info("Differential", "Complete");
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����ۺϲ���
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     test_motor();
// ��ע��Ϣ     �ۺϲ������е�����ܣ������������ܡ��ٶȿ��ƺͲ����˶�
//-------------------------------------------------------------------------------------------------------------------
void test_motor(void)
{
    test_init_display();
    test_display_info("Motor Full Test", "Starting...");
    
    // �������ܲ���
    test_display_info("Phase 1", "Basic Test");
    test_motor_basic();
    system_delay_ms(TEST_DELAY_LONG);
    
    // �ٶȿ��Ʋ���
    test_display_info("Phase 2", "Speed Test");
    test_motor_speed();
    system_delay_ms(TEST_DELAY_LONG);
    
    // �����˶�����
    test_display_info("Phase 3", "Diff Test");
    test_motor_differential();
    system_delay_ms(TEST_DELAY_LONG);
    
    test_display_info("Full Test", "Complete!");
    
    // ������ɺ󱣳���ʾ
    while(1)
    {
        system_delay_ms(1000);
    }
}

//=================================================�ڲ�����ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ������Ϣ��ʾ���ڲ�������
// ����˵��     test_name           ������Ŀ����
// ����˵��     status              ����״̬
// ���ز���     void
// ��ע��Ϣ     ����Ļ����ʾ������Ϣ
//-------------------------------------------------------------------------------------------------------------------
static void test_display_info(const char* test_name, const char* status)
{
    char display_buffer[50];
    static uint8 line_count = 0;
    
    // ��ʽ����ʾ�ַ���
    sprintf(display_buffer, "%s: %s", test_name, status);
    
    // ��ʾ����Ļ��
    ips114_show_string(0, line_count * 16, display_buffer);
    
    // �����м�����������Ļ�߶�ʱ�������¿�ʼ
    line_count++;
    if (line_count > 7)  // IPS114��Ļ��Լ����ʾ8��
    {
        line_count = 0;
        system_delay_ms(2000);  // ��ͣ2�����û�������Ϣ
        ips114_clear();
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ������ʾ��ʼ�����ڲ�������
// ����˵��     void
// ���ز���     void
// ��ע��Ϣ     ��ʼ��������ʾ��Ļ
//-------------------------------------------------------------------------------------------------------------------
static void test_init_display(void)
{
    // ��ʼ����Ļ
    ips114_set_dir(IPS114_PORTAIT);
    ips114_set_color(RGB565_WHITE, RGB565_BLACK);
    ips114_init();
    ips114_clear();
}
