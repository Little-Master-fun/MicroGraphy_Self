/*********************************************************************************************************************
* �ļ�����          test_motor.c
* ����˵��          ����������Գ��򣨼򻯰棩
* ����              LittleMaster
* �汾��Ϣ          v3.0
* �޸ļ�¼
* ����              ����                �汾              ��ע
* 2025-09-17        LittleMaster       1.0v              ��������������Թ���
* 2025-09-22        LittleMaster       2.0v              ��Ϊ�������Ʋ���ϵͳ
* 2025-09-22        LittleMaster       3.0v              �޸�IPS114��ʾ���Դ���
* 
* �ļ�����˵����
* ���ļ�Ϊ��������ļ򻯲��Գ����ṩ�������Ƶĵ�����Թ���
* 
* ���Թ��ܣ�
* - KEY1(P00_3): ���ҵ����1m/s�ٶ�ת��
* - KEY2(P00_2): ���ҵ����3.5m/s�ٶ�ת��  
* - KEY3(P01_0): ��1m/sǰ��һ�׺�ͣ��
* - KEY4(P01_1): ��3.5m/sǰ��һ�׺�ͣ��
* 
* IPS114��ʾ��ȫ���� (240��135���أ�8��16����)��
* - ��������y=0-16    - "Motor Test"
* - �˵�����y=32-80   - ����˵��  
* - ״̬����y=96-112  - ʵʱ״̬��ʾ
* - ���ȫy���꣺119 (135-16=119��������Դ���)
* 
* ʹ�õ��Ż�������
* - driver_motor.c (TB67H420FTG������)
* - driver_encoder.c (�򻯰�) 
* - motor_control.c (�򻯰�PID����)
********************************************************************************************************************/

#include "test_motor.h"
#include "driver_motor.h"
#include "driver_encoder.h" 
#include "motor_control.h"
#include "zf_device_ips114.h"
#include "zf_common_headfile.h"
#include <stdio.h>
#include <math.h>

//=================================================��������================================================
#define KEY1                    (P00_3)
#define KEY2                    (P00_2)
#define KEY3                    (P01_0)
#define KEY4                    (P01_1)

//=================================================���Բ���================================================
#define TEST_SPEED_LOW          (1.0f)      // ���٣�1 m/s
#define TEST_SPEED_HIGH         (3.5f)      // ���٣�3.5 m/s
#define TEST_DISTANCE           (1000.0f)   // ���Ծ��룺1�� (1000mm)

//=================================================�ڲ���������================================================
static void test_init_keys(void);
static void test_display_menu(void);
static void test_display_status(const char* action, float speed);
static uint8 test_read_keys(void);
static void test_move_distance(float speed, float distance);

//=================================================�ⲿ�ӿ�ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ����������Ʋ���
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     test_motor_key_control();
// ��ע��Ϣ     �򻯵İ������Ʋ��ԣ�֧�ֲ�ͬ�ٶȺ;������
//-------------------------------------------------------------------------------------------------------------------
void test_motor_key_control(void)
{
    uint8 key_pressed = 0;
    
    // ��ʼ��Ӳ��
    printf("���ڳ�ʼ��Ӳ��...\n");
    
    // ��ʼ������
    test_init_keys();
    
    // ��ʼ����ʾ��
    ips114_init();
    ips114_clear();
    
    // ��ʼ���������
    if (motor_init() != MOTOR_STATUS_OK) {
        printf("���󣺵����ʼ��ʧ��\n");
        return;
    }
    
    // ��ʼ��������
    if (encoder_init() != ENCODER_STATUS_OK) {
        printf("���󣺱�������ʼ��ʧ��\n");
        return;
    }
    
    // ��ʼ��PID����ϵͳ
    if (motor_pid_init() != MOTOR_PID_OK) {
        printf("����PID����ϵͳ��ʼ��ʧ��\n");
        return;
    }
    
    printf("Ӳ����ʼ�����\n");
    
    // ��ʾ���˵�
    test_display_menu();
    
    // ������ѭ��
    while(1)
    {
        // ��ȡ����
        key_pressed = test_read_keys();
        
        switch(key_pressed)
        {
            case 1:  // KEY1 - 1m/s����ת��
                test_display_status("Move", TEST_SPEED_LOW);
                motor_set_target_speed(TEST_SPEED_LOW, TEST_SPEED_LOW);
                break;
                
            case 2:  // KEY2 - 3.5m/s����ת��
                test_display_status("Move", TEST_SPEED_HIGH);
                motor_set_target_speed(TEST_SPEED_HIGH, TEST_SPEED_HIGH);
                break;
                
            case 3:  // KEY3 - 1m/sǰ��һ��
                test_display_status("Move1m", TEST_SPEED_LOW);
                test_move_distance(1.0f, TEST_DISTANCE);
                break;
                
            case 4:  // KEY4 - 3.5m/sǰ��һ��
                test_display_status("Move1m", TEST_SPEED_HIGH);
                test_move_distance(TEST_SPEED_HIGH, TEST_DISTANCE);
                break;
                
            default:
                // �ް�������ʱֹͣ���
                motor_set_target_speed(0.0f, 0.0f);
                break;
        }
        
        system_delay_ms(50);  // 50ms��������
    }
}

//=================================================�ڲ�����ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ʼ������GPIO
//-------------------------------------------------------------------------------------------------------------------
static void test_init_keys(void)
{
    // ��ʼ������Ϊ����ģʽ����������
    gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ʾ���˵�
//-------------------------------------------------------------------------------------------------------------------
static void test_display_menu(void)
{
    ips114_clear();
    
    // ������ʾ��ɫΪ��ɫ (��ɫ������ɫ��)
    ips114_set_color(RGB565_WHITE, RGB565_BLACK);
    
    // ���� - ʹ��Ӣ��
    ips114_show_string(5, 0, "Motor Test");
    ips114_show_string(5, 16, "==========");
    
    // ����˵�� - Ӣ�ļ�д�������ڰ�ȫ��Χ��
    ips114_show_string(5, 32, "KEY1: 1.0m/s Cont");
    ips114_show_string(5, 48, "KEY2: 3.5m/s Cont");
    ips114_show_string(5, 64, "KEY3: 1.0m/s 1m");
    ips114_show_string(5, 80, "KEY4: 3.5m/s 1m");
    
    // ״̬���� - �ڰ�ȫ���귶Χ�� (y < 135)
    ips114_show_string(5, 96, "Status: Wait Key");
    ips114_show_string(5, 112, "Speed: 0.0 m/s");
    
    printf("Menu display complete\n");
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ʾ��ǰ״̬
//-------------------------------------------------------------------------------------------------------------------
static void test_display_status(const char* action, float speed)
{
    char status_str[20];
    char speed_str[20];
    
    // ������ʾ��ɫ (��ɫ������ɫ��)
    ips114_set_color(RGB565_WHITE, RGB565_BLACK);
    
    // ���״̬���� - ʹ�ð�ȫ����
    ips114_show_string(5, 96, "               ");
    ips114_show_string(5, 112, "               ");
    
    // ��ʾ��״̬ - Ӣ�ļ�д
    sprintf(status_str, "St: %s", action);
    sprintf(speed_str, "Spd: %.1f m/s", speed);
    
    ips114_show_string(5, 96, status_str);
    ips114_show_string(5, 112, speed_str);
    
    printf("Status: %s - Speed: %.1f m/s\n", action, speed);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ����״̬
// ���ز���     uint8   �������(1-4)��0��ʾ�ް���
//-------------------------------------------------------------------------------------------------------------------
static uint8 test_read_keys(void)
{
    static uint8 key_last_state[4] = {1, 1, 1, 1};  // �ϴΰ���״̬
    uint8 key_current_state[4];
    uint8 i;
    
    // ��ȡ��ǰ����״̬���͵�ƽ��Ч��
    key_current_state[0] = gpio_get_level(KEY1);
    key_current_state[1] = gpio_get_level(KEY2);
    key_current_state[2] = gpio_get_level(KEY3);
    key_current_state[3] = gpio_get_level(KEY4);
    
    // ��ⰴ�����£��½��ش�����
    for(i = 0; i < 4; i++)
    {
        if(key_last_state[i] == 1 && key_current_state[i] == 0)
        {
            // ������ʱ
            system_delay_ms(20);
            
            // �ٴ�ȷ�ϰ���״̬
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
    
    return 0;  // �ް�������
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ָ���ٶ�ǰ��ָ������
// ����˵��     speed       Ŀ���ٶ� (m/s)
// ����˵��     distance    Ŀ����� (mm)
//-------------------------------------------------------------------------------------------------------------------
static void test_move_distance(float speed, float distance)
{
    float left_distance, right_distance;
    float avg_distance;
    char distance_str[32];
    
    // ���ñ���������
    encoder_reset(ENCODER_ID_BOTH);
    
    printf("��ʼǰ�� %.1f�ף��ٶ� %.1f m/s\n", distance/1000.0f, speed);
    
    // ��ʼ�ƶ�
    motor_set_target_speed(speed, speed);
    
    // ��ؾ���ֱ���ﵽĿ��
    while(1)
    {
        // ��ȡ��ǰ��ʻ����
        left_distance = encoder_get_distance(ENCODER_ID_LEFT);
        right_distance = encoder_get_distance(ENCODER_ID_RIGHT);
        avg_distance = (fabs(left_distance) + fabs(right_distance)) / 2.0f;
        
        // ��ʾ��ǰ���� - ʹ�ð�ȫ�����Ӣ�� (y=119��ȫ��Χ)
        sprintf(distance_str, "D:%.0f/%.0fmm", avg_distance, distance);
        ips114_show_string(5, 112, "               "); // ����speed��ʾ��
        ips114_show_string(5, 112, distance_str);
        
        // ����Ƿ�ﵽĿ�����
        if(avg_distance >= distance)
        {
            break;
        }
        
        system_delay_ms(50);
    }
    
    // ֹͣ���
    motor_set_target_speed(0.0f, 0.0f);
    
    // ��ʾ���״̬ - Ӣ��
    ips114_show_string(5, 96, "St: Done!     ");
    ips114_show_string(5, 112, "Spd: 0.0 m/s  ");
    
    printf("ǰ����ɣ�ʵ�ʾ���: %.1fmm\n", avg_distance);
    
    // �ȴ�һ��ʱ����ʾ���
    system_delay_ms(2000);
    
    // ���������ʾ - �ָ�speed��
    ips114_show_string(5, 112, "Spd: 0.0 m/s  ");
    
    // �ָ��˵���ʾ - Ӣ��
    ips114_show_string(5, 96, "Status: Wait Key");
}
