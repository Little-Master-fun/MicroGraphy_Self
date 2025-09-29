/*********************************************************************************************************************
* �ļ�����          test_imu.c
* ����˵��          IMU��̬����ϵͳ���Գ���ʵ���ļ� - ʹ��SCH16TK10��ʵ������
* ����              LittleMaster  
* �汾��Ϣ          v1.0
* �޸ļ�¼
* ����              ����                �汾
* 2025-09-28        LittleMaster       1.0v
* 
* IPS114��ʾ��ȫ���� (240��135���أ�8��16����)��
* - ��������y=0-16     - "IMU Test System"
* - �Ƕ�����y=32-80    - ʵʱ�Ƕ���ʾ����������ת��ƫ����
* - ״̬����y=80-112   - ϵͳ״̬��ͳ����Ϣ
* 
* �ܹ���ƣ�
* 1. 1ms�жϲ���IMU���� - 1kHz��Ƶ���ݲɼ� (��imu_attitude.c��ʵ��)
* 2. ��ѭ��������̬���� - 10Hz����Ĵ���Ƶ��  
* 3. ��ʾ���� - 10Hz�����û��۲�����
* 4. ʹ��SCH16TK10�߾���IMU������
********************************************************************************************************************/

#include "test_imu.h"
#include "imu_attitude.h"
#include "zf_device_ips114.h"
#include "zf_common_headfile.h"
#include "zf_driver_timer.h"
#include "zf_driver_delay.h"
#include <stdio.h>
#include <math.h>

//=================================================ȫ�ֱ�������================================================
imu_test_system_t imu_test_system;

// ����ʱ������Ķ�ʱ��
static uint8 test_timer_initialized = 0;

//=================================================�ڲ���������================================================
static void display_realtime_angles(void);
static void display_system_status(void);
static void update_statistics(imu_euler_t *euler);
static uint32 test_get_time_ms(void);
static void test_init_timer(void);
static void draw_filled_rectangle(uint16 x, uint16 y, uint16 width, uint16 height, uint16 color);

//=================================================�ⲿ�ӿ�ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     IMU����ϵͳ������ - ʹ��SCH16TK10������
//-------------------------------------------------------------------------------------------------------------------
void test_imu_system(void)
{
    printf("=== IMU��̬����ϵͳ���Կ�ʼ (SCH16TK10) ===\r\n");
    
    // ��ʼ������ϵͳ
    if(!test_imu_init(IMU_TEST_MODE_REALTIME))
    {
        printf("IMU����ϵͳ��ʼ��ʧ�ܣ�\r\n");
        return;
    }
    
    printf("IMU����ϵͳ��ʼ���ɹ�\r\n");
    printf("��ʼʵʱ�Ƕ���ʾ...\r\n");
    printf("���ݲ���Ƶ��: 1kHz (1ms�жϣ��ڿ���ģ����ʵ��)\r\n");
    printf("��̬����Ƶ��: 10Hz\r\n");
    printf("��ʾ����Ƶ��: 10Hz\r\n");
    printf("ע�⣺����1ms�ж��е��� imu_data_collection_handler() ����\r\n");
    
    imu_test_system.status = IMU_TEST_STATUS_RUNNING;
    
    // ��ѭ�� - ֻ������̬�������ʾ����
    while(1)
    {
        uint32 current_time = test_get_time_ms();
        
        // ��̬������� - 10Hz (IMU���ݲ�����1ms�жϴ���)
        if(current_time - imu_test_system.last_attitude_time >= imu_test_system.config.attitude_update_ms)
        {
            imu_test_system.last_attitude_time = current_time;
            
            // ������̬���� (���������ж��и��µ�imu_attitudeģ��)
            imu_attitude_update();
            
            // ����ͳ����Ϣ
            imu_euler_t euler;
            if(imu_get_euler_angles(&euler) == IMU_STATUS_OK)
            {
                update_statistics(&euler);
            }
        }
        
        // ��ʾ���� - 10Hz
        if(current_time - imu_test_system.last_display_time >= imu_test_system.config.display_refresh_ms)
        {
            imu_test_system.last_display_time = current_time;
            display_realtime_angles();
            display_system_status();
        }
        
        system_delay_ms(10); // 100Hz��ѭ����ȷ����Ӧ��ʱ
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     1ms�ж��е�IMU���ݲ������� (�����ݽӿ�)
// ��ע��Ϣ     �ú�����Ǩ�Ƶ� imu_attitude.c���˴�������������
//-------------------------------------------------------------------------------------------------------------------
void imu_1ms_interrupt_handler(void)
{
    // ���ÿ���ģ���е����ݲɼ�����
    imu_data_collection_handler();
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ʼ��IMU����ϵͳ
//-------------------------------------------------------------------------------------------------------------------
uint8 test_imu_init(imu_test_mode_enum test_mode)
{
    // ��ղ���ϵͳ�ṹ��
    memset(&imu_test_system, 0, sizeof(imu_test_system_t));
    
    // �������ò���
    imu_test_system.config.test_mode = test_mode;
    imu_test_system.config.auto_calibration = 1;
    imu_test_system.config.display_refresh_ms = TEST_IMU_DISPLAY_REFRESH_MS;
    imu_test_system.config.attitude_update_ms = TEST_IMU_ATTITUDE_UPDATE_MS;
    
    // ��ʼ��ͳ����Ϣ
    imu_test_system.stats.max_pitch = -180.0f;
    imu_test_system.stats.min_pitch = 180.0f;
    imu_test_system.stats.max_roll = -180.0f;
    imu_test_system.stats.min_roll = 180.0f;
    imu_test_system.stats.max_yaw = -180.0f;
    imu_test_system.stats.min_yaw = 180.0f;
    imu_test_system.stats.start_time = test_get_time_ms();
    
    // ��ʼ��IMU��̬����ϵͳ (������������ʼ��)
    if(imu_attitude_init(IMU_MODE_COMPLEMENTARY) != IMU_STATUS_OK)
    {
        printf("IMU��̬ϵͳ��ʼ��ʧ��\r\n");
        return 0;
    }
    
    // ��ʼ����ʾ��
    ips114_init();
    ips114_clear();
    ips114_set_color(RGB565_WHITE, RGB565_BLACK);
    ips114_show_string(5, 0, "IMU Test - SCH16TK10");
    
    imu_test_system.status = IMU_TEST_STATUS_IDLE;
    imu_test_system.system_initialized = 1;
    
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ����ϵͳʱ���
//-------------------------------------------------------------------------------------------------------------------
uint32 test_imu_get_timestamp(void)
{
    return test_get_time_ms();
}

//=================================================�ڲ�����ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ʾʵʱ�Ƕ���Ϣ
//-------------------------------------------------------------------------------------------------------------------
static void display_realtime_angles(void)
{
    imu_euler_t euler;
    char angle_str[32];
    
    if(imu_get_euler_angles(&euler) == IMU_STATUS_OK)
    {
        // // ����Ƕ���ʾ����
        // draw_filled_rectangle(0, 32, 240, 48, RGB565_BLACK);
        // ips114_set_color(RGB565_WHITE, RGB565_BLACK);
        
        // ��ʾ������
        sprintf(angle_str, "Pitch: %6.2f deg", euler.pitch_deg);
        ips114_show_string(5, 32, angle_str);
        
        // ��ʾ��ת��
        sprintf(angle_str, "Roll:  %6.2f deg", euler.roll_deg);
        ips114_show_string(5, 48, angle_str);
        
        // ��ʾƫ����
        sprintf(angle_str, "Yaw:   %6.2f deg", euler.yaw_deg);
        ips114_show_string(5, 64, angle_str);
        
        // ��ʾ�Ƕȱ仯�ļ�ͼ��ָʾ
        // ������ָʾ��
        int pitch_bar = (int)((euler.pitch_deg + 90.0f) * 100.0f / 180.0f);
        if(pitch_bar < 0) pitch_bar = 0;
        if(pitch_bar > 100) pitch_bar = 100;
        
        draw_filled_rectangle(200, 32, pitch_bar/5, 4, RGB565_GREEN);
        draw_filled_rectangle(200 + pitch_bar/5, 32, 240 - (200 + pitch_bar/5), 4, RGB565_BLACK);
        
        // ��ת��ָʾ��
        int roll_bar = (int)((euler.roll_deg + 90.0f) * 100.0f / 180.0f);
        if(roll_bar < 0) roll_bar = 0;
        if(roll_bar > 100) roll_bar = 100;
        
        draw_filled_rectangle(200, 48, roll_bar/5, 4, RGB565_BLUE);
        draw_filled_rectangle(200 + roll_bar/5, 48, 240 - (200 + roll_bar/5), 4, RGB565_BLACK);
        
        // ƫ����ָʾ��
        int yaw_bar = (int)((euler.yaw_deg + 180.0f) * 100.0f / 360.0f);
        if(yaw_bar < 0) yaw_bar = 0;
        if(yaw_bar > 100) yaw_bar = 100;
        
        draw_filled_rectangle(200, 64, yaw_bar/5, 4, RGB565_RED);
        draw_filled_rectangle(200 + yaw_bar/5, 64, 240 - (200 + yaw_bar/5), 4, RGB565_BLACK);
        
        ips114_set_color(RGB565_WHITE, RGB565_BLACK);
    }
    else
    {
        ips114_show_string(5, 32, "IMU Data Error!");
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ʾϵͳ״̬
//-------------------------------------------------------------------------------------------------------------------
static void display_system_status(void)
{
    char status_str[32];
    
    // ����ʱ��
    uint32 runtime = test_get_time_ms() - imu_test_system.stats.start_time;
    sprintf(status_str, "Runtime: %d.%ds", (int)(runtime/1000), (int)((runtime%1000)/100));
    ips114_show_string(5, 80, status_str);
    
    // ���´���
    sprintf(status_str, "Updates: %d", (int)imu_test_system.stats.update_count);
    ips114_show_string(5, 96, status_str);
    
    // �������
    sprintf(status_str, "Errors: %d", (int)imu_test_system.stats.error_count);
    ips114_show_string(5, 112, status_str);
    
    // IMUϵͳ״̬
    imu_attitude_system_t imu_system;
    if(imu_get_system_status(&imu_system) == IMU_STATUS_OK)
    {
        const char* imu_status_text = "OK";
        switch(imu_system.status)
        {
            case IMU_STATUS_OK: imu_status_text = "OK"; break;
            case IMU_STATUS_CALIBRATING: imu_status_text = "CAL"; break;
            case IMU_STATUS_ERROR: imu_status_text = "ERR"; break;
            case IMU_STATUS_NOT_INIT: imu_status_text = "INIT"; break;
            default: imu_status_text = "UNK"; break;
        }
        
        sprintf(status_str, "IMU: %s", imu_status_text);
        ips114_show_string(120, 96, status_str);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����ͳ����Ϣ
//-------------------------------------------------------------------------------------------------------------------
static void update_statistics(imu_euler_t *euler)
{
    imu_test_system.stats.update_count++;
    
    // ���½Ƕȷ�Χ
    if(euler->pitch_deg > imu_test_system.stats.max_pitch)
        imu_test_system.stats.max_pitch = euler->pitch_deg;
    if(euler->pitch_deg < imu_test_system.stats.min_pitch)
        imu_test_system.stats.min_pitch = euler->pitch_deg;
    
    if(euler->roll_deg > imu_test_system.stats.max_roll)
        imu_test_system.stats.max_roll = euler->roll_deg;
    if(euler->roll_deg < imu_test_system.stats.min_roll)
        imu_test_system.stats.min_roll = euler->roll_deg;
    
    if(euler->yaw_deg > imu_test_system.stats.max_yaw)
        imu_test_system.stats.max_yaw = euler->yaw_deg;
    if(euler->yaw_deg < imu_test_system.stats.min_yaw)
        imu_test_system.stats.min_yaw = euler->yaw_deg;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ��ǰʱ�䣨���룩
//-------------------------------------------------------------------------------------------------------------------
static uint32 test_get_time_ms(void)
{
    if(!test_timer_initialized)
    {
        test_init_timer();
    }
    return timer_get(TC_TIME2_CH1);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ʼ������ʱ�������ʱ��
//-------------------------------------------------------------------------------------------------------------------
static void test_init_timer(void)
{
    if(!test_timer_initialized)
    {
        timer_init(TC_TIME2_CH1, TIMER_MS);
        timer_clear(TC_TIME2_CH1);
        timer_start(TC_TIME2_CH1);
        test_timer_initialized = 1;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ���������Σ���������ڵ�ips114_show_rectangle������
//-------------------------------------------------------------------------------------------------------------------
static void draw_filled_rectangle(uint16 x, uint16 y, uint16 width, uint16 height, uint16 color)
{
    // �߽����ֹ������Ļ��Χ (240x135)
    if(x >= 240 || y >= 135) return;
    
    // ���ƿ�Ⱥ͸߶�����Ļ��Χ��
    if(x + width > 240) width = 240 - x;
    if(y + height > 135) height = 135 - y;
    
    // ʹ�ö����߻��ƾ���
    for(uint16 i = 0; i < height; i++)
    {
        ips114_draw_line(x, y + i, x + width - 1, y + i, color);
    }
}