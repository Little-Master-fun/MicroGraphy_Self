/*********************************************************************************************************************
* �ļ�����          test_nav.c
* ����˵��          ����ϵͳ���Գ���ʵ���ļ�
* ����              LittleMaster
* �汾��Ϣ          v1.0
* �޸ļ�¼
* ����              ����                �汾
* 2025-09-19        LittleMaster       1.0v
* 
* �ļ�����˵����
* ���ļ�ʵ�ֵ���ϵͳ���������Թ��ܣ�����������·�����ɡ��������ơ�״̬��ʾ��
* 
* �������̣�
* 1. ��ʼ������ϵͳ�Ͱ���ϵͳ
* 2. ����1��1m�����β���·��
* 3. �������Ƶ�������/ֹͣ
* 4. ʵʱ��ʾ����״̬��ͳ����Ϣ
* 5. ѭ������ֱ���ֶ�ֹͣ
* 
* �������壺
* KEY_1: ���ɲ�����������·��
* KEY_2: ��ʼ/��ͣ����
* KEY_3: ֹͣ����������
* KEY_4: ����ͣ��
********************************************************************************************************************/

#include "test_nav.h"
#include "driver_flash_road.h"
#include "zf_driver_timer.h"

//=================================================ȫ�ֱ�������================================================
static test_nav_status_enum test_status = TEST_NAV_IDLE;    // ����״̬
static test_nav_stats_t test_stats = {0};                   // ����ͳ����Ϣ
static uint32 last_display_update_ms = 0;                   // �ϴ���ʾ����ʱ��
static uint8 key_initialized = 0;                          // ������ʼ����־
static uint8 timer_initialized = 0;                        // ��ʱ����ʼ����־

//=================================================�ڲ���������================================================
static void test_nav_init_system(void);
static void test_nav_clear_display(void);
static float test_nav_calculate_distance(const nav_position_t *p1, const nav_position_t *p2);
static void test_nav_update_statistics(void);
static const char* test_nav_status_to_string(nav_status_enum status);
static uint32 test_nav_get_system_time_ms(void);

//=================================================�����Ժ���ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ��������ϵͳ����
//-------------------------------------------------------------------------------------------------------------------
void test_nav_complete(void)
{
    // ��ʼ��ϵͳ
    test_nav_init_system();
    
    // ��ʾ��ӭ����
    test_nav_clear_display();
    ips114_show_string(0, 0, "Navigation Test v1.0");
    ips114_show_string(0, 16, "KEY1: Create Path");
    ips114_show_string(0, 32, "KEY2: Start/Pause");
    ips114_show_string(0, 48, "KEY3: Stop & Return");
    ips114_show_string(0, 64, "KEY4: Emergency Stop");
    ips114_show_string(0, 96, "Press KEY1 to begin...");
    
    test_status = TEST_NAV_IDLE;
    uint32 last_update_ms = 0;
    
    // ������ѭ��
    while(1)
    {
        uint32 current_time = test_nav_get_system_time_ms();
        
        // ����ɨ�谴����ÿ10ms��
        if (current_time - last_update_ms >= 10)
        {
            key_scanner();
            last_update_ms = current_time;
        }
        
        // ����������
        if (test_nav_handle_keys())
        {
            // �а�������ʱ������ʾ
            test_nav_display_status();
        }
        
        // ���ڸ�����ʾ��״̬
        if (current_time - last_display_update_ms >= TEST_DISPLAY_UPDATE_MS)
        {
            test_nav_display_status();
            last_display_update_ms = current_time;
        }
        
        // ����״̬���͸���
        if (test_status == TEST_NAV_NAVIGATING)
        {
            // ���µ���ϵͳ
            nav_update();
            
            // ��鵼��״̬
            nav_status_enum nav_status = nav_get_status();
            if (nav_status == NAV_STATUS_COMPLETED)
            {
                test_status = TEST_NAV_COMPLETED;
                test_stats.loops_completed++;
                
                // �Զ����¿�ʼ������ѭ�����ԣ�
                system_delay_ms(1000);
                if (nav_start(NAV_MODE_PURE_PURSUIT))
                {
                    test_status = TEST_NAV_NAVIGATING;
                }
            }
            else if (nav_status == NAV_STATUS_ERROR || nav_status == NAV_STATUS_EMERGENCY_STOP)
            {
                test_status = TEST_NAV_ERROR;
            }
            
            // ����ͳ����Ϣ
            test_nav_update_statistics();
        }
        
        // �˳��������
        if (test_status == TEST_NAV_IDLE && 
            key_get_state(KEY_3) == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_3);
            break;  // ����KEY3�˳�����
        }
        
        system_delay_ms(5);  // ������ʱ������CPUռ��
    }
    
    // ���Խ�������
    nav_stop(0);
    test_nav_clear_display();
    ips114_show_string(0, 0, "Navigation Test End");
    ips114_show_string(0, 16, "Thank you!");
    
    // ��ʾ����ͳ��
    char stats_info[50];
    sprintf(stats_info, "Loops: %d", test_stats.loops_completed);
    ips114_show_string(0, 32, stats_info);
    sprintf(stats_info, "Max Error: %.2fm", test_stats.max_cross_track_error);
    ips114_show_string(0, 48, stats_info);
    
    system_delay_ms(3000);  // ��ʾ3��ͳ����Ϣ
}

//=================================================·�����ɺ���ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ����1��1m�����β���·��
//-------------------------------------------------------------------------------------------------------------------
uint8 test_nav_create_square_path(void)
{
    static nav_path_t square_path = {0};
    
    // ���·������
    memset(&square_path, 0, sizeof(nav_path_t));
    
    // ����·��������Ϣ
    strcpy(square_path.path_name, "�����β���·��");
    square_path.waypoint_count = 5;  // 4���ǵ� + 1���ص����
    square_path.current_waypoint = 0;
    
    // ���������ε�4���ǵ㣨�Ե�ǰλ��Ϊ��㣩
    
    // ·����0: ��� (0, 0)
    square_path.waypoints[0].position.x = 0.0f;
    square_path.waypoints[0].position.y = 0.0f;
    square_path.waypoints[0].target_speed = TEST_SQUARE_SPEED;
    square_path.waypoints[0].tolerance = TEST_WAYPOINT_TOLERANCE;
    square_path.waypoints[0].point_type = 0;  // ��ͨ��
    
    // ·����1: ��ǰ�� (1, 0)
    square_path.waypoints[1].position.x = TEST_SQUARE_SIZE;
    square_path.waypoints[1].position.y = 0.0f;
    square_path.waypoints[1].target_speed = TEST_CORNER_SPEED;
    square_path.waypoints[1].tolerance = TEST_WAYPOINT_TOLERANCE;
    square_path.waypoints[1].point_type = 1;  // ת���
    
    // ·����2: �Һ�� (1, 1)  
    square_path.waypoints[2].position.x = TEST_SQUARE_SIZE;
    square_path.waypoints[2].position.y = TEST_SQUARE_SIZE;
    square_path.waypoints[2].target_speed = TEST_CORNER_SPEED;
    square_path.waypoints[2].tolerance = TEST_WAYPOINT_TOLERANCE;
    square_path.waypoints[2].point_type = 1;  // ת���
    
    // ·����3: ���� (0, 1)
    square_path.waypoints[3].position.x = 0.0f;
    square_path.waypoints[3].position.y = TEST_SQUARE_SIZE;
    square_path.waypoints[3].target_speed = TEST_CORNER_SPEED;
    square_path.waypoints[3].tolerance = TEST_WAYPOINT_TOLERANCE;
    square_path.waypoints[3].point_type = 1;  // ת���
    
    // ·����4: �ص���� (0, 0)
    square_path.waypoints[4].position.x = 0.0f;
    square_path.waypoints[4].position.y = 0.0f;
    square_path.waypoints[4].target_speed = TEST_SQUARE_SPEED;
    square_path.waypoints[4].tolerance = TEST_WAYPOINT_TOLERANCE;
    square_path.waypoints[4].point_type = 2;  // �յ�
    
    // ����·���ܳ���
    square_path.total_length = 4.0f * TEST_SQUARE_SIZE;  // �������ܳ�
    
    // ����·��������ϵͳ
    if (!nav_load_path("�����β���·��"))
    {
        // ֱ������·�����ݣ��ƹ�Flash�洢��
        memcpy(&nav_system.current_path, &square_path, sizeof(nav_path_t));
        
        test_status = TEST_NAV_PATH_READY;
        return 1;
    }
    
    test_status = TEST_NAV_PATH_READY;
    return 1;
}

//=================================================��ʾ����ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ����״̬ʵʱ��ʾ
//-------------------------------------------------------------------------------------------------------------------
void test_nav_display_status(void)
{
    char display_info[50];
    
    // ����
    test_nav_clear_display();
    
    // ��ʾ����
    ips114_show_string(0, 0, "Nav Test Status");
    
    // ��ʾ����״̬
    const char* status_text = "";
    switch(test_status)
    {
        case TEST_NAV_IDLE:           status_text = "IDLE"; break;
        case TEST_NAV_PATH_READY:     status_text = "PATH_READY"; break;
        case TEST_NAV_NAVIGATING:     status_text = "NAVIGATING"; break;
        case TEST_NAV_PAUSED:         status_text = "PAUSED"; break;
        case TEST_NAV_COMPLETED:      status_text = "COMPLETED"; break;
        case TEST_NAV_ERROR:          status_text = "ERROR"; break;
        default:                      status_text = "UNKNOWN"; break;
    }
    
    sprintf(display_info, "Status: %s", status_text);
    ips114_show_string(0, 16, display_info);
    
    // ��ʾ����ϵͳ״̬
    nav_status_enum nav_status = nav_get_status();
    const char* nav_status_text = test_nav_status_to_string(nav_status);
    sprintf(display_info, "Nav: %s", nav_status_text);
    ips114_show_string(0, 32, display_info);
    
    // ��ʾ��ǰλ��
    nav_position_t current_pos = {0};
    if (nav_get_current_position(&current_pos))
    {
        sprintf(display_info, "Pos: %.2f,%.2f", current_pos.x, current_pos.y);
        ips114_show_string(0, 48, display_info);
    }
    
    // ��ʾ����ƫ��
    float cross_error = nav_get_cross_track_error();
    sprintf(display_info, "Error: %.3fm", cross_error);
    ips114_show_string(0, 64, display_info);
    
    // ��ʾͳ����Ϣ
    sprintf(display_info, "Loops: %d", test_stats.loops_completed);
    ips114_show_string(0, 80, display_info);
    
    // ��ʾ��ǰ�ٶȣ���������У�
    if (test_status == TEST_NAV_NAVIGATING)
    {
        sprintf(display_info, "Speed: %.2fm/s", nav_system.sensor_fusion.fused_pose.velocity.linear);
        ips114_show_string(0, 96, display_info);
    }
    
    // ��ʾ������ʾ������״̬��
    if (test_status == TEST_NAV_IDLE)
    {
        ips114_show_string(0, 112, "KEY1: Create Path");
    }
    else if (test_status == TEST_NAV_PATH_READY)
    {
        ips114_show_string(0, 112, "KEY2: Start Nav");
    }
    else if (test_status == TEST_NAV_NAVIGATING)
    {
        ips114_show_string(0, 112, "KEY2:Pause KEY4:Stop");
    }
}

//=================================================����������ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     �������ƴ���
//-------------------------------------------------------------------------------------------------------------------
uint8 test_nav_handle_keys(void)
{
    uint8 key_action = 0;
    
    // KEY_1: ���ɲ�����������·��
    if (key_get_state(KEY_1) == KEY_SHORT_PRESS)
    {
        key_clear_state(KEY_1);
        key_action = 1;
        
        test_nav_clear_display();
        ips114_show_string(0, 0, "Creating Path...");
        
        if (test_nav_create_square_path())
        {
            ips114_show_string(0, 16, "Path Created!");
            ips114_show_string(0, 32, "Size: 1x1m Square");
            ips114_show_string(0, 48, "Points: 5");
            ips114_show_string(0, 64, "Press KEY2 to start");
        }
        else
        {
            ips114_show_string(0, 16, "Path Create Failed!");
            test_status = TEST_NAV_ERROR;
        }
        
        system_delay_ms(1500);  // ��ʾ���1.5��
    }
    
    // KEY_2: ��ʼ/��ͣ����
    else if (key_get_state(KEY_2) == KEY_SHORT_PRESS)
    {
        key_clear_state(KEY_2);
        key_action = 1;
        
        if (test_status == TEST_NAV_PATH_READY)
        {
            // ��ʼ����
            test_nav_clear_display();
            ips114_show_string(0, 0, "Starting Navigation...");
            
            if (nav_start(NAV_MODE_PURE_PURSUIT))
            {
                test_status = TEST_NAV_NAVIGATING;
                test_stats.test_start_time_ms = test_nav_get_system_time_ms();
                ips114_show_string(0, 16, "Navigation Started!");
                system_delay_ms(1000);
            }
            else
            {
                ips114_show_string(0, 16, "Start Failed!");
                test_status = TEST_NAV_ERROR;
                system_delay_ms(1500);
            }
        }
        else if (test_status == TEST_NAV_NAVIGATING)
        {
            // ��ͣ����
            nav_pause(1);
            test_status = TEST_NAV_PAUSED;
            ips114_show_string(0, 112, "Navigation Paused");
            system_delay_ms(500);
        }
        else if (test_status == TEST_NAV_PAUSED)
        {
            // �ָ�����
            nav_pause(0);
            test_status = TEST_NAV_NAVIGATING;
            ips114_show_string(0, 112, "Navigation Resumed");
            system_delay_ms(500);
        }
    }
    
    // KEY_3: ֹͣ����������
    else if (key_get_state(KEY_3) == KEY_SHORT_PRESS)
    {
        key_clear_state(KEY_3);
        key_action = 1;
        
        if (test_status == TEST_NAV_NAVIGATING || test_status == TEST_NAV_PAUSED)
        {
            nav_stop(0);  // ����ͣ��
            test_status = TEST_NAV_IDLE;
            
            test_nav_clear_display();
            ips114_show_string(0, 0, "Navigation Stopped");
            ips114_show_string(0, 16, "Returning to idle...");
            system_delay_ms(1500);
        }
        else
        {
            test_status = TEST_NAV_IDLE;
        }
    }
    
    // KEY_4: ����ͣ��
    else if (key_get_state(KEY_4) == KEY_SHORT_PRESS)
    {
        key_clear_state(KEY_4);
        key_action = 1;
        
        test_nav_emergency_stop();
    }
    
    return key_action;
}

//=================================================��������ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ����·����֤����
//-------------------------------------------------------------------------------------------------------------------
uint8 test_nav_verify_path(void)
{
    nav_path_t *path = &nav_system.current_path;
    
    // ������֤
    if (path->waypoint_count != 5)
    {
        return 0;
    }
    
    // ��֤·��������
    if (fabsf(path->waypoints[0].position.x - 0.0f) > 0.01f ||
        fabsf(path->waypoints[0].position.y - 0.0f) > 0.01f)
    {
        return 0;
    }
    
    // ��֤·������
    float calculated_length = 0.0f;
    for (uint16 i = 1; i < path->waypoint_count; i++)
    {
        calculated_length += test_nav_calculate_distance(
            &path->waypoints[i-1].position, 
            &path->waypoints[i].position
        );
    }
    
    if (fabsf(calculated_length - path->total_length) > 0.1f)
    {
        return 0;
    }
    
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��������ͳ��
//-------------------------------------------------------------------------------------------------------------------
uint8 test_nav_get_statistics(test_nav_stats_t *stats)
{
    if (stats == NULL) return 0;
    
    *stats = test_stats;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����ͣ������
//-------------------------------------------------------------------------------------------------------------------
void test_nav_emergency_stop(void)
{
    nav_stop(1);  // ����ͣ��
    test_status = TEST_NAV_ERROR;
    
    test_nav_clear_display();
    ips114_show_string(0, 0, "EMERGENCY STOP!");
    ips114_show_string(0, 16, "System Halted");
    ips114_show_string(0, 32, "Check Safety");
    ips114_show_string(0, 48, "Press KEY3 to reset");
    
    // �ȴ��û�ȷ��
    while(1)
    {
        key_scanner();
        if (key_get_state(KEY_3) == KEY_SHORT_PRESS)
        {
            key_clear_state(KEY_3);
            break;
        }
        system_delay_ms(100);
    }
    
    test_nav_reset();
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����ϵͳ��λ
//-------------------------------------------------------------------------------------------------------------------
void test_nav_reset(void)
{
    test_status = TEST_NAV_IDLE;
    memset(&test_stats, 0, sizeof(test_nav_stats_t));
    
    test_nav_clear_display();
    ips114_show_string(0, 0, "System Reset");
    ips114_show_string(0, 16, "Ready for new test");
    system_delay_ms(1000);
}

//=================================================�ڲ�����ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ʼ������ϵͳ
//-------------------------------------------------------------------------------------------------------------------
static void test_nav_init_system(void)
{
    // ��ʼ������ϵͳ
    if (!nav_init())
    {
        ips114_clear();
        ips114_show_string(0, 0, "Nav Init Failed!");
        while(1); // ��ʼ��ʧ����ֹͣ
    }
    
    // ��ʼ������ϵͳ
    if (!key_initialized)
    {
        key_init(10);  // 10msɨ������
        key_initialized = 1;
    }
    
    // ��հ���״̬
    key_clear_all_state();
    
    // ���ò���״̬
    test_status = TEST_NAV_IDLE;
    memset(&test_stats, 0, sizeof(test_nav_stats_t));
    last_display_update_ms = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     �����ʾ
//-------------------------------------------------------------------------------------------------------------------
static void test_nav_clear_display(void)
{
    ips114_clear();
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ������������
//-------------------------------------------------------------------------------------------------------------------
static float test_nav_calculate_distance(const nav_position_t *p1, const nav_position_t *p2)
{
    if (p1 == NULL || p2 == NULL) return 0.0f;
    
    float dx = p2->x - p1->x;
    float dy = p2->y - p1->y;
    return sqrtf(dx * dx + dy * dy);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����ͳ����Ϣ
//-------------------------------------------------------------------------------------------------------------------
static void test_nav_update_statistics(void)
{
    if (test_stats.test_start_time_ms == 0) return;
    
    // ���²���ʱ��
    test_stats.total_test_time_ms = test_nav_get_system_time_ms() - test_stats.test_start_time_ms;
    
    // �������������
    float current_error = fabsf(nav_get_cross_track_error());
    if (current_error > test_stats.max_cross_track_error)
    {
        test_stats.max_cross_track_error = current_error;
    }
    
    // ����ƽ���������򻯰汾��
    static float error_sum = 0.0f;
    static uint32 error_count = 0;
    
    error_sum += current_error;
    error_count++;
    test_stats.avg_cross_track_error = error_sum / error_count;
    
    // ������ʻ����
    nav_statistics_t nav_stats;
    if (nav_get_statistics(&nav_stats))
    {
        test_stats.total_distance = nav_stats.total_distance_traveled;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����״̬ת��Ϊ�ַ���
//-------------------------------------------------------------------------------------------------------------------
static const char* test_nav_status_to_string(nav_status_enum status)
{
    switch(status)
    {
        case NAV_STATUS_IDLE:             return "IDLE";
        case NAV_STATUS_INITIALIZING:     return "INIT";
        case NAV_STATUS_PATH_LOADING:     return "LOADING";
        case NAV_STATUS_NAVIGATING:       return "NAVIGATING";
        case NAV_STATUS_PAUSED:           return "PAUSED";
        case NAV_STATUS_COMPLETED:        return "COMPLETED";
        case NAV_STATUS_ERROR:            return "ERROR";
        case NAV_STATUS_EMERGENCY_STOP:   return "EMERGENCY";
        default:                          return "UNKNOWN";
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡϵͳʱ�䣨���룩
//-------------------------------------------------------------------------------------------------------------------
static uint32 test_nav_get_system_time_ms(void)
{
    // ʹ��ϵͳ��ʱ����ȡ����ʱ��
    if (!timer_initialized) {
        timer_init(TC_TIME2_CH1, TIMER_MS);  // ��ʼ����ʱ��Ϊ����ģʽ
        timer_start(TC_TIME2_CH1);           // ������ʱ��
        timer_initialized = 1;
    }
    return timer_get(TC_TIME2_CH1);
}
