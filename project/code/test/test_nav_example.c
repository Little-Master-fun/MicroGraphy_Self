/*********************************************************************************************************************
* �ļ�����          test_nav_example.c
* ����˵��          ��������ϵͳʹ��ʾ��
* ����              LittleMaster
* �汾��Ϣ          v1.0
* �޸ļ�¼
* ����              ����                �汾
* 2025-09-19        LittleMaster       1.0v
* 
* ʹ��˵����
* ���ļ�չʾ������������е��õ�������ϵͳ
* ����ֱ�ӽ� test_nav_main_example() ���������ݸ��Ƶ�ʵ�ʵ�main������ʹ��
********************************************************************************************************************/

#include "test_nav.h"
#include "zf_common_headfile.h"

//-------------------------------------------------------------------------------------------------------------------
// �������     ��������������ʾ��
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     ��main.c�е��ã�test_nav_main_example();
//-------------------------------------------------------------------------------------------------------------------
void test_nav_main_example(void)
{
    // 1. ��ʼ��ϵͳʱ�ӣ������Ҫ��
    // system_clock_init();
    
    // 2. ��ʼ�����Դ��ڣ������Ҫ��
    // debug_init();
    
    // 3. ��ʼ����ʾ��
    ips114_init();
    ips114_clear();
    
    // 4. ��ʾ������Ϣ
    ips114_show_string(0, 0, "MicroGraphy 2.0");
    ips114_show_string(0, 16, "Navigation Test");
    ips114_show_string(0, 32, "Initializing...");
    system_delay_ms(2000);
    
    // 5. ����������������
    test_nav_complete();
    
    // 6. ���Խ�����Ĵ���
    ips114_clear();
    ips114_show_string(0, 0, "Test Complete");
    ips114_show_string(0, 16, "System Halt");
    
    // 7. ��������ѭ��
    while(1)
    {
        system_delay_ms(1000);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     �򻯵ĵ�������ʾ��
// ����˵��     void  
// ���ز���     void
// ʹ��ʾ��     ֻ����·�����ɺ���֤��������ʵ�ʵ���
//-------------------------------------------------------------------------------------------------------------------
void test_nav_path_only_example(void)
{
    // ��ʼ����ʾ��
    ips114_init();
    ips114_clear();
    
    // ��ʾ������Ϣ
    ips114_show_string(0, 0, "Path Generation Test");
    
    // ����·������
    if (test_nav_create_square_path())
    {
        ips114_show_string(0, 16, "Path Created: OK");
        
        // ��֤·��
        if (test_nav_verify_path())
        {
            ips114_show_string(0, 32, "Path Verified: OK");
        }
        else
        {
            ips114_show_string(0, 32, "Path Verified: FAIL");
        }
        
        // ��ʾ·����Ϣ
        nav_path_t *path = &nav_system.current_path;
        char info[50];
        sprintf(info, "Points: %d", path->waypoint_count);
        ips114_show_string(0, 48, info);
        sprintf(info, "Length: %.1fm", path->total_length);
        ips114_show_string(0, 64, info);
    }
    else
    {
        ips114_show_string(0, 16, "Path Created: FAIL");
    }
    
    // ��ʾ�����Ϣ
    ips114_show_string(0, 96, "Test Complete");
    
    while(1)
    {
        system_delay_ms(1000);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ������ʾ���������棩
//-------------------------------------------------------------------------------------------------------------------
int main(void)
{
    // ����ϵͳ��ʼ��
    clock_init(SYSTEM_CLOCK_600M);      // ��ʼ��ϵͳʱ��
    debug_init();                        // ��ʼ�����Խӿ�
    
    // ������������
    test_nav_main_example();
    
    return 0;
}
