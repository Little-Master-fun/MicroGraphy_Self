/*********************************************************************************************************************
* CYT4BB Opensourec Library ���� CYT4BB ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ��� CYT4BB ��Դ���һ����
*
* CYT4BB ��Դ�� ��������
* �����Ը��������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
*
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
*
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
*
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ���֤Э�� �����������Ϊ���İ汾
* �������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ���֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
*
* �ļ�����          main_cm7_1
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          IAR 9.40.1
* ����ƽ̨          CYT4BB
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2024-1-4       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "nav_control_ahrs.h"
#include "dual_core_comm.h"
#include "test_nav_ahrs.h"

// ���µĹ��̻��߹����ƶ���λ�����ִ�����²���
// ��һ�� �ر��������д򿪵��ļ�
// �ڶ��� project->clean  �ȴ��·�����������

// ˫�˼ܹ�˵����
// CM7_1���ģ������㷨�;���
//   - ��CM7_0���մ���������
//   - ִ�е����㷨���� (5ms)
//   - ·���滮�;���
//   - ���Ϳ���ָ���CM7_0

// **************************** �������� ****************************

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// ʱ�����ü�ϵͳ��ʼ��<��ر���>
    debug_info_init();                  // ���Դ�����Ϣ��ʼ��
    
    printf("\n");
    printf("�X�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�[\n");
    printf("�U     CM7_1���ģ������㷨�����             �U\n");
    printf("�^�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�a\n\n");
    
    // 1. ��ʼ��˫��ͨ��
    printf("[CM7_1] ��ʼ��˫��ͨ��...\n");
    dual_core_comm_init_core1();
    
    // 2. �ȴ�CM7_0��ʼ�����
    printf("[CM7_1] �ȴ�CM7_0��ʼ�����...\n");
    system_delay_ms(2000);
    
    // 3. ��ʼ������ϵͳ
    printf("[CM7_1] ��ʼ������ϵͳ...\n");
    nav_ahrs_init();
    
    // 4. ���ɲ���·������ѡ�������λ�ֱ�ߣ�
    printf("[CM7_1] ���ɵ���·��...\n");
    
    // ѡ��1: ����������·�� (1m �� 1m)
    // test_nav_ahrs_generate_square_path(1.0f);
    
    // ѡ��2: ����ֱ��·�� (2m, 0�ȷ���)
    test_nav_ahrs_generate_straight_path(2.0f, 0.0f);
    
    // 5. ��ʱ�ȴ�ϵͳ�ȶ�
    printf("[CM7_1] �ȴ�ϵͳ�ȶ�...\n");
    system_delay_ms(1000);
    
    // 6. ���������㷨��ʱ��
    printf("[CM7_1] ���������㷨��ʱ��...\n");
    pit_ms_init(PIT_CH0, 5);  // �����㷨 (5ms)
    
    // 7. ��������
    printf("[CM7_1] ��������ģʽ...\n");
    nav_ahrs_reset();
    nav_ahrs_set_speed(2.5f);  // ���û����ٶ� 2.5 m/s
    nav_ahrs_set_mode(NAV_AHRS_MODE_REPLAY);
    
    
    // ��ѭ��
    uint32 loop_count = 0;
    while(true)
    {
        
        
        system_delay_ms(100);
    }
}

// **************************** �������� ****************************
