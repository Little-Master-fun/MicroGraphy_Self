/*********************************************************************************************************************
* �ļ�����          test_nav_ahrs.c
* ����˵��          AHRS����ϵͳ�����ļ�
* ����              AI Assistant
* �汾��Ϣ          v1.2
* �޸ļ�¼
* ����              ����                �汾
* 2025-01-XX        AI Assistant       1.0v
* 2025-01-XX        AI Assistant       1.1v - ��Ϊһ������
* 2025-01-XX        AI Assistant       1.2v - ���·�����ɺ���
* 
* �ļ�����˵����
* ���ļ��ṩAHRS����ϵͳ�Ĳ���·�����ɺ���������
********************************************************************************************************************/

#include "test_nav_ahrs.h"
#include "nav_control_ahrs.h"
#include "imu_ahrs_complementary.h"
#include "driver_encoder.h"
#include "motor_control.h"
#include "zf_common_headfile.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

//=================================================·�����ɺ���================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ���������β���·��
// ����˵��     size            �����α߳� (m)
// ���ز���     ״̬
//-------------------------------------------------------------------------------------------------------------------
nav_ahrs_status_enum test_nav_ahrs_generate_square_path(float size)
{
    if (!nav_ahrs.initialized) {
        return NAV_AHRS_STATUS_NOT_INIT;
    }
    
    printf("[NAV_TEST] ����������·��: %.2fm �� %.2fm\n", size, size);
    
    // �������
    float perimeter = 4.0f * size;  // �ܳ� (m)
    float total_distance = perimeter * 1000.0f;  // �ܾ��� (mm)
    
    // ÿ���ߵľ��� (mm)
    float distance_per_side = total_distance / 4.0f;
    
    printf("[NAV_TEST] ���ܳ�: %.2fm, �ܾ���: %.2fmm\n", perimeter, total_distance);
    printf("[NAV_TEST] ÿ�߾���: %.2fmm\n", distance_per_side);
    
    // ����·����
    uint16 point_idx = 0;
    float current_distance = 0.0f;
    
    // ��һ���ߣ�0�� (����)
    float yaw = 0.0f;
    while (current_distance < distance_per_side && point_idx < NAV_AHRS_MAX_POINTS) {
        nav_ahrs.path.points[point_idx].yaw = yaw;
        nav_ahrs.path.points[point_idx].distance = current_distance;
        point_idx++;
        current_distance += NAV_AHRS_DISTANCE_PER_POINT;
    }
    
    // �ڶ����ߣ�90�� (����)
    yaw = 90.0f;
    float side2_end = 2.0f * distance_per_side;
    while (current_distance < side2_end && point_idx < NAV_AHRS_MAX_POINTS) {
        nav_ahrs.path.points[point_idx].yaw = yaw;
        nav_ahrs.path.points[point_idx].distance = current_distance;
        point_idx++;
        current_distance += NAV_AHRS_DISTANCE_PER_POINT;
    }
    
    // �������ߣ�180�� (����)
    yaw = 180.0f;
    float side3_end = 3.0f * distance_per_side;
    while (current_distance < side3_end && point_idx < NAV_AHRS_MAX_POINTS) {
        nav_ahrs.path.points[point_idx].yaw = yaw;
        nav_ahrs.path.points[point_idx].distance = current_distance;
        point_idx++;
        current_distance += NAV_AHRS_DISTANCE_PER_POINT;
    }
    
    // �������ߣ�270�� (����)
    yaw = 270.0f;
    while (current_distance < total_distance && point_idx < NAV_AHRS_MAX_POINTS) {
        nav_ahrs.path.points[point_idx].yaw = yaw;
        nav_ahrs.path.points[point_idx].distance = current_distance;
        point_idx++;
        current_distance += NAV_AHRS_DISTANCE_PER_POINT;
    }
    
    // ���ת����ɵ㣨ƽ��ת�ǣ�
    // ��ÿ��ת��ǰ����ӹ��ɽǶ�
    uint16 corner_indices[] = {
        (uint16)(distance_per_side / NAV_AHRS_DISTANCE_PER_POINT),           // ��һ��ת��
        (uint16)(2 * distance_per_side / NAV_AHRS_DISTANCE_PER_POINT),       // �ڶ���ת��
        (uint16)(3 * distance_per_side / NAV_AHRS_DISTANCE_PER_POINT)        // ������ת��
    };
    
    // ƽ��ת�ǣ���ӹ��ɶΣ�
    for (int i = 0; i < 3; i++) {
        uint16 corner_idx = corner_indices[i];
        if (corner_idx > 5 && corner_idx < point_idx - 5) {
            float angle_start = nav_ahrs.path.points[corner_idx - 5].yaw;
            float angle_end = nav_ahrs.path.points[corner_idx + 5].yaw;
            
            // ����Ƕȿ�Խ����
            if (fabs(angle_end - angle_start) > 180.0f) {
                if (angle_end > angle_start) {
                    angle_start += 360.0f;
                } else {
                    angle_end += 360.0f;
                }
            }
            
            // ���Բ�ֵ���ɶ�
            for (int j = -4; j <= 4; j++) {
                float ratio = (j + 4) / 8.0f;
                float smooth_yaw = angle_start + (angle_end - angle_start) * ratio;
                // ��һ���Ƕ�
                while (smooth_yaw > 180.0f) smooth_yaw -= 360.0f;
                while (smooth_yaw <= -180.0f) smooth_yaw += 360.0f;
                nav_ahrs.path.points[corner_idx + j].yaw = smooth_yaw;
            }
        }
    }
    
    nav_ahrs.path.total_points = point_idx;
    nav_ahrs.path.current_index = 0;
    nav_ahrs.path.loop_mode = 1;  // ѭ��ģʽ
    nav_ahrs.path_loaded = 1;
    
    printf("[NAV_TEST] ·��������ɣ��� %d ����\n", point_idx);
    printf("[NAV_TEST] ת��λ��: %d, %d, %d\n", 
           corner_indices[0], corner_indices[1], corner_indices[2]);
    
    return NAV_AHRS_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����ֱ�߲���·��
// ����˵��     length          ֱ�߳��� (m)
// ����˵��     direction       ǰ������Ƕ� (��), 0=����, 90=����, 180=����, 270=����
// ���ز���     ״̬
//-------------------------------------------------------------------------------------------------------------------
nav_ahrs_status_enum test_nav_ahrs_generate_straight_path(float length, float direction)
{
    if (!nav_ahrs.initialized) {
        return NAV_AHRS_STATUS_NOT_INIT;
    }
    
    printf("[NAV_TEST] ����ֱ��·��: %.2fm, ����: %.1f��\n", length, direction);
    
    // �������
    float total_distance = length * 1000.0f;  // �ܾ��� (mm)
    
    printf("[NAV_TEST] �ܾ���: %.2fmm\n", total_distance);
    
    // ����·����
    uint16 point_idx = 0;
    float current_distance = 0.0f;
    
    // ��ֱ�߷������ɵ�
    while (current_distance <= total_distance && point_idx < NAV_AHRS_MAX_POINTS) {
        nav_ahrs.path.points[point_idx].yaw = direction;
        nav_ahrs.path.points[point_idx].distance = current_distance;
        point_idx++;
        current_distance += NAV_AHRS_DISTANCE_PER_POINT;
    }
    
    nav_ahrs.path.total_points = point_idx;
    nav_ahrs.path.current_index = 0;
    nav_ahrs.path.loop_mode = 0;  // ��ѭ��ģʽ
    nav_ahrs.path_loaded = 1;
    
    printf("[NAV_TEST] ·��������ɣ��� %d ����\n", point_idx);
    
    return NAV_AHRS_STATUS_OK;
}

//=================================================���ĺ���================================================

/**
 * @brief  һ������AHRS����ϵͳ
 * @note   ���ô˺���������г�ʼ������ʼ����
 * @return �Ƿ�ɹ�
 */
uint8 test_nav_ahrs_quick_start(void)
{
    printf("\n�X�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�[\n");
    printf("�U     AHRS����ϵͳһ������                   �U\n");
    printf("�^�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�a\n\n");
    
    // 1. ��ʼ��AHRS��̬����ϵͳ
    printf("[1/6] ��ʼ��AHRS��̬����...\n");
    if (ahrs_complementary_init() != AHRS_STATUS_OK) {
        printf("? ����AHRS��ʼ��ʧ��\n");
        return 0;
    }
    printf("? AHRS��ʼ�����\n\n");
    
    // 2. ��ʼ��������
    printf("[2/6] ��ʼ��������...\n");
    if (encoder_init() != ENCODER_STATUS_OK) {
        printf("? ���󣺱�������ʼ��ʧ��\n");
        return 0;
    }
    printf("? ��������ʼ�����\n\n");
    
    // 3. ��ʼ���������
    printf("[3/6] ��ʼ���������...\n");
    if (motor_pid_init() != MOTOR_PID_OK) {
        printf("? ���󣺵�����Ƴ�ʼ��ʧ��\n");
        return 0;
    }
    printf("? ������Ƴ�ʼ�����\n\n");
    
    // 4. ��ʼ��AHRS����ϵͳ
    printf("[4/6] ��ʼ��AHRS����ϵͳ...\n");
    if (nav_ahrs_init() != NAV_AHRS_STATUS_OK) {
        printf("? ���󣺵���ϵͳ��ʼ��ʧ��\n");
        return 0;
    }
    printf("? ����ϵͳ��ʼ�����\n\n");
    
    // 5. ����1m��1m������·��
    printf("[5/6] ����1m��1m������·��...\n");
    if (test_nav_ahrs_generate_square_path(1.0f) != NAV_AHRS_STATUS_OK) {
        printf("? ����·������ʧ��\n");
        return 0;
    }
    printf("? ·���������\n\n");
    
    // 6. ��������
    printf("[6/6] ���������ط�ģʽ...\n");
    nav_ahrs_reset();                      // ����״̬
    nav_ahrs_set_speed(2.5f);              // ���û����ٶ� 2.5 m/s
    nav_ahrs_set_mode(NAV_AHRS_MODE_REPLAY);  // �����ط�ģʽ
    printf("? ����������\n\n");
    
    printf("�X�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�[\n");
    printf("�U     ��ʼ����ɣ�С����ʼ����               �U\n");
    printf("�U     ���ڶ�ʱ���е��� test_nav_ahrs_loop() �U\n");
    printf("�^�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�a\n\n");
    
    return 1;
}

/**
 * @brief  һ������AHRS����ϵͳ��ֱ��·����
 * @param  length       ֱ�߳��� (m)
 * @param  direction    ����Ƕ� (��)
 * @note   ���ô˺���������г�ʼ������ʼֱ�ߵ���
 * @return �Ƿ�ɹ�
 */
uint8 test_nav_ahrs_quick_start_straight(float length, float direction)
{
    printf("\n�X�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�[\n");
    printf("�U     AHRS����ϵͳһ��������ֱ�ߣ�           �U\n");
    printf("�^�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�a\n\n");
    
    // 1. ��ʼ��AHRS��̬����ϵͳ
    printf("[1/6] ��ʼ��AHRS��̬����...\n");
    if (ahrs_complementary_init() != AHRS_STATUS_OK) {
        printf("? ����AHRS��ʼ��ʧ��\n");
        return 0;
    }
    printf("? AHRS��ʼ�����\n\n");
    
    // 2. ��ʼ��������
    printf("[2/6] ��ʼ��������...\n");
    if (encoder_init() != ENCODER_STATUS_OK) {
        printf("? ���󣺱�������ʼ��ʧ��\n");
        return 0;
    }
    printf("? ��������ʼ�����\n\n");
    
    // 3. ��ʼ���������
    printf("[3/6] ��ʼ���������...\n");
    if (motor_pid_init() != MOTOR_PID_OK) {
        printf("? ���󣺵�����Ƴ�ʼ��ʧ��\n");
        return 0;
    }
    printf("? ������Ƴ�ʼ�����\n\n");
    
    // 4. ��ʼ��AHRS����ϵͳ
    printf("[4/6] ��ʼ��AHRS����ϵͳ...\n");
    if (nav_ahrs_init() != NAV_AHRS_STATUS_OK) {
        printf("? ���󣺵���ϵͳ��ʼ��ʧ��\n");
        return 0;
    }
    printf("? ����ϵͳ��ʼ�����\n\n");
    
    // 5. ����ֱ��·��
    printf("[5/6] ����%.2fmֱ��·��������%.1f�㣩...\n", length, direction);
    if (test_nav_ahrs_generate_straight_path(length, direction) != NAV_AHRS_STATUS_OK) {
        printf("? ����·������ʧ��\n");
        return 0;
    }
    printf("? ·���������\n\n");
    
    // 6. ��������
    printf("[6/6] ���������ط�ģʽ...\n");
    nav_ahrs_reset();                      // ����״̬
    nav_ahrs_set_speed(2.5f);              // ���û����ٶ� 2.5 m/s
    nav_ahrs_set_mode(NAV_AHRS_MODE_REPLAY);  // �����ط�ģʽ
    printf("? ����������\n\n");
    
    printf("�X�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�[\n");
    printf("�U     ��ʼ����ɣ�С����ʼ����               �U\n");
    printf("�U     ���ڶ�ʱ���е��� test_nav_ahrs_loop() �U\n");
    printf("�^�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�T�a\n\n");
    
    return 1;
}

/**
 * @brief  ����ϵͳ��ѭ�����ڶ�ʱ���е��ã�
 * @note   ����5-10ms���ڵ���
 * @param  dt: �������ڣ��룩������0.01��ʾ10ms
 */
void test_nav_ahrs_loop(float dt)
{
    // 1. ���µ���ϵͳ
    nav_ahrs_update(dt);
    
    // 2. ִ�е�����ƣ��ڲ��Զ�����motor_set_target_speed��
    nav_ahrs_get_motor_output(NULL, NULL);
}

