/*********************************************************************************************************************
* �ļ�����          test_nav_ahrs.h
* ����˵��          AHRS����ϵͳ����ͷ�ļ�
* ����              AI Assistant
* �汾��Ϣ          v1.2
* �޸ļ�¼
* ����              ����                �汾
* 2025-01-XX        AI Assistant       1.0v
* 2025-01-XX        AI Assistant       1.2v - ���·�����ɺ���
* 
* �ļ�����˵����
* ���ļ��ṩAHRS����ϵͳ�Ĳ���·�����ɺ�������������
********************************************************************************************************************/

#ifndef _TEST_NAV_AHRS_H_
#define _TEST_NAV_AHRS_H_

#include "nav_control_ahrs.h"
#include "zf_common_typedef.h"

//=================================================·�����ɺ�������================================================

/**
 * @brief  ���������β���·��
 * @param  size     �����α߳� (m)
 * @retval ״̬
 * @note   ����һ��������·����ÿ��NAV_AHRS_DISTANCE_PER_POINT��¼һ�������
 */
nav_ahrs_status_enum test_nav_ahrs_generate_square_path(float size);

/**
 * @brief  ����ֱ�߲���·��
 * @param  length       ֱ�߳��� (m)
 * @param  direction    ǰ������Ƕ� (��), 0=����, 90=����, 180=����, 270=����
 * @retval ״̬
 * @note   ����һ��ָ�����Ⱥͷ����ֱ��·��
 */
nav_ahrs_status_enum test_nav_ahrs_generate_straight_path(float length, float direction);

//=================================================������������================================================

/**
 * @brief  һ������AHRS����ϵͳ��������·����
 * @retval �Ƿ�ɹ�
 * @note   ���ô˺���������г�ʼ������ʼ����
 */
uint8 test_nav_ahrs_quick_start(void);

/**
 * @brief  һ������AHRS����ϵͳ��ֱ��·����
 * @param  length       ֱ�߳��� (m)
 * @param  direction    ����Ƕ� (��)
 * @retval �Ƿ�ɹ�
 * @note   ���ô˺���������г�ʼ������ʼֱ�ߵ���
 */
uint8 test_nav_ahrs_quick_start_straight(float length, float direction);

/**
 * @brief  ����ϵͳ��ѭ�����ڶ�ʱ���е��ã�
 * @param  dt: �������ڣ��룩������0.01��ʾ10ms
 * @note   ����5-10ms���ڵ���
 */
void test_nav_ahrs_loop(float dt);

#endif // _TEST_NAV_AHRS_H_
