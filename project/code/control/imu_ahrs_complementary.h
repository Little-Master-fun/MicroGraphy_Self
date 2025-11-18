/*********************************************************************************************************************
* �ļ�����          imu_ahrs_complementary.h
* ����˵��          ������Ԫ����AHRS�����˲���ͷ�ļ�
* ����              LittleMaster
* �汾��Ϣ          v1.0
* �޸ļ�¼
* ����              ����                �汾
* 2025-01-XX        LittleMaster       1.0v
********************************************************************************************************************/

#ifndef __IMU_AHRS_COMPLEMENTARY_H_
#define __IMU_AHRS_COMPLEMENTARY_H_

#include "zf_common_typedef.h"
#include "driver_sch16tk10.h"

//=================================================ö�ٶ���================================================
// AHRSϵͳ״̬ö��
typedef enum
{
    AHRS_STATUS_OK = 0,             // ��������
    AHRS_STATUS_ERROR,              // ����
    AHRS_STATUS_NOT_INIT,           // δ��ʼ��
    AHRS_STATUS_CALIBRATING,        // ����У׼
} ahrs_status_enum;

//=================================================�ṹ�嶨��================================================

// ��Ԫ���ṹ��
typedef struct
{
    float q0;           // ��Ԫ��ʵ�� (w)
    float q1;           // ��Ԫ���鲿 (x)
    float q2;           // ��Ԫ���鲿 (y)
    float q3;           // ��Ԫ���鲿 (z)
} ahrs_quaternion_t;

// ŷ���ǽṹ��
typedef struct
{
    float pitch;                // ������ (��)
    float roll;                 // ��ת�� (��)
    float yaw;                  // ƫ���� (��) [-180, 180]
    float yaw_accumulated;      // �ۻ�ƫ���� (��)��������
} ahrs_euler_angles_t;

// ��������ƫ�ṹ��
typedef struct
{
    float x;            // X����ƫ (rad/s)
    float y;            // Y����ƫ (rad/s)
    float z;            // Z����ƫ (rad/s)
} ahrs_gyro_offset_t;

// IMU���ݽṹ��
typedef struct
{
    float gyro_x;       // ������X�� (rad/s)
    float gyro_y;       // ������Y�� 
    float gyro_z;       // ������Z�� 
    float acc_x;        // ���ٶȼ�X�� (g)
    float acc_y;        // ���ٶȼ�Y�� 
    float acc_z;        // ���ٶȼ�Z�� 
} ahrs_imu_data_t;

// PI�������ṹ��
typedef struct
{
    float integral_ex;  // X��������
    float integral_ey;  // Y��������
    float integral_ez;  // Z��������
} ahrs_pi_controller_t;

// AHRSϵͳ�ṹ��
typedef struct
{
    uint8 initialized;          // ��ʼ����־
    uint8 ready;                // ϵͳ׼��������־
    uint32 update_count;        // ���´�������
} ahrs_system_t;

//=================================================�ⲿ��������================================================

/**
 * @brief  ��ʼ��AHRS�����˲���ϵͳ
 * @return ahrs_status_enum  ��ʼ��״̬
 * @note   �ú������Զ���ʼ��������ƫУ׼���̣��豣���豸��ֹԼ2��
 */
ahrs_status_enum ahrs_complementary_init(void);

/**
 * @brief  AHRSϵͳ���£��ڶ�ʱ�ж��е��ã�
 * @param  raw_data  SCH16TK10������ԭʼ����ָ��
 * @return ahrs_status_enum  ����״̬
 * @note   ������1ms���ڵĶ�ʱ�ж��е��ô˺���
 * @note   �����ڲ����ۼ�AVG_FACTOR��ԭʼ���ݺ����ƽ�����Խ�������
 */
ahrs_status_enum ahrs_complementary_update(SCH1_raw_data *raw_data);

/**
 * @brief  ��ȡ��ǰŷ����
 * @param  euler  ŷ�������ָ��
 * @return ahrs_status_enum  ״̬
 */
ahrs_status_enum ahrs_get_euler_angles(ahrs_euler_angles_t *euler);

/**
 * @brief  ��ȡ��ǰ��Ԫ��
 * @param  quat  ��Ԫ�����ָ��
 * @return ahrs_status_enum  ״̬
 */
ahrs_status_enum ahrs_get_quaternion(ahrs_quaternion_t *quat);

/**
 * @brief  ��ȡ��������ƫ
 * @param  offset  ��ƫ���ָ��
 * @return ahrs_status_enum  ״̬
 */
ahrs_status_enum ahrs_get_gyro_offset(ahrs_gyro_offset_t *offset);

/**
 * @brief  ���ú�������
 * @return ahrs_status_enum  ״̬
 * @note   ���ô˺����󣬵�ǰ����ǻᱻ����Ϊ���
 */
ahrs_status_enum ahrs_reset_yaw(void);

/**
 * @brief  ����PI����������
 * @param  kp  ��������
 * @param  ki  ��������
 * @return ahrs_status_enum  ״̬
 * @note   Ĭ��ֵΪ0�����Ը���ʵ����������Բ���������Ư��
 */
ahrs_status_enum ahrs_set_pi_params(float kp, float ki);

/**
 * @brief  ���AHRSϵͳ�Ƿ�׼������
 * @return uint8  1-������У׼��ɣ� 0-δ����������У׼��δ��ʼ����
 */
/**
 * @brief  纯陀螺航向角（度, [-180, 180]）
 * @param  yaw_deg 陀螺积分航向
 * @return ahrs_status_enum
 */
ahrs_status_enum ahrs_get_yaw_gyro(float *yaw_deg);
uint8 ahrs_is_ready(void);

#endif /* __IMU_AHRS_COMPLEMENTARY_H_ */

