/*********************************************************************************************************************
* �ļ�����          motor_control.h
* ����˵��          ���PID�ջ�����ϵͳͷ�ļ�
* ����              LittleMaster
* �汾��Ϣ          v1.0
* �޸ļ�¼
* ����              ����                �汾              ��ע
* 2025-09-19        LittleMaster        v1.0              �������PID����ϵͳ
*
* �ļ�����˵����
* ���ļ�Ϊ���PID�ջ�����ϵͳ��ͷ�ļ����ṩ˫����ľ�ȷ�ٶȿ���
* 
* ��Ҫ���ܣ�
* 1. ˫�������PID������
* 2. ���ٿ���֧��
* 3. ������̬����
* 4. ���ַ����ͱ���
* 5. ʵʱ״̬���
*
* PID�������ã�
* - ����ϵ�� Kp: 45.0
* - ����ϵ�� Ki: 3.0
* - ΢��ϵ�� Kd: 0.0
********************************************************************************************************************/

#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_

#include "zf_common_typedef.h"

//=================================================���ò�������================================================
#define MOTOR_PID_KP_DEFAULT        (45.0f)     // Ĭ�ϱ���ϵ��
#define MOTOR_PID_KI_DEFAULT        (3.0f)      // Ĭ�ϻ���ϵ��
#define MOTOR_PID_KD_DEFAULT        (0.0f)      // Ĭ��΢��ϵ��
#define MOTOR_PID_INTEGRAL_MAX      (2000.0f)   // �����޷�
#define MOTOR_PID_OUTPUT_MAX        (9999.0f)   // ����޷�
#define MOTOR_PID_MIN_DT            (0.001f)    // ��Сʱ���� (s)

//=================================================ö�����Ͷ���================================================
// ���IDö��
typedef enum
{
    MOTOR_PID_LEFT      = 0,                    // ����
    MOTOR_PID_RIGHT     = 1,                    // �ҵ��
    MOTOR_PID_BOTH      = 2,                    // ˫���
} motor_pid_id_enum;

// PID������״̬ö��
typedef enum
{
    MOTOR_PID_OK        = 0,                    // ����
    MOTOR_PID_ERROR     = 1,                    // ����
    MOTOR_PID_NOT_INIT  = 2,                    // δ��ʼ��
} motor_pid_status_enum;

//=================================================���ݽṹ����================================================
// PID���������ݽṹ
typedef struct {
    float kp, ki, kd;          // PID����
    float integral;            // �����ۻ�
    float last_error;          // �ϴ����
    float integral_max;        // �����޷�
    float output_max;          // ����޷�
    uint8 initialized;         // ��ʼ����־
    
    // ͳ����Ϣ
    uint32 update_count;       // ���´���
    float max_error;           // �������¼
    float average_error;       // ƽ�����
} motor_pid_controller_t;

// �������ϵͳ״̬�ṹ
typedef struct {
    motor_pid_controller_t left_pid;            // ����PID������
    motor_pid_controller_t right_pid;           // �ҵ��PID������
    
    // ϵͳ״̬
    uint8 system_enabled;                       // ϵͳʹ��״̬
    uint32 total_update_count;                  // �ܸ��´���
    float control_frequency;                    // ����Ƶ�� (Hz)
    
    // ��ǰ����ֵ
    float left_target_speed;                    // ����Ŀ���ٶ�
    float right_target_speed;                   // �ҵ��Ŀ���ٶ�
    float left_current_speed;                   // ������ǰ�ٶ�
    float right_current_speed;                  // �ҵ����ǰ�ٶ�
    float left_output;                          // �������
    float right_output;                         // �ҵ�����
} motor_control_system_t;

//=================================================ȫ�ֱ�������================================================
extern motor_control_system_t motor_control_system;

//=================================================��������================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ʼ�����PID������
// ����˵��     void
// ���ز���     motor_pid_status_enum   ��ʼ��״̬
// ʹ��ʾ��     motor_pid_init();
// ��ע��Ϣ     ʹ��Ĭ��PID������ʼ�����ҵ�������� (Kp=45, Ki=3, Kd=0)
//-------------------------------------------------------------------------------------------------------------------
motor_pid_status_enum motor_pid_init(void);

//-------------------------------------------------------------------------------------------------------------------
// �������     ���õ��Ŀ���ٶ�
// ����˵��     left_speed          ����Ŀ���ٶ� (m/s)
// ����˵��     right_speed         �ҵ��Ŀ���ٶ� (m/s)
// ���ز���     motor_pid_status_enum   ִ��״̬
// ʹ��ʾ��     motor_set_target_speed(1.0f, 1.0f);
// ��ע��Ϣ     ����PID���ƺ͵��������������ƽӿ�
//-------------------------------------------------------------------------------------------------------------------
motor_pid_status_enum motor_set_target_speed(float left_speed, float right_speed);

#endif // _MOTOR_CONTROL_H_
