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
// �������     �����ٶȱջ�����
// ����˵��     target_speed        Ŀ���ٶ� (m/s)
// ����˵��     current_speed       ��ǰ�ٶ� (m/s)
// ���ز���     float               ������� (PWMֵ -9999~9999)
// ʹ��ʾ��     pwm = motor_left_speed_control(1.5f, actual_speed);
// ��ע��Ϣ     �����ٶ�PID�ջ�����
//-------------------------------------------------------------------------------------------------------------------
float motor_left_speed_control(float target_speed, float current_speed);

//-------------------------------------------------------------------------------------------------------------------
// �������     �ҵ���ٶȱջ�����
// ����˵��     target_speed        Ŀ���ٶ� (m/s)
// ����˵��     current_speed       ��ǰ�ٶ� (m/s)
// ���ز���     float               ������� (PWMֵ -9999~9999)
// ʹ��ʾ��     pwm = motor_right_speed_control(1.5f, actual_speed);
// ��ע��Ϣ     �ҵ���ٶ�PID�ջ�����
//-------------------------------------------------------------------------------------------------------------------
float motor_right_speed_control(float target_speed, float current_speed);

//-------------------------------------------------------------------------------------------------------------------
// �������     ˫������ٱջ�����
// ����˵��     left_target         ����Ŀ���ٶ� (m/s)
// ����˵��     right_target        �ҵ��Ŀ���ٶ� (m/s)
// ����˵��     left_current        ������ǰ�ٶ� (m/s)
// ����˵��     right_current       �ҵ����ǰ�ٶ� (m/s)
// ����˵��     left_output         �������ָ��
// ����˵��     right_output        �ҵ�����ָ��
// ���ز���     motor_pid_status_enum   ִ��״̬
// ʹ��ʾ��     motor_differential_speed_control(1.0f, 1.2f, act_l, act_r, &pwm_l, &pwm_r);
// ��ע��Ϣ     ˫�������PID�ջ�����
//-------------------------------------------------------------------------------------------------------------------
motor_pid_status_enum motor_differential_speed_control(float left_target, float right_target,
                                                      float left_current, float right_current,
                                                      float *left_output, float *right_output);

//-------------------------------------------------------------------------------------------------------------------
// �������     ����PID������
// ����˵��     motor_id            ���ID (MOTOR_PID_LEFT/RIGHT/BOTH)
// ���ز���     motor_pid_status_enum   ����״̬
// ʹ��ʾ��     motor_pid_reset(MOTOR_PID_BOTH);
// ��ע��Ϣ     ���PID���ֺ���ʷ�����ڿ���������
//-------------------------------------------------------------------------------------------------------------------
motor_pid_status_enum motor_pid_reset(motor_pid_id_enum motor_id);

//-------------------------------------------------------------------------------------------------------------------
// �������     ����PID����
// ����˵��     motor_id            ���ID (MOTOR_PID_LEFT/RIGHT/BOTH)
// ����˵��     kp                  ����ϵ��
// ����˵��     ki                  ����ϵ��
// ����˵��     kd                  ΢��ϵ��
// ���ز���     motor_pid_status_enum   ����״̬
// ʹ��ʾ��     motor_pid_set_params(MOTOR_PID_BOTH, 45.0f, 3.0f, 0.0f);
// ��ע��Ϣ     ��̬����PID����
//-------------------------------------------------------------------------------------------------------------------
motor_pid_status_enum motor_pid_set_params(motor_pid_id_enum motor_id, float kp, float ki, float kd);

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡPID������״̬
// ����˵��     motor_id            ���ID (MOTOR_PID_LEFT/RIGHT)
// ����˵��     kp, ki, kd          PID�������ָ��
// ����˵��     integral            ����ֵ���ָ��
// ����˵��     last_error          �ϴ�������ָ��
// ���ز���     motor_pid_status_enum   ��ȡ״̬
// ʹ��ʾ��     motor_pid_get_status(MOTOR_PID_LEFT, &kp, &ki, &kd, &integral, &error);
// ��ע��Ϣ     ��ȡPID�������ĵ�ǰ״̬��Ϣ
//-------------------------------------------------------------------------------------------------------------------
motor_pid_status_enum motor_pid_get_status(motor_pid_id_enum motor_id, float *kp, float *ki, float *kd, 
                                          float *integral, float *last_error);

//-------------------------------------------------------------------------------------------------------------------
// �������     ʹ��/���õ������ϵͳ
// ����˵��     enable              ʹ�ܱ�־ (1=ʹ��, 0=����)
// ���ز���     motor_pid_status_enum   ����״̬
// ʹ��ʾ��     motor_control_enable(1);
// ��ע��Ϣ     ����ʱ�����������
//-------------------------------------------------------------------------------------------------------------------
motor_pid_status_enum motor_control_enable(uint8 enable);

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ�������ϵͳ״̬
// ����˵��     system_info         ϵͳ״̬���ָ��
// ���ز���     motor_pid_status_enum   ��ȡ״̬
// ʹ��ʾ��     motor_control_get_system_status(&info);
// ��ע��Ϣ     ��ȡ�����������ϵͳ��״̬��Ϣ
//-------------------------------------------------------------------------------------------------------------------
motor_pid_status_enum motor_control_get_system_status(motor_control_system_t *system_info);

//-------------------------------------------------------------------------------------------------------------------
// �������     �������ϵͳ�����º���
// ����˵��     left_target         ����Ŀ���ٶ�
// ����˵��     right_target        �ҵ��Ŀ���ٶ�
// ���ز���     motor_pid_status_enum   ����״̬
// ʹ��ʾ��     motor_control_update(1.0f, 1.0f);
// ��ע��Ϣ     ���ɱ�������ȡ��PID���㡢����������������ѭ��
//-------------------------------------------------------------------------------------------------------------------
motor_pid_status_enum motor_control_update(float left_target, float right_target);

#endif // _MOTOR_CONTROL_H_
