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
// ǰ�����Ʋ��� - ��������ֱ����� (2000,1.1), (3000,1.7)
#define MOTOR_FEEDFORWARD_KFF       (1666.7f)   // ǰ��ϵ�� Kff = 1/b
#define MOTOR_FEEDFORWARD_A         (-0.1f)     // �ؾ� a = v - b*PWM
#define MOTOR_FEEDFORWARD_B         (0.0006f)   // б�� b = (1.7-1.1)/(3000-2000)

// ����PI���Ʋ���(20ms��Ӧ��)
#define MOTOR_LEFT_PI_KP_DEFAULT    (2500.0f)    // ��������ϵ��
#define MOTOR_LEFT_PI_KI_DEFAULT    (525.0f)     // ��������ϵ��
#define MOTOR_LEFT_PI_KD_DEFAULT    (0.0f)       // ����΢��ϵ��

// �ҵ��PI���Ʋ���  
#define MOTOR_RIGHT_PI_KP_DEFAULT   (2500.0f)    // �ҵ������ϵ��
#define MOTOR_RIGHT_PI_KI_DEFAULT   (525.0f)     // �ҵ������ϵ��
#define MOTOR_RIGHT_PI_KD_DEFAULT   (0.0f)       // �ҵ��΢��ϵ��

// ͨ��PI���Ʋ���
#define MOTOR_PI_INTEGRAL_MAX       (7.0f)       // �����޷� (m) - ��ֹ���ֱ���
#define MOTOR_PI_OUTPUT_MAX         (9999.0f)    // ����޷� (PWM)
#define MOTOR_PI_CONTROL_PERIOD     (0.002f)     // �������� (s) - 2ms

// PWM���ݼ�¼����
#define MOTOR_PWM_RECORD_SIZE       (100)       // ��¼PWMֵ�������С
#define MOTOR_PWM_RECORD_THRESHOLD  (10.0f)     // PWM��¼��ֵ������ֵ���ڴ�ֵ�ż�¼��

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
// ����ʽPI���������ݽṹ
typedef struct {
    // PI���Ʋ���
    float kp, ki, kd;          // PI���� (kd��ʹ��)
    float integral;            // �����ۻ� (��λ: m*s) - ����ʽPI�в�ʹ��
    float last_error;          // �ϴ����
    float last_output;         // �ϴ����ֵ (����ʽPI����)
    float integral_max;        // �����޷� (m*s)
    float output_max;          // ����޷� (PWM)
    
    // ǰ�����Ʋ��� (����ʽPI�в�ʹ��)
    float kff;                 // ǰ��ϵ�� (PWM/(m/s))
    float feedforward_a;       // ������Ͻؾ� (m/s)
    float feedforward_b;       // �������б�� (m/s per PWM)
    
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
    
    // PWM���ݼ�¼�����ڷ�����Ӧ���ߣ�
    float left_pwm_record[MOTOR_PWM_RECORD_SIZE];   // ����PWM��¼����
    float right_pwm_record[MOTOR_PWM_RECORD_SIZE];  // �ҵ��PWM��¼����
    uint16 left_pwm_record_count;               // ����PWM��¼����
    uint16 right_pwm_record_count;              // �ҵ��PWM��¼����
    uint8 pwm_record_enabled;                   // PWM��¼ʹ�ܱ�־
} motor_control_system_t;

//=================================================ȫ�ֱ�������================================================
extern motor_control_system_t motor_control_system;

//=================================================��������================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ʼ�����PID������
// ����˵��     void
// ���ز���     motor_pid_status_enum   ��ʼ��״̬
// ʹ��ʾ��     motor_pid_init();
// ��ע��Ϣ     ʹ��Ĭ������ʽPI������ʼ�����ҵ�������� (Kp=250, Ki=420, Kd=0)
//-------------------------------------------------------------------------------------------------------------------
motor_pid_status_enum motor_pid_init(void);

//-------------------------------------------------------------------------------------------------------------------
// �������     ���õ��Ŀ���ٶȣ�������Ŀ�꣬�����п��Ƽ��㣩
// ����˵��     left_speed          ����Ŀ���ٶ� (m/s)
// ����˵��     right_speed         �ҵ��Ŀ���ٶ� (m/s)
// ���ز���     motor_pid_status_enum   ִ��״̬
// ʹ��ʾ��     motor_set_target_speed(1.0f, 1.0f);
// ��ע��Ϣ     ֻ��������Ŀ���ٶ�ֵ�����Ƽ�����motor_control_update�����
//-------------------------------------------------------------------------------------------------------------------
motor_pid_status_enum motor_set_target_speed(float left_speed, float right_speed);

//-------------------------------------------------------------------------------------------------------------------
// �������     ������Ƹ��º������ڶ�ʱ�ж��е��ã�
// ����˵��     void
// ���ز���     motor_pid_status_enum   ִ��״̬
// ʹ��ʾ��     motor_control_update();  // ��10ms��ʱ�ж��е���
// ��ע��Ϣ     ִ�б�������ȡ��PID���㡢PWM�������������ѭ�������ڹ̶������ж��е���
//-------------------------------------------------------------------------------------------------------------------
motor_pid_status_enum motor_control_update(void);

//-------------------------------------------------------------------------------------------------------------------
// �������     ����PWM���ݼ�¼
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     motor_start_pwm_record();
// ��ע��Ϣ     ���֮ǰ�ļ�¼���ݲ���ʼ��¼�µ�PWMֵ�����ڷ�����Ӧ����
//-------------------------------------------------------------------------------------------------------------------
void motor_start_pwm_record(void);

//-------------------------------------------------------------------------------------------------------------------
// �������     ֹͣPWM���ݼ�¼
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     motor_stop_pwm_record();
// ��ע��Ϣ     ֹͣ��¼PWMֵ�������Ѽ�¼�����ݹ�����
//-------------------------------------------------------------------------------------------------------------------
void motor_stop_pwm_record(void);

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡPWM��¼����
// ����˵��     motor_id            ���ID (MOTOR_PID_LEFT/MOTOR_PID_RIGHT)
// ����˵��     data_buffer         ���ݻ�����ָ��
// ����˵��     buffer_size         ��������С
// ���ز���     uint16              ʵ�ʸ��Ƶ����ݸ���
// ʹ��ʾ��     count = motor_get_pwm_record(MOTOR_PID_LEFT, buffer, 100);
// ��ע��Ϣ     ����¼��PWM���ݸ��Ƶ�ָ��������������ʵ�ʸ��Ƶ����ݸ���
//-------------------------------------------------------------------------------------------------------------------
uint16 motor_get_pwm_record(motor_pid_id_enum motor_id, float *data_buffer, uint16 buffer_size);

//-------------------------------------------------------------------------------------------------------------------
// �������     ���õ��PI����
// ����˵��     motor_id            ���ID (MOTOR_PID_LEFT/MOTOR_PID_RIGHT/MOTOR_PID_BOTH)
// ����˵��     kp                  ����ϵ��
// ����˵��     ki                  ����ϵ��  
// ����˵��     kd                  ΢��ϵ��
// ���ز���     motor_pid_status_enum   ����״̬
// ʹ��ʾ��     motor_set_pi_params(MOTOR_PID_LEFT, 2000.0f, 400.0f, 0.0f);
// ��ע��Ϣ     ����ʱ��̬����ָ�������PI������֧�����ҵ����������
//-------------------------------------------------------------------------------------------------------------------
motor_pid_status_enum motor_set_pi_params(motor_pid_id_enum motor_id, float kp, float ki, float kd);

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ���PI����
// ����˵��     motor_id            ���ID (MOTOR_PID_LEFT/MOTOR_PID_RIGHT)
// ����˵��     kp                  ����ϵ��ָ��
// ����˵��     ki                  ����ϵ��ָ��
// ����˵��     kd                  ΢��ϵ��ָ��
// ���ز���     motor_pid_status_enum   ��ȡ״̬
// ʹ��ʾ��     motor_get_pi_params(MOTOR_PID_LEFT, &kp, &ki, &kd);
// ��ע��Ϣ     ��ȡָ�������ǰ��PI����ֵ
//-------------------------------------------------------------------------------------------------------------------
motor_pid_status_enum motor_get_pi_params(motor_pid_id_enum motor_id, float *kp, float *ki, float *kd);

#endif // _MOTOR_CONTROL_H_
