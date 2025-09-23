/*********************************************************************************************************************
* �ļ�����          motor_control.c
* ����˵��          ���PID�ջ�����ϵͳʵ���ļ�
* ����              LittleMaster
* �汾��Ϣ          v1.0
* �޸ļ�¼
* ����              ����                �汾              ��ע
* 2025-09-19        LittleMaster        v1.0              �������PID����ϵͳʵ��
*
* �ļ�����˵����
* ���ļ�ʵ�ֵ��PID�ջ�����ϵͳ�����й��ܣ��ṩ˫����ľ�ȷ�ٶȿ���
* ֧�ֲ��ٿ��ơ�������̬���������ַ����͡�ʵʱ״̬��صȹ���
*
* ��Ҫ���ԣ�
* 1. �߾���PID�����㷨
* 2. ˫�����������
* 3. �����˶�֧��
* 4. ʵʱ���ܼ��
* 5. �������ߵ���
********************************************************************************************************************/

#include "motor_control.h"
#include "zf_common_headfile.h"
#include "driver_encoder.h"
#include "driver_motor.h"

//=================================================ȫ�ֱ�������================================================
motor_control_system_t motor_control_system = {0};

//=================================================�ڲ���������================================================
static float motor_pid_calculate(motor_pid_controller_t *pid, float target, float current);
static motor_pid_status_enum motor_pid_init_single(motor_pid_controller_t *pid, const char* name);
static void motor_pid_update_statistics(motor_pid_controller_t *pid, float error);

//=================================================��Ҫ�ӿں���================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ʼ�����PID������
//-------------------------------------------------------------------------------------------------------------------
motor_pid_status_enum motor_pid_init(void)
{
    // ����ϵͳ�ṹ��
    memset(&motor_control_system, 0, sizeof(motor_control_system_t));
    
    // ��ʼ������PID������
    if (motor_pid_init_single(&motor_control_system.left_pid, "����") != MOTOR_PID_OK) {
        printf("��������PID��ʼ��ʧ��\n");
        return MOTOR_PID_ERROR;
    }
    
    // ��ʼ���ҵ��PID������
    if (motor_pid_init_single(&motor_control_system.right_pid, "�ҵ��") != MOTOR_PID_OK) {
        printf("�����ҵ��PID��ʼ��ʧ��\n");
        return MOTOR_PID_ERROR;
    }
    
    // ����ϵͳ����
    motor_control_system.system_enabled = 1;
    motor_control_system.control_frequency = 100.0f;  // Ĭ��100Hz
    motor_control_system.total_update_count = 0;
    
    printf("���PID�ջ�����ϵͳ��ʼ�����\n");
    printf("��������: Kp=%.1f Ki=%.1f Kd=%.1f\n", 
           MOTOR_PID_KP_DEFAULT, MOTOR_PID_KI_DEFAULT, MOTOR_PID_KD_DEFAULT);
    
    return MOTOR_PID_OK;
}



//-------------------------------------------------------------------------------------------------------------------
// �������     ���õ��Ŀ���ٶȣ�������Ŀ�꣬�����п��Ƽ��㣩
//-------------------------------------------------------------------------------------------------------------------
motor_pid_status_enum motor_set_target_speed(float left_speed, float right_speed)
{
    if (!motor_control_system.system_enabled) {
        return MOTOR_PID_NOT_INIT;
    }
    
    // ֻ��������Ŀ���ٶ�
    motor_control_system.left_target_speed = left_speed;
    motor_control_system.right_target_speed = right_speed;
    
    return MOTOR_PID_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ������Ƹ��º������ڶ�ʱ�ж��е��ã���10ms�жϣ�
//-------------------------------------------------------------------------------------------------------------------
motor_pid_status_enum motor_control_update(void)
{
    if (!motor_control_system.system_enabled) {
        return MOTOR_PID_NOT_INIT;
    }
    
    // ���±���������
    encoder_update();
    
    // ��ȡ��ǰ�ٶ�
    float left_current = encoder_get_speed(ENCODER_ID_LEFT);
    float right_current = encoder_get_speed(ENCODER_ID_RIGHT);
    
    // ����ϵͳ״̬
    motor_control_system.left_current_speed = left_current;
    motor_control_system.right_current_speed = right_current;
    
    // ����PID���
    float left_output = motor_pid_calculate(&motor_control_system.left_pid, 
                                          motor_control_system.left_target_speed, 
                                          left_current);
    float right_output = motor_pid_calculate(&motor_control_system.right_pid, 
                                           motor_control_system.right_target_speed, 
                                           right_current);
    
    // �������ֵ
    motor_control_system.left_output = left_output;
    motor_control_system.right_output = right_output;
    
    // ���PWM�����
    motor_set_pwm(MOTOR_LEFT, (int16)left_output);
    motor_set_pwm(MOTOR_RIGHT, (int16)right_output);
    
    // ͳ�Ƽ���
    motor_control_system.total_update_count++;
    
    return MOTOR_PID_OK;
}







//=================================================�ڲ�����ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     PID���������㣨�ڲ�������
//-------------------------------------------------------------------------------------------------------------------
static float motor_pid_calculate(motor_pid_controller_t *pid, float target, float current)
{
    if (pid == NULL || !pid->initialized) {
        return 0.0f;
    }
    
    // ʹ�ù̶��Ŀ������ڣ�Ƕ��ʽϵͳ��׼������
    const float dt = 0.01f;  // �̶�10ms�������ڣ�������100HzƵ�ʵ���
    
    // �������
    float error = target - current;
    
    // ������
    float proportional = pid->kp * error;
    
    // ������
    pid->integral += error * dt;
    
    // �����޷�����ֹ���ֱ��ͣ�
    if (pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if (pid->integral < -pid->integral_max) {
        pid->integral = -pid->integral_max;
    }
    
    float integral = pid->ki * pid->integral;
    
    // ΢����
    float derivative = pid->kd * (error - pid->last_error) / dt;
    
    // PID���
    float output = proportional + integral + derivative;
    
    // ����޷�
    if (output > pid->output_max) {
        output = pid->output_max;
    } else if (output < -pid->output_max) {
        output = -pid->output_max;
    }
    
    // ������ʷֵ
    pid->last_error = error;
    
    // ����ͳ����Ϣ
    motor_pid_update_statistics(pid, error);
    
    return output;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ʼ������PID���������ڲ�������
//-------------------------------------------------------------------------------------------------------------------
static motor_pid_status_enum motor_pid_init_single(motor_pid_controller_t *pid, const char* name)
{
    if (pid == NULL) {
        return MOTOR_PID_ERROR;
    }
    
    // ����PID����
    pid->kp = MOTOR_PID_KP_DEFAULT;
    pid->ki = MOTOR_PID_KI_DEFAULT;
    pid->kd = MOTOR_PID_KD_DEFAULT;
    
    // ����״̬����
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->integral_max = MOTOR_PID_INTEGRAL_MAX;
    pid->output_max = MOTOR_PID_OUTPUT_MAX;
    
    // ����ͳ����Ϣ
    pid->update_count = 0;
    pid->max_error = 0.0f;
    pid->average_error = 0.0f;
    
    // ���Ϊ�ѳ�ʼ��
    pid->initialized = 1;
    
    printf("%sPID��������ʼ���ɹ�\n", name);
    
    return MOTOR_PID_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����PIDͳ����Ϣ���ڲ�������
//-------------------------------------------------------------------------------------------------------------------
static void motor_pid_update_statistics(motor_pid_controller_t *pid, float error)
{
    if (pid == NULL) {
        return;
    }
    
    // ���¼���
    pid->update_count++;
    
    // ����������
    float abs_error = fabs(error);
    if (abs_error > pid->max_error) {
        pid->max_error = abs_error;
    }
    
    // ����ƽ��������ƽ����
    if (pid->update_count == 1) {
        pid->average_error = abs_error;
    } else {
        pid->average_error = 0.95f * pid->average_error + 0.05f * abs_error;
    }
}
