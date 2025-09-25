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
    
    // ��ʼ��PWM��¼��ر���
    motor_control_system.left_pwm_record_count = 0;
    motor_control_system.right_pwm_record_count = 0;
    motor_control_system.pwm_record_enabled = 0;
    memset(motor_control_system.left_pwm_record, 0, sizeof(motor_control_system.left_pwm_record));
    memset(motor_control_system.right_pwm_record, 0, sizeof(motor_control_system.right_pwm_record));
    
    printf("�������ʽPI�ջ�����ϵͳ��ʼ�����\n");
    printf("����ʽPI����: Kp=%.1f Ki=%.1f ����޷�=%.1f\n", 
           MOTOR_PI_KP_DEFAULT, MOTOR_PI_KI_DEFAULT, MOTOR_PI_OUTPUT_MAX);
    
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
    //encoder_update();
    
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
    motor_set_pwm(MOTOR_RIGHT, (int16)left_output);
    motor_set_pwm(MOTOR_LEFT, (int16)right_output);
    
    // ��¼PWMֵ���ڷ�����Ӧ����
    if (motor_control_system.pwm_record_enabled) {
        // ��¼����PWMֵ��ʵ��������ҵ����
        if (fabs(left_output) >= MOTOR_PWM_RECORD_THRESHOLD && 
            motor_control_system.left_pwm_record_count < MOTOR_PWM_RECORD_SIZE) {
            motor_control_system.left_pwm_record[motor_control_system.left_pwm_record_count] = left_output;
            motor_control_system.left_pwm_record_count++;
        }
        
        // ��¼�ҵ��PWMֵ��ʵ�������������
        if (fabs(right_output) >= MOTOR_PWM_RECORD_THRESHOLD && 
            motor_control_system.right_pwm_record_count < MOTOR_PWM_RECORD_SIZE) {
            motor_control_system.right_pwm_record[motor_control_system.right_pwm_record_count] = right_output;
            motor_control_system.right_pwm_record_count++;
        }
        
        // ��������������¼���ˣ��Զ�ֹͣ��¼
        if (motor_control_system.left_pwm_record_count >= MOTOR_PWM_RECORD_SIZE &&
            motor_control_system.right_pwm_record_count >= MOTOR_PWM_RECORD_SIZE) {
            motor_control_system.pwm_record_enabled = 0;
            printf("PWM��¼�������Զ�ֹͣ��¼\n");
        }
    }
    
    // ͳ�Ƽ���
    motor_control_system.total_update_count++;
    
    return MOTOR_PID_OK;
}







//=================================================�ڲ�����ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ����ʽPI���������㣨�ڲ�������
//-------------------------------------------------------------------------------------------------------------------
static float motor_pid_calculate(motor_pid_controller_t *pid, float target, float current)
{
    if (pid == NULL || !pid->initialized) {
        return 0.0f;
    }
    
    // ���㵱ǰ���
    float error = -target + current;
    
    // ========== ����ʽPI�����㷨 ==========
    // ����ʽPI���ƹ�ʽ: ��u(k) = Kp * [e(k) - e(k-1)] + Ki * e(k)
    // ����: e(k)Ϊ��ǰ���, e(k-1)Ϊ�ϴ����
    
    // ����������: Kp * [e(k) - e(k-1)]
    float proportional_increment = pid->kp * (error - pid->last_error);
    
    // ����������: Ki * e(k)  
    float integral_increment = pid->ki * error;
    
    // �������������: ��u(k) = Kp*[e(k)-e(k-1)] + Ki*e(k)
    float output_increment = proportional_increment + integral_increment;
    
    // ���㵱ǰ���: u(k) = u(k-1) + ��u(k)
    float output = pid->last_output + output_increment;
    
    // ����޷���ȷ��PWM����Ч��Χ��
    if (output > pid->output_max) {
        output = pid->output_max;
    } else if (output < -pid->output_max) {
        output = -pid->output_max;
    }
    
    // ������ʷֵ��Ϊ�´μ�����׼����
    pid->last_error = error;      // ���浱ǰ�����Ϊ�´ε��ϴ����
    pid->last_output = output;    // ���浱ǰ�����Ϊ�´ε��ϴ����
    
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
    
    // ����PI���Ʋ���  
    pid->kp = MOTOR_PI_KP_DEFAULT;
    pid->ki = MOTOR_PI_KI_DEFAULT;
    pid->kd = MOTOR_PI_KD_DEFAULT;
    
    // ����ǰ�����Ʋ���
    pid->kff = MOTOR_FEEDFORWARD_KFF;
    pid->feedforward_a = MOTOR_FEEDFORWARD_A;
    pid->feedforward_b = MOTOR_FEEDFORWARD_B;
    
    // ����״̬����
    pid->integral = 0.0f;              // ����ʽPI�в�ʹ�ã�������
    pid->last_error = 0.0f;            // �ϴ�����ʼ��Ϊ0
    pid->last_output = 0.0f;           // �ϴ������ʼ��Ϊ0������ʽPI��Ҫ������
    pid->integral_max = MOTOR_PI_INTEGRAL_MAX;
    pid->output_max = MOTOR_PI_OUTPUT_MAX;
    
    // ����ͳ����Ϣ
    pid->update_count = 0;
    pid->max_error = 0.0f;
    pid->average_error = 0.0f;
    
    // ���Ϊ�ѳ�ʼ��
    pid->initialized = 1;
    
    printf("%s����ʽPI��������ʼ���ɹ�\n", name);
    
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

//=================================================PWM���ݼ�¼����================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ����PWM���ݼ�¼
//-------------------------------------------------------------------------------------------------------------------
void motor_start_pwm_record(void)
{
    if (!motor_control_system.system_enabled) {
        printf("���󣺵������ϵͳδ��ʼ�����޷�����PWM��¼\n");
        return;
    }
    
    // ���֮ǰ�ļ�¼����
    motor_control_system.left_pwm_record_count = 0;
    motor_control_system.right_pwm_record_count = 0;
    memset(motor_control_system.left_pwm_record, 0, sizeof(motor_control_system.left_pwm_record));
    memset(motor_control_system.right_pwm_record, 0, sizeof(motor_control_system.right_pwm_record));
    
    // ������¼
    motor_control_system.pwm_record_enabled = 1;
    
    printf("PWM���ݼ�¼������������¼ǰ%d����Ϊ���PWMֵ\n", MOTOR_PWM_RECORD_SIZE);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ֹͣPWM���ݼ�¼
//-------------------------------------------------------------------------------------------------------------------
void motor_stop_pwm_record(void)
{
    motor_control_system.pwm_record_enabled = 0;
    
    printf("PWM���ݼ�¼��ֹͣ\n");
    printf("������¼��%d��PWMֵ���ҵ����¼��%d��PWMֵ\n", 
           motor_control_system.left_pwm_record_count, 
           motor_control_system.right_pwm_record_count);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡPWM��¼����
//-------------------------------------------------------------------------------------------------------------------
uint16 motor_get_pwm_record(motor_pid_id_enum motor_id, float *data_buffer, uint16 buffer_size)
{
    if (data_buffer == NULL || buffer_size == 0) {
        return 0;
    }
    
    uint16 copy_count = 0;
    
    switch (motor_id) {
        case MOTOR_PID_LEFT:
            copy_count = (motor_control_system.left_pwm_record_count < buffer_size) ? 
                         motor_control_system.left_pwm_record_count : buffer_size;
            memcpy(data_buffer, motor_control_system.left_pwm_record, copy_count * sizeof(float));
            break;
            
        case MOTOR_PID_RIGHT:
            copy_count = (motor_control_system.right_pwm_record_count < buffer_size) ? 
                         motor_control_system.right_pwm_record_count : buffer_size;
            memcpy(data_buffer, motor_control_system.right_pwm_record, copy_count * sizeof(float));
            break;
            
        default:
            printf("������Ч�ĵ��ID\n");
            return 0;
    }
    
    return copy_count;
}
