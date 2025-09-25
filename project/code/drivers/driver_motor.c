/*********************************************************************************************************************
* �ļ�����          driver_motor.c
* ����˵��          ˫�����������ʵ���ļ���˫PWM���ƣ�
* ����              LittleMaster
* �汾��Ϣ          v3.0
* �޸ļ�¼
* ����              ����                �汾
* 2025-09-17        LittleMaster       1.0v 
* 2025-09-17        LittleMaster       2.0v              ������������
* 2025-09-23        LittleMaster       3.0v              ����˫PWM���ƣ��Ƴ�����GPIO
* 
* �ļ�����˵����
* ���ļ���˫�������ϵͳ��ʵ���ļ�������˫PWM���Ʒ�ʽ
* �����߼�����תʹ��PWM1����תʹ��PWM2������PWMͨ�����⹤��
* 
********************************************************************************************************************/

#include "driver_motor.h"
#include "zf_common_headfile.h"



//=================================================�ⲿ�ӿ�ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ���ϵͳ��ʼ��
// ����˵��     void
// ���ز���     motor_status_enum   ��ʼ��״̬
// ʹ��ʾ��     motor_init();
// ��ע��Ϣ     ��ʼ��˫PWMͨ��������GPIO�������
//-------------------------------------------------------------------------------------------------------------------
motor_status_enum motor_init(void)
{
    // ��ʼ������˫PWMͨ��
    pwm_init(MOTOR_LEFT_PWM1, MOTOR_PWM_FREQUENCY, MOTOR_PWM_INIT_DUTY);   // ������תͨ��
    pwm_init(MOTOR_LEFT_PWM2, MOTOR_PWM_FREQUENCY, MOTOR_PWM_INIT_DUTY);   // ������תͨ��
    
    // ��ʼ���ҵ��˫PWMͨ��
    pwm_init(MOTOR_RIGHT_PWM1, MOTOR_PWM_FREQUENCY, MOTOR_PWM_INIT_DUTY);  // �ҵ����תͨ��
    pwm_init(MOTOR_RIGHT_PWM2, MOTOR_PWM_FREQUENCY, MOTOR_PWM_INIT_DUTY);  // �ҵ����תͨ��
    
    // ȷ�������ʼ״̬Ϊֹͣ
    pwm_set_duty(MOTOR_LEFT_PWM1, 0);
    pwm_set_duty(MOTOR_LEFT_PWM2, 0);
    pwm_set_duty(MOTOR_RIGHT_PWM1, 0);
    pwm_set_duty(MOTOR_RIGHT_PWM2, 0);
    
    return MOTOR_STATUS_OK;
}




//-------------------------------------------------------------------------------------------------------------------
// �������     PWMֵ��������
// ����˵��     pwm_input           ����PWMֵ
// ���ز���     int16               �������PWMֵ
// ʹ��ʾ��     protected_pwm = motor_pwm_protect(input_pwm);
// ��ע��Ϣ     ����PWMֵ����Ч��Χ�ڣ���ֹ�����
//-------------------------------------------------------------------------------------------------------------------
int16 motor_pwm_protect(int16 pwm_input)
{
    int16 pwm_output = 0;
    
    // ���ޱ���
    if (pwm_input > MOTOR_PWM_MAX)
    {
        pwm_output = MOTOR_PWM_MAX;
    }
    // ���ޱ���
    else if (pwm_input < -MOTOR_PWM_MAX)
    {
        pwm_output = -MOTOR_PWM_MAX;
    }
    // ������������ֹ���������
    else if ((pwm_input > -MOTOR_PWM_MIN) && (pwm_input < MOTOR_PWM_MIN))
      {
          pwm_output = 0;
      }
      else
      {
          pwm_output = pwm_input;
      } 
      
      return pwm_output;
}


//-------------------------------------------------------------------------------------------------------------------
// �������     ���õ��PWM��˫PWM���Ʒ�ʽ��
// ����˵��     motor_id            ���ID (MOTOR_LEFT/MOTOR_RIGHT/MOTOR_BOTH)
// ����˵��     pwm_value           ���PWMֵ (-MOTOR_PWM_MAX ~ MOTOR_PWM_MAX)
// ���ز���     motor_status_enum   ִ��״̬
// ʹ��ʾ��     motor_set_pwm(MOTOR_LEFT, 5000);
// ��ע��Ϣ     ��ֵ��תʹ��PWM1����ֵ��תʹ��PWM2����PWMͨ������
//-------------------------------------------------------------------------------------------------------------------
motor_status_enum motor_set_pwm(motor_id_enum motor_id, int16 pwm_value)
{
    motor_status_enum status = MOTOR_STATUS_OK;
    
    // PWMֵ����
    int16 protected_pwm = motor_pwm_protect(pwm_value);
    
    // ����MOTOR_BOTH��������е����������ҵ��
    if (motor_id == MOTOR_BOTH)
    {
        status |= motor_set_pwm(MOTOR_LEFT, protected_pwm);
        status |= motor_set_pwm(MOTOR_RIGHT, protected_pwm);
        return status;
    }
    
    // ˫PWM�����߼��������ٶȺͷ���
    int16 abs_pwm = (protected_pwm >= 0) ? protected_pwm : -protected_pwm;
    uint8 is_forward = (protected_pwm >= 0) ? 0 : 1;
    
    // ��������
    if (motor_id == MOTOR_LEFT)
    {
        if (abs_pwm == 0)
        {
            // ֹͣģʽ��PWM1=0, PWM2=0
            pwm_set_duty(MOTOR_LEFT_PWM1, 0);
            pwm_set_duty(MOTOR_LEFT_PWM2, 0);
        }
        else if (is_forward)
        {
            // ��ת��PWM1=�ٶ�, PWM2=0
            pwm_set_duty(MOTOR_LEFT_PWM1, abs_pwm);
            pwm_set_duty(MOTOR_LEFT_PWM2, 0);
        }
        else
        {
            // ��ת��PWM1=0, PWM2=�ٶ�
            pwm_set_duty(MOTOR_LEFT_PWM1, 0);
            pwm_set_duty(MOTOR_LEFT_PWM2, abs_pwm);
        }
    }
    // �����ҵ��  
    else if (motor_id == MOTOR_RIGHT)
    {
        if (abs_pwm == 0)
        {
            // ֹͣģʽ��PWM1=0, PWM2=0
            pwm_set_duty(MOTOR_RIGHT_PWM1, 0);
            pwm_set_duty(MOTOR_RIGHT_PWM2, 0);
        }
        else if (is_forward)
        {
            // ��ת��PWM1=�ٶ�, PWM2=0
            pwm_set_duty(MOTOR_RIGHT_PWM1, abs_pwm);
            pwm_set_duty(MOTOR_RIGHT_PWM2, 0);
        }
        else
        {
            // ��ת��PWM1=0, PWM2=�ٶ�
            pwm_set_duty(MOTOR_RIGHT_PWM1, 0);
            pwm_set_duty(MOTOR_RIGHT_PWM2, abs_pwm);
        }
    }
    else
    {
        return MOTOR_STATUS_ERROR;
    }
    
    return MOTOR_STATUS_OK;
}
