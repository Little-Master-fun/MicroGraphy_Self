/*********************************************************************************************************************
* �ļ�����          driver_motor.c
* ����˵��          ˫�����������ʵ���ļ�
* ����              LittleMaster
* �汾��Ϣ          v1.0
* �޸ļ�¼
* ����              ����                �汾
* 2025-09-17        LittleMaster       1.0v
* 
* �ļ�����˵����
* ���ļ���˫�������ϵͳ��ʵ���ļ����ṩ�����ĵ�����ƹ���
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
// ��ע��Ϣ     ��ʼ��PWMͨ����GPIO����
//-------------------------------------------------------------------------------------------------------------------
motor_status_enum motor_init(void)
{
    // ��ʼ������PWMͨ��
    pwm_init(MOTOR_LEFT_PWM1, MOTOR_PWM_FREQUENCY, MOTOR_PWM_INIT_DUTY);
    pwm_init(MOTOR_LEFT_PWM2, MOTOR_PWM_FREQUENCY, MOTOR_PWM_INIT_DUTY);
    
    // ��ʼ���ҵ��PWMͨ��
    pwm_init(MOTOR_RIGHT_PWM1, MOTOR_PWM_FREQUENCY, MOTOR_PWM_INIT_DUTY);
    pwm_init(MOTOR_RIGHT_PWM2, MOTOR_PWM_FREQUENCY, MOTOR_PWM_INIT_DUTY);
    
    // // ��ʼ�������������GPIO
    // gpio_init(MOTOR_LEFT_IN1, GPO, GPIO_LOW, GPO_PUSH_PULL);
    // gpio_init(MOTOR_LEFT_IN2, GPO, GPIO_LOW, GPO_PUSH_PULL);
    
    // // ��ʼ���ҵ���������GPIO
    // gpio_init(MOTOR_RIGHT_IN1, GPO, GPIO_LOW, GPO_PUSH_PULL);
    // gpio_init(MOTOR_RIGHT_IN2, GPO, GPIO_LOW, GPO_PUSH_PULL);
    
    // ȷ�������ʼ״̬Ϊֹͣ - TB67H420FTGֹͣģʽ
    pwm_set_duty(MOTOR_LEFT_PWM1, 0);
    pwm_set_duty(MOTOR_LEFT_PWM2, 0);
    pwm_set_duty(MOTOR_RIGHT_PWM1, 0);
    pwm_set_duty(MOTOR_RIGHT_PWM2, 0);
    
    // // TB67H420FTG: ֹͣ״̬�������з�������Ϊ�͵�ƽ
    // gpio_set_level(MOTOR_LEFT_IN1, GPIO_LOW);
    // gpio_set_level(MOTOR_LEFT_IN2, GPIO_LOW);
    // gpio_set_level(MOTOR_RIGHT_IN1, GPIO_LOW);
    // gpio_set_level(MOTOR_RIGHT_IN2, GPIO_LOW);
    
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
// �������     ���õ��PWM
// ����˵��     motor_id            ���ID (MOTOR_LEFT/MOTOR_RIGHT/MOTOR_BOTH)
// ����˵��     pwm_value           ���PWMֵ (-MOTOR_PWM_MAX ~ MOTOR_PWM_MAX)
// ���ز���     motor_status_enum   ִ��״̬
// ʹ��ʾ��     motor_set_pwm_single(MOTOR_LEFT, 5000);
// ��ע��Ϣ     ���õ����PWMֵ����ֵ��ת����ֵ��ת��֧��˫���ͬʱ����
//-------------------------------------------------------------------------------------------------------------------
motor_status_enum motor_set_pwm(motor_id_enum motor_id, int16 pwm_value)
{
    motor_status_enum status = MOTOR_STATUS_OK;
    
    // PWMֵ����
    int16 protected_pwm = motor_pwm_protect(pwm_value);
    
    // ����MOTOR_BOTH������ݹ�����������ҵ��
    if (motor_id == MOTOR_BOTH)
    {
        status |= motor_set_pwm(MOTOR_LEFT, protected_pwm);
        status |= motor_set_pwm(MOTOR_RIGHT, protected_pwm);
        return status;
    }
    
    // TB67H420FTG�����߼������ж��Ƿ�ֹͣ�������÷���
    int16 abs_pwm = (protected_pwm >= 0) ? protected_pwm : -protected_pwm;
    uint8 is_forward = (protected_pwm >= 0) ? 1 : 0;
    
    // ��������
    if (motor_id == MOTOR_LEFT)
    {
        if (abs_pwm == 0)
        {
            // ֹͣģʽ��TB67H420FTG IN1=L, IN2=L, PWM=0
            pwm_set_duty(MOTOR_LEFT_PWM1, 0);
            pwm_set_duty(MOTOR_LEFT_PWM2, 0);  // ��������PWMͨ��
            // gpio_set_level(MOTOR_LEFT_IN1, GPIO_LOW);
            // gpio_set_level(MOTOR_LEFT_IN2, GPIO_LOW);
        }
        else if (is_forward)
        {
            // ��ת��TB67H420FTG IN1=H, IN2=L, PWM=�ٶ�
            pwm_set_duty(MOTOR_LEFT_PWM1, abs_pwm);  // ��PWMͨ��
            pwm_set_duty(MOTOR_LEFT_PWM2, 0);        // ����ͨ����Ϊ0
            // gpio_set_level(MOTOR_LEFT_IN1, GPIO_HIGH);
            // gpio_set_level(MOTOR_LEFT_IN2, GPIO_LOW);
        }
        else
        {
            // ��ת��TB67H420FTG IN1=L, IN2=H, PWM=�ٶ�
            pwm_set_duty(MOTOR_LEFT_PWM1, abs_pwm);  // ��PWMͨ��
            pwm_set_duty(MOTOR_LEFT_PWM2, 0);        // ����ͨ����Ϊ0
            // gpio_set_level(MOTOR_LEFT_IN1, GPIO_LOW);
            // gpio_set_level(MOTOR_LEFT_IN2, GPIO_HIGH);
        }
    }
    // �����ҵ��  
    else if (motor_id == MOTOR_RIGHT)
    {
        if (abs_pwm == 0)
        {
            // ֹͣģʽ��TB67H420FTG IN1=L, IN2=L, PWM=0
            pwm_set_duty(MOTOR_RIGHT_PWM1, 0);
            pwm_set_duty(MOTOR_RIGHT_PWM2, 0);  // ��������PWMͨ��
            // gpio_set_level(MOTOR_RIGHT_IN1, GPIO_LOW);
            // gpio_set_level(MOTOR_RIGHT_IN2, GPIO_LOW);
        }
        else if (is_forward)
        {
            // ��ת��TB67H420FTG IN1=H, IN2=L, PWM=�ٶ�
            pwm_set_duty(MOTOR_RIGHT_PWM1, abs_pwm);  // ��PWMͨ��
            pwm_set_duty(MOTOR_RIGHT_PWM2, 0);        // ����ͨ����Ϊ0
            // gpio_set_level(MOTOR_RIGHT_IN1, GPIO_HIGH);
            // gpio_set_level(MOTOR_RIGHT_IN2, GPIO_LOW);
        }
        else
        {
            // ��ת��TB67H420FTG IN1=L, IN2=H, PWM=�ٶ�
            pwm_set_duty(MOTOR_RIGHT_PWM1, abs_pwm);  // ��PWMͨ��
            pwm_set_duty(MOTOR_RIGHT_PWM2, 0);        // ����ͨ����Ϊ0
            // gpio_set_level(MOTOR_RIGHT_IN1, GPIO_LOW);
            // gpio_set_level(MOTOR_RIGHT_IN2, GPIO_HIGH);
        }
    }
    else
    {
        return MOTOR_STATUS_ERROR;
    }
    
    return MOTOR_STATUS_OK;
}
