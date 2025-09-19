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

//=================================================�ڲ���������================================================
static motor_status_enum motor_set_pwm_single(motor_id_enum motor_id, int16 pwm_value);
static motor_status_enum motor_set_direction_single(motor_id_enum motor_id, motor_direction_enum direction);

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
    
    // ��ʼ�������������GPIO
    gpio_init(MOTOR_LEFT_IN1, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(MOTOR_LEFT_IN2, GPO, GPIO_LOW, GPO_PUSH_PULL);
    
    // ��ʼ���ҵ���������GPIO
    gpio_init(MOTOR_RIGHT_IN1, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(MOTOR_RIGHT_IN2, GPO, GPIO_LOW, GPO_PUSH_PULL);
    
    // ȷ�������ʼ״̬Ϊֹͣ
    motor_stop(MOTOR_BOTH);
    
    return MOTOR_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ���õ������
// ����˵��     motor_id            ���ID (MOTOR_LEFT/MOTOR_RIGHT/MOTOR_BOTH)
// ����˵��     direction           ������� (MOTOR_STOP/MOTOR_FORWARD/MOTOR_BACKWARD)
// ���ز���     motor_status_enum   ִ��״̬
// ʹ��ʾ��     motor_set_direction(MOTOR_LEFT, MOTOR_FORWARD);
// ��ע��Ϣ     ���õ��ת������
//-------------------------------------------------------------------------------------------------------------------
motor_status_enum motor_set_direction(motor_id_enum motor_id, motor_direction_enum direction)
{
    motor_status_enum status = MOTOR_STATUS_OK;
    
    if (motor_id == MOTOR_LEFT || motor_id == MOTOR_BOTH)
    {
        status |= motor_set_direction_single(MOTOR_LEFT, direction);
    }
    
    if (motor_id == MOTOR_RIGHT || motor_id == MOTOR_BOTH)
    {
        status |= motor_set_direction_single(MOTOR_RIGHT, direction);
    }
    
    return status;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ���õ���ٶ�
// ����˵��     motor_id            ���ID (MOTOR_LEFT/MOTOR_RIGHT/MOTOR_BOTH)
// ����˵��     speed               ����ٶ� (-MOTOR_PWM_MAX ~ MOTOR_PWM_MAX)
// ���ز���     motor_status_enum   ִ��״̬
// ʹ��ʾ��     motor_set_speed(MOTOR_LEFT, 5000);
// ��ע��Ϣ     ��ֵ��ת����ֵ��ת���Զ����������
//-------------------------------------------------------------------------------------------------------------------
motor_status_enum motor_set_speed(motor_id_enum motor_id, int16 speed)
{
    motor_status_enum status = MOTOR_STATUS_OK;
    
    // PWMֵ����
    int16 protected_speed = motor_pwm_protect(speed);
    
    // �����ٶ�����ȷ������
    motor_direction_enum direction;
    int16 abs_speed;
    
    if (protected_speed > 0)
    {
        direction = MOTOR_FORWARD;
        abs_speed = protected_speed;
    }
    else if (protected_speed < 0)
    {
        direction = MOTOR_BACKWARD;
        abs_speed = -protected_speed;
    }
    else
    {
        direction = MOTOR_STOP;
        abs_speed = 0;
    }
    
    // ���÷���
    status |= motor_set_direction(motor_id, direction);
    
    // ����PWM
    if (direction != MOTOR_STOP)
    {
        if (motor_id == MOTOR_LEFT || motor_id == MOTOR_BOTH)
        {
            status |= motor_set_pwm_single(MOTOR_LEFT, abs_speed);
        }
        
        if (motor_id == MOTOR_RIGHT || motor_id == MOTOR_BOTH)
        {
            status |= motor_set_pwm_single(MOTOR_RIGHT, abs_speed);
        }
    }
    
    return status;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ֹͣ���
// ����˵��     motor_id            ���ID (MOTOR_LEFT/MOTOR_RIGHT/MOTOR_BOTH)
// ���ز���     motor_status_enum   ִ��״̬
// ʹ��ʾ��     motor_stop(MOTOR_BOTH);
// ��ע��Ϣ     ָֹͣ�����
//-------------------------------------------------------------------------------------------------------------------
motor_status_enum motor_stop(motor_id_enum motor_id)
{
    motor_status_enum status = MOTOR_STATUS_OK;
    
    if (motor_id == MOTOR_LEFT || motor_id == MOTOR_BOTH)
    {
        // ֹͣ����
        gpio_set_level(MOTOR_LEFT_IN1, GPIO_LOW);
        gpio_set_level(MOTOR_LEFT_IN2, GPIO_LOW);
        pwm_set_duty(MOTOR_LEFT_PWM1, 0);
        pwm_set_duty(MOTOR_LEFT_PWM2, 0);
    }
    
    if (motor_id == MOTOR_RIGHT || motor_id == MOTOR_BOTH)
    {
        // ֹͣ�ҵ��
        gpio_set_level(MOTOR_RIGHT_IN1, GPIO_LOW);
        gpio_set_level(MOTOR_RIGHT_IN2, GPIO_LOW);
        pwm_set_duty(MOTOR_RIGHT_PWM1, 0);
        pwm_set_duty(MOTOR_RIGHT_PWM2, 0);
    }
    
    return status;
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
// �������     ������Ժ���
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     motor_test();
// ��ע��Ϣ     ���ڲ��Ե����������
//-------------------------------------------------------------------------------------------------------------------
void motor_test(void)
{
    // ����������ת
    motor_set_speed(MOTOR_LEFT, 3000);
    system_delay_ms(2000);
    
    // ����������ת
    motor_set_speed(MOTOR_LEFT, -3000);
    system_delay_ms(2000);
    
    // ֹͣ����
    motor_stop(MOTOR_LEFT);
    system_delay_ms(1000);
    
    // �����ҵ����ת
    motor_set_speed(MOTOR_RIGHT, 3000);
    system_delay_ms(2000);
    
    // �����ҵ����ת
    motor_set_speed(MOTOR_RIGHT, -3000);
    system_delay_ms(2000);
    
    // ֹͣ�ҵ��
    motor_stop(MOTOR_RIGHT);
    system_delay_ms(1000);
    
    // ����˫���ͬ��
    motor_set_speed(MOTOR_BOTH, 2000);
    system_delay_ms(2000);
    
    // ֹͣ���е��
    motor_stop(MOTOR_BOTH);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     �����˶�����
// ����˵��     linear_speed        ���ٶ� (-MOTOR_PWM_MAX ~ MOTOR_PWM_MAX)
// ����˵��     angular_speed       ���ٶ� (-MOTOR_PWM_MAX ~ MOTOR_PWM_MAX)
// ���ز���     motor_status_enum   ִ��״̬
// ʹ��ʾ��     motor_differential_drive(3000, 1000);
// ��ע��Ϣ     �������ٶȺͽ��ٶȿ��Ʋ����˶�
//-------------------------------------------------------------------------------------------------------------------
motor_status_enum motor_differential_drive(int16 linear_speed, int16 angular_speed)
{
    motor_status_enum status = MOTOR_STATUS_OK;
    
    // �����˶�ѧ����
    int16 left_speed = linear_speed - angular_speed;
    int16 right_speed = linear_speed + angular_speed;
    
    // �������ҵ���ٶ�
    status |= motor_set_speed(MOTOR_LEFT, left_speed);
    status |= motor_set_speed(MOTOR_RIGHT, right_speed);
    
    return status;
}

//=================================================�ڲ�����ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     �������PWM���ã��ڲ�������
// ����˵��     motor_id            ���ID (MOTOR_LEFT/MOTOR_RIGHT)
// ����˵��     pwm_value           PWMֵ (0 ~ MOTOR_PWM_MAX)
// ���ز���     motor_status_enum   ִ��״̬
// ��ע��Ϣ     �ڲ��������������õ��������PWM���
//-------------------------------------------------------------------------------------------------------------------
static motor_status_enum motor_set_pwm_single(motor_id_enum motor_id, int16 pwm_value)
{
    if (motor_id == MOTOR_LEFT)
    {
        pwm_set_duty(MOTOR_LEFT_PWM1, pwm_value);
        pwm_set_duty(MOTOR_LEFT_PWM2, 0);
    }
    else if (motor_id == MOTOR_RIGHT)
    {
        pwm_set_duty(MOTOR_RIGHT_PWM1, pwm_value);
        pwm_set_duty(MOTOR_RIGHT_PWM2, 0);
    }
    else
    {
        return MOTOR_STATUS_ERROR;
    }
    
    return MOTOR_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��������������ã��ڲ�������
// ����˵��     motor_id            ���ID (MOTOR_LEFT/MOTOR_RIGHT)
// ����˵��     direction           �������
// ���ز���     motor_status_enum   ִ��״̬
// ��ע��Ϣ     �ڲ��������������õ��������ת������
//-------------------------------------------------------------------------------------------------------------------
static motor_status_enum motor_set_direction_single(motor_id_enum motor_id, motor_direction_enum direction)
{
    if (motor_id == MOTOR_LEFT)
    {
        switch (direction)
        {
            case MOTOR_FORWARD:
                gpio_set_level(MOTOR_LEFT_IN1, GPIO_LOW);
                gpio_set_level(MOTOR_LEFT_IN2, GPIO_HIGH);
                break;
            case MOTOR_BACKWARD:
                gpio_set_level(MOTOR_LEFT_IN1, GPIO_HIGH);
                gpio_set_level(MOTOR_LEFT_IN2, GPIO_LOW);
                break;
            case MOTOR_STOP:
            default:
                gpio_set_level(MOTOR_LEFT_IN1, GPIO_LOW);
                gpio_set_level(MOTOR_LEFT_IN2, GPIO_LOW);
                break;
        }
    }
    else if (motor_id == MOTOR_RIGHT)
    {
        switch (direction)
        {
            case MOTOR_FORWARD:
                gpio_set_level(MOTOR_RIGHT_IN1, GPIO_HIGH);
                gpio_set_level(MOTOR_RIGHT_IN2, GPIO_LOW);
                break;
            case MOTOR_BACKWARD:
                gpio_set_level(MOTOR_RIGHT_IN1, GPIO_LOW);
                gpio_set_level(MOTOR_RIGHT_IN2, GPIO_HIGH);
                break;
            case MOTOR_STOP:
            default:
                gpio_set_level(MOTOR_RIGHT_IN1, GPIO_LOW);
                gpio_set_level(MOTOR_RIGHT_IN2, GPIO_LOW);
                break;
        }
    }
    else
    {
        return MOTOR_STATUS_ERROR;
    }
    
    return MOTOR_STATUS_OK;
}