/*********************************************************************************************************************
* �ļ�����          driver_encoder.c
* ����˵��          ˫��������������ʵ���ļ�
* ����              LittleMaster
* �汾��Ϣ          v1.0
* �޸ļ�¼
* ����              ����                �汾
* 2025-09-17        LittleMaster       1.0v
* 
* �ļ�����˵����
* ���ļ���˫����������ϵͳ��ʵ���ļ����ṩ�����ı��������ݲɼ��ʹ�����
* ֧����ʽ�����˵���̼Ʋ������ٶȷ������˶�����
********************************************************************************************************************/

#include "driver_encoder.h"
#include "zf_common_headfile.h"
#include <math.h>

//=================================================ȫ�ֱ�������================================================
encoder_system_struct encoder_system = {0};

//=================================================�ڲ���������================================================
static encoder_status_enum encoder_read_single(encoder_id_enum encoder_id);
static encoder_status_enum encoder_calculate_single_speed(encoder_id_enum encoder_id);
static void encoder_reset_single(encoder_id_enum encoder_id);
static float encoder_pulse_to_distance(int32 pulse_count);

//=================================================�ⲿ�ӿ�ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ������ϵͳ��ʼ��
//-------------------------------------------------------------------------------------------------------------------
encoder_status_enum encoder_init(void)
{
    // ��ʼ���������Ӳ��
    encoder_dir_init(ENCODER_LEFT, ENCODER_LEFT_A, ENCODER_LEFT_B);
    
    // ��ʼ���ұ�����Ӳ��
    encoder_dir_init(ENCODER_RIGHT, ENCODER_RIGHT_A, ENCODER_RIGHT_B);
    
    // ��ʼ�����ݽṹ
    memset(&encoder_system, 0, sizeof(encoder_system_struct));
    
    return ENCODER_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ����������
//-------------------------------------------------------------------------------------------------------------------
encoder_status_enum encoder_read_data(encoder_id_enum encoder_id)
{
    encoder_status_enum status = ENCODER_STATUS_OK;
    
    if (encoder_id == ENCODER_ID_LEFT || encoder_id == ENCODER_ID_BOTH)
    {
        status |= encoder_read_single(ENCODER_ID_LEFT);
    }
    
    if (encoder_id == ENCODER_ID_RIGHT || encoder_id == ENCODER_ID_BOTH)
    {
        status |= encoder_read_single(ENCODER_ID_RIGHT);
    }
    
    return status;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ�������������
//-------------------------------------------------------------------------------------------------------------------
int32 encoder_get_pulse_count(encoder_id_enum encoder_id)
{
    switch (encoder_id)
    {
        case ENCODER_ID_LEFT:
            return encoder_system.left.pulse_count;
        case ENCODER_ID_RIGHT:
            return encoder_system.right.pulse_count;
        default:
            return 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ���������ݸ��£����ڵ��ã�
//-------------------------------------------------------------------------------------------------------------------
encoder_status_enum encoder_update(void)
{
    encoder_status_enum status = ENCODER_STATUS_OK;
    
    // ��ȡ����������
    status |= encoder_read_data(ENCODER_ID_BOTH);
    
    // �����˶�����
    encoder_system.linear_velocity = (encoder_system.left.speed_mps + encoder_system.right.speed_mps) / 2.0f;
    encoder_system.angular_velocity = (encoder_system.right.speed_mps - encoder_system.left.speed_mps) / (ENCODER_WHEEL_DIAMETER / 1000.0f);
    
    return status;
}

//=================================================�ڲ�����ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ�������������ݣ��ڲ�������
//-------------------------------------------------------------------------------------------------------------------
static encoder_status_enum encoder_read_single(encoder_id_enum encoder_id)
{
    encoder_data_struct *encoder_data;
    int32 raw_count;
    
    // ��ȡ��Ӧ�ı���������ָ���Ӳ�����
    if (encoder_id == ENCODER_ID_LEFT)
    {
        encoder_data = &encoder_system.left;
        raw_count = encoder_get_count(ENCODER_LEFT);
        encoder_clear_count(ENCODER_LEFT);
    }
    else if (encoder_id == ENCODER_ID_RIGHT)
    {
        encoder_data = &encoder_system.right;
        raw_count = encoder_get_count(ENCODER_RIGHT);
        encoder_clear_count(ENCODER_RIGHT);
    }
    else
    {
        return ENCODER_STATUS_ERROR;
    }
    
    // �����������
    encoder_data->pulse_count = raw_count;
    encoder_data->pulse_count_abs = (raw_count < 0) ? -raw_count : raw_count;
    
    // �������������
    encoder_data->total_pulse += encoder_data->pulse_count_abs;
    
    // �жϷ���
    if (raw_count > 0)
    {
        encoder_data->direction = ENCODER_DIR_FORWARD;
    }
    else if (raw_count < 0)
    {
        encoder_data->direction = ENCODER_DIR_BACKWARD;
    }
    else
    {
        encoder_data->direction = ENCODER_DIR_STOP;
    }
    
    // �����ٶ� (�򻯰汾)
    float wheel_circumference = M_PI * (ENCODER_WHEEL_DIAMETER / 1000.0f);  // ת��Ϊ��
    encoder_data->speed_mps = ((float)encoder_data->pulse_count_abs / ENCODER_PPR) * 
                             wheel_circumference * (1000.0f / ENCODER_SAMPLE_TIME);
    
    // ���Ƿ���
    if (encoder_data->direction == ENCODER_DIR_BACKWARD)
    {
        encoder_data->speed_mps = -encoder_data->speed_mps;
    }
    
    return ENCODER_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����ת��Ϊ���루�ڲ�������
//-------------------------------------------------------------------------------------------------------------------
static float encoder_pulse_to_distance(int32 pulse_count)
{
    float wheel_circumference = M_PI * ENCODER_WHEEL_DIAMETER;  // mm
    float distance_per_pulse = wheel_circumference / ENCODER_PPR;
    
    return (float)pulse_count * distance_per_pulse;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����ת��ΪRPM���ڲ�������
//-------------------------------------------------------------------------------------------------------------------
// static float encoder_pulse_to_rpm(int32 pulse_count, uint32 time_interval_ms)
// {
//     float revolutions = (float)pulse_count / ENCODER_PPR;
//     float time_minutes = (float)time_interval_ms / 60000.0f;
    
//     return revolutions / time_minutes;
// }

//-------------------------------------------------------------------------------------------------------------------
// �������     ���㵥���������ٶȣ��ڲ�������
//-------------------------------------------------------------------------------------------------------------------
static encoder_status_enum encoder_calculate_single_speed(encoder_id_enum encoder_id)
{
    encoder_data_struct *encoder_data;
    
    if (encoder_id == ENCODER_ID_LEFT)
    {
        encoder_data = &encoder_system.left;
    }
    else if (encoder_id == ENCODER_ID_RIGHT)
    {
        encoder_data = &encoder_system.right;
    }
    else
    {
        return ENCODER_STATUS_ERROR;
    }
    
    // ����ת�� (RPM)
    encoder_data->speed_rpm = ((float)encoder_data->pulse_count_abs / ENCODER_PPR) * 
                             (60000.0f / ENCODER_SAMPLE_TIME);
    
    // �������ٶ� (m/s)
    float wheel_circumference = M_PI * (ENCODER_WHEEL_DIAMETER / 1000.0f);
    encoder_data->speed_mps = ((float)encoder_data->pulse_count_abs / ENCODER_PPR) * 
                             wheel_circumference * (1000.0f / ENCODER_SAMPLE_TIME);
    
    // ������� (mm)
    encoder_data->distance_mm = encoder_pulse_to_distance(encoder_data->total_pulse);
    
    // ���Ƿ���
    if (encoder_data->direction == ENCODER_DIR_BACKWARD)
    {
        encoder_data->speed_mps = -encoder_data->speed_mps;
        encoder_data->speed_rpm = -encoder_data->speed_rpm;
    }
    
    return ENCODER_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ���õ������������ڲ�������
//-------------------------------------------------------------------------------------------------------------------
static void encoder_reset_single(encoder_id_enum encoder_id)
{
    encoder_data_struct *encoder_data;
    
    if (encoder_id == ENCODER_ID_LEFT)
    {
        encoder_data = &encoder_system.left;
        encoder_clear_count(ENCODER_LEFT);
    }
    else if (encoder_id == ENCODER_ID_RIGHT)
    {
        encoder_data = &encoder_system.right;
        encoder_clear_count(ENCODER_RIGHT);
    }
    else
    {
        return;
    }
    
    // ������������
    encoder_data->pulse_count = 0;
    encoder_data->pulse_count_abs = 0;
    encoder_data->total_pulse = 0;
    encoder_data->speed_rpm = 0.0f;
    encoder_data->speed_mps = 0.0f;
    encoder_data->distance_mm = 0.0f;
    encoder_data->direction = ENCODER_DIR_STOP;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ�������ٶ�
//-------------------------------------------------------------------------------------------------------------------
float encoder_get_speed(encoder_id_enum encoder_id)
{
    switch (encoder_id)
    {
        case ENCODER_ID_LEFT:
            return encoder_system.left.speed_mps;
        case ENCODER_ID_RIGHT:
            return encoder_system.right.speed_mps;
        default:
            return 0.0f;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ����������
//-------------------------------------------------------------------------------------------------------------------
float encoder_get_distance(encoder_id_enum encoder_id)
{
    switch (encoder_id)
    {
        case ENCODER_ID_LEFT:
            return encoder_system.left.distance_mm;
        case ENCODER_ID_RIGHT:
            return encoder_system.right.distance_mm;
        default:
            return 0.0f;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����������ٶ�
//-------------------------------------------------------------------------------------------------------------------
encoder_status_enum encoder_calculate_speed(encoder_id_enum encoder_id)
{
    encoder_status_enum status = ENCODER_STATUS_OK;
    
    if (encoder_id == ENCODER_ID_LEFT || encoder_id == ENCODER_ID_BOTH)
    {
        status |= encoder_calculate_single_speed(ENCODER_ID_LEFT);
    }
    
    if (encoder_id == ENCODER_ID_RIGHT || encoder_id == ENCODER_ID_BOTH)
    {
        status |= encoder_calculate_single_speed(ENCODER_ID_RIGHT);
    }
    
    return status;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ���ñ���������
//-------------------------------------------------------------------------------------------------------------------
encoder_status_enum encoder_reset(encoder_id_enum encoder_id)
{
    if (encoder_id == ENCODER_ID_LEFT || encoder_id == ENCODER_ID_BOTH)
    {
        encoder_reset_single(ENCODER_ID_LEFT);
    }
    
    if (encoder_id == ENCODER_ID_RIGHT || encoder_id == ENCODER_ID_BOTH)
    {
        encoder_reset_single(ENCODER_ID_RIGHT);
    }
    
    return ENCODER_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ�������ٶ���Ϣ
//-------------------------------------------------------------------------------------------------------------------
encoder_status_enum encoder_get_robot_velocity(float *linear_velocity, float *angular_velocity)
{
    if (linear_velocity != NULL)
    {
        *linear_velocity = encoder_system.linear_velocity;
    }
    
    if (angular_velocity != NULL)
    {
        *angular_velocity = encoder_system.angular_velocity;
    }
    
    return ENCODER_STATUS_OK;
}