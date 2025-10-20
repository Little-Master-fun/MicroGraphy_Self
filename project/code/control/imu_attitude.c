/*********************************************************************************************************************
* �ļ�����          imu_attitude.c
* ����˵��          IMU��̬����ϵͳʵ���ļ�
* ����              LittleMaster
* �汾��Ϣ          v1.0
* �޸ļ�¼
* ����              ����                �汾
* 2025-09-28        LittleMaster       1.0v
* 
* �ļ�����˵����
* ���ļ�ʵ��IMU��̬����ϵͳ�����й��ܣ����������Ǻͼ��ٶȼ������ںϡ�ŷ���Ǽ����
* 
* ��Ҫ���ԣ�
* 1. �����˲����㷨���ں������Ǻͼ��ٶȼ�����
* 2. ��Ԫ����ŷ���Ǳ�ʾ��̬
* 3. ������У׼����ƫ����
* 4. �����˲��㷨֧��
* 5. ʵʱ״̬��غ͵�����Ϣ
* 6. SCH16TK10���������ݲɼ�
********************************************************************************************************************/

#include "imu_attitude.h"
#include "driver_sch16tk10.h"
#include "config_infineon.h"
#include "zf_common_headfile.h"
#include "zf_driver_timer.h"
#include "zf_driver_delay.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

//=================================================ȫ�ֱ�������================================================
imu_attitude_system_t imu_attitude_system = {0};

// ����ʱ������Ķ�ʱ��
static uint8 imu_timer_initialized = 0;

// SCH16TK10 ��������ر���
static SCH1_raw_data sch1_raw_data;
static SCH1_raw_data sch1_summed_data;  // �ۼ����ݻ�����
static SCH1_result sch1_result_data;
static uint8 sch1_initialized = 0;
static uint32 sch1_sample_count = 0;    // ����������

//=================================================�ڲ���������================================================
static imu_status_enum imu_apply_calibration(void);
static imu_status_enum imu_complementary_filter_update(void);
static imu_status_enum imu_mahony_filter_update(void);
static imu_status_enum imu_simple_gyro_integration(void);
static imu_status_enum imu_kalman_filter_update(void);
static void imu_quaternion_to_euler(imu_quaternion_t q, imu_euler_t *euler);
static void imu_euler_to_quaternion(imu_euler_t euler, imu_quaternion_t *q);
static void imu_quaternion_normalize(imu_quaternion_t *q);
static float imu_normalize_angle(float angle);
static void imu_update_dt(void);
static uint32 imu_get_time_ms(void);
static void imu_init_timer(void);

// �������˲�����غ���
static void imu_kalman_init(void);
static void imu_kalman_predict(float dt, imu_vector3_t *gyro);
static void imu_kalman_update_accel(imu_vector3_t *accel);
static void imu_matrix_multiply(float *A, float *B, float *C, int m, int n, int p);
static void imu_matrix_transpose(float *A, float *AT, int m, int n);
static void imu_matrix_inverse_3x3(float A[3][3], float Ainv[3][3]);
static void imu_matrix_add(float *A, float *B, float *C, int m, int n);
static void imu_matrix_subtract(float *A, float *B, float *C, int m, int n);

// Madgwick�˲�����غ���
static void imu_madgwick_init(void);
static imu_status_enum imu_madgwick_filter_update(void);
static void imu_madgwick_update_imu(float dt, imu_vector3_t *gyro, imu_vector3_t *accel);
static float imu_inv_sqrt(float x);

// UKF�˲�����غ���
static void imu_ukf_init(void);
static imu_status_enum imu_ukf_filter_update(void);
static void imu_ukf_generate_sigma_points(void);
static void imu_ukf_predict(float dt, imu_vector3_t *gyro);
static void imu_ukf_update_accel(imu_vector3_t *accel);
static void imu_ukf_measurement_function(float *state, float *measurement);
static void imu_matrix_cholesky_decomposition(float A[IMU_UKF_STATE_SIZE][IMU_UKF_STATE_SIZE], 
                                             float L[IMU_UKF_STATE_SIZE][IMU_UKF_STATE_SIZE]);

//=================================================�ⲿ�ӿ�ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ʼ��IMU��̬����ϵͳ
//-------------------------------------------------------------------------------------------------------------------
imu_status_enum imu_attitude_init(imu_calc_mode_enum calc_mode)
{
    // ���ϵͳ�ṹ��
    memset(&imu_attitude_system, 0, sizeof(imu_attitude_system_t));
    
    // ���ó�ʼ����
    imu_attitude_system.calc_mode = calc_mode;
    imu_attitude_system.complementary_alpha = IMU_COMPLEMENTARY_ALPHA;
    imu_attitude_system.mahony_kp = IMU_MAHONY_KP;
    imu_attitude_system.mahony_ki = IMU_MAHONY_KI;
    
    // ���ó�ʼ�ο��������ٶȣ�У׼�����£�
    imu_attitude_system.calibration.reference_gravity = IMU_GRAVITY_ACCEL;
    
    // ��ʼ����Ԫ��Ϊ��λ��Ԫ��
    imu_attitude_system.processed_data.quaternion.w = 1.0f;
    imu_attitude_system.processed_data.quaternion.x = 0.0f;
    imu_attitude_system.processed_data.quaternion.y = 0.0f;
    imu_attitude_system.processed_data.quaternion.z = 0.0f;
    
    // ��ʼ���˲���
    if(calc_mode == IMU_MODE_KALMAN)
    {
        imu_kalman_init();
        printf("�������˲����ѳ�ʼ��\r\n");
    }
    else if(calc_mode == IMU_MODE_MADGWICK)
    {
        imu_madgwick_init();
        printf("Madgwick�˲����ѳ�ʼ��\r\n");
    }
    else if(calc_mode == IMU_MODE_UKF)
    {
        imu_ukf_init();
        printf("UKF�˲����ѳ�ʼ��\r\n");
    }
    
    // ��ʼ����ʱ��
    imu_init_timer();
    
    // ��ʼ��������
    if(imu_init_sensor() != IMU_STATUS_OK)
    {
        printf("SCH16TK10��������ʼ��ʧ��\r\n");
        imu_attitude_system.status = IMU_STATUS_ERROR;
        return IMU_STATUS_ERROR;
    }
    
    imu_attitude_system.status = IMU_STATUS_OK;
    imu_attitude_system.system_initialized = 1;
    imu_attitude_system.last_update_time = imu_get_time_ms();
    
    printf("IMU��̬����ϵͳ��ʼ���ɹ�\r\n");
    return IMU_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����IMU���������ݣ�֧���ֶ�������Զ��ɼ���
//-------------------------------------------------------------------------------------------------------------------
imu_status_enum imu_update_sensor_data(float gyro_x, float gyro_y, float gyro_z, 
                                       float accel_x, float accel_y, float accel_z)
{
    if(imu_attitude_system.status == IMU_STATUS_NOT_INIT)
        return IMU_STATUS_NOT_INIT;
    
    // ������Ч�Լ�飺���NaN��Inf
    if(isnan(gyro_x) || isnan(gyro_y) || isnan(gyro_z) ||
       isnan(accel_x) || isnan(accel_y) || isnan(accel_z) ||
       isinf(gyro_x) || isinf(gyro_y) || isinf(gyro_z) ||
       isinf(accel_x) || isinf(accel_y) || isinf(accel_z))
    {
        printf("�����������쳣(NaN/Inf): gyro(%.3f,%.3f,%.3f) accel(%.3f,%.3f,%.3f)\r\n",
               gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z);
        imu_attitude_system.raw_data.data_valid = 0;
        return IMU_STATUS_DATA_INVALID;
    }
    
    // �����Լ�飺�����Ƿ�Χ��2000 deg/s = ��34.9 rad/s
    if(fabs(gyro_x) > 35.0f || fabs(gyro_y) > 35.0f || fabs(gyro_z) > 35.0f)
    {
        printf("���������ݳ�����Χ: gyro(%.3f,%.3f,%.3f)\r\n", gyro_x, gyro_y, gyro_z);
        imu_attitude_system.raw_data.data_valid = 0;
        return IMU_STATUS_DATA_INVALID;
    }
    
    // �����Լ�飺���ٶȼƷ�Χ��16g = ��156.8 m/s?
    if(fabs(accel_x) > 156.8f || fabs(accel_y) > 156.8f || fabs(accel_z) > 156.8f)
    {
        printf("���ٶȼ����ݳ�����Χ: accel(%.3f,%.3f,%.3f)\r\n", accel_x, accel_y, accel_z);
        imu_attitude_system.raw_data.data_valid = 0;
        return IMU_STATUS_DATA_INVALID;
    }
    
    // ����ԭʼ����
    imu_attitude_system.raw_data.gyro_raw.x = gyro_x;
    imu_attitude_system.raw_data.gyro_raw.y = gyro_y;
    imu_attitude_system.raw_data.gyro_raw.z = gyro_z;
    
    imu_attitude_system.raw_data.accel_raw.x = accel_x;
    imu_attitude_system.raw_data.accel_raw.y = accel_y;
    imu_attitude_system.raw_data.accel_raw.z = accel_z;
    
    imu_attitude_system.raw_data.data_valid = 1;
    imu_attitude_system.raw_data.timestamp = imu_get_time_ms();
    
    return IMU_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ִ��IMU��̬�������
//-------------------------------------------------------------------------------------------------------------------
imu_status_enum imu_attitude_update(void)
{
    if(!imu_attitude_system.system_initialized)
        return IMU_STATUS_NOT_INIT;
        
    if(!imu_attitude_system.raw_data.data_valid)
        return IMU_STATUS_DATA_INVALID;
    
    // ����ʱ����
    imu_update_dt();
    
    // Ӧ��У׼����
    imu_apply_calibration();
    
    // ���ݼ���ģʽѡ���㷨
    imu_status_enum status = IMU_STATUS_OK;
    switch(imu_attitude_system.calc_mode)
    {
        case IMU_MODE_COMPLEMENTARY:
            status = imu_complementary_filter_update();
            break;
            
        case IMU_MODE_MAHONY:
            status = imu_mahony_filter_update();
            break;
            
        case IMU_MODE_SIMPLE_GYRO:
            status = imu_simple_gyro_integration();
            break;
            
        case IMU_MODE_KALMAN:
            status = imu_kalman_filter_update();
            break;
            
        case IMU_MODE_MADGWICK:
            status = imu_madgwick_filter_update();
            break;
            
        case IMU_MODE_UKF:
            status = imu_ukf_filter_update();
            break;
            
        default:
            return IMU_STATUS_ERROR;
    }
    
    // ת����Ԫ����ŷ����
    imu_quaternion_to_euler(imu_attitude_system.processed_data.quaternion, 
                           &imu_attitude_system.processed_data.euler_angles);
    
    return status;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ��̬���ݣ�ŷ���Ǻ���Ԫ����
//-------------------------------------------------------------------------------------------------------------------
imu_status_enum imu_get_attitude_data(imu_euler_t *euler, imu_quaternion_t *quaternion)
{
    if(!imu_attitude_system.system_initialized)
        return IMU_STATUS_ERROR;
    
    // ��ȡŷ���ǣ���ΪNULL��
    if(euler != NULL)
        *euler = imu_attitude_system.processed_data.euler_angles;
    
    // ��ȡ��Ԫ������ΪNULL��
    if(quaternion != NULL)
        *quaternion = imu_attitude_system.processed_data.quaternion;
    
    return IMU_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����IMUУ׼
//-------------------------------------------------------------------------------------------------------------------
imu_status_enum imu_start_calibration(uint32 calibration_time)
{
    if(!imu_attitude_system.system_initialized)
        return IMU_STATUS_NOT_INIT;
    
    // ����У׼����
    imu_attitude_system.calibration.calibration_enabled = 1;
    imu_attitude_system.calibration.calibration_time = calibration_time;
    imu_attitude_system.calibration.calibration_start_time = imu_get_time_ms();
    imu_attitude_system.calibration.calibration_samples = 0;
    
    // �����ۼ�ֵ
    memset(&imu_attitude_system.calibration.gyro_sum, 0, sizeof(imu_vector3_t));
    memset(&imu_attitude_system.calibration.accel_sum, 0, sizeof(imu_vector3_t));
    
    imu_attitude_system.status = IMU_STATUS_CALIBRATING;
    
    printf("IMUУ׼��ʼ���뱣���豸��ֹ %d ��\r\n", calibration_time/1000);
    return IMU_STATUS_CALIBRATING;
}


//-------------------------------------------------------------------------------------------------------------------
// �������     ������̬����
//-------------------------------------------------------------------------------------------------------------------
imu_status_enum imu_attitude_reset(void)
{
    // ������Ԫ��Ϊ��λ��Ԫ��
    imu_attitude_system.processed_data.quaternion.w = 1.0f;
    imu_attitude_system.processed_data.quaternion.x = 0.0f;
    imu_attitude_system.processed_data.quaternion.y = 0.0f;
    imu_attitude_system.processed_data.quaternion.z = 0.0f;
    
    // ����ŷ����
    memset(&imu_attitude_system.processed_data.euler_angles, 0, sizeof(imu_euler_t));
    
    // ���û������
    memset(&imu_attitude_system.integral_error, 0, sizeof(imu_vector3_t));
    
    return IMU_STATUS_OK;
}



//-------------------------------------------------------------------------------------------------------------------
// �������     ��SCH16TK10�������Զ��ɼ����� (������1ms�ж��е���)
// ��ע��Ϣ     ʵ��AVG_FACTOR�β���ƽ������������
//-------------------------------------------------------------------------------------------------------------------
imu_status_enum imu_update_from_sensor(void)
{
    if(!sch1_initialized || !imu_attitude_system.system_initialized)
        return IMU_STATUS_NOT_INIT;
        
    // ��ȡSCH16TK10ԭʼ����
    SCH1_getData(&sch1_raw_data);
    
    // �������֡�Ƿ��д���
    if (sch1_raw_data.frame_error)
    {
        return IMU_STATUS_DATA_INVALID;
    }
    
    // �ۼ�ԭʼ����
    sch1_summed_data.Rate1_raw[AXIS_X] += sch1_raw_data.Rate1_raw[AXIS_X];
    sch1_summed_data.Rate1_raw[AXIS_Y] += sch1_raw_data.Rate1_raw[AXIS_Y];
    sch1_summed_data.Rate1_raw[AXIS_Z] += sch1_raw_data.Rate1_raw[AXIS_Z];
    
    sch1_summed_data.Rate2_raw[AXIS_X] += sch1_raw_data.Rate2_raw[AXIS_X];
    sch1_summed_data.Rate2_raw[AXIS_Y] += sch1_raw_data.Rate2_raw[AXIS_Y];
    sch1_summed_data.Rate2_raw[AXIS_Z] += sch1_raw_data.Rate2_raw[AXIS_Z];
    
    sch1_summed_data.Acc1_raw[AXIS_X] += sch1_raw_data.Acc1_raw[AXIS_X];
    sch1_summed_data.Acc1_raw[AXIS_Y] += sch1_raw_data.Acc1_raw[AXIS_Y];
    sch1_summed_data.Acc1_raw[AXIS_Z] += sch1_raw_data.Acc1_raw[AXIS_Z];
    
    sch1_summed_data.Acc2_raw[AXIS_X] += sch1_raw_data.Acc2_raw[AXIS_X];
    sch1_summed_data.Acc2_raw[AXIS_Y] += sch1_raw_data.Acc2_raw[AXIS_Y];
    sch1_summed_data.Acc2_raw[AXIS_Z] += sch1_raw_data.Acc2_raw[AXIS_Z];
    
    sch1_summed_data.Acc3_raw[AXIS_X] += sch1_raw_data.Acc3_raw[AXIS_X];
    sch1_summed_data.Acc3_raw[AXIS_Y] += sch1_raw_data.Acc3_raw[AXIS_Y];
    sch1_summed_data.Acc3_raw[AXIS_Z] += sch1_raw_data.Acc3_raw[AXIS_Z];
    
    sch1_summed_data.Temp_raw += sch1_raw_data.Temp_raw;
    
    // ������������
    sch1_sample_count++;
    
    // ���ۼӴ����ﵽAVG_FACTORʱ��ת�����ݲ�������̬
    if (sch1_sample_count >= AVG_FACTOR)
    {
        // ת��Ϊ���������Զ�����AVG_FACTOR����ƽ����
        SCH1_convert_data(&sch1_summed_data, &sch1_result_data);
        
        // ��SCH16TK10���ݸ��µ�IMU��̬����ģ��
        // SCH16TK10: Rate1Ϊ���ٶ�(rad/s), Acc2Ϊ���ٶ�(m/s?)
        imu_status_enum status = imu_update_sensor_data(
            sch1_result_data.Rate1[AXIS_X],    // ������X�� (rad/s)
            sch1_result_data.Rate1[AXIS_Y],    // ������Y�� (rad/s) 
            sch1_result_data.Rate1[AXIS_Z],    // ������Z�� (rad/s)
            sch1_result_data.Acc2[AXIS_X],     // ���ٶȼ�X�� (m/s?)
            sch1_result_data.Acc2[AXIS_Y],     // ���ٶȼ�Y�� (m/s?)
            sch1_result_data.Acc2[AXIS_Z]      // ���ٶȼ�Z�� (m/s?)
        );
        
        // ���ò����������ۼӻ�����
        sch1_sample_count = 0;
        memset(&sch1_summed_data, 0, sizeof(SCH1_raw_data));
        
        return status;
    }
    
    // ��δ�ۼ��㹻����������OK����������̬
    return IMU_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ʼ��SCH16TK10������
//-------------------------------------------------------------------------------------------------------------------
imu_status_enum imu_init_sensor(void)
{
    printf("���ڳ�ʼ��SCH16TK10������...\r\n");
    
    // ����SCH16TK10������ʹ��config_infineon.h�е����ã�
    SCH1_filter sFilter;
    SCH1_sensitivity sSensitivity;
    SCH1_decimation sDecimation;

    // �����˲������� (Hz)
    sFilter.Rate12 = FILTER_RATE;    // �������˲�Ƶ��
    sFilter.Acc12 = FILTER_ACC12;    // ���ٶȼ�1,2�˲�Ƶ��
    sFilter.Acc3 = FILTER_ACC3;      // ���ٶȼ�3�˲�Ƶ��

    // ���������Ȳ��� (LSB/unit)
    sSensitivity.Rate1 = SENSITIVITY_RATE1;   // ������1������
    sSensitivity.Rate2 = SENSITIVITY_RATE2;   // ������2������
    sSensitivity.Acc1 = SENSITIVITY_ACC1;     // ���ٶȼ�1������
    sSensitivity.Acc2 = SENSITIVITY_ACC2;     // ���ٶȼ�2������
    sSensitivity.Acc3 = SENSITIVITY_ACC3;     // ���ٶȼ�3������

    // ���ó�ȡ����
    sDecimation.Rate2 = DECIMATION_RATE;   // ������2��ȡ��
    sDecimation.Acc2 = DECIMATION_ACC;     // ���ٶȼ�2��ȡ��

    // ��ʼ��������
    int init_result = SCH1_init(sFilter, sSensitivity, sDecimation, false);
    
    if (init_result != SCH1_OK)
    {
        printf("SCH16TK10��ʼ��ʧ�ܣ�������: %d\r\n", init_result);
        
        // ��ȡ״̬�Ĵ����������
        SCH1_status status;
        if (SCH1_getStatus(&status) == SCH1_OK)
        {
            printf("״̬��� - Summary: 0x%04X, Common: 0x%04X\r\n", 
                   status.Summary, status.Common);
        }
        return IMU_STATUS_ERROR;
    }
    
    printf("SCH16TK10��������ʼ���ɹ���\r\n");
    
    // ����SPIͨ���ȶ���
    if (SCH1_testSPIStability())
    {
        printf("SPIͨ�Ų���ͨ��\r\n");
    }
    else
    {
        printf("SPIͨ�Ų���ʧ�ܣ�����������\r\n");
    }
    
    sch1_initialized = 1;
    return IMU_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ�������˲���״̬��Ϣ
//-------------------------------------------------------------------------------------------------------------------
imu_kalman_filter_t* imu_get_kalman_info(void)
{
    if(imu_attitude_system.calc_mode == IMU_MODE_KALMAN && 
       imu_attitude_system.kalman.initialized)
    {
        return &imu_attitude_system.kalman;
    }
    return NULL;
}

//=================================================�ڲ�����ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     Ӧ��У׼����
//-------------------------------------------------------------------------------------------------------------------
static imu_status_enum imu_apply_calibration(void)
{
    // ����Ƿ���У׼������
    if(imu_attitude_system.calibration.calibration_enabled)
    {
        uint32 current_time = imu_get_time_ms();
        uint32 elapsed_time = current_time - imu_attitude_system.calibration.calibration_start_time;
        
        if(elapsed_time < imu_attitude_system.calibration.calibration_time)
        {
            // У׼�ڼ䣬�ۼƴ���������
            imu_attitude_system.calibration.gyro_sum.x += imu_attitude_system.raw_data.gyro_raw.x;
            imu_attitude_system.calibration.gyro_sum.y += imu_attitude_system.raw_data.gyro_raw.y;
            imu_attitude_system.calibration.gyro_sum.z += imu_attitude_system.raw_data.gyro_raw.z;
            
            imu_attitude_system.calibration.accel_sum.x += imu_attitude_system.raw_data.accel_raw.x;
            imu_attitude_system.calibration.accel_sum.y += imu_attitude_system.raw_data.accel_raw.y;
            imu_attitude_system.calibration.accel_sum.z += imu_attitude_system.raw_data.accel_raw.z;
            
            imu_attitude_system.calibration.calibration_samples++;
            return IMU_STATUS_CALIBRATING;
        }
        else
        {
            // У׼��ɣ�������ƫ
            if(imu_attitude_system.calibration.calibration_samples > 0)
            {
                float samples = (float)imu_attitude_system.calibration.calibration_samples;
                
                imu_attitude_system.raw_data.gyro_bias.x = imu_attitude_system.calibration.gyro_sum.x / samples;
                imu_attitude_system.raw_data.gyro_bias.y = imu_attitude_system.calibration.gyro_sum.y / samples;
                imu_attitude_system.raw_data.gyro_bias.z = imu_attitude_system.calibration.gyro_sum.z / samples;
                
                // ���㾲ֹʱ��ƽ�����ٶ�
                float accel_avg_x = imu_attitude_system.calibration.accel_sum.x / samples;
                float accel_avg_y = imu_attitude_system.calibration.accel_sum.y / samples;
                float accel_avg_z = imu_attitude_system.calibration.accel_sum.z / samples;
                
                // ���㾲ֹʱ�ļ��ٶȷ�ֵ��Ϊ�ο��������ٶ�
                imu_attitude_system.calibration.reference_gravity = 
                    sqrtf(accel_avg_x * accel_avg_x + accel_avg_y * accel_avg_y + accel_avg_z * accel_avg_z);
                
                // ���ٶȼ�У׼������У׼�ڼ��豸��ֹ��������ƫ
                imu_attitude_system.raw_data.accel_bias.x = accel_avg_x;
                imu_attitude_system.raw_data.accel_bias.y = accel_avg_y;
                imu_attitude_system.raw_data.accel_bias.z = accel_avg_z - imu_attitude_system.calibration.reference_gravity;
                
                printf("IMUУ׼��ɣ�\r\n");
                printf("��������ƫ: [%.4f, %.4f, %.4f] rad/s\r\n", 
                       imu_attitude_system.raw_data.gyro_bias.x,
                       imu_attitude_system.raw_data.gyro_bias.y,
                       imu_attitude_system.raw_data.gyro_bias.z);
                printf("���ٶȼ���ƫ: [%.4f, %.4f, %.4f] m/s?\r\n",
                       imu_attitude_system.raw_data.accel_bias.x,
                       imu_attitude_system.raw_data.accel_bias.y,
                       imu_attitude_system.raw_data.accel_bias.z);
                printf("�ο��������ٶ�: %.4f m/s? (����ӦУ׼)\r\n",
                       imu_attitude_system.calibration.reference_gravity);
            }
            
            imu_attitude_system.calibration.calibration_enabled = 0;
            imu_attitude_system.status = IMU_STATUS_OK;
        }
    }
    
    // Ӧ��У׼����
    imu_attitude_system.processed_data.gyro_filtered.x = 
        imu_attitude_system.raw_data.gyro_raw.x - imu_attitude_system.raw_data.gyro_bias.x;
    imu_attitude_system.processed_data.gyro_filtered.y = 
        imu_attitude_system.raw_data.gyro_raw.y - imu_attitude_system.raw_data.gyro_bias.y;
    imu_attitude_system.processed_data.gyro_filtered.z = 
        imu_attitude_system.raw_data.gyro_raw.z - imu_attitude_system.raw_data.gyro_bias.z;
        
    imu_attitude_system.processed_data.accel_filtered.x = 
        imu_attitude_system.raw_data.accel_raw.x - imu_attitude_system.raw_data.accel_bias.x;
    imu_attitude_system.processed_data.accel_filtered.y = 
        imu_attitude_system.raw_data.accel_raw.y - imu_attitude_system.raw_data.accel_bias.y;
    imu_attitude_system.processed_data.accel_filtered.z = 
        imu_attitude_system.raw_data.accel_raw.z - imu_attitude_system.raw_data.accel_bias.z;
    
    return IMU_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     �����˲�������
//-------------------------------------------------------------------------------------------------------------------
static imu_status_enum imu_complementary_filter_update(void)
{
    float alpha = imu_attitude_system.complementary_alpha;
    float dt = imu_attitude_system.processed_data.dt;
    
    // ��ȡ�����Ĵ���������
    imu_vector3_t *gyro = &imu_attitude_system.processed_data.gyro_filtered;
    imu_vector3_t *accel = &imu_attitude_system.processed_data.accel_filtered;
    
    // ���ٶȼƼ�����ĸ����Ǻ͹�ת��
    float acc_pitch = atan2f(accel->x, sqrtf(accel->y * accel->y + accel->z * accel->z));
    float acc_roll = atan2f(-accel->y, accel->z);
    
    // �����ǻ���
    imu_attitude_system.processed_data.euler_angles.pitch_deg += gyro->x * dt * 180.0f / M_PI;
    imu_attitude_system.processed_data.euler_angles.roll_deg += gyro->y * dt * 180.0f / M_PI;
    imu_attitude_system.processed_data.euler_angles.yaw_deg += gyro->z * dt * 180.0f / M_PI;
    
    // �����˲����ں�
    imu_attitude_system.processed_data.euler_angles.pitch_deg = 
        alpha * imu_attitude_system.processed_data.euler_angles.pitch_deg + 
        (1.0f - alpha) * (acc_pitch * 180.0f / M_PI);
        
    imu_attitude_system.processed_data.euler_angles.roll_deg = 
        alpha * imu_attitude_system.processed_data.euler_angles.roll_deg + 
        (1.0f - alpha) * (acc_roll * 180.0f / M_PI);
    
    // �Ƕȹ�һ��
    imu_attitude_system.processed_data.euler_angles.pitch_deg = 
        imu_normalize_angle(imu_attitude_system.processed_data.euler_angles.pitch_deg);
    imu_attitude_system.processed_data.euler_angles.roll_deg = 
        imu_normalize_angle(imu_attitude_system.processed_data.euler_angles.roll_deg);
    imu_attitude_system.processed_data.euler_angles.yaw_deg = 
        imu_normalize_angle(imu_attitude_system.processed_data.euler_angles.yaw_deg);
    
    // ������Ԫ��
    imu_euler_to_quaternion(imu_attitude_system.processed_data.euler_angles,
                           &imu_attitude_system.processed_data.quaternion);
    
    return IMU_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     Mahony�˲�������
//-------------------------------------------------------------------------------------------------------------------
static imu_status_enum imu_mahony_filter_update(void)
{
    float dt = imu_attitude_system.processed_data.dt;
    float kp = imu_attitude_system.mahony_kp;
    float ki = imu_attitude_system.mahony_ki;
    
    // ��ȡ�����Ĵ���������
    imu_vector3_t *gyro = &imu_attitude_system.processed_data.gyro_filtered;
    imu_vector3_t *accel = &imu_attitude_system.processed_data.accel_filtered;
    
    // ��һ�����ٶȼ�����
    float acc_norm = sqrtf(accel->x * accel->x + accel->y * accel->y + accel->z * accel->z);
    if (acc_norm > 0.0f)
    {
        accel->x /= acc_norm;
        accel->y /= acc_norm;
        accel->z /= acc_norm;
    }
    
    // ��ǰ��Ԫ��
    imu_quaternion_t *q = &imu_attitude_system.processed_data.quaternion;
    
    // ����Ԫ��������������
    float gx = 2.0f * (q->x * q->z - q->w * q->y);
    float gy = 2.0f * (q->w * q->x + q->y * q->z);
    float gz = q->w * q->w - q->x * q->x - q->y * q->y + q->z * q->z;
    
    // ������������ٶȼƷ������������Ʒ���Ĳ��
    float ex = (accel->y * gz - accel->z * gy);
    float ey = (accel->z * gx - accel->x * gz);
    float ez = (accel->x * gy - accel->y * gx);
    
    // �������
    if (ki > 0.0f)
    {
        imu_attitude_system.integral_error.x += ex * dt;
        imu_attitude_system.integral_error.y += ey * dt;
        imu_attitude_system.integral_error.z += ez * dt;
    }
    else
    {
        imu_attitude_system.integral_error.x = 0.0f;
        imu_attitude_system.integral_error.y = 0.0f;
        imu_attitude_system.integral_error.z = 0.0f;
    }
    
    // У���������������
    float gx_corrected = gyro->x + kp * ex + ki * imu_attitude_system.integral_error.x;
    float gy_corrected = gyro->y + kp * ey + ki * imu_attitude_system.integral_error.y;
    float gz_corrected = gyro->z + kp * ez + ki * imu_attitude_system.integral_error.z;
    
    // ��Ԫ��΢�ַ���
    float qw_dot = 0.5f * (-q->x * gx_corrected - q->y * gy_corrected - q->z * gz_corrected);
    float qx_dot = 0.5f * (q->w * gx_corrected + q->y * gz_corrected - q->z * gy_corrected);
    float qy_dot = 0.5f * (q->w * gy_corrected - q->x * gz_corrected + q->z * gx_corrected);
    float qz_dot = 0.5f * (q->w * gz_corrected + q->x * gy_corrected - q->y * gx_corrected);
    
    // ���ָ�����Ԫ��
    q->w += qw_dot * dt;
    q->x += qx_dot * dt;
    q->y += qy_dot * dt;
    q->z += qz_dot * dt;
    
    // ��һ����Ԫ��
    imu_quaternion_normalize(q);
    
    return IMU_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     �������ǻ���
//-------------------------------------------------------------------------------------------------------------------
static imu_status_enum imu_simple_gyro_integration(void)
{
    float dt = imu_attitude_system.processed_data.dt;
    imu_vector3_t *gyro = &imu_attitude_system.processed_data.gyro_filtered;
    
    // ֱ�ӻ�������������
    imu_attitude_system.processed_data.euler_angles.pitch_deg += gyro->x * dt * 180.0f / M_PI;
    imu_attitude_system.processed_data.euler_angles.roll_deg += gyro->y * dt * 180.0f / M_PI;
    imu_attitude_system.processed_data.euler_angles.yaw_deg += gyro->z * dt * 180.0f / M_PI;
    
    // �Ƕȹ�һ��
    imu_attitude_system.processed_data.euler_angles.pitch_deg = 
        imu_normalize_angle(imu_attitude_system.processed_data.euler_angles.pitch_deg);
    imu_attitude_system.processed_data.euler_angles.roll_deg = 
        imu_normalize_angle(imu_attitude_system.processed_data.euler_angles.roll_deg);
    imu_attitude_system.processed_data.euler_angles.yaw_deg = 
        imu_normalize_angle(imu_attitude_system.processed_data.euler_angles.yaw_deg);
    
    // ������Ԫ��
    imu_euler_to_quaternion(imu_attitude_system.processed_data.euler_angles,
                           &imu_attitude_system.processed_data.quaternion);
    
    return IMU_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��Ԫ��תŷ����
//-------------------------------------------------------------------------------------------------------------------
static void imu_quaternion_to_euler(imu_quaternion_t q, imu_euler_t *euler)
{
    // ��ת�� (��X����ת)
    float sin_r_cp = 2.0f * (q.w * q.x + q.y * q.z);
    float cos_r_cp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    euler->roll_deg = atan2f(sin_r_cp, cos_r_cp) * 180.0f / M_PI;

    // ������ (��Y����ת)
    float sin_p = 2.0f * (q.w * q.y - q.z * q.x);
    if (fabsf(sin_p) >= 1.0f)
        euler->pitch_deg = copysignf(90.0f, sin_p); // ���������
    else
        euler->pitch_deg = asinf(sin_p) * 180.0f / M_PI;

    // ƫ���� (��Z����ת)
    float sin_y_cp = 2.0f * (q.w * q.z + q.x * q.y);
    float cos_y_cp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    euler->yaw_deg = atan2f(sin_y_cp, cos_y_cp) * 180.0f / M_PI;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ŷ����ת��Ԫ��
//-------------------------------------------------------------------------------------------------------------------
static void imu_euler_to_quaternion(imu_euler_t euler, imu_quaternion_t *q)
{
    float roll_rad = euler.roll_deg * M_PI / 180.0f;
    float pitch_rad = euler.pitch_deg * M_PI / 180.0f;
    float yaw_rad = euler.yaw_deg * M_PI / 180.0f;
    
    float cr = cosf(roll_rad * 0.5f);
    float sr = sinf(roll_rad * 0.5f);
    float cp = cosf(pitch_rad * 0.5f);
    float sp = sinf(pitch_rad * 0.5f);
    float cy = cosf(yaw_rad * 0.5f);
    float sy = sinf(yaw_rad * 0.5f);

    q->w = cr * cp * cy + sr * sp * sy;
    q->x = sr * cp * cy - cr * sp * sy;
    q->y = cr * sp * cy + sr * cp * sy;
    q->z = cr * cp * sy - sr * sp * cy;
}


//-------------------------------------------------------------------------------------------------------------------
// �������     ��Ԫ����һ��
//-------------------------------------------------------------------------------------------------------------------
static void imu_quaternion_normalize(imu_quaternion_t *q)
{
    float norm = sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    if (norm > 0.0f)
    {
        float inv_norm = 1.0f / norm;
        q->w *= inv_norm;
        q->x *= inv_norm;
        q->y *= inv_norm;
        q->z *= inv_norm;
    }
}


//-------------------------------------------------------------------------------------------------------------------
// �������     �Ƕȹ�һ��
//-------------------------------------------------------------------------------------------------------------------
static float imu_normalize_angle(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����ʱ����
//-------------------------------------------------------------------------------------------------------------------
static void imu_update_dt(void)
{
    uint32 current_time = imu_get_time_ms();
    uint32 dt_ms = current_time - imu_attitude_system.last_update_time;
    
    if (dt_ms > 0)
    {
        imu_attitude_system.processed_data.dt = (float)dt_ms / 1000.0f;
        imu_attitude_system.last_update_time = current_time;
    }
    else
    {
        imu_attitude_system.processed_data.dt = 1.0f / IMU_ATTITUDE_UPDATE_FREQ;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ��ǰʱ�䣨���룩
//-------------------------------------------------------------------------------------------------------------------
static uint32 imu_get_time_ms(void)
{
    if(!imu_timer_initialized)
    {
        imu_init_timer();
    }
    return timer_get(TC_TIME2_CH1);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ʼ��ʱ�������ʱ��
//-------------------------------------------------------------------------------------------------------------------
static void imu_init_timer(void)
{
    if(!imu_timer_initialized)
    {
        timer_init(TC_TIME2_CH1, TIMER_MS);
        timer_clear(TC_TIME2_CH1);
        timer_start(TC_TIME2_CH1);
        imu_timer_initialized = 1;
    }
}

//=================================================�������˲���ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ʼ���������˲���
//-------------------------------------------------------------------------------------------------------------------
static void imu_kalman_init(void)
{
    imu_kalman_filter_t *kf = &imu_attitude_system.kalman;
    
    // ������о���
    memset(kf, 0, sizeof(imu_kalman_filter_t));
    
    // ��ʼ��״̬���� [q0, q1, q2, q3, bx, by, bz]
    kf->state[0] = 1.0f;  // q0 (w)
    kf->state[1] = 0.0f;  // q1 (x)
    kf->state[2] = 0.0f;  // q2 (y)
    kf->state[3] = 0.0f;  // q3 (z)
    kf->state[4] = 0.0f;  // bias_x
    kf->state[5] = 0.0f;  // bias_y
    kf->state[6] = 0.0f;  // bias_z
    
    // ��ʼ��״̬Э������� P (�Խ���Ԫ��)
    for(int i = 0; i < IMU_KALMAN_STATE_SIZE; i++)
    {
        for(int j = 0; j < IMU_KALMAN_STATE_SIZE; j++)
        {
            if(i == j)
            {
                if(i < 4)  // ��Ԫ������
                    kf->P[i][j] = 0.1f;
                else       // ������ƫ���
                    kf->P[i][j] = 0.01f;
            }
            else
            {
                kf->P[i][j] = 0.0f;
            }
        }
    }
    
    // ��ʼ����������Э������� Q
    for(int i = 0; i < IMU_KALMAN_STATE_SIZE; i++)
    {
        for(int j = 0; j < IMU_KALMAN_STATE_SIZE; j++)
        {
            if(i == j)
            {
                if(i < 4)  // ��Ԫ����������
                    kf->Q[i][j] = IMU_KALMAN_Q_GYRO;
                else       // ������ƫ���������
                    kf->Q[i][j] = IMU_KALMAN_Q_BIAS;
            }
            else
            {
                kf->Q[i][j] = 0.0f;
            }
        }
    }
    
    // ��ʼ����������Э������� R
    for(int i = 0; i < IMU_KALMAN_MEASUREMENT_SIZE; i++)
    {
        for(int j = 0; j < IMU_KALMAN_MEASUREMENT_SIZE; j++)
        {
            if(i == j)
                kf->R[i][j] = IMU_KALMAN_R_ACCEL;
            else
                kf->R[i][j] = 0.0f;
        }
    }
    
    kf->initialized = 1;
    kf->update_count = 0;
    
    printf("�������˲�����ʼ�����\r\n");
}

//-------------------------------------------------------------------------------------------------------------------
// �������     �������˲��������º���
//-------------------------------------------------------------------------------------------------------------------
static imu_status_enum imu_kalman_filter_update(void)
{
    imu_kalman_filter_t *kf = &imu_attitude_system.kalman;
    
    if(!kf->initialized)
    {
        imu_kalman_init();
    }
    
    float dt = imu_attitude_system.processed_data.dt;
    imu_vector3_t *gyro = &imu_attitude_system.processed_data.gyro_filtered;
    imu_vector3_t *accel = &imu_attitude_system.processed_data.accel_filtered;
    
    // 1. Ԥ�ⲽ��
    imu_kalman_predict(dt, gyro);
    
    // 2. ���²��裨ʹ�ü��ٶȼ����ݣ�
    float accel_norm = sqrtf(accel->x * accel->x + accel->y * accel->y + accel->z * accel->z);
    // ֻ�ڼ��ٶȷ�ֵ�ӽ��ο��������ٶ�ʱ��ʹ�ü��ٶȼ�У�� (�����50%��ƫ��)
    float accel_min = imu_attitude_system.calibration.reference_gravity * 0.5f;
    float accel_max = imu_attitude_system.calibration.reference_gravity * 1.5f;
    if(accel_norm >= accel_min && accel_norm <= accel_max)
    {
        imu_kalman_update_accel(accel);
    }
    
    // 3. ���������Ԫ��
    imu_attitude_system.processed_data.quaternion.w = kf->state[0];
    imu_attitude_system.processed_data.quaternion.x = kf->state[1];
    imu_attitude_system.processed_data.quaternion.y = kf->state[2];
    imu_attitude_system.processed_data.quaternion.z = kf->state[3];
    
    // 4. ��һ����Ԫ��
    imu_quaternion_normalize(&imu_attitude_system.processed_data.quaternion);
    
    // 5. ����״̬�����е���Ԫ��������һ���ԣ�
    kf->state[0] = imu_attitude_system.processed_data.quaternion.w;
    kf->state[1] = imu_attitude_system.processed_data.quaternion.x;
    kf->state[2] = imu_attitude_system.processed_data.quaternion.y;
    kf->state[3] = imu_attitude_system.processed_data.quaternion.z;
    
    kf->update_count++;
    
    return IMU_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     �������˲���Ԥ�ⲽ��
//-------------------------------------------------------------------------------------------------------------------
static void imu_kalman_predict(float dt, imu_vector3_t *gyro)
{
    imu_kalman_filter_t *kf = &imu_attitude_system.kalman;
    
    // ��ȡ��ǰ״̬
    float q0 = kf->state[0], q1 = kf->state[1], q2 = kf->state[2], q3 = kf->state[3];
    float bx = kf->state[4], by = kf->state[5], bz = kf->state[6];
    
    // У���������������
    float wx = gyro->x - bx;
    float wy = gyro->y - by;
    float wz = gyro->z - bz;
    
    // ״̬ת�Ʒ���: ��Ԫ������
    // q_new = q_old + 0.5 * dt * ��(��) * q_old
    float dq0 = 0.5f * dt * (-q1 * wx - q2 * wy - q3 * wz);
    float dq1 = 0.5f * dt * (q0 * wx - q3 * wy + q2 * wz);
    float dq2 = 0.5f * dt * (q3 * wx + q0 * wy - q1 * wz);
    float dq3 = 0.5f * dt * (-q2 * wx + q1 * wy + q0 * wz);
    
    // ����״̬����
    kf->state[0] += dq0;  // q0
    kf->state[1] += dq1;  // q1
    kf->state[2] += dq2;  // q2
    kf->state[3] += dq3;  // q3
    // ƫ��ֲ���
    // kf->state[4] = bx;  // bias_x
    // kf->state[5] = by;  // bias_y
    // kf->state[6] = bz;  // bias_z
    
    // ��һ����Ԫ������
    float q_norm = sqrtf(kf->state[0]*kf->state[0] + kf->state[1]*kf->state[1] + 
                        kf->state[2]*kf->state[2] + kf->state[3]*kf->state[3]);
    if(q_norm > 0.0f)
    {
        kf->state[0] /= q_norm;
        kf->state[1] /= q_norm;
        kf->state[2] /= q_norm;
        kf->state[3] /= q_norm;
    }
    
    // ״̬Э����Ԥ��: P = F*P*F' + Q
    // ����򻯴���ֱ�Ӽӹ�������
    for(int i = 0; i < IMU_KALMAN_STATE_SIZE; i++)
    {
        kf->P[i][i] += kf->Q[i][i] * dt;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     �������˲������ٶȼƸ��²���
//-------------------------------------------------------------------------------------------------------------------
static void imu_kalman_update_accel(imu_vector3_t *accel)
{
    imu_kalman_filter_t *kf = &imu_attitude_system.kalman;
    
    // ��һ�����ٶȼ�����
    float norm = sqrtf(accel->x * accel->x + accel->y * accel->y + accel->z * accel->z);
    float ax = accel->x / norm;
    float ay = accel->y / norm;
    float az = accel->z / norm;
    
    // �ӵ�ǰ��Ԫ��������������
    float q0 = kf->state[0], q1 = kf->state[1], q2 = kf->state[2], q3 = kf->state[3];
    
    // Ԥ����������� (body frame)
    float gx_pred = 2.0f * (q1 * q3 - q0 * q2);
    float gy_pred = 2.0f * (q0 * q1 + q2 * q3);
    float gz_pred = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    
    // ���£��в
    float y[3];
    y[0] = ax - gx_pred;
    y[1] = ay - gy_pred;
    y[2] = az - gz_pred;
    
    // �۲���� H���ſɱȾ���
    // H = d(h)/d(x), ���� h �ǹ۲⺯����x ��״̬����
    memset(kf->H, 0, sizeof(kf->H));
    
    kf->H[0][0] = -2.0f * q2;  // dh1/dq0
    kf->H[0][1] = 2.0f * q3;   // dh1/dq1
    kf->H[0][2] = -2.0f * q0;  // dh1/dq2
    kf->H[0][3] = 2.0f * q1;   // dh1/dq3
    
    kf->H[1][0] = 2.0f * q1;   // dh2/dq0
    kf->H[1][1] = 2.0f * q0;   // dh2/dq1
    kf->H[1][2] = 2.0f * q3;   // dh2/dq2
    kf->H[1][3] = 2.0f * q2;   // dh2/dq3
    
    kf->H[2][0] = 2.0f * q0;   // dh3/dq0
    kf->H[2][1] = -2.0f * q1;  // dh3/dq1
    kf->H[2][2] = -2.0f * q2;  // dh3/dq2
    kf->H[2][3] = 2.0f * q3;   // dh3/dq3
    
    // ���㴴��Э���� S = H*P*H' + R
    // �򻯼��㣬ֻ���ǶԽ�Ԫ��
    for(int i = 0; i < 3; i++)
    {
        kf->S[i][i] = kf->R[i][i];
        for(int j = 0; j < 4; j++)  // ֻ������Ԫ������
        {
            kf->S[i][i] += kf->H[i][j] * kf->P[j][j] * kf->H[i][j];
        }
    }
    
    // ���㿨�������� K = P*H'*S^(-1)
    // �򻯼���
    for(int i = 0; i < IMU_KALMAN_STATE_SIZE; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            if(i < 4 && kf->S[j][j] > 0.0f)  // ֻ������Ԫ������
            {
                kf->K[i][j] = kf->P[i][i] * kf->H[j][i] / kf->S[j][j];
            }
            else
            {
                kf->K[i][j] = 0.0f;
            }
        }
    }
    
    // ״̬���� x = x + K*y
    for(int i = 0; i < IMU_KALMAN_STATE_SIZE; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            kf->state[i] += kf->K[i][j] * y[j];
        }
    }
    
    // Э������� P = (I - K*H)*P
    // ��Ϊ P = P - K*H*P
    for(int i = 0; i < 4; i++)  // ֻ������Ԫ����ص�Э����
    {
        for(int j = 0; j < 3; j++)
        {
            kf->P[i][i] *= (1.0f - kf->K[i][j] * kf->H[j][i]);
        }
        // ����Э������Сֵ
        if(kf->P[i][i] < 0.001f)
            kf->P[i][i] = 0.001f;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����˷� (�򻯰汾�������ڱ�Ҫ����)
//-------------------------------------------------------------------------------------------------------------------
static void imu_matrix_multiply(float *A, float *B, float *C, int m, int n, int p)
{
    // C = A * B
    // A: m x n, B: n x p, C: m x p
    for(int i = 0; i < m; i++)
    {
        for(int j = 0; j < p; j++)
        {
            C[i * p + j] = 0.0f;
            for(int k = 0; k < n; k++)
            {
                C[i * p + j] += A[i * n + k] * B[k * p + j];
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����ת��
//-------------------------------------------------------------------------------------------------------------------
static void imu_matrix_transpose(float *A, float *AT, int m, int n)
{
    // AT = A^T
    // A: m x n, AT: n x m
    for(int i = 0; i < m; i++)
    {
        for(int j = 0; j < n; j++)
        {
            AT[j * m + i] = A[i * n + j];
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     3x3�������� (���ڴ���Э�������)
//-------------------------------------------------------------------------------------------------------------------
static void imu_matrix_inverse_3x3(float A[3][3], float Ainv[3][3])
{
    float det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
              - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
              + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
    
    if(fabsf(det) < 1e-6f)
    {
        // �������죬ʹ�õ�λ����
        memset(Ainv, 0, sizeof(float) * 9);
        Ainv[0][0] = Ainv[1][1] = Ainv[2][2] = 1.0f;
        return;
    }
    
    float inv_det = 1.0f / det;
    
    Ainv[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) * inv_det;
    Ainv[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) * inv_det;
    Ainv[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * inv_det;
    
    Ainv[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) * inv_det;
    Ainv[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * inv_det;
    Ainv[1][2] = (A[0][2] * A[1][0] - A[0][0] * A[1][2]) * inv_det;
    
    Ainv[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) * inv_det;
    Ainv[2][1] = (A[0][1] * A[2][0] - A[0][0] * A[2][1]) * inv_det;
    Ainv[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * inv_det;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����ӷ�
//-------------------------------------------------------------------------------------------------------------------
static void imu_matrix_add(float *A, float *B, float *C, int m, int n)
{
    // C = A + B
    for(int i = 0; i < m * n; i++)
    {
        C[i] = A[i] + B[i];
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     �������
//-------------------------------------------------------------------------------------------------------------------
static void imu_matrix_subtract(float *A, float *B, float *C, int m, int n)
{
    // C = A - B
    for(int i = 0; i < m * n; i++)
    {
        C[i] = A[i] - B[i];
    }
}

//=================================================Madgwick�˲���ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ʼ��Madgwick�˲���
//-------------------------------------------------------------------------------------------------------------------
static void imu_madgwick_init(void)
{
    imu_madgwick_filter_t *mf = &imu_attitude_system.madgwick;
    
    // ����˲����ṹ��
    memset(mf, 0, sizeof(imu_madgwick_filter_t));
    
    // ����Ĭ�ϲ���
    mf->beta = IMU_MADGWICK_BETA_DEFAULT;
    mf->zeta = IMU_MADGWICK_ZETA;
    
    // ���������
    memset(&mf->w_err_integral, 0, sizeof(imu_vector3_t));
    
    mf->initialized = 1;
    mf->update_count = 0;
    
    printf("Madgwick�˲�����ʼ����ɣ�beta=%.3f\r\n", mf->beta);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     Madgwick�˲��������º���
//-------------------------------------------------------------------------------------------------------------------
static imu_status_enum imu_madgwick_filter_update(void)
{
    imu_madgwick_filter_t *mf = &imu_attitude_system.madgwick;
    
    if(!mf->initialized)
    {
        imu_madgwick_init();
    }
    
    float dt = imu_attitude_system.processed_data.dt;
    imu_vector3_t *gyro = &imu_attitude_system.processed_data.gyro_filtered;
    imu_vector3_t *accel = &imu_attitude_system.processed_data.accel_filtered;
    
    // �����ٶȼƷ�ֵ�Ƿ�ӽ��ο��������ٶ� (�����50%��ƫ��)
    float accel_norm = sqrtf(accel->x * accel->x + accel->y * accel->y + accel->z * accel->z);
    float accel_min = imu_attitude_system.calibration.reference_gravity * 0.5f;
    float accel_max = imu_attitude_system.calibration.reference_gravity * 1.5f;
    
    // ֻ�ڼ��ٶȷ�ֵ����ʱ��ʹ�ü��ٶȼ�У��
    // ����ֻ�������ǻ���(���⶯̬���ٶȸ���)
    if(accel_norm >= accel_min && accel_norm <= accel_max)
    {
        // ����Madgwick�㷨����(�����ٶȼ�У��)
        imu_madgwick_update_imu(dt, gyro, accel);
    }
    else
    {
        // ��ʹ�������Ǹ���
        imu_vector3_t zero_accel = {0.0f, 0.0f, 0.0f};
        imu_madgwick_update_imu(dt, gyro, &zero_accel);
    }
    
    mf->update_count++;
    
    return IMU_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     Madgwick�㷨IMU����
//-------------------------------------------------------------------------------------------------------------------
static void imu_madgwick_update_imu(float dt, imu_vector3_t *gyro, imu_vector3_t *accel)
{
    imu_madgwick_filter_t *mf = &imu_attitude_system.madgwick;
    imu_quaternion_t *q = &imu_attitude_system.processed_data.quaternion;
    
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
    
    // ��ȡ��ǰ��Ԫ��ֵ
    float q0 = q->w, q1 = q->x, q2 = q->y, q3 = q->z;
    
    // �����ǽ��ٶ�
    float gx = gyro->x, gy = gyro->y, gz = gyro->z;
    float ax = accel->x, ay = accel->y, az = accel->z;
    
    // ������ٶȼ�������Ч
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 
    {
        // ��һ�����ٶȼƲ���ֵ
        recipNorm = imu_inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        
        // ���������������ظ�����
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;
        
        // �ݶ��½��㷨������Ŀ�꺯�����ݶ�
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        
        recipNorm = imu_inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // ��һ���ݶ�
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;
        
        // �������������
        float w_err_x = _2q0 * s1 - _2q1 * s0 - _2q2 * s3 + _2q3 * s2;
        float w_err_y = _2q0 * s2 + _2q1 * s3 - _2q2 * s0 - _2q3 * s1;
        float w_err_z = _2q0 * s3 - _2q1 * s2 + _2q2 * s1 - _2q3 * s0;
        
        // ���������������������zeta������
        if(mf->zeta > 0.0f)
        {
            mf->w_err_integral.x += w_err_x * dt;
            mf->w_err_integral.y += w_err_y * dt;
            mf->w_err_integral.z += w_err_z * dt;
            
            // Ӧ�û��ַ���
            gx += mf->zeta * mf->w_err_integral.x;
            gy += mf->zeta * mf->w_err_integral.y;
            gz += mf->zeta * mf->w_err_integral.z;
        }
        
        // Ӧ�ñ�������
        gx -= mf->beta * s0;
        gy -= mf->beta * s1;
        gz -= mf->beta * s2;
    }
    
    // ��Ԫ������
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
    
    // ������Ԫ��
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;
    
    // ��һ����Ԫ��
    recipNorm = imu_inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q->w = q0 * recipNorm;
    q->x = q1 * recipNorm;
    q->y = q2 * recipNorm;
    q->z = q3 * recipNorm;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����ƽ���������㷨
//-------------------------------------------------------------------------------------------------------------------
static float imu_inv_sqrt(float x)
{
    // ����ƽ���������㷨 (Quake III�㷨�ĸĽ���)
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));   // ��ѡ�ĵڶ��ε�������߾���
    return y;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡMadgwick�˲���״̬��Ϣ
//-------------------------------------------------------------------------------------------------------------------
imu_madgwick_filter_t* imu_get_madgwick_info(void)
{
    if(imu_attitude_system.calc_mode == IMU_MODE_MADGWICK && 
       imu_attitude_system.madgwick.initialized)
    {
        return &imu_attitude_system.madgwick;
    }
    return NULL;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����Madgwick�˲�������
//-------------------------------------------------------------------------------------------------------------------
imu_status_enum imu_set_madgwick_params(float beta, float zeta)
{
    if(!imu_attitude_system.system_initialized)
        return IMU_STATUS_NOT_INIT;
    
    // ������Χ���
    if(beta < IMU_MADGWICK_BETA_MIN || beta > IMU_MADGWICK_BETA_MAX)
    {
        printf("Madgwick beta����������Χ [%.3f, %.3f]\r\n", 
               IMU_MADGWICK_BETA_MIN, IMU_MADGWICK_BETA_MAX);
        return IMU_STATUS_ERROR;
    }
    
    if(zeta < 0.0f || zeta > 1.0f)
    {
        printf("Madgwick zeta����������Χ [0.0, 1.0]\r\n");
        return IMU_STATUS_ERROR;
    }
    
    // ���ò���
    imu_attitude_system.madgwick.beta = beta;
    imu_attitude_system.madgwick.zeta = zeta;
    
    printf("Madgwick�����Ѹ��£�beta=%.3f, zeta=%.3f\r\n", beta, zeta);
    return IMU_STATUS_OK;
}

//=================================================UKF�˲���ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ʼ��UKF�˲���
//-------------------------------------------------------------------------------------------------------------------
static void imu_ukf_init(void)
{
    imu_ukf_filter_t *ukf = &imu_attitude_system.ukf;
    
    // ����˲����ṹ��
    memset(ukf, 0, sizeof(imu_ukf_filter_t));
    
    // ����UKF����
    ukf->alpha = IMU_UKF_ALPHA;
    ukf->beta = IMU_UKF_BETA;
    ukf->kappa = IMU_UKF_KAPPA;
    ukf->lambda = ukf->alpha * ukf->alpha * (IMU_UKF_STATE_SIZE + ukf->kappa) - IMU_UKF_STATE_SIZE;
    
    // ��ʼ��״̬���� [q0, q1, q2, q3, bx, by, bz]
    ukf->state[0] = 1.0f;  // q0 (w)
    ukf->state[1] = 0.0f;  // q1 (x)
    ukf->state[2] = 0.0f;  // q2 (y)
    ukf->state[3] = 0.0f;  // q3 (z)
    ukf->state[4] = 0.0f;  // bias_x
    ukf->state[5] = 0.0f;  // bias_y
    ukf->state[6] = 0.0f;  // bias_z
    
    // ��ʼ��״̬Э������� P
    for(int i = 0; i < IMU_UKF_STATE_SIZE; i++)
    {
        for(int j = 0; j < IMU_UKF_STATE_SIZE; j++)
        {
            if(i == j)
            {
                if(i < 4)  // ��Ԫ������
                    ukf->P[i][j] = 0.1f;
                else       // ������ƫ���
                    ukf->P[i][j] = 0.01f;
            }
            else
            {
                ukf->P[i][j] = 0.0f;
            }
        }
    }
    
    // ��ʼ����������Э������� Q
    for(int i = 0; i < IMU_UKF_STATE_SIZE; i++)
    {
        for(int j = 0; j < IMU_UKF_STATE_SIZE; j++)
        {
            if(i == j)
            {
                if(i < 4)  // ��Ԫ����������
                    ukf->Q[i][j] = IMU_UKF_Q_GYRO;
                else       // ������ƫ���������
                    ukf->Q[i][j] = IMU_UKF_Q_BIAS;
            }
            else
            {
                ukf->Q[i][j] = 0.0f;
            }
        }
    }
    
    // ��ʼ����������Э������� R
    for(int i = 0; i < IMU_UKF_MEASUREMENT_SIZE; i++)
    {
        for(int j = 0; j < IMU_UKF_MEASUREMENT_SIZE; j++)
        {
            if(i == j)
                ukf->R[i][j] = IMU_UKF_R_ACCEL;
            else
                ukf->R[i][j] = 0.0f;
        }
    }
    
    // ����Sigma��Ȩ��
    float w_m_0 = ukf->lambda / (IMU_UKF_STATE_SIZE + ukf->lambda);
    float w_c_0 = w_m_0 + (1 - ukf->alpha * ukf->alpha + ukf->beta);
    float w_i = 1.0f / (2.0f * (IMU_UKF_STATE_SIZE + ukf->lambda));
    
    ukf->sigma_weights_mean[0] = w_m_0;
    ukf->sigma_weights_cov[0] = w_c_0;
    
    for(int i = 1; i < IMU_UKF_SIGMA_POINTS; i++)
    {
        ukf->sigma_weights_mean[i] = w_i;
        ukf->sigma_weights_cov[i] = w_i;
    }
    
    ukf->initialized = 1;
    ukf->update_count = 0;
    
    printf("UKF�˲�����ʼ����ɣ�lambda=%.6f\r\n", ukf->lambda);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     UKF�˲��������º���
//-------------------------------------------------------------------------------------------------------------------
static imu_status_enum imu_ukf_filter_update(void)
{
    imu_ukf_filter_t *ukf = &imu_attitude_system.ukf;
    
    if(!ukf->initialized)
    {
        imu_ukf_init();
    }
    
    float dt = imu_attitude_system.processed_data.dt;
    imu_vector3_t *gyro = &imu_attitude_system.processed_data.gyro_filtered;
    imu_vector3_t *accel = &imu_attitude_system.processed_data.accel_filtered;
    
    // ���NaN - ��Ԥ��ǰ���״̬
    for(int i = 0; i < IMU_UKF_STATE_SIZE; i++)
    {
        if(isnan(ukf->state[i]) || isinf(ukf->state[i]))
        {
            printf("��⵽UKF״̬NaN/Inf�����³�ʼ�� state[%d]=%.3f\r\n", i, ukf->state[i]);
            imu_ukf_init();
            return IMU_STATUS_ERROR;
        }
    }
    
    // 1. Ԥ�ⲽ��
    imu_ukf_predict(dt, gyro);
    
    // ���NaN - ��Ԥ�����״̬
    for(int i = 0; i < IMU_UKF_STATE_SIZE; i++)
    {
        if(isnan(ukf->state[i]) || isinf(ukf->state[i]))
        {
            printf("Ԥ����⵽UKF״̬NaN/Inf�����³�ʼ�� state[%d]=%.3f\r\n", i, ukf->state[i]);
            imu_ukf_init();
            return IMU_STATUS_ERROR;
        }
    }
    
    // 2. ���²��裨ʹ�ü��ٶȼ����ݣ�
    float accel_norm = sqrtf(accel->x * accel->x + accel->y * accel->y + accel->z * accel->z);
    // ֻ�ڼ��ٶȷ�ֵ�ӽ��ο��������ٶ�ʱ��ʹ�ü��ٶȼ�У�� (�����50%��ƫ��)
    float accel_min = imu_attitude_system.calibration.reference_gravity * 0.5f;
    float accel_max = imu_attitude_system.calibration.reference_gravity * 1.5f;
    if(accel_norm >= accel_min && accel_norm <= accel_max)
    {
        imu_ukf_update_accel(accel);
        
        // ���NaN - �ڸ��º���״̬
        for(int i = 0; i < IMU_UKF_STATE_SIZE; i++)
        {
            if(isnan(ukf->state[i]) || isinf(ukf->state[i]))
            {
                printf("���º��⵽UKF״̬NaN/Inf�����³�ʼ�� state[%d]=%.3f\r\n", i, ukf->state[i]);
                imu_ukf_init();
                return IMU_STATUS_ERROR;
            }
        }
    }
    
    // 3. ���������Ԫ��
    imu_attitude_system.processed_data.quaternion.w = ukf->state[0];
    imu_attitude_system.processed_data.quaternion.x = ukf->state[1];
    imu_attitude_system.processed_data.quaternion.y = ukf->state[2];
    imu_attitude_system.processed_data.quaternion.z = ukf->state[3];
    
    // 4. ��һ����Ԫ��
    imu_quaternion_normalize(&imu_attitude_system.processed_data.quaternion);
    
    // 5. ����״̬�����е���Ԫ��������һ���ԣ�
    ukf->state[0] = imu_attitude_system.processed_data.quaternion.w;
    ukf->state[1] = imu_attitude_system.processed_data.quaternion.x;
    ukf->state[2] = imu_attitude_system.processed_data.quaternion.y;
    ukf->state[3] = imu_attitude_system.processed_data.quaternion.z;
    
    ukf->update_count++;
    
    return IMU_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����UKF Sigma��
//-------------------------------------------------------------------------------------------------------------------
static void imu_ukf_generate_sigma_points(void)
{
    imu_ukf_filter_t *ukf = &imu_attitude_system.ukf;
    
    // ����Э��������Cholesky�ֽ�
    float L[IMU_UKF_STATE_SIZE][IMU_UKF_STATE_SIZE];
    imu_matrix_cholesky_decomposition(ukf->P, L);
    
    // ������������
    float sqrt_lambda_n = sqrtf(IMU_UKF_STATE_SIZE + ukf->lambda);
    
    // ��һ��Sigma����״̬��ֵ
    for(int i = 0; i < IMU_UKF_STATE_SIZE; i++)
    {
        ukf->sigma_points[0][i] = ukf->state[i];
    }
    
    // ���������Sigma��
    for(int i = 0; i < IMU_UKF_STATE_SIZE; i++)
    {
        for(int j = 0; j < IMU_UKF_STATE_SIZE; j++)
        {
            // ����Sigma��
            ukf->sigma_points[i+1][j] = ukf->state[j] + sqrt_lambda_n * L[i][j];
            // ����Sigma��
            ukf->sigma_points[i+1+IMU_UKF_STATE_SIZE][j] = ukf->state[j] - sqrt_lambda_n * L[i][j];
        }
    }
    
    // ��һ����Ԫ�����֣�ȷ��ÿ��Sigma�����Ԫ�����ǵ�λ��Ԫ����
    for(int i = 0; i < IMU_UKF_SIGMA_POINTS; i++)
    {
        float q_norm = sqrtf(ukf->sigma_points[i][0] * ukf->sigma_points[i][0] + 
                           ukf->sigma_points[i][1] * ukf->sigma_points[i][1] + 
                           ukf->sigma_points[i][2] * ukf->sigma_points[i][2] + 
                           ukf->sigma_points[i][3] * ukf->sigma_points[i][3]);
        if(q_norm > 0.001f)
        {
            ukf->sigma_points[i][0] /= q_norm;
            ukf->sigma_points[i][1] /= q_norm;
            ukf->sigma_points[i][2] /= q_norm;
            ukf->sigma_points[i][3] /= q_norm;
        }
        else
        {
            // �����Ԫ��ģ���쳣������Ϊ��λ��Ԫ��
            ukf->sigma_points[i][0] = 1.0f;
            ukf->sigma_points[i][1] = 0.0f;
            ukf->sigma_points[i][2] = 0.0f;
            ukf->sigma_points[i][3] = 0.0f;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     UKFԤ�ⲽ��
//-------------------------------------------------------------------------------------------------------------------
static void imu_ukf_predict(float dt, imu_vector3_t *gyro)
{
    imu_ukf_filter_t *ukf = &imu_attitude_system.ukf;
    
    // 1. ����Sigma��
    imu_ukf_generate_sigma_points();
    
    // 2. ����ÿ��Sigma��ͨ������ģ��
    for(int i = 0; i < IMU_UKF_SIGMA_POINTS; i++)
    {
        // ��ȡ��ǰSigma���״̬
        float q0 = ukf->sigma_points[i][0], q1 = ukf->sigma_points[i][1];
        float q2 = ukf->sigma_points[i][2], q3 = ukf->sigma_points[i][3];
        float bx = ukf->sigma_points[i][4], by = ukf->sigma_points[i][5], bz = ukf->sigma_points[i][6];
        
        // У���������������
        float wx = gyro->x - bx;
        float wy = gyro->y - by;
        float wz = gyro->z - bz;
        
        // ��Ԫ�����֣�״̬ת�Ʒ��̣�
        float dq0 = 0.5f * dt * (-q1 * wx - q2 * wy - q3 * wz);
        float dq1 = 0.5f * dt * (q0 * wx - q3 * wy + q2 * wz);
        float dq2 = 0.5f * dt * (q3 * wx + q0 * wy - q1 * wz);
        float dq3 = 0.5f * dt * (-q2 * wx + q1 * wy + q0 * wz);
        
        // ����Sigma��
        ukf->sigma_points[i][0] = q0 + dq0;
        ukf->sigma_points[i][1] = q1 + dq1;
        ukf->sigma_points[i][2] = q2 + dq2;
        ukf->sigma_points[i][3] = q3 + dq3;
        // ƫ��ֲ���
        // ukf->sigma_points[i][4] = bx;
        // ukf->sigma_points[i][5] = by;
        // ukf->sigma_points[i][6] = bz;
        
        // ��һ����Ԫ��
        float q_norm = sqrtf(ukf->sigma_points[i][0] * ukf->sigma_points[i][0] + 
                           ukf->sigma_points[i][1] * ukf->sigma_points[i][1] + 
                           ukf->sigma_points[i][2] * ukf->sigma_points[i][2] + 
                           ukf->sigma_points[i][3] * ukf->sigma_points[i][3]);
        if(q_norm > 0.001f)
        {
            ukf->sigma_points[i][0] /= q_norm;
            ukf->sigma_points[i][1] /= q_norm;
            ukf->sigma_points[i][2] /= q_norm;
            ukf->sigma_points[i][3] /= q_norm;
        }
        else
        {
            // �����Ԫ��ģ���쳣������Ϊ��λ��Ԫ��
            ukf->sigma_points[i][0] = 1.0f;
            ukf->sigma_points[i][1] = 0.0f;
            ukf->sigma_points[i][2] = 0.0f;
            ukf->sigma_points[i][3] = 0.0f;
        }
    }
    
    // 3. ����Ԥ���״̬��ֵ
    for(int i = 0; i < IMU_UKF_STATE_SIZE; i++)
    {
        ukf->state[i] = 0.0f;
        for(int j = 0; j < IMU_UKF_SIGMA_POINTS; j++)
        {
            ukf->state[i] += ukf->sigma_weights_mean[j] * ukf->sigma_points[j][i];
        }
    }
    
    // ��һ��Ԥ�����Ԫ��
    float q_norm = sqrtf(ukf->state[0] * ukf->state[0] + ukf->state[1] * ukf->state[1] + 
                        ukf->state[2] * ukf->state[2] + ukf->state[3] * ukf->state[3]);
    if(q_norm > 0.001f)
    {
        ukf->state[0] /= q_norm;
        ukf->state[1] /= q_norm;
        ukf->state[2] /= q_norm;
        ukf->state[3] /= q_norm;
    }
    else
    {
        // �����Ԫ��ģ���쳣������Ϊ��λ��Ԫ��
        ukf->state[0] = 1.0f;
        ukf->state[1] = 0.0f;
        ukf->state[2] = 0.0f;
        ukf->state[3] = 0.0f;
    }
    
    // 4. ����Ԥ���Э�������
    // ��������Э�������
    for(int i = 0; i < IMU_UKF_STATE_SIZE; i++)
    {
        for(int j = 0; j < IMU_UKF_STATE_SIZE; j++)
        {
            ukf->P[i][j] = 0.0f;
        }
    }
    
    // ����Э����
    for(int k = 0; k < IMU_UKF_SIGMA_POINTS; k++)
    {
        for(int i = 0; i < IMU_UKF_STATE_SIZE; i++)
        {
            for(int j = 0; j < IMU_UKF_STATE_SIZE; j++)
            {
                float diff_i = ukf->sigma_points[k][i] - ukf->state[i];
                float diff_j = ukf->sigma_points[k][j] - ukf->state[j];
                ukf->P[i][j] += ukf->sigma_weights_cov[k] * diff_i * diff_j;
            }
        }
    }
    
    // 5. ��ӹ�������
    for(int i = 0; i < IMU_UKF_STATE_SIZE; i++)
    {
        ukf->P[i][i] += ukf->Q[i][i] * dt;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     UKF���ٶȼƸ��²���
//-------------------------------------------------------------------------------------------------------------------
static void imu_ukf_update_accel(imu_vector3_t *accel)
{
    imu_ukf_filter_t *ukf = &imu_attitude_system.ukf;
    
    // 1. ͨ��������������Sigma��
    for(int i = 0; i < IMU_UKF_SIGMA_POINTS; i++)
    {
        imu_ukf_measurement_function(ukf->sigma_points[i], ukf->predicted_measurements[i]);
    }
    
    // 2. ����Ԥ��Ĳ�����ֵ
    float z_mean[IMU_UKF_MEASUREMENT_SIZE] = {0.0f};
    for(int i = 0; i < IMU_UKF_MEASUREMENT_SIZE; i++)
    {
        for(int j = 0; j < IMU_UKF_SIGMA_POINTS; j++)
        {
            z_mean[i] += ukf->sigma_weights_mean[j] * ukf->predicted_measurements[j][i];
        }
    }
    
    // 3. ���㴴��Э������� Pzz
    float Pzz[IMU_UKF_MEASUREMENT_SIZE][IMU_UKF_MEASUREMENT_SIZE];
    for(int i = 0; i < IMU_UKF_MEASUREMENT_SIZE; i++)
    {
        for(int j = 0; j < IMU_UKF_MEASUREMENT_SIZE; j++)
        {
            Pzz[i][j] = 0.0f;
            for(int k = 0; k < IMU_UKF_SIGMA_POINTS; k++)
            {
                float diff_i = ukf->predicted_measurements[k][i] - z_mean[i];
                float diff_j = ukf->predicted_measurements[k][j] - z_mean[j];
                Pzz[i][j] += ukf->sigma_weights_cov[k] * diff_i * diff_j;
            }
            // ��Ӳ�������
            if(i == j)
                Pzz[i][j] += ukf->R[i][j];
        }
    }
    
    // 4. ����״̬-��������Э������� Pxz
    float Pxz[IMU_UKF_STATE_SIZE][IMU_UKF_MEASUREMENT_SIZE];
    for(int i = 0; i < IMU_UKF_STATE_SIZE; i++)
    {
        for(int j = 0; j < IMU_UKF_MEASUREMENT_SIZE; j++)
        {
            Pxz[i][j] = 0.0f;
            for(int k = 0; k < IMU_UKF_SIGMA_POINTS; k++)
            {
                float diff_x = ukf->sigma_points[k][i] - ukf->state[i];
                float diff_z = ukf->predicted_measurements[k][j] - z_mean[j];
                Pxz[i][j] += ukf->sigma_weights_cov[k] * diff_x * diff_z;
            }
        }
    }
    
    // 5. ���㿨�������� K = Pxz * Pzz^(-1)
    // �򻯰汾��ʹ�öԽ��������
    float K[IMU_UKF_STATE_SIZE][IMU_UKF_MEASUREMENT_SIZE];
    for(int i = 0; i < IMU_UKF_STATE_SIZE; i++)
    {
        for(int j = 0; j < IMU_UKF_MEASUREMENT_SIZE; j++)
        {
            if(Pzz[j][j] > 0.0f)
                K[i][j] = Pxz[i][j] / Pzz[j][j];
            else
                K[i][j] = 0.0f;
        }
    }
    
    // 6. ���㴴�£��в
    float innovation[IMU_UKF_MEASUREMENT_SIZE];
    float accel_norm = sqrtf(accel->x * accel->x + accel->y * accel->y + accel->z * accel->z);
    
    // ������������ٶ�ģ���쳣�������и���
    if(accel_norm < 0.1f || accel_norm > 20.0f)
    {
        // ���ٶ������쳣���������θ���
        return;
    }
    
    innovation[0] = accel->x / accel_norm - z_mean[0];
    innovation[1] = accel->y / accel_norm - z_mean[1];
    innovation[2] = accel->z / accel_norm - z_mean[2];
    
    // 7. ״̬����
    for(int i = 0; i < IMU_UKF_STATE_SIZE; i++)
    {
        for(int j = 0; j < IMU_UKF_MEASUREMENT_SIZE; j++)
        {
            ukf->state[i] += K[i][j] * innovation[j];
        }
    }
    
    // 8. Э������� P = P - K * Pzz * K'
    // �򻯰汾
    for(int i = 0; i < IMU_UKF_STATE_SIZE; i++)
    {
        for(int j = 0; j < IMU_UKF_MEASUREMENT_SIZE; j++)
        {
            ukf->P[i][i] -= K[i][j] * Pzz[j][j] * K[i][j];
        }
        // ����Э������Сֵ
        if(ukf->P[i][i] < 0.001f)
            ukf->P[i][i] = 0.001f;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     UKF������������״̬��Ԥ�ڼ��ٶȼƶ�����
//-------------------------------------------------------------------------------------------------------------------
static void imu_ukf_measurement_function(float *state, float *measurement)
{
    // ����Ԫ������Ԥ�ڵ����������ڴ���������ϵ�У�
    float q0 = state[0], q1 = state[1], q2 = state[2], q3 = state[3];
    
    // �����ڴ���������ϵ�еı�ʾ
    measurement[0] = 2.0f * (q1 * q3 - q0 * q2);        // gx
    measurement[1] = 2.0f * (q0 * q1 + q2 * q3);        // gy
    measurement[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;  // gz
}

//-------------------------------------------------------------------------------------------------------------------
// �������     Cholesky�ֽ⣨����UKF Sigma�����ɣ�
//-------------------------------------------------------------------------------------------------------------------
static void imu_matrix_cholesky_decomposition(float A[IMU_UKF_STATE_SIZE][IMU_UKF_STATE_SIZE], 
                                             float L[IMU_UKF_STATE_SIZE][IMU_UKF_STATE_SIZE])
{
    // ��ʼ��L����Ϊ��
    for(int i = 0; i < IMU_UKF_STATE_SIZE; i++)
    {
        for(int j = 0; j < IMU_UKF_STATE_SIZE; j++)
        {
            L[i][j] = 0.0f;
        }
    }
    
    // Cholesky�ֽ�: A = L * L'
    for(int i = 0; i < IMU_UKF_STATE_SIZE; i++)
    {
        for(int j = 0; j <= i; j++)
        {
            if(i == j)  // �Խ���Ԫ��
            {
                float sum = 0.0f;
                for(int k = 0; k < j; k++)
                {
                    sum += L[i][k] * L[i][k];
                }
                
                float val = A[i][i] - sum;
                if(val > 0.0f)
                    L[i][j] = sqrtf(val);
                else
                    L[i][j] = 0.001f;  // ��ֹ��ֵ����
            }
            else  // ������Ԫ��
            {
                float sum = 0.0f;
                for(int k = 0; k < j; k++)
                {
                    sum += L[i][k] * L[j][k];
                }
                
                if(L[j][j] > 0.0f)
                    L[i][j] = (A[i][j] - sum) / L[j][j];
                else
                    L[i][j] = 0.0f;
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡUKF�˲���״̬��Ϣ
//-------------------------------------------------------------------------------------------------------------------
imu_ukf_filter_t* imu_get_ukf_info(void)
{
    if(imu_attitude_system.calc_mode == IMU_MODE_UKF && 
       imu_attitude_system.ukf.initialized)
    {
        return &imu_attitude_system.ukf;
    }
    return NULL;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����UKF�˲�������
//-------------------------------------------------------------------------------------------------------------------
imu_status_enum imu_set_ukf_params(float alpha, float beta, float kappa)
{
    if(!imu_attitude_system.system_initialized)
        return IMU_STATUS_NOT_INIT;
    
    // ������Χ���
    if(alpha <= 0.0f || alpha > 1.0f)
    {
        printf("UKF alpha����������Χ (0.0, 1.0]\r\n");
        return IMU_STATUS_ERROR;
    }
    
    if(beta < 0.0f)
    {
        printf("UKF beta��������Ϊ����\r\n");
        return IMU_STATUS_ERROR;
    }
    
    // ���ò���
    imu_attitude_system.ukf.alpha = alpha;
    imu_attitude_system.ukf.beta = beta;
    imu_attitude_system.ukf.kappa = kappa;
    imu_attitude_system.ukf.lambda = alpha * alpha * (IMU_UKF_STATE_SIZE + kappa) - IMU_UKF_STATE_SIZE;
    
    // ���¼���Ȩ��
    float w_m_0 = imu_attitude_system.ukf.lambda / (IMU_UKF_STATE_SIZE + imu_attitude_system.ukf.lambda);
    float w_c_0 = w_m_0 + (1 - alpha * alpha + beta);
    float w_i = 1.0f / (2.0f * (IMU_UKF_STATE_SIZE + imu_attitude_system.ukf.lambda));
    
    imu_attitude_system.ukf.sigma_weights_mean[0] = w_m_0;
    imu_attitude_system.ukf.sigma_weights_cov[0] = w_c_0;
    
    for(int i = 1; i < IMU_UKF_SIGMA_POINTS; i++)
    {
        imu_attitude_system.ukf.sigma_weights_mean[i] = w_i;
        imu_attitude_system.ukf.sigma_weights_cov[i] = w_i;
    }
    
    printf("UKF�����Ѹ��£�alpha=%.6f, beta=%.3f, kappa=%.3f, lambda=%.6f\r\n", 
           alpha, beta, kappa, imu_attitude_system.ukf.lambda);
    return IMU_STATUS_OK;
}