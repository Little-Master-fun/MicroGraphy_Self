/*********************************************************************************************************************
* �ļ�����          imu_ahrs_complementary.c
* ����˵��          ������Ԫ����AHRS�����˲���ʵ�֣��ο�dog\DOG_KEY��Ŀ��
* ����              LittleMaster
* �汾��Ϣ          v1.0
* �޸ļ�¼
* ����              ����                �汾
* 2025-10-11        LittleMaster       1.0v
* 2025-10-20        LittleMaster       2.0v
* 
* �ļ�����˵����
* ���ļ�ʵ�ֻ�����Ԫ����AHRS��̬�����㷨���������¹��ܣ�
* 1. ��������ƫ��̬У׼������ʱ�Զ�У׼��
* 2. ��Ԫ����̬���£���PI������Ư�Ʋ�����
* 3. ŷ���Ǽ��㼰������ۻ�����
* 4. ����ƽ���������㷨�Ż�
* 
* �޸���¼��
* 2025-11-11: �޸���DEG_TO_RAD��λת�����󣨴�1.0��Ϊ��/180���������תһȦ��2��ƫ�������
********************************************************************************************************************/

#include "imu_ahrs_complementary.h"
#include "driver_sch16tk10.h"
#include "config_infineon.h"
#include "zf_common_headfile.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

//=================================================��������================================================


//  �������ڼ��㣺1ms�ж� �� AVG_FACTOR(2) = 2ms = 0.002��
// ������ʵ�⣬ʵ�ʲ�������ԼΪ 1.126ms����� DELTA_T = 0.001126 * AVG_FACTOR(2) �� 0.002252�룬��Ҫ����Ϊʲô��0.032���ʾ��Ǿ���
// �⣬Ϊʲôû�л�������������ô����أ����뵽���������ֲ�ſ��������������ö����ԣ�sb�������


#define DELTA_T              0.002f         // �������� 
#define ALPHA                0.2f           // ���ٶȼƵ�ͨ�˲�ϵ��
#define G_TO_M_S2            9.80665f       // �������ٶ�ת��ϵ��
#define DEG_TO_RAD           0.017453292519943295f  // ��ת���� (��/180)�������������dps��Ҫת��
#define AHRS_PI              3.1415926535f  // Բ����

// ��������ƫУ׼����
#define GYRO_CALIB_START     400            // ��ʼУ׼�Ĳ����� (800ms��ʼ) 400
#define GYRO_CALIB_END       1400           // ����У׼�Ĳ����� (2800ms)
#define GYRO_CALIB_SAMPLES   1000           // У׼�������� (�ۼ�1000�Σ�����2��)
#define GYRO_CALIB_FINISH    1500           // У׼��ɵ� (3000ms)
#define GYRO_READY_COUNT     1800           // ϵͳ׼�������� (3600ms��Լ3.6��)

//=================================================ȫ�ֱ�������================================================
static ahrs_system_t ahrs_system = {0};
static ahrs_quaternion_t quaternion = {1.0f, 0.0f, 0.0f, 0.0f};
static ahrs_euler_angles_t euler_angles = {0};
static ahrs_gyro_offset_t gyro_offset = {0};
static ahrs_imu_data_t imu_data = {0};
static ahrs_pi_controller_t pi_controller = {0};

// 1D \xd2\xf9\xcc\xab\xd1\xd0\xba\xcd\xb2\xbf (\xb6\xc8)\r
static float yaw_gyro = 0.0f;\r
static uint8 yaw_gyro_synced = 0;  // Ã¿±???

// ���Ա�����ͨ�����ߵ��Բ鿴��
typedef struct {
    // ������������Ϣ
    uint16_t actual_sens_rate1;        // ʵ�����õ�������Rate1
    uint16_t actual_sens_rate2;        // ʵ�����õ�������Rate2
    uint16_t actual_dec_rate2;         // ʵ�����õĳ�ȡ��
    float    config_sens_rate1;        // ���������õ�������
    
    // ԭʼLSB���ݣ��ۼӺ�
    int32_t  raw_gyro_x;               // ������XԭʼLSB
    int32_t  raw_gyro_y;               // ������YԭʼLSB
    int32_t  raw_gyro_z;               // ������ZԭʼLSB
    
    // ת�����������
    float    gyro_x_dps;               // ������X (��/��)
    float    gyro_y_dps;               // ������Y (��/��)
    float    gyro_z_dps;               // ������Z (��/��)
    float    gyro_x_rads;              // ������X (����/��)
    float    gyro_y_rads;              // ������Y (����/��)
    float    gyro_z_rads;              // ������Z (����/��)
    
    // ��̬��
    float    pitch_deg;                // ������ (��)
    float    roll_deg;                 // ��ת�� (��)
    float    yaw_deg;                  // ƫ���� (��)
    float    yaw_accumulated_deg;      // �ۻ�ƫ���� (��)
    
    // ת��ϵ����֤
    float    expected_sensitivity;     // ������������ = SENSITIVITY_RATE1 * AVG_FACTOR
    float    actual_conversion_factor; // ʵ��ת������
    
    // ���¼���
    uint32_t update_count;             // ��̬���¼�����
    
} ahrs_debug_info_t;

// ����Ϊȫ�ֱ����Ա�������鿴
ahrs_debug_info_t g_ahrs_debug = {0};

// �����������ۼӻ�����������AVG_FACTOR�β���ƽ����
static SCH1_raw_data raw_data_summed = {0};
static uint32 sample_count = 0;

// У׼��ر���
static uint32 calibration_count = 0;
static uint8 system_ready = 0;
static float gyro_sum_x = 0.0f;
static float gyro_sum_y = 0.0f;
static float gyro_sum_z = 0.0f;

// PI����������
static float param_kp = 0.0f;   // �������棨�ɸ�����Ҫ������
static float param_ki = 0.0f;   // �������棨�ɸ�����Ҫ������

// ����Ǵ���
static float yaw_offset = 0.0f;
static int8 direction_change = 0;
static float last_yaw = 0.0f;

//=================================================�ڲ���������================================================
static float fast_inv_sqrt(float x);
static void ahrs_get_sensor_values(SCH1_result *sensor_data);
static void ahrs_update_quaternion(float gx, float gy, float gz, float ax, float ay, float az);
static void ahrs_quaternion_to_euler(void);

//=================================================�ⲿ�ӿ�ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ʼ��AHRSϵͳ
// ����˵��     ��
// ���ز���     ��ʼ��״̬
//-------------------------------------------------------------------------------------------------------------------
ahrs_status_enum ahrs_complementary_init(void)
{
    // ����������ݽṹ
    memset(&ahrs_system, 0, sizeof(ahrs_system_t));
    memset(&gyro_offset, 0, sizeof(ahrs_gyro_offset_t));
    memset(&pi_controller, 0, sizeof(ahrs_pi_controller_t));
    
    // ��ʼ����Ԫ��Ϊ��λ��Ԫ��
    quaternion.q0 = 1.0f;
    quaternion.q1 = 0.0f;
    quaternion.q2 = 0.0f;
    quaternion.q3 = 0.0f;
    
    // ����ŷ����
    euler_angles.pitch = 0.0f;
    euler_angles.roll = 0.0f;
    euler_angles.yaw = 0.0f;
    
    // ��ʼ�������������ۼӻ�����
    memset(&raw_data_summed, 0, sizeof(SCH1_raw_data));
    sample_count = 0;
    
    // ��ʼ��У׼����
    calibration_count = 0;
    system_ready = 0;
    gyro_sum_x = 0.0f;
    gyro_sum_y = 0.0f;
    gyro_sum_z = 0.0f;
    
    // ��ʼ������Ǵ�������
    yaw_offset = 0.0f;
    direction_change = 0;
    last_yaw = 0.0f;
    
    // ����PI����������
    param_kp = 0.0f;  // ���Ը���ʵ���������
    param_ki = 0.0f;  // ���Ը���ʵ���������
    
    // ��ʼ��SCH16TK10������
    printf("���ڳ�ʼ��SCH16TK10������...\r\n");
    
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
        return AHRS_STATUS_ERROR;
    }
    
    printf("SCH16TK10��������ʼ���ɹ���\r\n");
    
    // ��ȡ����֤ʵ������������
    uint16_t actual_sens_rate1, actual_sens_rate2, actual_dec_rate2;
    if (SCH1_getRateSensDec(&actual_sens_rate1, &actual_sens_rate2, &actual_dec_rate2) == SCH1_OK)
    {
        // ���浽���Խṹ��
        g_ahrs_debug.actual_sens_rate1 = actual_sens_rate1;
        g_ahrs_debug.actual_sens_rate2 = actual_sens_rate2;
        g_ahrs_debug.actual_dec_rate2 = actual_dec_rate2;
        g_ahrs_debug.config_sens_rate1 = SENSITIVITY_RATE1;
        g_ahrs_debug.expected_sensitivity = SENSITIVITY_RATE1 * (float)AVG_FACTOR;
        
        printf("������ʵ������ - ������Rate1: %d LSB/dps, Rate2: %d LSB/dps, ��ȡ��: %d\r\n", 
               actual_sens_rate1, actual_sens_rate2, actual_dec_rate2);
        printf("������ʹ�õ�������: %.0f LSB/dps\r\n", SENSITIVITY_RATE1);
        printf("����ת������: %.0f (SENS * AVG_FACTOR)\r\n", g_ahrs_debug.expected_sensitivity);
        
        if(actual_sens_rate1 != (uint16_t)SENSITIVITY_RATE1)
        {
            printf("���棺ʵ����������������ò�ƥ�䣡\r\n");
        }
    }
    
    // ����SPIͨ���ȶ���
    if (SCH1_testSPIStability())
    {
        printf("SPIͨ�Ų���ͨ��\r\n");
    }
    else
    {
        printf("SPIͨ�Ų���ʧ�ܣ�����������\r\n");
    }
    
    ahrs_system.initialized = 1;
    
    printf("AHRS�����˲�����ʼ�����\r\n");
    printf("���ڽ�����������ƫУ׼���뱣���豸��ֹ...\r\n");
    
    return AHRS_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     AHRSϵͳ���£��ڶ�ʱ�ж��е��ã�����1ms���ڣ�
// ����˵��     raw_data        ������ԭʼ����
// ���ز���     ����״̬
// ��ע��Ϣ     ʵ��AVG_FACTOR�β���ƽ������������
//-------------------------------------------------------------------------------------------------------------------
ahrs_status_enum ahrs_complementary_update(SCH1_raw_data *raw_data)
{
    if(!ahrs_system.initialized)
        return AHRS_STATUS_NOT_INIT;
    
    // �������֡����
    if(raw_data->frame_error)
        return AHRS_STATUS_ERROR;
    
    // �ۼ�ԭʼ���ݣ����ڶ�β���ƽ�����룩
    raw_data_summed.Rate1_raw[AXIS_X] += raw_data->Rate1_raw[AXIS_X];
    raw_data_summed.Rate1_raw[AXIS_Y] += raw_data->Rate1_raw[AXIS_Y];
    raw_data_summed.Rate1_raw[AXIS_Z] += raw_data->Rate1_raw[AXIS_Z];
    
    raw_data_summed.Acc1_raw[AXIS_X] += raw_data->Acc1_raw[AXIS_X];
    raw_data_summed.Acc1_raw[AXIS_Y] += raw_data->Acc1_raw[AXIS_Y];
    raw_data_summed.Acc1_raw[AXIS_Z] += raw_data->Acc1_raw[AXIS_Z];
    
    raw_data_summed.Temp_raw += raw_data->Temp_raw;
    
    sample_count++;
    
    // ���ۼӴ����ﵽAVG_FACTORʱ���Ž�������ת������̬����
    if(sample_count >= AVG_FACTOR)
    {
        // ת��Ϊ��������SCH1_convert_data���Զ�����AVG_FACTOR����ƽ����
        SCH1_result sensor_data;
        SCH1_convert_data(&raw_data_summed, &sensor_data);
        
        // У׼�׶�
        if(!system_ready)
        {
            calibration_count++;
            
            // �ۻ�����������������ƫ���㣨��401��1400����1000�Σ�
            if(calibration_count > GYRO_CALIB_START && calibration_count <= GYRO_CALIB_END)
            {
                gyro_sum_x += sensor_data.Rate1[AXIS_X];
                gyro_sum_y += sensor_data.Rate1[AXIS_Y];
                gyro_sum_z += sensor_data.Rate1[AXIS_Z];
            }
            
            // ������ƫƽ��ֵ
            if(calibration_count == GYRO_CALIB_FINISH)
            {
                gyro_offset.x = gyro_sum_x / GYRO_CALIB_SAMPLES;
                gyro_offset.y = gyro_sum_y / GYRO_CALIB_SAMPLES;
                gyro_offset.z = gyro_sum_z / GYRO_CALIB_SAMPLES;
                
                printf("��������ƫУ׼���:\r\n");
                printf("  X��ƫ��: %.6f dps\r\n", gyro_offset.x);
                printf("  Y��ƫ��: %.6f dps\r\n", gyro_offset.y);
                printf("  Z��ƫ��: %.6f dps\r\n", gyro_offset.z);
            }
            
            // ϵͳ׼������
            if(calibration_count > GYRO_READY_COUNT)
            {
                system_ready = 1;
                printf("AHRSϵͳ׼����������ʼ��̬����\r\n");
            }
        }
        else
        {
            // ���������׶Σ���ȡ���������ݲ�������̬
            ahrs_get_sensor_values(&sensor_data);
            
            // ���µ�����Ϣ
            g_ahrs_debug.update_count++;
            
            // ��¼ԭʼLSB����
            g_ahrs_debug.raw_gyro_x = raw_data_summed.Rate1_raw[AXIS_X];
            g_ahrs_debug.raw_gyro_y = raw_data_summed.Rate1_raw[AXIS_Y];
            g_ahrs_debug.raw_gyro_z = raw_data_summed.Rate1_raw[AXIS_Z];
            
            // ��¼ת���������(dps)
            g_ahrs_debug.gyro_x_dps = sensor_data.Rate1[AXIS_X];
            g_ahrs_debug.gyro_y_dps = sensor_data.Rate1[AXIS_Y];
            g_ahrs_debug.gyro_z_dps = sensor_data.Rate1[AXIS_Z];
            
            // ��¼ת���������(rad/s)
            g_ahrs_debug.gyro_x_rads = imu_data.gyro_x;
            g_ahrs_debug.gyro_y_rads = imu_data.gyro_y;
            g_ahrs_debug.gyro_z_rads = imu_data.gyro_z;
            
            // ����ʵ��ת�����ӣ���LSB��dps��
            if(g_ahrs_debug.gyro_z_dps != 0)
            {
                g_ahrs_debug.actual_conversion_factor = (float)g_ahrs_debug.raw_gyro_z / g_ahrs_debug.gyro_z_dps;
            }
            
            // ִ��AHRS����
            ahrs_update_quaternion(imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z,
                                  imu_data.acc_x, imu_data.acc_y, imu_data.acc_z);
            
            // ת��Ϊŷ����
            ahrs_quaternion_to_euler();
            
            // ��¼��̬��
            g_ahrs_debug.pitch_deg = euler_angles.pitch;
            g_ahrs_debug.roll_deg = euler_angles.roll;
            g_ahrs_debug.yaw_deg = euler_angles.yaw;
            g_ahrs_debug.yaw_accumulated_deg = euler_angles.yaw_accumulated;
        }
        
        // ���ò����������ۼӻ�����
        sample_count = 0;
        memset(&raw_data_summed, 0, sizeof(SCH1_raw_data));
    }
    
    return system_ready ? AHRS_STATUS_OK : AHRS_STATUS_CALIBRATING;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ��ǰŷ����
// ����˵��     euler           ŷ�������ָ��
// ���ز���     ״̬
//-------------------------------------------------------------------------------------------------------------------
ahrs_status_enum ahrs_get_euler_angles(ahrs_euler_angles_t *euler)
{
    if(!system_ready || euler == NULL)
        return AHRS_STATUS_ERROR;
    
    *euler = euler_angles;
    return AHRS_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ��ǰ��Ԫ��
// ����˵��     quat            ��Ԫ�����ָ��
// ���ز���     ״̬
//-------------------------------------------------------------------------------------------------------------------
ahrs_status_enum ahrs_get_quaternion(ahrs_quaternion_t *quat)
{
    if(!system_ready || quat == NULL)
        return AHRS_STATUS_ERROR;
    
    *quat = quaternion;
    return AHRS_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ��������ƫ
// ����˵��     offset          ��ƫ���ָ��
// ���ز���     ״̬
//-------------------------------------------------------------------------------------------------------------------
ahrs_status_enum ahrs_get_gyro_offset(ahrs_gyro_offset_t *offset)
{
    if(!system_ready || offset == NULL)
        return AHRS_STATUS_ERROR;
    
    *offset = gyro_offset;
    return AHRS_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ���ú�������
// ����˵��     ��
// ���ز���     ״̬
//-------------------------------------------------------------------------------------------------------------------
ahrs_status_enum ahrs_reset_yaw(void)
{
    yaw_offset = 0.0f;
    direction_change = 0;
    last_yaw = euler_angles.yaw;
    
    printf("�����������\r\n");
    return AHRS_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����PI����������
// ����˵��     kp              ��������
//             ki              ��������
// ���ز���     ״̬
//-------------------------------------------------------------------------------------------------------------------
ahrs_status_enum ahrs_set_pi_params(float kp, float ki)
{
    param_kp = kp;
    param_ki = ki;
    
    printf("PI�����������Ѹ���: Kp=%.3f, Ki=%.3f\r\n", kp, ki);
    return AHRS_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ���ϵͳ�Ƿ�׼������
// ����˵��     ��
// ���ز���     1-���� 0-δ����
//-------------------------------------------------------------------------------------------------------------------
uint8 ahrs_is_ready(void)
{
    return system_ready;
}

//=================================================�ڲ�����ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ����ƽ���������㷨��Quake III�㷨��
// ����˵��     x               ����ֵ
// ���ز���     1/sqrt(x)
//-------------------------------------------------------------------------------------------------------------------
static float fast_inv_sqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ����������������
// ����˵��     sensor_data     ԭʼ����������
// ���ز���     ��
//-------------------------------------------------------------------------------------------------------------------
static void ahrs_get_sensor_values(SCH1_result *sensor_data)
{
    // ���ٶȼ����ݴ�������ͨ�˲� + ��λת����
    imu_data.acc_x = ALPHA * (sensor_data->Acc1[AXIS_X] / G_TO_M_S2) + 
                     (1.0f - ALPHA) * imu_data.acc_x;
    imu_data.acc_y = ALPHA * (sensor_data->Acc1[AXIS_Y] / G_TO_M_S2) + 
                     (1.0f - ALPHA) * imu_data.acc_y;
    imu_data.acc_z = ALPHA * (sensor_data->Acc1[AXIS_Z] / G_TO_M_S2) + 
                     (1.0f - ALPHA) * imu_data.acc_z;
    
    // ���������ݴ�������ƫ������
    imu_data.gyro_x = (sensor_data->Rate1[AXIS_X] - gyro_offset.x) * DEG_TO_RAD;
    imu_data.gyro_y = (sensor_data->Rate1[AXIS_Y] - gyro_offset.y) * DEG_TO_RAD;
    imu_data.gyro_z = (sensor_data->Rate1[AXIS_Z] - gyro_offset.z) * DEG_TO_RAD;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     AHRS��Ԫ�����£���PI������Ư�Ʋ��������о�Խ��ԽƮ�����ã�
// ����˵��     gx, gy, gz      ���������� (rad/s)
//             ax, ay, az      ���ٶȼ����� (g)
// ���ز���     ��
//-------------------------------------------------------------------------------------------------------------------
static void ahrs_update_quaternion(float gx, float gy, float gz, float ax, float ay, float az)
{
    float half_t = 0.5f * DELTA_T;
    float vx, vy, vz;
    float ex, ey, ez;
    
    // ��ǰ��Ԫ��ֵ
    float q0 = quaternion.q0;
    float q1 = quaternion.q1;
    float q2 = quaternion.q2;
    float q3 = quaternion.q3;
    
    // Ԥ���㳣����
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q1q1 = q1 * q1;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;
    
    // ��һ�����ٶȼ�����
    float norm = fast_inv_sqrt(ax * ax + ay * ay + az * az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;
    
    // �����������򣨴���Ԫ�����㣩
    vx = 2.0f * (q1q3 - q0q2);
    vy = 2.0f * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    
    // �����ٶȼƲ���ֵ�����ֵ�Ĳ��
    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;
    
    // PI�������������
    pi_controller.integral_ex += half_t * ex;
    pi_controller.integral_ey += half_t * ey;
    pi_controller.integral_ez += half_t * ez;
    
    // Ӧ��PI����������������Ư��
    gx = gx + param_kp * ex + param_ki * pi_controller.integral_ex;
    gy = gy + param_kp * ey + param_ki * pi_controller.integral_ey;
    gz = gz + param_kp * ez + param_ki * pi_controller.integral_ez;
    
    // ��Ԫ��һ�׸���
    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * half_t;
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * half_t;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * half_t;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * half_t;
    
    // ��������
    float delta_2 = (2.0f * half_t * gx) * (2.0f * half_t * gx) +
                    (2.0f * half_t * gy) * (2.0f * half_t * gy) +
                    (2.0f * half_t * gz) * (2.0f * half_t * gz);
    
    q0 = (1.0f - delta_2 / 8.0f) * q0 + (-q1 * gx - q2 * gy - q3 * gz) * half_t;
    q1 = (1.0f - delta_2 / 8.0f) * q1 + (q0 * gx + q2 * gz - q3 * gy) * half_t;
    q2 = (1.0f - delta_2 / 8.0f) * q2 + (q0 * gy - q1 * gz + q3 * gx) * half_t;
    q3 = (1.0f - delta_2 / 8.0f) * q3 + (q0 * gz + q1 * gy - q2 * gx) * half_t;
    
    // ��Ԫ����һ��
    norm = fast_inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    quaternion.q0 = q0 * norm;
    quaternion.q1 = q1 * norm;
    quaternion.q2 = q2 * norm;
    quaternion.q3 = q3 * norm;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��Ԫ��תŷ���ǣ���������ۻ�������
// ����˵��     ��
// ���ز���     ��
//-------------------------------------------------------------------------------------------------------------------
static void ahrs_quaternion_to_euler(void)
{
    float q0 = quaternion.q0;
    float q1 = quaternion.q1;
    float q2 = quaternion.q2;
    float q3 = quaternion.q3;
    
    // ���㸩���� (Pitch)
    euler_angles.pitch = asin(-2.0f * q1 * q3 + 2.0f * q0 * q2) * 180.0f / AHRS_PI;
    
    // �����ת�� (Roll)
    euler_angles.roll = atan2(2.0f * q2 * q3 + 2.0f * q0 * q1, 
                              -2.0f * q1 * q1 - 2.0f * q2 * q2 + 1.0f) * 180.0f / AHRS_PI;
    
    // ����ƫ���� (Yaw)
    euler_angles.yaw = atan2(2.0f * q1 * q2 + 2.0f * q0 * q3, 
                             -2.0f * q2 * q2 - 2.0f * q3 * q3 + 1.0f) * 180.0f / AHRS_PI;
    
    // �Ƕȷ�Χ���Ƶ� [-180, 180]
    if(euler_angles.yaw >= 180.0f)
        euler_angles.yaw -= 360.0f;
    else if(euler_angles.yaw <= -180.0f)
        euler_angles.yaw += 360.0f;
    
    // ������ۻ�������������Խ��180��������
    if((euler_angles.yaw - last_yaw) < -350.0f)
        direction_change++;
    else if((euler_angles.yaw - last_yaw) > 350.0f)
        direction_change--;
    
    // �����ۻ������
    euler_angles.yaw_accumulated = -(360.0f * direction_change + euler_angles.yaw - yaw_offset);
    
    last_yaw = euler_angles.yaw;
}

