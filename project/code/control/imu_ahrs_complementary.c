/*********************************************************************************************************************
* �ļ�����          imu_ahrs_complementary.c
* ����˵��          ������Ԫ����AHRS�����˲���ʵ�֣��ο�dog\DOG_KEY��Ŀ��
* ����              LittleMaster
* �汾��Ϣ          v1.0
* �޸ļ�¼
* ����              ����                �汾
* 2025-01-XX        LittleMaster       1.0v
* 
* �ļ�����˵����
* ���ļ�ʵ�ֻ�����Ԫ����AHRS��̬�����㷨���������¹��ܣ�
* 1. ��������ƫ��̬У׼������ʱ�Զ�У׼��
* 2. ��Ԫ����̬���£���PI������Ư�Ʋ�����
* 3. ŷ���Ǽ��㼰������ۻ�����
* 4. ����ƽ���������㷨�Ż�
********************************************************************************************************************/

#include "imu_ahrs_complementary.h"
#include "driver_sch16tk10.h"
#include "config_infineon.h"
#include "zf_common_headfile.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

//=================================================��������================================================
#define DELTA_T              0.000563f         // �������� 1ms (����AVG_FACTOR���)
// #define DELTA_T              0.000563f        // �������� 1ms (����AVG_FACTOR���)
#define ALPHA                1.0f           // ���ٶȼƵ�ͨ�˲�ϵ��
#define G_TO_M_S2            9.80665f       // �������ٶ�ת��ϵ��
#define DEG_TO_RAD           1.0f  // ��ת���� (��/180)�������������dps��Ҫת��// #define DEG_TO_RAD           0.017453292519943295f  // ��ת���� (��/180)�������������dps��Ҫת��#define GNSS_PI              3.1415926535f  // Բ����

// ��������ƫУ׼������ע�⣺����AVG_FACTOR=2��ʵ��ʱ��Ϊ����������2ms��
#define GYRO_CALIB_START     400            // ��ʼУ׼�Ĳ����� (800ms��ʼ)
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
    
    // ��ʼ������Ǵ������
    yaw_offset = 0.0f;
    direction_change = 0;
    last_yaw = 0.0f;
    
    // ����PI����������
    param_kp = 0.0f;  // ���Ը���ʵ���������
    param_ki = 0.0f;  // ���Ը���ʵ���������
    
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
            
            // ִ��AHRS����
            ahrs_update_quaternion(imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z,
                                  imu_data.acc_x, imu_data.acc_y, imu_data.acc_z);
            
            // ת��Ϊŷ����
            ahrs_quaternion_to_euler();
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
// �������     ��ȡ��������������
// ����˵��     sensor_data     ԭʼ����������
// ���ز���     ��
//-------------------------------------------------------------------------------------------------------------------
static void ahrs_get_sensor_values(SCH1_result *sensor_data)
{
    // ���ٶȼ����ݴ�����ͨ�˲� + ��λת����
    imu_data.acc_x = ALPHA * (sensor_data->Acc1[AXIS_X] / G_TO_M_S2) + 
                     (1.0f - ALPHA) * imu_data.acc_x;
    imu_data.acc_y = ALPHA * (sensor_data->Acc1[AXIS_Y] / G_TO_M_S2) + 
                     (1.0f - ALPHA) * imu_data.acc_y;
    imu_data.acc_z = ALPHA * (sensor_data->Acc1[AXIS_Z] / G_TO_M_S2) + 
                     (1.0f - ALPHA) * imu_data.acc_z;
    
    // ���������ݴ�����ƫ������
    imu_data.gyro_x = (sensor_data->Rate1[AXIS_X] - gyro_offset.x) * DEG_TO_RAD;
    imu_data.gyro_y = (sensor_data->Rate1[AXIS_Y] - gyro_offset.y) * DEG_TO_RAD;
    imu_data.gyro_z = (sensor_data->Rate1[AXIS_Z] - gyro_offset.z) * DEG_TO_RAD;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     AHRS��Ԫ�����£���PI������Ư�Ʋ�����
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
    
    // ��Ԫ��һ������-����������
    float delta_2 = (2.0f * half_t * gx) * (2.0f * half_t * gx) +
                    (2.0f * half_t * gy) * (2.0f * half_t * gy) +
                    (2.0f * half_t * gz) * (2.0f * half_t * gz);
    
    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * half_t;
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * half_t;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * half_t;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * half_t;
    
    // ��������
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
// �������     ��Ԫ��תŷ���ǣ���������ۻ�����
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
    euler_angles.pitch = asin(-2.0f * q1 * q3 + 2.0f * q0 * q2) * 180.0f / GNSS_PI;
    
    // �����ת�� (Roll)
    euler_angles.roll = atan2(2.0f * q2 * q3 + 2.0f * q0 * q1, 
                              -2.0f * q1 * q1 - 2.0f * q2 * q2 + 1.0f) * 180.0f / GNSS_PI;
    
    // ����ƫ���� (Yaw)
    euler_angles.yaw = atan2(2.0f * q1 * q2 + 2.0f * q0 * q3, 
                             -2.0f * q2 * q2 - 2.0f * q3 * q3 + 1.0f) * 180.0f / GNSS_PI;
    
    // �Ƕȷ�Χ���Ƶ� [-180, 180]
    if(euler_angles.yaw >= 180.0f)
        euler_angles.yaw -= 360.0f;
    else if(euler_angles.yaw <= -180.0f)
        euler_angles.yaw += 360.0f;
    
    // ������ۻ����������Խ��180��������
    if((euler_angles.yaw - last_yaw) < -350.0f)
        direction_change++;
    else if((euler_angles.yaw - last_yaw) > 350.0f)
        direction_change--;
    
    // �����ۻ������
    euler_angles.yaw_accumulated = -(360.0f * direction_change + euler_angles.yaw - yaw_offset);
    
    last_yaw = euler_angles.yaw;
}

