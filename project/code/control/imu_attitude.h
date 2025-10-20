/*********************************************************************************************************************
* �ļ�����          imu_attitude.h  
* ����˵��          IMU��̬����ϵͳͷ�ļ�
* ����              LittleMaster
* �汾��Ϣ          v1.0
* �޸ļ�¼
* ����              ����                �汾
* 2025-09-28        LittleMaster       1.0v
*
* �ļ�����˵����
* ���ļ���IMU��̬����ϵͳ��ͷ�ļ������������ݽṹ�������ӿںͳ�������
*
* ��Ҫ���ܣ�
* 1. IMU���ݽṹ����
* 2. ��̬�����㷨�ӿ�
* 3. ������У׼�ӿ�
* 4. ״̬��ѯ�ӿ�
********************************************************************************************************************/

#ifndef _IMU_ATTITUDE_H_
#define _IMU_ATTITUDE_H_

#include "zf_common_typedef.h"

//=================================================���ò�������================================================
#define IMU_ATTITUDE_UPDATE_FREQ        (1000.0f)        // ��̬����Ƶ�� (Hz) 
#define IMU_COMPLEMENTARY_ALPHA         (0.98f)         // �����˲���ϵ��
#define IMU_MAHONY_KP                   (2.0f)          // Mahony�˲�����������
#define IMU_MAHONY_KI                   (0.0f)          // Mahony�˲�����������
#define IMU_GYRO_THRESHOLD              (0.01f)         // ��������ֵ (rad/s)
#define IMU_ACCEL_THRESHOLD             (0.98f)         // ���ٶȼ���ֵ (m/s?)
#define IMU_GYRO_LPF_ALPHA              (0.2f)          // �����ǵ�ͨ�˲�ϵ�� (0~1)
#define IMU_ACCEL_LPF_ALPHA             (0.2f)          // ���ٶȼƵ�ͨ�˲�ϵ�� (0~1)

// �������˲�������
#define IMU_KALMAN_Q_GYRO               (0.001f)        // �����ǹ�������
#define IMU_KALMAN_Q_BIAS               (0.0001f)       // ������ƫ���������
#define IMU_KALMAN_R_ACCEL              (0.5f)          // ���ٶȼƲ�������
#define IMU_KALMAN_STATE_SIZE           (7)             // ״̬����ά�� (��Ԫ��4 + ������ƫ��3)
#define IMU_KALMAN_MEASUREMENT_SIZE     (3)             // ��������ά�� (���ٶȼ�3��)

// Madgwick�˲�������
#define IMU_MADGWICK_BETA_DEFAULT       (0.033f)        // Madgwick�㷨�²��� (�����Լ����������ж�)
#define IMU_MADGWICK_BETA_MIN           (0.01f)         // ����Сֵ
#define IMU_MADGWICK_BETA_MAX           (0.5f)          // �����ֵ
#define IMU_MADGWICK_ZETA               (0.0f)          // Madgwick�㷨�Ʋ���

// UKF�˲�������
#define IMU_UKF_STATE_SIZE              (7)             // UKF״̬����ά�� (��Ԫ��4 + ������ƫ��3)
#define IMU_UKF_MEASUREMENT_SIZE        (3)             // UKF��������ά�� (���ٶȼ�3��)
#define IMU_UKF_SIGMA_POINTS            (15)            // Sigma������ (2n+1)
#define IMU_UKF_ALPHA                   (0.1f)          // UKF���Ų����� (�����Ը���Sigma��ֲ�)
#define IMU_UKF_BETA                    (2.0f)          // UKF�ֲ�������
#define IMU_UKF_KAPPA                   (0.0f)          // UKF���Ų�����
#define IMU_UKF_Q_GYRO                  (0.001f)        // UKF�����ǹ�������
#define IMU_UKF_Q_BIAS                  (0.0001f)       // UKF������ƫ���������
#define IMU_UKF_R_ACCEL                 (0.1f)          // UKF���ٶȼƲ������� (����������ȶ���)

// ����M_PI�ظ�����
#ifndef M_PI
#define M_PI                            (3.1415926f)     // Բ����
#endif

// �������ٶȶ��壨��׼�������ٶȣ�
#define IMU_GRAVITY_ACCEL               (9.8f)         // �������ٶ� (m/s?)

//=================================================ö�����Ͷ���================================================
// IMU״̬ö��
typedef enum
{
    IMU_STATUS_OK           = 0,    // ����״̬
    IMU_STATUS_NOT_INIT     = 1,    // δ��ʼ��
    IMU_STATUS_CALIBRATING  = 2,    // У׼��
    IMU_STATUS_ERROR        = 3,    // ����״̬
    IMU_STATUS_DATA_INVALID = 4,    // ������Ч
} imu_status_enum;

// IMU����ģʽö��
typedef enum
{
    IMU_MODE_COMPLEMENTARY  = 0,    // �����˲���
    IMU_MODE_MAHONY        = 1,     // Mahony�˲���
    IMU_MODE_SIMPLE_GYRO   = 2,     // �������ǻ���
    IMU_MODE_KALMAN        = 3,     // �������˲���
    IMU_MODE_MADGWICK      = 4,     // Madgwick�˲���
    IMU_MODE_UKF           = 5,     // �޼��������˲���
} imu_calc_mode_enum;

//=================================================���ݽṹ����================================================
// �������ݽṹ
typedef struct
{
    float x;
    float y;
    float z;
} imu_vector3_t;

// ŷ���ǽṹ�� (����)
typedef struct
{
    float pitch_deg;    // ������ (��)
    float roll_deg;     // ��ת�� (��)
    float yaw_deg;      // ƫ���� (��)
} imu_euler_t;

// ��Ԫ���ṹ��
typedef struct
{
    float w, x, y, z;
} imu_quaternion_t;

// IMUԭʼ���ݽṹ��
typedef struct
{
    imu_vector3_t gyro_raw;         // ������ԭʼ���� (rad/s)
    imu_vector3_t accel_raw;        // ���ٶȼ�ԭʼ���� (m/s?)
    imu_vector3_t gyro_bias;        // ��������ƫ
    imu_vector3_t accel_bias;       // ���ٶȼ���ƫ
    uint8 data_valid;               // ������Ч��־
    uint32 timestamp;               // ʱ���
} imu_raw_data_t;

// IMU��������ݽṹ��  
typedef struct
{
    imu_vector3_t gyro_filtered;    // �˲�������������
    imu_vector3_t accel_filtered;   // �˲�����ٶȼ�����
    imu_euler_t euler_angles;       // ŷ���� (��)
    imu_quaternion_t quaternion;    // ��Ԫ��
    float dt;                       // ʱ���� (s)
} imu_processed_data_t;

// IMUУ׼�����ṹ��
typedef struct
{
    uint8 calibration_enabled;      // У׼ʹ��
    uint32 calibration_time;        // У׼����ʱ�� (ms)
    uint32 calibration_start_time;  // У׼��ʼʱ��
    uint32 calibration_samples;     // У׼������
    imu_vector3_t gyro_sum;         // �������ۼ�ֵ
    imu_vector3_t accel_sum;        // ���ٶȼ��ۼ�ֵ
    float reference_gravity;        // �ο��������ٶȷ�ֵ (����ӦУ׼�õ�)
} imu_calibration_t;

// �������˲���״̬�ṹ��
typedef struct
{
    // ״̬���� [q0, q1, q2, q3, bx, by, bz]^T
    // q0-q3: ��Ԫ�� (w, x, y, z)
    // bx-bz: ������ƫ��
    float state[IMU_KALMAN_STATE_SIZE];
    
    // ״̬Э������� P (7x7)
    float P[IMU_KALMAN_STATE_SIZE][IMU_KALMAN_STATE_SIZE];
    
    // ��������Э������� Q (7x7)
    float Q[IMU_KALMAN_STATE_SIZE][IMU_KALMAN_STATE_SIZE];
    
    // ��������Э������� R (3x3)
    float R[IMU_KALMAN_MEASUREMENT_SIZE][IMU_KALMAN_MEASUREMENT_SIZE];
    
    // ��ʱ�������
    float H[IMU_KALMAN_MEASUREMENT_SIZE][IMU_KALMAN_STATE_SIZE];    // �۲����
    float K[IMU_KALMAN_STATE_SIZE][IMU_KALMAN_MEASUREMENT_SIZE];    // ����������
    float S[IMU_KALMAN_MEASUREMENT_SIZE][IMU_KALMAN_MEASUREMENT_SIZE];  // ����Э����
    
    // ״̬��־
    uint8 initialized;               // �˲�����ʼ����־
    uint32 update_count;            // ���¼�����
} imu_kalman_filter_t;

// Madgwick�˲���״̬�ṹ��
typedef struct
{
    float beta;                         // �㷨�������
    float zeta;                         // ������Ư�Ʋ�������
    imu_vector3_t w_err_integral;       // ������������
    uint8 initialized;                  // �˲�����ʼ����־
    uint32 update_count;               // ���¼�����
} imu_madgwick_filter_t;

// UKF�˲���״̬�ṹ��
typedef struct
{
    // ״̬���� [q0, q1, q2, q3, bx, by, bz]^T
    float state[IMU_UKF_STATE_SIZE];
    
    // ״̬Э������� P (7x7)
    float P[IMU_UKF_STATE_SIZE][IMU_UKF_STATE_SIZE];
    
    // ��������Э������� Q (7x7)
    float Q[IMU_UKF_STATE_SIZE][IMU_UKF_STATE_SIZE];
    
    // ��������Э������� R (3x3)
    float R[IMU_UKF_MEASUREMENT_SIZE][IMU_UKF_MEASUREMENT_SIZE];
    
    // Sigma�㼯��
    float sigma_points[IMU_UKF_SIGMA_POINTS][IMU_UKF_STATE_SIZE];
    float sigma_weights_mean[IMU_UKF_SIGMA_POINTS];     // ��ֵȨ��
    float sigma_weights_cov[IMU_UKF_SIGMA_POINTS];      // Э����Ȩ��
    
    // Ԥ��Ĳ���ֵ
    float predicted_measurements[IMU_UKF_SIGMA_POINTS][IMU_UKF_MEASUREMENT_SIZE];
    
    // UKF����
    float alpha;                        // ���Ų���
    float beta;                         // �ֲ�����
    float kappa;                        // ���Ų���
    float lambda;                       // �ϳ����Ų���
    
    // ״̬��־
    uint8 initialized;                  // �˲�����ʼ����־
    uint32 update_count;               // ���¼�����
} imu_ukf_filter_t;

// IMUϵͳ���ṹ��
typedef struct
{
    imu_raw_data_t raw_data;        // ԭʼ����
    imu_processed_data_t processed_data; // ���������
    imu_calibration_t calibration;  // У׼����
    imu_calc_mode_enum calc_mode;   // ����ģʽ
    imu_status_enum status;         // ϵͳ״̬
    float complementary_alpha;      // �����˲���ϵ��
    float mahony_kp, mahony_ki;     // Mahony�˲�������
    imu_vector3_t integral_error;   // Mahony�������
    imu_kalman_filter_t kalman;     // �������˲���
    imu_madgwick_filter_t madgwick; // Madgwick�˲���
    imu_ukf_filter_t ukf;           // UKF�˲���
    uint32 last_update_time;        // �ϴθ���ʱ��
    uint8 system_initialized;       // ϵͳ��ʼ����־
} imu_attitude_system_t;

//=================================================ȫ�ֱ�������================================================
extern imu_attitude_system_t imu_attitude_system;

//=================================================��������================================================
//-------------------------------------------------------------------------------------------------------------------
// �������     ��ʼ��IMU��̬����ϵͳ
// ����˵��     calc_mode           ����ģʽ
// ���ز���     imu_status_enum     ��ʼ��״̬
// ʹ��ʾ��     imu_attitude_init(IMU_MODE_COMPLEMENTARY);
// ��ע��Ϣ     ���IMUϵͳ�ĳ�ʼ������
//-------------------------------------------------------------------------------------------------------------------
imu_status_enum imu_attitude_init(imu_calc_mode_enum calc_mode);

//-------------------------------------------------------------------------------------------------------------------
// �������     ����IMU���������ݣ�֧���ֶ�������Զ��ɼ���
// ����˵��     gyro_x, gyro_y, gyro_z      �������������� (rad/s)
// ����˵��     accel_x, accel_y, accel_z   ���ٶȼ��������� (m/s?)
// ���ز���     imu_status_enum             ����״̬
// ʹ��ʾ��     imu_update_sensor_data(gx, gy, gz, ax, ay, az);
// ��ע��Ϣ     ����IMU���������ݵ�ϵͳ�У�֧���ֶ���������
//-------------------------------------------------------------------------------------------------------------------
imu_status_enum imu_update_sensor_data(float gyro_x, float gyro_y, float gyro_z, 
                                       float accel_x, float accel_y, float accel_z);

//-------------------------------------------------------------------------------------------------------------------
// �������     ִ��IMU��̬�������
// ����˵��     void
// ���ز���     imu_status_enum     ����״̬
// ʹ��ʾ��     imu_attitude_update();
// ��ע��Ϣ     ִ����̬���㣬����ŷ���Ǻ���Ԫ��
//-------------------------------------------------------------------------------------------------------------------
imu_status_enum imu_attitude_update(void);

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ��̬���ݣ�ŷ���Ǻ���Ԫ����
// ����˵��     euler               ŷ�������ָ�루��ΪNULL��
// ����˵��     quaternion          ��Ԫ�����ָ�루��ΪNULL��
// ���ز���     imu_status_enum     ��ȡ״̬
// ʹ��ʾ��     imu_get_attitude_data(&euler, &quat); �� imu_get_attitude_data(&euler, NULL);
// ��ע��Ϣ     ��ȡ��ǰ����õ�����̬���ݣ���ѡ��ֻ��ȡŷ���ǻ���Ԫ��
//-------------------------------------------------------------------------------------------------------------------
imu_status_enum imu_get_attitude_data(imu_euler_t *euler, imu_quaternion_t *quaternion);

//-------------------------------------------------------------------------------------------------------------------
// �������     ����IMUУ׼
// ����˵��     calibration_time    У׼ʱ�� (ms)
// ���ز���     imu_status_enum     У׼״̬
// ʹ��ʾ��     imu_start_calibration(10000); // У׼10��
// ��ע��Ϣ     ���������Ǻͼ��ٶȼ�У׼��У׼�ڼ���Ҫ�����豸��ֹ
//-------------------------------------------------------------------------------------------------------------------
imu_status_enum imu_start_calibration(uint32 calibration_time);


//-------------------------------------------------------------------------------------------------------------------
// �������     ������̬����
// ����˵��     void
// ���ز���     imu_status_enum     ����״̬
// ʹ��ʾ��     imu_attitude_reset();
// ��ע��Ϣ     ������̬�Ƕ�Ϊ��ʼ״̬
//-------------------------------------------------------------------------------------------------------------------
imu_status_enum imu_attitude_reset(void);



//-------------------------------------------------------------------------------------------------------------------
// �������     ��SCH16TK10�������Զ��ɼ�����
// ����˵��     void
// ���ز���     imu_status_enum     �ɼ�״̬
// ʹ��ʾ��     imu_update_from_sensor();
// ��ע��Ϣ     �Զ���ȡSCH16TK10���������ݲ����µ�ϵͳ��������1ms�ж��е���
//-------------------------------------------------------------------------------------------------------------------
imu_status_enum imu_update_from_sensor(void);

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ʼ��SCH16TK10������
// ����˵��     void
// ���ز���     imu_status_enum     ��ʼ��״̬
// ʹ��ʾ��     imu_init_sensor();
// ��ע��Ϣ     ��ʼ��SCH16TK10������Ӳ�������ò���
//-------------------------------------------------------------------------------------------------------------------
imu_status_enum imu_init_sensor(void);

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ�������˲���״̬��Ϣ
// ����˵��     void
// ���ز���     imu_kalman_filter_t*    �������˲���ָ�루���ڿ�����ģʽ����Ч��
// ʹ��ʾ��     imu_kalman_filter_t *kf = imu_get_kalman_info();
// ��ע��Ϣ     ��ȡ�������˲������ڲ�״̬�����ڵ��Ժͼ��
//-------------------------------------------------------------------------------------------------------------------
imu_kalman_filter_t* imu_get_kalman_info(void);

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡMadgwick�˲���״̬��Ϣ
// ����˵��     void
// ���ز���     imu_madgwick_filter_t*  Madgwick�˲���ָ�루����Madgwickģʽ����Ч��
// ʹ��ʾ��     imu_madgwick_filter_t *mf = imu_get_madgwick_info();
// ��ע��Ϣ     ��ȡMadgwick�˲������ڲ�״̬�����ڵ��Ժͼ��
//-------------------------------------------------------------------------------------------------------------------
imu_madgwick_filter_t* imu_get_madgwick_info(void);

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡUKF�˲���״̬��Ϣ
// ����˵��     void
// ���ز���     imu_ukf_filter_t*       UKF�˲���ָ�루����UKFģʽ����Ч��
// ʹ��ʾ��     imu_ukf_filter_t *ukf = imu_get_ukf_info();
// ��ע��Ϣ     ��ȡUKF�˲������ڲ�״̬�����ڵ��Ժͼ��
//-------------------------------------------------------------------------------------------------------------------
imu_ukf_filter_t* imu_get_ukf_info(void);

//-------------------------------------------------------------------------------------------------------------------
// �������     ����Madgwick�˲�������
// ����˵��     beta                    �㷨������� (0.01~0.5)
// ����˵��     zeta                    ������Ư�Ʋ������� (ͨ��Ϊ0)
// ���ز���     imu_status_enum         ����״̬
// ʹ��ʾ��     imu_set_madgwick_params(0.1f, 0.0f);
// ��ע��Ϣ     ����Madgwick�㷨������betaԽ������Խ�쵫����Խ��
//-------------------------------------------------------------------------------------------------------------------
imu_status_enum imu_set_madgwick_params(float beta, float zeta);

//-------------------------------------------------------------------------------------------------------------------
// �������     ����UKF�˲�������
// ����˵��     alpha                   ���Ų��� (0.001~1.0)
// ����˵��     beta                    �ֲ����� (ͨ��Ϊ2.0)
// ����˵��     kappa                   ���Ų��� (ͨ��Ϊ0)
// ���ز���     imu_status_enum         ����״̬
// ʹ��ʾ��     imu_set_ukf_params(0.001f, 2.0f, 0.0f);
// ��ע��Ϣ     ����UKF�㷨������Ӱ��Sigma��ķֲ���Ȩ��
//-------------------------------------------------------------------------------------------------------------------
imu_status_enum imu_set_ukf_params(float alpha, float beta, float kappa);

#endif // _IMU_ATTITUDE_H_