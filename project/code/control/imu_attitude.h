/*********************************************************************************************************************
* 文件名称          imu_attitude.h  
* 功能说明          IMU姿态计算系统头文件
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本
* 2025-09-28        LittleMaster       1.0v
*
* 文件作用说明：
* 本文件是IMU姿态计算系统的头文件，定义了数据结构、函数接口和常量定义
*
* 主要功能：
* 1. IMU数据结构定义
* 2. 姿态解算算法接口
* 3. 传感器校准接口
* 4. 状态查询接口
********************************************************************************************************************/

#ifndef _IMU_ATTITUDE_H_
#define _IMU_ATTITUDE_H_

#include "zf_common_typedef.h"

//=================================================配置参数定义================================================
#define IMU_ATTITUDE_UPDATE_FREQ        (1000.0f)        // 姿态更新频率 (Hz) 
#define IMU_COMPLEMENTARY_ALPHA         (0.98f)         // 互补滤波器系数
#define IMU_MAHONY_KP                   (2.0f)          // Mahony滤波器比例增益
#define IMU_MAHONY_KI                   (0.0f)          // Mahony滤波器积分增益
#define IMU_GYRO_THRESHOLD              (0.01f)         // 陀螺仪阈值 (rad/s)
#define IMU_ACCEL_THRESHOLD             (0.98f)         // 加速度计阈值 (m/s?)
#define IMU_GYRO_LPF_ALPHA              (0.2f)          // 陀螺仪低通滤波系数 (0~1)
#define IMU_ACCEL_LPF_ALPHA             (0.2f)          // 加速度计低通滤波系数 (0~1)

// 卡尔曼滤波器参数
#define IMU_KALMAN_Q_GYRO               (0.001f)        // 陀螺仪过程噪声
#define IMU_KALMAN_Q_BIAS               (0.0001f)       // 陀螺仪偏差过程噪声
#define IMU_KALMAN_R_ACCEL              (0.5f)          // 加速度计测量噪声
#define IMU_KALMAN_STATE_SIZE           (7)             // 状态向量维度 (四元数4 + 陀螺仪偏差3)
#define IMU_KALMAN_MEASUREMENT_SIZE     (3)             // 测量向量维度 (加速度计3轴)

// Madgwick滤波器参数
#define IMU_MADGWICK_BETA_DEFAULT       (0.033f)        // Madgwick算法β参数 (降低以减少噪声敏感度)
#define IMU_MADGWICK_BETA_MIN           (0.01f)         // β最小值
#define IMU_MADGWICK_BETA_MAX           (0.5f)          // β最大值
#define IMU_MADGWICK_ZETA               (0.0f)          // Madgwick算法ζ参数

// UKF滤波器参数
#define IMU_UKF_STATE_SIZE              (7)             // UKF状态向量维度 (四元数4 + 陀螺仪偏差3)
#define IMU_UKF_MEASUREMENT_SIZE        (3)             // UKF测量向量维度 (加速度计3轴)
#define IMU_UKF_SIGMA_POINTS            (15)            // Sigma点数量 (2n+1)
#define IMU_UKF_ALPHA                   (0.1f)          // UKF缩放参数α (增大以改善Sigma点分布)
#define IMU_UKF_BETA                    (2.0f)          // UKF分布参数β
#define IMU_UKF_KAPPA                   (0.0f)          // UKF缩放参数κ
#define IMU_UKF_Q_GYRO                  (0.001f)        // UKF陀螺仪过程噪声
#define IMU_UKF_Q_BIAS                  (0.0001f)       // UKF陀螺仪偏差过程噪声
#define IMU_UKF_R_ACCEL                 (0.1f)          // UKF加速度计测量噪声 (降低以提高稳定性)

// 避免M_PI重复定义
#ifndef M_PI
#define M_PI                            (3.1415926f)     // 圆周率
#endif

// 重力加速度定义（标准重力加速度）
#define IMU_GRAVITY_ACCEL               (9.8f)         // 重力加速度 (m/s?)

//=================================================枚举类型定义================================================
// IMU状态枚举
typedef enum
{
    IMU_STATUS_OK           = 0,    // 正常状态
    IMU_STATUS_NOT_INIT     = 1,    // 未初始化
    IMU_STATUS_CALIBRATING  = 2,    // 校准中
    IMU_STATUS_ERROR        = 3,    // 错误状态
    IMU_STATUS_DATA_INVALID = 4,    // 数据无效
} imu_status_enum;

// IMU计算模式枚举
typedef enum
{
    IMU_MODE_COMPLEMENTARY  = 0,    // 互补滤波器
    IMU_MODE_MAHONY        = 1,     // Mahony滤波器
    IMU_MODE_SIMPLE_GYRO   = 2,     // 简单陀螺仪积分
    IMU_MODE_KALMAN        = 3,     // 卡尔曼滤波器
    IMU_MODE_MADGWICK      = 4,     // Madgwick滤波器
    IMU_MODE_UKF           = 5,     // 无迹卡尔曼滤波器
} imu_calc_mode_enum;

//=================================================数据结构定义================================================
// 三轴数据结构
typedef struct
{
    float x;
    float y;
    float z;
} imu_vector3_t;

// 欧拉角结构体 (度数)
typedef struct
{
    float pitch_deg;    // 俯仰角 (度)
    float roll_deg;     // 滚转角 (度)
    float yaw_deg;      // 偏航角 (度)
} imu_euler_t;

// 四元数结构体
typedef struct
{
    float w, x, y, z;
} imu_quaternion_t;

// IMU原始数据结构体
typedef struct
{
    imu_vector3_t gyro_raw;         // 陀螺仪原始数据 (rad/s)
    imu_vector3_t accel_raw;        // 加速度计原始数据 (m/s?)
    imu_vector3_t gyro_bias;        // 陀螺仪零偏
    imu_vector3_t accel_bias;       // 加速度计零偏
    uint8 data_valid;               // 数据有效标志
    uint32 timestamp;               // 时间戳
} imu_raw_data_t;

// IMU处理后数据结构体  
typedef struct
{
    imu_vector3_t gyro_filtered;    // 滤波后陀螺仪数据
    imu_vector3_t accel_filtered;   // 滤波后加速度计数据
    imu_euler_t euler_angles;       // 欧拉角 (度)
    imu_quaternion_t quaternion;    // 四元数
    float dt;                       // 时间间隔 (s)
} imu_processed_data_t;

// IMU校准参数结构体
typedef struct
{
    uint8 calibration_enabled;      // 校准使能
    uint32 calibration_time;        // 校准持续时间 (ms)
    uint32 calibration_start_time;  // 校准开始时间
    uint32 calibration_samples;     // 校准样本数
    imu_vector3_t gyro_sum;         // 陀螺仪累计值
    imu_vector3_t accel_sum;        // 加速度计累计值
    float reference_gravity;        // 参考重力加速度幅值 (自适应校准得到)
} imu_calibration_t;

// 卡尔曼滤波器状态结构体
typedef struct
{
    // 状态向量 [q0, q1, q2, q3, bx, by, bz]^T
    // q0-q3: 四元数 (w, x, y, z)
    // bx-bz: 陀螺仪偏差
    float state[IMU_KALMAN_STATE_SIZE];
    
    // 状态协方差矩阵 P (7x7)
    float P[IMU_KALMAN_STATE_SIZE][IMU_KALMAN_STATE_SIZE];
    
    // 过程噪声协方差矩阵 Q (7x7)
    float Q[IMU_KALMAN_STATE_SIZE][IMU_KALMAN_STATE_SIZE];
    
    // 测量噪声协方差矩阵 R (3x3)
    float R[IMU_KALMAN_MEASUREMENT_SIZE][IMU_KALMAN_MEASUREMENT_SIZE];
    
    // 临时计算矩阵
    float H[IMU_KALMAN_MEASUREMENT_SIZE][IMU_KALMAN_STATE_SIZE];    // 观测矩阵
    float K[IMU_KALMAN_STATE_SIZE][IMU_KALMAN_MEASUREMENT_SIZE];    // 卡尔曼增益
    float S[IMU_KALMAN_MEASUREMENT_SIZE][IMU_KALMAN_MEASUREMENT_SIZE];  // 创新协方差
    
    // 状态标志
    uint8 initialized;               // 滤波器初始化标志
    uint32 update_count;            // 更新计数器
} imu_kalman_filter_t;

// Madgwick滤波器状态结构体
typedef struct
{
    float beta;                         // 算法增益参数
    float zeta;                         // 陀螺仪漂移补偿参数
    imu_vector3_t w_err_integral;       // 陀螺仪误差积分
    uint8 initialized;                  // 滤波器初始化标志
    uint32 update_count;               // 更新计数器
} imu_madgwick_filter_t;

// UKF滤波器状态结构体
typedef struct
{
    // 状态向量 [q0, q1, q2, q3, bx, by, bz]^T
    float state[IMU_UKF_STATE_SIZE];
    
    // 状态协方差矩阵 P (7x7)
    float P[IMU_UKF_STATE_SIZE][IMU_UKF_STATE_SIZE];
    
    // 过程噪声协方差矩阵 Q (7x7)
    float Q[IMU_UKF_STATE_SIZE][IMU_UKF_STATE_SIZE];
    
    // 测量噪声协方差矩阵 R (3x3)
    float R[IMU_UKF_MEASUREMENT_SIZE][IMU_UKF_MEASUREMENT_SIZE];
    
    // Sigma点集合
    float sigma_points[IMU_UKF_SIGMA_POINTS][IMU_UKF_STATE_SIZE];
    float sigma_weights_mean[IMU_UKF_SIGMA_POINTS];     // 均值权重
    float sigma_weights_cov[IMU_UKF_SIGMA_POINTS];      // 协方差权重
    
    // 预测的测量值
    float predicted_measurements[IMU_UKF_SIGMA_POINTS][IMU_UKF_MEASUREMENT_SIZE];
    
    // UKF参数
    float alpha;                        // 缩放参数
    float beta;                         // 分布参数
    float kappa;                        // 缩放参数
    float lambda;                       // 合成缩放参数
    
    // 状态标志
    uint8 initialized;                  // 滤波器初始化标志
    uint32 update_count;               // 更新计数器
} imu_ukf_filter_t;

// IMU系统主结构体
typedef struct
{
    imu_raw_data_t raw_data;        // 原始数据
    imu_processed_data_t processed_data; // 处理后数据
    imu_calibration_t calibration;  // 校准参数
    imu_calc_mode_enum calc_mode;   // 计算模式
    imu_status_enum status;         // 系统状态
    float complementary_alpha;      // 互补滤波器系数
    float mahony_kp, mahony_ki;     // Mahony滤波器参数
    imu_vector3_t integral_error;   // Mahony积分误差
    imu_kalman_filter_t kalman;     // 卡尔曼滤波器
    imu_madgwick_filter_t madgwick; // Madgwick滤波器
    imu_ukf_filter_t ukf;           // UKF滤波器
    uint32 last_update_time;        // 上次更新时间
    uint8 system_initialized;       // 系统初始化标志
} imu_attitude_system_t;

//=================================================全局变量声明================================================
extern imu_attitude_system_t imu_attitude_system;

//=================================================函数声明================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化IMU姿态计算系统
// 参数说明     calc_mode           计算模式
// 返回参数     imu_status_enum     初始化状态
// 使用示例     imu_attitude_init(IMU_MODE_COMPLEMENTARY);
// 备注信息     完成IMU系统的初始化设置
//-------------------------------------------------------------------------------------------------------------------
imu_status_enum imu_attitude_init(imu_calc_mode_enum calc_mode);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新IMU传感器数据（支持手动输入或自动采集）
// 参数说明     gyro_x, gyro_y, gyro_z      陀螺仪三轴数据 (rad/s)
// 参数说明     accel_x, accel_y, accel_z   加速度计三轴数据 (m/s?)
// 返回参数     imu_status_enum             更新状态
// 使用示例     imu_update_sensor_data(gx, gy, gz, ax, ay, az);
// 备注信息     更新IMU传感器数据到系统中，支持手动输入数据
//-------------------------------------------------------------------------------------------------------------------
imu_status_enum imu_update_sensor_data(float gyro_x, float gyro_y, float gyro_z, 
                                       float accel_x, float accel_y, float accel_z);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     执行IMU姿态计算更新
// 参数说明     void
// 返回参数     imu_status_enum     计算状态
// 使用示例     imu_attitude_update();
// 备注信息     执行姿态解算，更新欧拉角和四元数
//-------------------------------------------------------------------------------------------------------------------
imu_status_enum imu_attitude_update(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取姿态数据（欧拉角和四元数）
// 参数说明     euler               欧拉角输出指针（可为NULL）
// 参数说明     quaternion          四元数输出指针（可为NULL）
// 返回参数     imu_status_enum     获取状态
// 使用示例     imu_get_attitude_data(&euler, &quat); 或 imu_get_attitude_data(&euler, NULL);
// 备注信息     获取当前计算得到的姿态数据，可选择只获取欧拉角或四元数
//-------------------------------------------------------------------------------------------------------------------
imu_status_enum imu_get_attitude_data(imu_euler_t *euler, imu_quaternion_t *quaternion);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     启动IMU校准
// 参数说明     calibration_time    校准时间 (ms)
// 返回参数     imu_status_enum     校准状态
// 使用示例     imu_start_calibration(10000); // 校准10秒
// 备注信息     启动陀螺仪和加速度计校准，校准期间需要保持设备静止
//-------------------------------------------------------------------------------------------------------------------
imu_status_enum imu_start_calibration(uint32 calibration_time);


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     重置姿态解算
// 参数说明     void
// 返回参数     imu_status_enum     重置状态
// 使用示例     imu_attitude_reset();
// 备注信息     重置姿态角度为初始状态
//-------------------------------------------------------------------------------------------------------------------
imu_status_enum imu_attitude_reset(void);



//-------------------------------------------------------------------------------------------------------------------
// 函数简介     从SCH16TK10传感器自动采集数据
// 参数说明     void
// 返回参数     imu_status_enum     采集状态
// 使用示例     imu_update_from_sensor();
// 备注信息     自动读取SCH16TK10传感器数据并更新到系统，建议在1ms中断中调用
//-------------------------------------------------------------------------------------------------------------------
imu_status_enum imu_update_from_sensor(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化SCH16TK10传感器
// 参数说明     void
// 返回参数     imu_status_enum     初始化状态
// 使用示例     imu_init_sensor();
// 备注信息     初始化SCH16TK10传感器硬件和配置参数
//-------------------------------------------------------------------------------------------------------------------
imu_status_enum imu_init_sensor(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取卡尔曼滤波器状态信息
// 参数说明     void
// 返回参数     imu_kalman_filter_t*    卡尔曼滤波器指针（仅在卡尔曼模式下有效）
// 使用示例     imu_kalman_filter_t *kf = imu_get_kalman_info();
// 备注信息     获取卡尔曼滤波器的内部状态，用于调试和监控
//-------------------------------------------------------------------------------------------------------------------
imu_kalman_filter_t* imu_get_kalman_info(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取Madgwick滤波器状态信息
// 参数说明     void
// 返回参数     imu_madgwick_filter_t*  Madgwick滤波器指针（仅在Madgwick模式下有效）
// 使用示例     imu_madgwick_filter_t *mf = imu_get_madgwick_info();
// 备注信息     获取Madgwick滤波器的内部状态，用于调试和监控
//-------------------------------------------------------------------------------------------------------------------
imu_madgwick_filter_t* imu_get_madgwick_info(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取UKF滤波器状态信息
// 参数说明     void
// 返回参数     imu_ukf_filter_t*       UKF滤波器指针（仅在UKF模式下有效）
// 使用示例     imu_ukf_filter_t *ukf = imu_get_ukf_info();
// 备注信息     获取UKF滤波器的内部状态，用于调试和监控
//-------------------------------------------------------------------------------------------------------------------
imu_ukf_filter_t* imu_get_ukf_info(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置Madgwick滤波器参数
// 参数说明     beta                    算法增益参数 (0.01~0.5)
// 参数说明     zeta                    陀螺仪漂移补偿参数 (通常为0)
// 返回参数     imu_status_enum         设置状态
// 使用示例     imu_set_madgwick_params(0.1f, 0.0f);
// 备注信息     调整Madgwick算法参数，beta越大收敛越快但噪声越大
//-------------------------------------------------------------------------------------------------------------------
imu_status_enum imu_set_madgwick_params(float beta, float zeta);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置UKF滤波器参数
// 参数说明     alpha                   缩放参数 (0.001~1.0)
// 参数说明     beta                    分布参数 (通常为2.0)
// 参数说明     kappa                   缩放参数 (通常为0)
// 返回参数     imu_status_enum         设置状态
// 使用示例     imu_set_ukf_params(0.001f, 2.0f, 0.0f);
// 备注信息     调整UKF算法参数，影响Sigma点的分布和权重
//-------------------------------------------------------------------------------------------------------------------
imu_status_enum imu_set_ukf_params(float alpha, float beta, float kappa);

#endif // _IMU_ATTITUDE_H_