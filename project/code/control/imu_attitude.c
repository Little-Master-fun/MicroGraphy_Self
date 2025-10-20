/*********************************************************************************************************************
* 文件名称          imu_attitude.c
* 功能说明          IMU姿态计算系统实现文件
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本
* 2025-09-28        LittleMaster       1.0v
* 
* 文件作用说明：
* 本文件实现IMU姿态计算系统的所有功能，包括陀螺仪和加速度计数据融合、欧拉角计算等
* 
* 主要特性：
* 1. 互补滤波器算法，融合陀螺仪和加速度计数据
* 2. 四元数和欧拉角表示姿态
* 3. 传感器校准和零偏补偿
* 4. 多种滤波算法支持
* 5. 实时状态监控和调试信息
* 6. SCH16TK10传感器数据采集
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

//=================================================全局变量定义================================================
imu_attitude_system_t imu_attitude_system = {0};

// 用于时间测量的定时器
static uint8 imu_timer_initialized = 0;

// SCH16TK10 传感器相关变量
static SCH1_raw_data sch1_raw_data;
static SCH1_raw_data sch1_summed_data;  // 累加数据缓冲区
static SCH1_result sch1_result_data;
static uint8 sch1_initialized = 0;
static uint32 sch1_sample_count = 0;    // 采样计数器

//=================================================内部函数声明================================================
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

// 卡尔曼滤波器相关函数
static void imu_kalman_init(void);
static void imu_kalman_predict(float dt, imu_vector3_t *gyro);
static void imu_kalman_update_accel(imu_vector3_t *accel);
static void imu_matrix_multiply(float *A, float *B, float *C, int m, int n, int p);
static void imu_matrix_transpose(float *A, float *AT, int m, int n);
static void imu_matrix_inverse_3x3(float A[3][3], float Ainv[3][3]);
static void imu_matrix_add(float *A, float *B, float *C, int m, int n);
static void imu_matrix_subtract(float *A, float *B, float *C, int m, int n);

// Madgwick滤波器相关函数
static void imu_madgwick_init(void);
static imu_status_enum imu_madgwick_filter_update(void);
static void imu_madgwick_update_imu(float dt, imu_vector3_t *gyro, imu_vector3_t *accel);
static float imu_inv_sqrt(float x);

// UKF滤波器相关函数
static void imu_ukf_init(void);
static imu_status_enum imu_ukf_filter_update(void);
static void imu_ukf_generate_sigma_points(void);
static void imu_ukf_predict(float dt, imu_vector3_t *gyro);
static void imu_ukf_update_accel(imu_vector3_t *accel);
static void imu_ukf_measurement_function(float *state, float *measurement);
static void imu_matrix_cholesky_decomposition(float A[IMU_UKF_STATE_SIZE][IMU_UKF_STATE_SIZE], 
                                             float L[IMU_UKF_STATE_SIZE][IMU_UKF_STATE_SIZE]);

//=================================================外部接口实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化IMU姿态计算系统
//-------------------------------------------------------------------------------------------------------------------
imu_status_enum imu_attitude_init(imu_calc_mode_enum calc_mode)
{
    // 清空系统结构体
    memset(&imu_attitude_system, 0, sizeof(imu_attitude_system_t));
    
    // 设置初始参数
    imu_attitude_system.calc_mode = calc_mode;
    imu_attitude_system.complementary_alpha = IMU_COMPLEMENTARY_ALPHA;
    imu_attitude_system.mahony_kp = IMU_MAHONY_KP;
    imu_attitude_system.mahony_ki = IMU_MAHONY_KI;
    
    // 设置初始参考重力加速度（校准后会更新）
    imu_attitude_system.calibration.reference_gravity = IMU_GRAVITY_ACCEL;
    
    // 初始化四元数为单位四元数
    imu_attitude_system.processed_data.quaternion.w = 1.0f;
    imu_attitude_system.processed_data.quaternion.x = 0.0f;
    imu_attitude_system.processed_data.quaternion.y = 0.0f;
    imu_attitude_system.processed_data.quaternion.z = 0.0f;
    
    // 初始化滤波器
    if(calc_mode == IMU_MODE_KALMAN)
    {
        imu_kalman_init();
        printf("卡尔曼滤波器已初始化\r\n");
    }
    else if(calc_mode == IMU_MODE_MADGWICK)
    {
        imu_madgwick_init();
        printf("Madgwick滤波器已初始化\r\n");
    }
    else if(calc_mode == IMU_MODE_UKF)
    {
        imu_ukf_init();
        printf("UKF滤波器已初始化\r\n");
    }
    
    // 初始化定时器
    imu_init_timer();
    
    // 初始化传感器
    if(imu_init_sensor() != IMU_STATUS_OK)
    {
        printf("SCH16TK10传感器初始化失败\r\n");
        imu_attitude_system.status = IMU_STATUS_ERROR;
        return IMU_STATUS_ERROR;
    }
    
    imu_attitude_system.status = IMU_STATUS_OK;
    imu_attitude_system.system_initialized = 1;
    imu_attitude_system.last_update_time = imu_get_time_ms();
    
    printf("IMU姿态计算系统初始化成功\r\n");
    return IMU_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新IMU传感器数据（支持手动输入或自动采集）
//-------------------------------------------------------------------------------------------------------------------
imu_status_enum imu_update_sensor_data(float gyro_x, float gyro_y, float gyro_z, 
                                       float accel_x, float accel_y, float accel_z)
{
    if(imu_attitude_system.status == IMU_STATUS_NOT_INIT)
        return IMU_STATUS_NOT_INIT;
    
    // 数据有效性检查：检测NaN和Inf
    if(isnan(gyro_x) || isnan(gyro_y) || isnan(gyro_z) ||
       isnan(accel_x) || isnan(accel_y) || isnan(accel_z) ||
       isinf(gyro_x) || isinf(gyro_y) || isinf(gyro_z) ||
       isinf(accel_x) || isinf(accel_y) || isinf(accel_z))
    {
        printf("传感器数据异常(NaN/Inf): gyro(%.3f,%.3f,%.3f) accel(%.3f,%.3f,%.3f)\r\n",
               gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z);
        imu_attitude_system.raw_data.data_valid = 0;
        return IMU_STATUS_DATA_INVALID;
    }
    
    // 合理性检查：陀螺仪范围±2000 deg/s = ±34.9 rad/s
    if(fabs(gyro_x) > 35.0f || fabs(gyro_y) > 35.0f || fabs(gyro_z) > 35.0f)
    {
        printf("陀螺仪数据超出范围: gyro(%.3f,%.3f,%.3f)\r\n", gyro_x, gyro_y, gyro_z);
        imu_attitude_system.raw_data.data_valid = 0;
        return IMU_STATUS_DATA_INVALID;
    }
    
    // 合理性检查：加速度计范围±16g = ±156.8 m/s?
    if(fabs(accel_x) > 156.8f || fabs(accel_y) > 156.8f || fabs(accel_z) > 156.8f)
    {
        printf("加速度计数据超出范围: accel(%.3f,%.3f,%.3f)\r\n", accel_x, accel_y, accel_z);
        imu_attitude_system.raw_data.data_valid = 0;
        return IMU_STATUS_DATA_INVALID;
    }
    
    // 更新原始数据
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
// 函数简介     执行IMU姿态计算更新
//-------------------------------------------------------------------------------------------------------------------
imu_status_enum imu_attitude_update(void)
{
    if(!imu_attitude_system.system_initialized)
        return IMU_STATUS_NOT_INIT;
        
    if(!imu_attitude_system.raw_data.data_valid)
        return IMU_STATUS_DATA_INVALID;
    
    // 更新时间间隔
    imu_update_dt();
    
    // 应用校准参数
    imu_apply_calibration();
    
    // 根据计算模式选择算法
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
    
    // 转换四元数到欧拉角
    imu_quaternion_to_euler(imu_attitude_system.processed_data.quaternion, 
                           &imu_attitude_system.processed_data.euler_angles);
    
    return status;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取姿态数据（欧拉角和四元数）
//-------------------------------------------------------------------------------------------------------------------
imu_status_enum imu_get_attitude_data(imu_euler_t *euler, imu_quaternion_t *quaternion)
{
    if(!imu_attitude_system.system_initialized)
        return IMU_STATUS_ERROR;
    
    // 获取欧拉角（可为NULL）
    if(euler != NULL)
        *euler = imu_attitude_system.processed_data.euler_angles;
    
    // 获取四元数（可为NULL）
    if(quaternion != NULL)
        *quaternion = imu_attitude_system.processed_data.quaternion;
    
    return IMU_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     启动IMU校准
//-------------------------------------------------------------------------------------------------------------------
imu_status_enum imu_start_calibration(uint32 calibration_time)
{
    if(!imu_attitude_system.system_initialized)
        return IMU_STATUS_NOT_INIT;
    
    // 设置校准参数
    imu_attitude_system.calibration.calibration_enabled = 1;
    imu_attitude_system.calibration.calibration_time = calibration_time;
    imu_attitude_system.calibration.calibration_start_time = imu_get_time_ms();
    imu_attitude_system.calibration.calibration_samples = 0;
    
    // 清零累计值
    memset(&imu_attitude_system.calibration.gyro_sum, 0, sizeof(imu_vector3_t));
    memset(&imu_attitude_system.calibration.accel_sum, 0, sizeof(imu_vector3_t));
    
    imu_attitude_system.status = IMU_STATUS_CALIBRATING;
    
    printf("IMU校准开始，请保持设备静止 %d 秒\r\n", calibration_time/1000);
    return IMU_STATUS_CALIBRATING;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     重置姿态解算
//-------------------------------------------------------------------------------------------------------------------
imu_status_enum imu_attitude_reset(void)
{
    // 重置四元数为单位四元数
    imu_attitude_system.processed_data.quaternion.w = 1.0f;
    imu_attitude_system.processed_data.quaternion.x = 0.0f;
    imu_attitude_system.processed_data.quaternion.y = 0.0f;
    imu_attitude_system.processed_data.quaternion.z = 0.0f;
    
    // 重置欧拉角
    memset(&imu_attitude_system.processed_data.euler_angles, 0, sizeof(imu_euler_t));
    
    // 重置积分误差
    memset(&imu_attitude_system.integral_error, 0, sizeof(imu_vector3_t));
    
    return IMU_STATUS_OK;
}



//-------------------------------------------------------------------------------------------------------------------
// 函数简介     从SCH16TK10传感器自动采集数据 (建议在1ms中断中调用)
// 备注信息     实现AVG_FACTOR次采样平均，减少噪声
//-------------------------------------------------------------------------------------------------------------------
imu_status_enum imu_update_from_sensor(void)
{
    if(!sch1_initialized || !imu_attitude_system.system_initialized)
        return IMU_STATUS_NOT_INIT;
        
    // 读取SCH16TK10原始数据
    SCH1_getData(&sch1_raw_data);
    
    // 检查数据帧是否有错误
    if (sch1_raw_data.frame_error)
    {
        return IMU_STATUS_DATA_INVALID;
    }
    
    // 累加原始数据
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
    
    // 采样计数递增
    sch1_sample_count++;
    
    // 当累加次数达到AVG_FACTOR时，转换数据并更新姿态
    if (sch1_sample_count >= AVG_FACTOR)
    {
        // 转换为物理量（自动除以AVG_FACTOR进行平均）
        SCH1_convert_data(&sch1_summed_data, &sch1_result_data);
        
        // 将SCH16TK10数据更新到IMU姿态解算模块
        // SCH16TK10: Rate1为角速度(rad/s), Acc2为加速度(m/s?)
        imu_status_enum status = imu_update_sensor_data(
            sch1_result_data.Rate1[AXIS_X],    // 陀螺仪X轴 (rad/s)
            sch1_result_data.Rate1[AXIS_Y],    // 陀螺仪Y轴 (rad/s) 
            sch1_result_data.Rate1[AXIS_Z],    // 陀螺仪Z轴 (rad/s)
            sch1_result_data.Acc2[AXIS_X],     // 加速度计X轴 (m/s?)
            sch1_result_data.Acc2[AXIS_Y],     // 加速度计Y轴 (m/s?)
            sch1_result_data.Acc2[AXIS_Z]      // 加速度计Z轴 (m/s?)
        );
        
        // 重置采样计数和累加缓冲区
        sch1_sample_count = 0;
        memset(&sch1_summed_data, 0, sizeof(SCH1_raw_data));
        
        return status;
    }
    
    // 尚未累加足够次数，返回OK但不更新姿态
    return IMU_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化SCH16TK10传感器
//-------------------------------------------------------------------------------------------------------------------
imu_status_enum imu_init_sensor(void)
{
    printf("正在初始化SCH16TK10传感器...\r\n");
    
    // 设置SCH16TK10参数（使用config_infineon.h中的配置）
    SCH1_filter sFilter;
    SCH1_sensitivity sSensitivity;
    SCH1_decimation sDecimation;

    // 设置滤波器参数 (Hz)
    sFilter.Rate12 = FILTER_RATE;    // 陀螺仪滤波频率
    sFilter.Acc12 = FILTER_ACC12;    // 加速度计1,2滤波频率
    sFilter.Acc3 = FILTER_ACC3;      // 加速度计3滤波频率

    // 设置灵敏度参数 (LSB/unit)
    sSensitivity.Rate1 = SENSITIVITY_RATE1;   // 陀螺仪1灵敏度
    sSensitivity.Rate2 = SENSITIVITY_RATE2;   // 陀螺仪2灵敏度
    sSensitivity.Acc1 = SENSITIVITY_ACC1;     // 加速度计1灵敏度
    sSensitivity.Acc2 = SENSITIVITY_ACC2;     // 加速度计2灵敏度
    sSensitivity.Acc3 = SENSITIVITY_ACC3;     // 加速度计3灵敏度

    // 设置抽取参数
    sDecimation.Rate2 = DECIMATION_RATE;   // 陀螺仪2抽取率
    sDecimation.Acc2 = DECIMATION_ACC;     // 加速度计2抽取率

    // 初始化传感器
    int init_result = SCH1_init(sFilter, sSensitivity, sDecimation, false);
    
    if (init_result != SCH1_OK)
    {
        printf("SCH16TK10初始化失败，错误码: %d\r\n", init_result);
        
        // 读取状态寄存器进行诊断
        SCH1_status status;
        if (SCH1_getStatus(&status) == SCH1_OK)
        {
            printf("状态诊断 - Summary: 0x%04X, Common: 0x%04X\r\n", 
                   status.Summary, status.Common);
        }
        return IMU_STATUS_ERROR;
    }
    
    printf("SCH16TK10传感器初始化成功！\r\n");
    
    // 测试SPI通信稳定性
    if (SCH1_testSPIStability())
    {
        printf("SPI通信测试通过\r\n");
    }
    else
    {
        printf("SPI通信测试失败，但继续运行\r\n");
    }
    
    sch1_initialized = 1;
    return IMU_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取卡尔曼滤波器状态信息
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

//=================================================内部函数实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     应用校准参数
//-------------------------------------------------------------------------------------------------------------------
static imu_status_enum imu_apply_calibration(void)
{
    // 检查是否在校准过程中
    if(imu_attitude_system.calibration.calibration_enabled)
    {
        uint32 current_time = imu_get_time_ms();
        uint32 elapsed_time = current_time - imu_attitude_system.calibration.calibration_start_time;
        
        if(elapsed_time < imu_attitude_system.calibration.calibration_time)
        {
            // 校准期间，累计传感器数据
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
            // 校准完成，计算零偏
            if(imu_attitude_system.calibration.calibration_samples > 0)
            {
                float samples = (float)imu_attitude_system.calibration.calibration_samples;
                
                imu_attitude_system.raw_data.gyro_bias.x = imu_attitude_system.calibration.gyro_sum.x / samples;
                imu_attitude_system.raw_data.gyro_bias.y = imu_attitude_system.calibration.gyro_sum.y / samples;
                imu_attitude_system.raw_data.gyro_bias.z = imu_attitude_system.calibration.gyro_sum.z / samples;
                
                // 计算静止时的平均加速度
                float accel_avg_x = imu_attitude_system.calibration.accel_sum.x / samples;
                float accel_avg_y = imu_attitude_system.calibration.accel_sum.y / samples;
                float accel_avg_z = imu_attitude_system.calibration.accel_sum.z / samples;
                
                // 计算静止时的加速度幅值作为参考重力加速度
                imu_attitude_system.calibration.reference_gravity = 
                    sqrtf(accel_avg_x * accel_avg_x + accel_avg_y * accel_avg_y + accel_avg_z * accel_avg_z);
                
                // 加速度计校准：假设校准期间设备静止，计算零偏
                imu_attitude_system.raw_data.accel_bias.x = accel_avg_x;
                imu_attitude_system.raw_data.accel_bias.y = accel_avg_y;
                imu_attitude_system.raw_data.accel_bias.z = accel_avg_z - imu_attitude_system.calibration.reference_gravity;
                
                printf("IMU校准完成！\r\n");
                printf("陀螺仪零偏: [%.4f, %.4f, %.4f] rad/s\r\n", 
                       imu_attitude_system.raw_data.gyro_bias.x,
                       imu_attitude_system.raw_data.gyro_bias.y,
                       imu_attitude_system.raw_data.gyro_bias.z);
                printf("加速度计零偏: [%.4f, %.4f, %.4f] m/s?\r\n",
                       imu_attitude_system.raw_data.accel_bias.x,
                       imu_attitude_system.raw_data.accel_bias.y,
                       imu_attitude_system.raw_data.accel_bias.z);
                printf("参考重力加速度: %.4f m/s? (自适应校准)\r\n",
                       imu_attitude_system.calibration.reference_gravity);
            }
            
            imu_attitude_system.calibration.calibration_enabled = 0;
            imu_attitude_system.status = IMU_STATUS_OK;
        }
    }
    
    // 应用校准补偿
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
// 函数简介     互补滤波器更新
//-------------------------------------------------------------------------------------------------------------------
static imu_status_enum imu_complementary_filter_update(void)
{
    float alpha = imu_attitude_system.complementary_alpha;
    float dt = imu_attitude_system.processed_data.dt;
    
    // 获取处理后的传感器数据
    imu_vector3_t *gyro = &imu_attitude_system.processed_data.gyro_filtered;
    imu_vector3_t *accel = &imu_attitude_system.processed_data.accel_filtered;
    
    // 加速度计计算出的俯仰角和滚转角
    float acc_pitch = atan2f(accel->x, sqrtf(accel->y * accel->y + accel->z * accel->z));
    float acc_roll = atan2f(-accel->y, accel->z);
    
    // 陀螺仪积分
    imu_attitude_system.processed_data.euler_angles.pitch_deg += gyro->x * dt * 180.0f / M_PI;
    imu_attitude_system.processed_data.euler_angles.roll_deg += gyro->y * dt * 180.0f / M_PI;
    imu_attitude_system.processed_data.euler_angles.yaw_deg += gyro->z * dt * 180.0f / M_PI;
    
    // 互补滤波器融合
    imu_attitude_system.processed_data.euler_angles.pitch_deg = 
        alpha * imu_attitude_system.processed_data.euler_angles.pitch_deg + 
        (1.0f - alpha) * (acc_pitch * 180.0f / M_PI);
        
    imu_attitude_system.processed_data.euler_angles.roll_deg = 
        alpha * imu_attitude_system.processed_data.euler_angles.roll_deg + 
        (1.0f - alpha) * (acc_roll * 180.0f / M_PI);
    
    // 角度归一化
    imu_attitude_system.processed_data.euler_angles.pitch_deg = 
        imu_normalize_angle(imu_attitude_system.processed_data.euler_angles.pitch_deg);
    imu_attitude_system.processed_data.euler_angles.roll_deg = 
        imu_normalize_angle(imu_attitude_system.processed_data.euler_angles.roll_deg);
    imu_attitude_system.processed_data.euler_angles.yaw_deg = 
        imu_normalize_angle(imu_attitude_system.processed_data.euler_angles.yaw_deg);
    
    // 更新四元数
    imu_euler_to_quaternion(imu_attitude_system.processed_data.euler_angles,
                           &imu_attitude_system.processed_data.quaternion);
    
    return IMU_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     Mahony滤波器更新
//-------------------------------------------------------------------------------------------------------------------
static imu_status_enum imu_mahony_filter_update(void)
{
    float dt = imu_attitude_system.processed_data.dt;
    float kp = imu_attitude_system.mahony_kp;
    float ki = imu_attitude_system.mahony_ki;
    
    // 获取处理后的传感器数据
    imu_vector3_t *gyro = &imu_attitude_system.processed_data.gyro_filtered;
    imu_vector3_t *accel = &imu_attitude_system.processed_data.accel_filtered;
    
    // 归一化加速度计数据
    float acc_norm = sqrtf(accel->x * accel->x + accel->y * accel->y + accel->z * accel->z);
    if (acc_norm > 0.0f)
    {
        accel->x /= acc_norm;
        accel->y /= acc_norm;
        accel->z /= acc_norm;
    }
    
    // 当前四元数
    imu_quaternion_t *q = &imu_attitude_system.processed_data.quaternion;
    
    // 从四元数估计重力方向
    float gx = 2.0f * (q->x * q->z - q->w * q->y);
    float gy = 2.0f * (q->w * q->x + q->y * q->z);
    float gz = q->w * q->w - q->x * q->x - q->y * q->y + q->z * q->z;
    
    // 误差向量：加速度计方向与重力估计方向的叉积
    float ex = (accel->y * gz - accel->z * gy);
    float ey = (accel->z * gx - accel->x * gz);
    float ez = (accel->x * gy - accel->y * gx);
    
    // 积分误差
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
    
    // 校正后的陀螺仪数据
    float gx_corrected = gyro->x + kp * ex + ki * imu_attitude_system.integral_error.x;
    float gy_corrected = gyro->y + kp * ey + ki * imu_attitude_system.integral_error.y;
    float gz_corrected = gyro->z + kp * ez + ki * imu_attitude_system.integral_error.z;
    
    // 四元数微分方程
    float qw_dot = 0.5f * (-q->x * gx_corrected - q->y * gy_corrected - q->z * gz_corrected);
    float qx_dot = 0.5f * (q->w * gx_corrected + q->y * gz_corrected - q->z * gy_corrected);
    float qy_dot = 0.5f * (q->w * gy_corrected - q->x * gz_corrected + q->z * gx_corrected);
    float qz_dot = 0.5f * (q->w * gz_corrected + q->x * gy_corrected - q->y * gx_corrected);
    
    // 积分更新四元数
    q->w += qw_dot * dt;
    q->x += qx_dot * dt;
    q->y += qy_dot * dt;
    q->z += qz_dot * dt;
    
    // 归一化四元数
    imu_quaternion_normalize(q);
    
    return IMU_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     简单陀螺仪积分
//-------------------------------------------------------------------------------------------------------------------
static imu_status_enum imu_simple_gyro_integration(void)
{
    float dt = imu_attitude_system.processed_data.dt;
    imu_vector3_t *gyro = &imu_attitude_system.processed_data.gyro_filtered;
    
    // 直接积分陀螺仪数据
    imu_attitude_system.processed_data.euler_angles.pitch_deg += gyro->x * dt * 180.0f / M_PI;
    imu_attitude_system.processed_data.euler_angles.roll_deg += gyro->y * dt * 180.0f / M_PI;
    imu_attitude_system.processed_data.euler_angles.yaw_deg += gyro->z * dt * 180.0f / M_PI;
    
    // 角度归一化
    imu_attitude_system.processed_data.euler_angles.pitch_deg = 
        imu_normalize_angle(imu_attitude_system.processed_data.euler_angles.pitch_deg);
    imu_attitude_system.processed_data.euler_angles.roll_deg = 
        imu_normalize_angle(imu_attitude_system.processed_data.euler_angles.roll_deg);
    imu_attitude_system.processed_data.euler_angles.yaw_deg = 
        imu_normalize_angle(imu_attitude_system.processed_data.euler_angles.yaw_deg);
    
    // 更新四元数
    imu_euler_to_quaternion(imu_attitude_system.processed_data.euler_angles,
                           &imu_attitude_system.processed_data.quaternion);
    
    return IMU_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     四元数转欧拉角
//-------------------------------------------------------------------------------------------------------------------
static void imu_quaternion_to_euler(imu_quaternion_t q, imu_euler_t *euler)
{
    // 滚转角 (绕X轴旋转)
    float sin_r_cp = 2.0f * (q.w * q.x + q.y * q.z);
    float cos_r_cp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    euler->roll_deg = atan2f(sin_r_cp, cos_r_cp) * 180.0f / M_PI;

    // 俯仰角 (绕Y轴旋转)
    float sin_p = 2.0f * (q.w * q.y - q.z * q.x);
    if (fabsf(sin_p) >= 1.0f)
        euler->pitch_deg = copysignf(90.0f, sin_p); // 万向锁情况
    else
        euler->pitch_deg = asinf(sin_p) * 180.0f / M_PI;

    // 偏航角 (绕Z轴旋转)
    float sin_y_cp = 2.0f * (q.w * q.z + q.x * q.y);
    float cos_y_cp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    euler->yaw_deg = atan2f(sin_y_cp, cos_y_cp) * 180.0f / M_PI;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     欧拉角转四元数
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
// 函数简介     四元数归一化
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
// 函数简介     角度归一化
//-------------------------------------------------------------------------------------------------------------------
static float imu_normalize_angle(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新时间间隔
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
// 函数简介     获取当前时间（毫秒）
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
// 函数简介     初始化时间测量定时器
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

//=================================================卡尔曼滤波器实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化卡尔曼滤波器
//-------------------------------------------------------------------------------------------------------------------
static void imu_kalman_init(void)
{
    imu_kalman_filter_t *kf = &imu_attitude_system.kalman;
    
    // 清空所有矩阵
    memset(kf, 0, sizeof(imu_kalman_filter_t));
    
    // 初始化状态向量 [q0, q1, q2, q3, bx, by, bz]
    kf->state[0] = 1.0f;  // q0 (w)
    kf->state[1] = 0.0f;  // q1 (x)
    kf->state[2] = 0.0f;  // q2 (y)
    kf->state[3] = 0.0f;  // q3 (z)
    kf->state[4] = 0.0f;  // bias_x
    kf->state[5] = 0.0f;  // bias_y
    kf->state[6] = 0.0f;  // bias_z
    
    // 初始化状态协方差矩阵 P (对角线元素)
    for(int i = 0; i < IMU_KALMAN_STATE_SIZE; i++)
    {
        for(int j = 0; j < IMU_KALMAN_STATE_SIZE; j++)
        {
            if(i == j)
            {
                if(i < 4)  // 四元数方差
                    kf->P[i][j] = 0.1f;
                else       // 陀螺仪偏差方差
                    kf->P[i][j] = 0.01f;
            }
            else
            {
                kf->P[i][j] = 0.0f;
            }
        }
    }
    
    // 初始化过程噪声协方差矩阵 Q
    for(int i = 0; i < IMU_KALMAN_STATE_SIZE; i++)
    {
        for(int j = 0; j < IMU_KALMAN_STATE_SIZE; j++)
        {
            if(i == j)
            {
                if(i < 4)  // 四元数过程噪声
                    kf->Q[i][j] = IMU_KALMAN_Q_GYRO;
                else       // 陀螺仪偏差过程噪声
                    kf->Q[i][j] = IMU_KALMAN_Q_BIAS;
            }
            else
            {
                kf->Q[i][j] = 0.0f;
            }
        }
    }
    
    // 初始化测量噪声协方差矩阵 R
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
    
    printf("卡尔曼滤波器初始化完成\r\n");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     卡尔曼滤波器主更新函数
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
    
    // 1. 预测步骤
    imu_kalman_predict(dt, gyro);
    
    // 2. 更新步骤（使用加速度计数据）
    float accel_norm = sqrtf(accel->x * accel->x + accel->y * accel->y + accel->z * accel->z);
    // 只在加速度幅值接近参考重力加速度时才使用加速度计校正 (允许±50%的偏差)
    float accel_min = imu_attitude_system.calibration.reference_gravity * 0.5f;
    float accel_max = imu_attitude_system.calibration.reference_gravity * 1.5f;
    if(accel_norm >= accel_min && accel_norm <= accel_max)
    {
        imu_kalman_update_accel(accel);
    }
    
    // 3. 更新输出四元数
    imu_attitude_system.processed_data.quaternion.w = kf->state[0];
    imu_attitude_system.processed_data.quaternion.x = kf->state[1];
    imu_attitude_system.processed_data.quaternion.y = kf->state[2];
    imu_attitude_system.processed_data.quaternion.z = kf->state[3];
    
    // 4. 归一化四元数
    imu_quaternion_normalize(&imu_attitude_system.processed_data.quaternion);
    
    // 5. 更新状态向量中的四元数（保持一致性）
    kf->state[0] = imu_attitude_system.processed_data.quaternion.w;
    kf->state[1] = imu_attitude_system.processed_data.quaternion.x;
    kf->state[2] = imu_attitude_system.processed_data.quaternion.y;
    kf->state[3] = imu_attitude_system.processed_data.quaternion.z;
    
    kf->update_count++;
    
    return IMU_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     卡尔曼滤波器预测步骤
//-------------------------------------------------------------------------------------------------------------------
static void imu_kalman_predict(float dt, imu_vector3_t *gyro)
{
    imu_kalman_filter_t *kf = &imu_attitude_system.kalman;
    
    // 获取当前状态
    float q0 = kf->state[0], q1 = kf->state[1], q2 = kf->state[2], q3 = kf->state[3];
    float bx = kf->state[4], by = kf->state[5], bz = kf->state[6];
    
    // 校正后的陀螺仪数据
    float wx = gyro->x - bx;
    float wy = gyro->y - by;
    float wz = gyro->z - bz;
    
    // 状态转移方程: 四元数积分
    // q_new = q_old + 0.5 * dt * Ω(ω) * q_old
    float dq0 = 0.5f * dt * (-q1 * wx - q2 * wy - q3 * wz);
    float dq1 = 0.5f * dt * (q0 * wx - q3 * wy + q2 * wz);
    float dq2 = 0.5f * dt * (q3 * wx + q0 * wy - q1 * wz);
    float dq3 = 0.5f * dt * (-q2 * wx + q1 * wy + q0 * wz);
    
    // 更新状态向量
    kf->state[0] += dq0;  // q0
    kf->state[1] += dq1;  // q1
    kf->state[2] += dq2;  // q2
    kf->state[3] += dq3;  // q3
    // 偏差保持不变
    // kf->state[4] = bx;  // bias_x
    // kf->state[5] = by;  // bias_y
    // kf->state[6] = bz;  // bias_z
    
    // 归一化四元数部分
    float q_norm = sqrtf(kf->state[0]*kf->state[0] + kf->state[1]*kf->state[1] + 
                        kf->state[2]*kf->state[2] + kf->state[3]*kf->state[3]);
    if(q_norm > 0.0f)
    {
        kf->state[0] /= q_norm;
        kf->state[1] /= q_norm;
        kf->state[2] /= q_norm;
        kf->state[3] /= q_norm;
    }
    
    // 状态协方差预测: P = F*P*F' + Q
    // 这里简化处理，直接加过程噪声
    for(int i = 0; i < IMU_KALMAN_STATE_SIZE; i++)
    {
        kf->P[i][i] += kf->Q[i][i] * dt;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     卡尔曼滤波器加速度计更新步骤
//-------------------------------------------------------------------------------------------------------------------
static void imu_kalman_update_accel(imu_vector3_t *accel)
{
    imu_kalman_filter_t *kf = &imu_attitude_system.kalman;
    
    // 归一化加速度计数据
    float norm = sqrtf(accel->x * accel->x + accel->y * accel->y + accel->z * accel->z);
    float ax = accel->x / norm;
    float ay = accel->y / norm;
    float az = accel->z / norm;
    
    // 从当前四元数估计重力方向
    float q0 = kf->state[0], q1 = kf->state[1], q2 = kf->state[2], q3 = kf->state[3];
    
    // 预测的重力方向 (body frame)
    float gx_pred = 2.0f * (q1 * q3 - q0 * q2);
    float gy_pred = 2.0f * (q0 * q1 + q2 * q3);
    float gz_pred = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    
    // 创新（残差）
    float y[3];
    y[0] = ax - gx_pred;
    y[1] = ay - gy_pred;
    y[2] = az - gz_pred;
    
    // 观测矩阵 H（雅可比矩阵）
    // H = d(h)/d(x), 其中 h 是观测函数，x 是状态向量
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
    
    // 计算创新协方差 S = H*P*H' + R
    // 简化计算，只考虑对角元素
    for(int i = 0; i < 3; i++)
    {
        kf->S[i][i] = kf->R[i][i];
        for(int j = 0; j < 4; j++)  // 只考虑四元数部分
        {
            kf->S[i][i] += kf->H[i][j] * kf->P[j][j] * kf->H[i][j];
        }
    }
    
    // 计算卡尔曼增益 K = P*H'*S^(-1)
    // 简化计算
    for(int i = 0; i < IMU_KALMAN_STATE_SIZE; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            if(i < 4 && kf->S[j][j] > 0.0f)  // 只更新四元数部分
            {
                kf->K[i][j] = kf->P[i][i] * kf->H[j][i] / kf->S[j][j];
            }
            else
            {
                kf->K[i][j] = 0.0f;
            }
        }
    }
    
    // 状态更新 x = x + K*y
    for(int i = 0; i < IMU_KALMAN_STATE_SIZE; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            kf->state[i] += kf->K[i][j] * y[j];
        }
    }
    
    // 协方差更新 P = (I - K*H)*P
    // 简化为 P = P - K*H*P
    for(int i = 0; i < 4; i++)  // 只更新四元数相关的协方差
    {
        for(int j = 0; j < 3; j++)
        {
            kf->P[i][i] *= (1.0f - kf->K[i][j] * kf->H[j][i]);
        }
        // 限制协方差最小值
        if(kf->P[i][i] < 0.001f)
            kf->P[i][i] = 0.001f;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     矩阵乘法 (简化版本，仅用于必要计算)
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
// 函数简介     矩阵转置
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
// 函数简介     3x3矩阵求逆 (用于创新协方差矩阵)
//-------------------------------------------------------------------------------------------------------------------
static void imu_matrix_inverse_3x3(float A[3][3], float Ainv[3][3])
{
    float det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
              - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
              + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
    
    if(fabsf(det) < 1e-6f)
    {
        // 矩阵奇异，使用单位矩阵
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
// 函数简介     矩阵加法
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
// 函数简介     矩阵减法
//-------------------------------------------------------------------------------------------------------------------
static void imu_matrix_subtract(float *A, float *B, float *C, int m, int n)
{
    // C = A - B
    for(int i = 0; i < m * n; i++)
    {
        C[i] = A[i] - B[i];
    }
}

//=================================================Madgwick滤波器实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化Madgwick滤波器
//-------------------------------------------------------------------------------------------------------------------
static void imu_madgwick_init(void)
{
    imu_madgwick_filter_t *mf = &imu_attitude_system.madgwick;
    
    // 清空滤波器结构体
    memset(mf, 0, sizeof(imu_madgwick_filter_t));
    
    // 设置默认参数
    mf->beta = IMU_MADGWICK_BETA_DEFAULT;
    mf->zeta = IMU_MADGWICK_ZETA;
    
    // 清空误差积分
    memset(&mf->w_err_integral, 0, sizeof(imu_vector3_t));
    
    mf->initialized = 1;
    mf->update_count = 0;
    
    printf("Madgwick滤波器初始化完成，beta=%.3f\r\n", mf->beta);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     Madgwick滤波器主更新函数
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
    
    // 检查加速度计幅值是否接近参考重力加速度 (允许±50%的偏差)
    float accel_norm = sqrtf(accel->x * accel->x + accel->y * accel->y + accel->z * accel->z);
    float accel_min = imu_attitude_system.calibration.reference_gravity * 0.5f;
    float accel_max = imu_attitude_system.calibration.reference_gravity * 1.5f;
    
    // 只在加速度幅值合理时才使用加速度计校正
    // 否则只用陀螺仪积分(避免动态加速度干扰)
    if(accel_norm >= accel_min && accel_norm <= accel_max)
    {
        // 调用Madgwick算法更新(含加速度计校正)
        imu_madgwick_update_imu(dt, gyro, accel);
    }
    else
    {
        // 仅使用陀螺仪更新
        imu_vector3_t zero_accel = {0.0f, 0.0f, 0.0f};
        imu_madgwick_update_imu(dt, gyro, &zero_accel);
    }
    
    mf->update_count++;
    
    return IMU_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     Madgwick算法IMU更新
//-------------------------------------------------------------------------------------------------------------------
static void imu_madgwick_update_imu(float dt, imu_vector3_t *gyro, imu_vector3_t *accel)
{
    imu_madgwick_filter_t *mf = &imu_attitude_system.madgwick;
    imu_quaternion_t *q = &imu_attitude_system.processed_data.quaternion;
    
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
    
    // 获取当前四元数值
    float q0 = q->w, q1 = q->x, q2 = q->y, q3 = q->z;
    
    // 陀螺仪角速度
    float gx = gyro->x, gy = gyro->y, gz = gyro->z;
    float ax = accel->x, ay = accel->y, az = accel->z;
    
    // 如果加速度计数据有效
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 
    {
        // 归一化加速度计测量值
        recipNorm = imu_inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        
        // 辅助变量，避免重复计算
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
        
        // 梯度下降算法，计算目标函数的梯度
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        
        recipNorm = imu_inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // 归一化梯度
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;
        
        // 计算陀螺仪误差
        float w_err_x = _2q0 * s1 - _2q1 * s0 - _2q2 * s3 + _2q3 * s2;
        float w_err_y = _2q0 * s2 + _2q1 * s3 - _2q2 * s0 - _2q3 * s1;
        float w_err_z = _2q0 * s3 - _2q1 * s2 + _2q2 * s1 - _2q3 * s0;
        
        // 积分陀螺仪误差（如果启用了zeta补偿）
        if(mf->zeta > 0.0f)
        {
            mf->w_err_integral.x += w_err_x * dt;
            mf->w_err_integral.y += w_err_y * dt;
            mf->w_err_integral.z += w_err_z * dt;
            
            // 应用积分反馈
            gx += mf->zeta * mf->w_err_integral.x;
            gy += mf->zeta * mf->w_err_integral.y;
            gz += mf->zeta * mf->w_err_integral.z;
        }
        
        // 应用比例反馈
        gx -= mf->beta * s0;
        gy -= mf->beta * s1;
        gz -= mf->beta * s2;
    }
    
    // 四元数积分
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
    
    // 积分四元数
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;
    
    // 归一化四元数
    recipNorm = imu_inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q->w = q0 * recipNorm;
    q->x = q1 * recipNorm;
    q->y = q2 * recipNorm;
    q->z = q3 * recipNorm;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     快速平方根倒数算法
//-------------------------------------------------------------------------------------------------------------------
static float imu_inv_sqrt(float x)
{
    // 快速平方根倒数算法 (Quake III算法的改进版)
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));   // 可选的第二次迭代，提高精度
    return y;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取Madgwick滤波器状态信息
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
// 函数简介     设置Madgwick滤波器参数
//-------------------------------------------------------------------------------------------------------------------
imu_status_enum imu_set_madgwick_params(float beta, float zeta)
{
    if(!imu_attitude_system.system_initialized)
        return IMU_STATUS_NOT_INIT;
    
    // 参数范围检查
    if(beta < IMU_MADGWICK_BETA_MIN || beta > IMU_MADGWICK_BETA_MAX)
    {
        printf("Madgwick beta参数超出范围 [%.3f, %.3f]\r\n", 
               IMU_MADGWICK_BETA_MIN, IMU_MADGWICK_BETA_MAX);
        return IMU_STATUS_ERROR;
    }
    
    if(zeta < 0.0f || zeta > 1.0f)
    {
        printf("Madgwick zeta参数超出范围 [0.0, 1.0]\r\n");
        return IMU_STATUS_ERROR;
    }
    
    // 设置参数
    imu_attitude_system.madgwick.beta = beta;
    imu_attitude_system.madgwick.zeta = zeta;
    
    printf("Madgwick参数已更新：beta=%.3f, zeta=%.3f\r\n", beta, zeta);
    return IMU_STATUS_OK;
}

//=================================================UKF滤波器实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化UKF滤波器
//-------------------------------------------------------------------------------------------------------------------
static void imu_ukf_init(void)
{
    imu_ukf_filter_t *ukf = &imu_attitude_system.ukf;
    
    // 清空滤波器结构体
    memset(ukf, 0, sizeof(imu_ukf_filter_t));
    
    // 设置UKF参数
    ukf->alpha = IMU_UKF_ALPHA;
    ukf->beta = IMU_UKF_BETA;
    ukf->kappa = IMU_UKF_KAPPA;
    ukf->lambda = ukf->alpha * ukf->alpha * (IMU_UKF_STATE_SIZE + ukf->kappa) - IMU_UKF_STATE_SIZE;
    
    // 初始化状态向量 [q0, q1, q2, q3, bx, by, bz]
    ukf->state[0] = 1.0f;  // q0 (w)
    ukf->state[1] = 0.0f;  // q1 (x)
    ukf->state[2] = 0.0f;  // q2 (y)
    ukf->state[3] = 0.0f;  // q3 (z)
    ukf->state[4] = 0.0f;  // bias_x
    ukf->state[5] = 0.0f;  // bias_y
    ukf->state[6] = 0.0f;  // bias_z
    
    // 初始化状态协方差矩阵 P
    for(int i = 0; i < IMU_UKF_STATE_SIZE; i++)
    {
        for(int j = 0; j < IMU_UKF_STATE_SIZE; j++)
        {
            if(i == j)
            {
                if(i < 4)  // 四元数方差
                    ukf->P[i][j] = 0.1f;
                else       // 陀螺仪偏差方差
                    ukf->P[i][j] = 0.01f;
            }
            else
            {
                ukf->P[i][j] = 0.0f;
            }
        }
    }
    
    // 初始化过程噪声协方差矩阵 Q
    for(int i = 0; i < IMU_UKF_STATE_SIZE; i++)
    {
        for(int j = 0; j < IMU_UKF_STATE_SIZE; j++)
        {
            if(i == j)
            {
                if(i < 4)  // 四元数过程噪声
                    ukf->Q[i][j] = IMU_UKF_Q_GYRO;
                else       // 陀螺仪偏差过程噪声
                    ukf->Q[i][j] = IMU_UKF_Q_BIAS;
            }
            else
            {
                ukf->Q[i][j] = 0.0f;
            }
        }
    }
    
    // 初始化测量噪声协方差矩阵 R
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
    
    // 计算Sigma点权重
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
    
    printf("UKF滤波器初始化完成，lambda=%.6f\r\n", ukf->lambda);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UKF滤波器主更新函数
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
    
    // 检测NaN - 在预测前检查状态
    for(int i = 0; i < IMU_UKF_STATE_SIZE; i++)
    {
        if(isnan(ukf->state[i]) || isinf(ukf->state[i]))
        {
            printf("检测到UKF状态NaN/Inf，重新初始化 state[%d]=%.3f\r\n", i, ukf->state[i]);
            imu_ukf_init();
            return IMU_STATUS_ERROR;
        }
    }
    
    // 1. 预测步骤
    imu_ukf_predict(dt, gyro);
    
    // 检测NaN - 在预测后检查状态
    for(int i = 0; i < IMU_UKF_STATE_SIZE; i++)
    {
        if(isnan(ukf->state[i]) || isinf(ukf->state[i]))
        {
            printf("预测后检测到UKF状态NaN/Inf，重新初始化 state[%d]=%.3f\r\n", i, ukf->state[i]);
            imu_ukf_init();
            return IMU_STATUS_ERROR;
        }
    }
    
    // 2. 更新步骤（使用加速度计数据）
    float accel_norm = sqrtf(accel->x * accel->x + accel->y * accel->y + accel->z * accel->z);
    // 只在加速度幅值接近参考重力加速度时才使用加速度计校正 (允许±50%的偏差)
    float accel_min = imu_attitude_system.calibration.reference_gravity * 0.5f;
    float accel_max = imu_attitude_system.calibration.reference_gravity * 1.5f;
    if(accel_norm >= accel_min && accel_norm <= accel_max)
    {
        imu_ukf_update_accel(accel);
        
        // 检测NaN - 在更新后检查状态
        for(int i = 0; i < IMU_UKF_STATE_SIZE; i++)
        {
            if(isnan(ukf->state[i]) || isinf(ukf->state[i]))
            {
                printf("更新后检测到UKF状态NaN/Inf，重新初始化 state[%d]=%.3f\r\n", i, ukf->state[i]);
                imu_ukf_init();
                return IMU_STATUS_ERROR;
            }
        }
    }
    
    // 3. 更新输出四元数
    imu_attitude_system.processed_data.quaternion.w = ukf->state[0];
    imu_attitude_system.processed_data.quaternion.x = ukf->state[1];
    imu_attitude_system.processed_data.quaternion.y = ukf->state[2];
    imu_attitude_system.processed_data.quaternion.z = ukf->state[3];
    
    // 4. 归一化四元数
    imu_quaternion_normalize(&imu_attitude_system.processed_data.quaternion);
    
    // 5. 更新状态向量中的四元数（保持一致性）
    ukf->state[0] = imu_attitude_system.processed_data.quaternion.w;
    ukf->state[1] = imu_attitude_system.processed_data.quaternion.x;
    ukf->state[2] = imu_attitude_system.processed_data.quaternion.y;
    ukf->state[3] = imu_attitude_system.processed_data.quaternion.z;
    
    ukf->update_count++;
    
    return IMU_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     生成UKF Sigma点
//-------------------------------------------------------------------------------------------------------------------
static void imu_ukf_generate_sigma_points(void)
{
    imu_ukf_filter_t *ukf = &imu_attitude_system.ukf;
    
    // 计算协方差矩阵的Cholesky分解
    float L[IMU_UKF_STATE_SIZE][IMU_UKF_STATE_SIZE];
    imu_matrix_cholesky_decomposition(ukf->P, L);
    
    // 计算缩放因子
    float sqrt_lambda_n = sqrtf(IMU_UKF_STATE_SIZE + ukf->lambda);
    
    // 第一个Sigma点是状态均值
    for(int i = 0; i < IMU_UKF_STATE_SIZE; i++)
    {
        ukf->sigma_points[0][i] = ukf->state[i];
    }
    
    // 生成其余的Sigma点
    for(int i = 0; i < IMU_UKF_STATE_SIZE; i++)
    {
        for(int j = 0; j < IMU_UKF_STATE_SIZE; j++)
        {
            // 正向Sigma点
            ukf->sigma_points[i+1][j] = ukf->state[j] + sqrt_lambda_n * L[i][j];
            // 负向Sigma点
            ukf->sigma_points[i+1+IMU_UKF_STATE_SIZE][j] = ukf->state[j] - sqrt_lambda_n * L[i][j];
        }
    }
    
    // 归一化四元数部分（确保每个Sigma点的四元数都是单位四元数）
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
            // 如果四元数模长异常，重置为单位四元数
            ukf->sigma_points[i][0] = 1.0f;
            ukf->sigma_points[i][1] = 0.0f;
            ukf->sigma_points[i][2] = 0.0f;
            ukf->sigma_points[i][3] = 0.0f;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UKF预测步骤
//-------------------------------------------------------------------------------------------------------------------
static void imu_ukf_predict(float dt, imu_vector3_t *gyro)
{
    imu_ukf_filter_t *ukf = &imu_attitude_system.ukf;
    
    // 1. 生成Sigma点
    imu_ukf_generate_sigma_points();
    
    // 2. 传播每个Sigma点通过过程模型
    for(int i = 0; i < IMU_UKF_SIGMA_POINTS; i++)
    {
        // 获取当前Sigma点的状态
        float q0 = ukf->sigma_points[i][0], q1 = ukf->sigma_points[i][1];
        float q2 = ukf->sigma_points[i][2], q3 = ukf->sigma_points[i][3];
        float bx = ukf->sigma_points[i][4], by = ukf->sigma_points[i][5], bz = ukf->sigma_points[i][6];
        
        // 校正后的陀螺仪数据
        float wx = gyro->x - bx;
        float wy = gyro->y - by;
        float wz = gyro->z - bz;
        
        // 四元数积分（状态转移方程）
        float dq0 = 0.5f * dt * (-q1 * wx - q2 * wy - q3 * wz);
        float dq1 = 0.5f * dt * (q0 * wx - q3 * wy + q2 * wz);
        float dq2 = 0.5f * dt * (q3 * wx + q0 * wy - q1 * wz);
        float dq3 = 0.5f * dt * (-q2 * wx + q1 * wy + q0 * wz);
        
        // 更新Sigma点
        ukf->sigma_points[i][0] = q0 + dq0;
        ukf->sigma_points[i][1] = q1 + dq1;
        ukf->sigma_points[i][2] = q2 + dq2;
        ukf->sigma_points[i][3] = q3 + dq3;
        // 偏差保持不变
        // ukf->sigma_points[i][4] = bx;
        // ukf->sigma_points[i][5] = by;
        // ukf->sigma_points[i][6] = bz;
        
        // 归一化四元数
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
            // 如果四元数模长异常，重置为单位四元数
            ukf->sigma_points[i][0] = 1.0f;
            ukf->sigma_points[i][1] = 0.0f;
            ukf->sigma_points[i][2] = 0.0f;
            ukf->sigma_points[i][3] = 0.0f;
        }
    }
    
    // 3. 计算预测的状态均值
    for(int i = 0; i < IMU_UKF_STATE_SIZE; i++)
    {
        ukf->state[i] = 0.0f;
        for(int j = 0; j < IMU_UKF_SIGMA_POINTS; j++)
        {
            ukf->state[i] += ukf->sigma_weights_mean[j] * ukf->sigma_points[j][i];
        }
    }
    
    // 归一化预测的四元数
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
        // 如果四元数模长异常，重置为单位四元数
        ukf->state[0] = 1.0f;
        ukf->state[1] = 0.0f;
        ukf->state[2] = 0.0f;
        ukf->state[3] = 0.0f;
    }
    
    // 4. 计算预测的协方差矩阵
    // 首先清零协方差矩阵
    for(int i = 0; i < IMU_UKF_STATE_SIZE; i++)
    {
        for(int j = 0; j < IMU_UKF_STATE_SIZE; j++)
        {
            ukf->P[i][j] = 0.0f;
        }
    }
    
    // 计算协方差
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
    
    // 5. 添加过程噪声
    for(int i = 0; i < IMU_UKF_STATE_SIZE; i++)
    {
        ukf->P[i][i] += ukf->Q[i][i] * dt;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UKF加速度计更新步骤
//-------------------------------------------------------------------------------------------------------------------
static void imu_ukf_update_accel(imu_vector3_t *accel)
{
    imu_ukf_filter_t *ukf = &imu_attitude_system.ukf;
    
    // 1. 通过测量函数传播Sigma点
    for(int i = 0; i < IMU_UKF_SIGMA_POINTS; i++)
    {
        imu_ukf_measurement_function(ukf->sigma_points[i], ukf->predicted_measurements[i]);
    }
    
    // 2. 计算预测的测量均值
    float z_mean[IMU_UKF_MEASUREMENT_SIZE] = {0.0f};
    for(int i = 0; i < IMU_UKF_MEASUREMENT_SIZE; i++)
    {
        for(int j = 0; j < IMU_UKF_SIGMA_POINTS; j++)
        {
            z_mean[i] += ukf->sigma_weights_mean[j] * ukf->predicted_measurements[j][i];
        }
    }
    
    // 3. 计算创新协方差矩阵 Pzz
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
            // 添加测量噪声
            if(i == j)
                Pzz[i][j] += ukf->R[i][j];
        }
    }
    
    // 4. 计算状态-测量交叉协方差矩阵 Pxz
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
    
    // 5. 计算卡尔曼增益 K = Pxz * Pzz^(-1)
    // 简化版本：使用对角线逆矩阵
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
    
    // 6. 计算创新（残差）
    float innovation[IMU_UKF_MEASUREMENT_SIZE];
    float accel_norm = sqrtf(accel->x * accel->x + accel->y * accel->y + accel->z * accel->z);
    
    // 保护：如果加速度模长异常，不进行更新
    if(accel_norm < 0.1f || accel_norm > 20.0f)
    {
        // 加速度数据异常，跳过本次更新
        return;
    }
    
    innovation[0] = accel->x / accel_norm - z_mean[0];
    innovation[1] = accel->y / accel_norm - z_mean[1];
    innovation[2] = accel->z / accel_norm - z_mean[2];
    
    // 7. 状态更新
    for(int i = 0; i < IMU_UKF_STATE_SIZE; i++)
    {
        for(int j = 0; j < IMU_UKF_MEASUREMENT_SIZE; j++)
        {
            ukf->state[i] += K[i][j] * innovation[j];
        }
    }
    
    // 8. 协方差更新 P = P - K * Pzz * K'
    // 简化版本
    for(int i = 0; i < IMU_UKF_STATE_SIZE; i++)
    {
        for(int j = 0; j < IMU_UKF_MEASUREMENT_SIZE; j++)
        {
            ukf->P[i][i] -= K[i][j] * Pzz[j][j] * K[i][j];
        }
        // 限制协方差最小值
        if(ukf->P[i][i] < 0.001f)
            ukf->P[i][i] = 0.001f;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UKF测量函数（从状态到预期加速度计读数）
//-------------------------------------------------------------------------------------------------------------------
static void imu_ukf_measurement_function(float *state, float *measurement)
{
    // 从四元数计算预期的重力方向（在传感器坐标系中）
    float q0 = state[0], q1 = state[1], q2 = state[2], q3 = state[3];
    
    // 重力在传感器坐标系中的表示
    measurement[0] = 2.0f * (q1 * q3 - q0 * q2);        // gx
    measurement[1] = 2.0f * (q0 * q1 + q2 * q3);        // gy
    measurement[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;  // gz
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     Cholesky分解（用于UKF Sigma点生成）
//-------------------------------------------------------------------------------------------------------------------
static void imu_matrix_cholesky_decomposition(float A[IMU_UKF_STATE_SIZE][IMU_UKF_STATE_SIZE], 
                                             float L[IMU_UKF_STATE_SIZE][IMU_UKF_STATE_SIZE])
{
    // 初始化L矩阵为零
    for(int i = 0; i < IMU_UKF_STATE_SIZE; i++)
    {
        for(int j = 0; j < IMU_UKF_STATE_SIZE; j++)
        {
            L[i][j] = 0.0f;
        }
    }
    
    // Cholesky分解: A = L * L'
    for(int i = 0; i < IMU_UKF_STATE_SIZE; i++)
    {
        for(int j = 0; j <= i; j++)
        {
            if(i == j)  // 对角线元素
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
                    L[i][j] = 0.001f;  // 防止数值问题
            }
            else  // 下三角元素
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
// 函数简介     获取UKF滤波器状态信息
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
// 函数简介     设置UKF滤波器参数
//-------------------------------------------------------------------------------------------------------------------
imu_status_enum imu_set_ukf_params(float alpha, float beta, float kappa)
{
    if(!imu_attitude_system.system_initialized)
        return IMU_STATUS_NOT_INIT;
    
    // 参数范围检查
    if(alpha <= 0.0f || alpha > 1.0f)
    {
        printf("UKF alpha参数超出范围 (0.0, 1.0]\r\n");
        return IMU_STATUS_ERROR;
    }
    
    if(beta < 0.0f)
    {
        printf("UKF beta参数不能为负数\r\n");
        return IMU_STATUS_ERROR;
    }
    
    // 设置参数
    imu_attitude_system.ukf.alpha = alpha;
    imu_attitude_system.ukf.beta = beta;
    imu_attitude_system.ukf.kappa = kappa;
    imu_attitude_system.ukf.lambda = alpha * alpha * (IMU_UKF_STATE_SIZE + kappa) - IMU_UKF_STATE_SIZE;
    
    // 重新计算权重
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
    
    printf("UKF参数已更新：alpha=%.6f, beta=%.3f, kappa=%.3f, lambda=%.6f\r\n", 
           alpha, beta, kappa, imu_attitude_system.ukf.lambda);
    return IMU_STATUS_OK;
}