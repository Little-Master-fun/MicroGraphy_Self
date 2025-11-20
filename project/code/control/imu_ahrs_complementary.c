/*********************************************************************************************************************
* 文件名称          imu_ahrs_complementary.c
* 功能说明          基于四元数的AHRS互补滤波器实现（参考dog\DOG_KEY项目）
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本
* 2025-10-11        LittleMaster       1.0v
* 2025-10-20        LittleMaster       2.0v
* 2025-11-18        LittleMaster       3.0v
* 
* 文件作用说明：
* 本文件实现基于四元数的AHRS姿态解算算法，包含以下功能：
* 1. 陀螺仪零偏动态校准（启动时自动校准）
* 2. 四元数姿态更新（含PI控制器漂移补偿）
* 3. 欧拉角计算及航向角累积处理
* 4. 快速平方根倒数算法优化
* 
* 修复记录：
* 2025-11-11: 修复了DEG_TO_RAD单位转换错误（从1.0改为π/180），解决了转一圈有2度偏差的问题
* 2025-11-18: 完成了航向角的调整，一切都被我想复杂了，大道至简呀，小车不是无人机，抛弃了四元数，欧拉角，各种复杂的滤波，简单的1D积分就行了
********************************************************************************************************************/

#include "imu_ahrs_complementary.h"
#include "driver_sch16tk10.h"
#include "config_infineon.h"
#include "zf_common_headfile.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

//=================================================常量定义================================================


//  更新周期计算：1ms中断 × AVG_FACTOR(2) = 2ms = 0.002秒
// 但经过实测，实际采样周期约为 1.126ms，因此 DELTA_T = 0.001126 * AVG_FACTOR(2) ≈ 0.002252秒，不要问我为什么是0.032，问就是经验
// 测，为什么没有怀疑往年代码设置错了呢，申请到完整数据手册才看见，灵敏度配置都不对，sb往年代码


#define DELTA_T              0.01f         // 更新周期 
#define ALPHA                0.2f           // 加速度计低通滤波系数
#define G_TO_M_S2            9.80665f       // 重力加速度转换系数
#define DEG_TO_RAD           0.017453292519943295f  // 度转弧度 (π/180)，传感器输出是dps需要转换
#define AHRS_PI              3.1415926535f  // 圆周率

// 陀螺仪零偏校准参数
#define GYRO_CALIB_START     40            // 开始校准的采样点 (800ms后开始) 400
#define GYRO_CALIB_END       140           // 结束校准的采样点 (2800ms)
#define GYRO_CALIB_SAMPLES   100           // 校准采样点数 (累加1000次，持续2秒)
#define GYRO_CALIB_FINISH    150           // 校准完成点 (3000ms)
#define GYRO_READY_COUNT     180           // 系统准备就绪点 (3600ms，约3.6秒)

//=================================================全局变量定义================================================
static ahrs_system_t ahrs_system = {0};
static ahrs_quaternion_t quaternion = {1.0f, 0.0f, 0.0f, 0.0f};
static ahrs_euler_angles_t euler_angles = {0};
static ahrs_gyro_offset_t gyro_offset = {0};
static ahrs_imu_data_t imu_data = {0};
static ahrs_pi_controller_t pi_controller = {0};

// 1D Yaw 积分器（纯陀螺积分的航向角）
static float yaw_gyro = 0.0f;  // 纯陀螺积分的航向角（度）[-180, 180]

// 调试变量（通过在线调试查看）
typedef struct {
    // 传感器配置信息
    uint16_t actual_sens_rate1;        // 实际配置的灵敏度Rate1
    uint16_t actual_sens_rate2;        // 实际配置的灵敏度Rate2
    uint16_t actual_dec_rate2;         // 实际配置的抽取率
    float    config_sens_rate1;        // 代码中配置的灵敏度
    
    // 原始LSB数据（累加后）
    int32_t  raw_gyro_x;               // 陀螺仪X原始LSB
    int32_t  raw_gyro_y;               // 陀螺仪Y原始LSB
    int32_t  raw_gyro_z;               // 陀螺仪Z原始LSB
    
    // 转换后的物理量
    float    gyro_x_dps;               // 陀螺仪X (度/秒)
    float    gyro_y_dps;               // 陀螺仪Y (度/秒)
    float    gyro_z_dps;               // 陀螺仪Z (度/秒)
    float    gyro_x_rads;              // 陀螺仪X (弧度/秒)
    float    gyro_y_rads;              // 陀螺仪Y (弧度/秒)
    float    gyro_z_rads;              // 陀螺仪Z (弧度/秒)
    
    // 姿态角
    float    pitch_deg;                // 俯仰角 (度)
    float    roll_deg;                 // 滚转角 (度)
    float    yaw_deg;                  // 偏航角 (度)
    float    yaw_accumulated_deg;      // 累积偏航角 (度)
    float    yaw_gyro_deg;             // 1D Yaw积分器航向角 (度) [-180, 180]
    
    // 转换系数验证
    float    expected_sensitivity;     // 期望的灵敏度 = SENSITIVITY_RATE1 * AVG_FACTOR
    float    actual_conversion_factor; // 实际转换因子
    
    // 更新计数
    uint32_t update_count;             // 姿态更新计数器
    
} ahrs_debug_info_t;

// 声明为全局变量以便调试器查看
ahrs_debug_info_t g_ahrs_debug = {0};

// 传感器数据累加缓冲区（用于AVG_FACTOR次采样平均）
static SCH1_raw_data raw_data_summed = {0};
static uint32 sample_count = 0;

// 校准相关变量
static uint32 calibration_count = 0;
static uint8 system_ready = 0;
static float gyro_sum_x = 0.0f;
static float gyro_sum_y = 0.0f;
static float gyro_sum_z = 0.0f;

// PI控制器参数
static float param_kp = 0.0f;   // 比例增益（可根据需要调整）
static float param_ki = 0.0f;   // 积分增益（可根据需要调整）

// 航向角处理
static float yaw_offset = 0.0f;
static int8 direction_change = 0;
static float last_yaw = 0.0f;

//=================================================内部函数声明================================================
static float fast_inv_sqrt(float x);
static void ahrs_get_sensor_values(SCH1_result *sensor_data);
static void ahrs_update_quaternion(float gx, float gy, float gz, float ax, float ay, float az);
static void ahrs_quaternion_to_euler(void);

//=================================================外部接口实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化AHRS系统
// 参数说明     无
// 返回参数     初始化状态
//-------------------------------------------------------------------------------------------------------------------
ahrs_status_enum ahrs_complementary_init(void)
{
    // 清空所有数据结构
    memset(&ahrs_system, 0, sizeof(ahrs_system_t));
    memset(&gyro_offset, 0, sizeof(ahrs_gyro_offset_t));
    memset(&pi_controller, 0, sizeof(ahrs_pi_controller_t));
    
    // 初始化四元数为单位四元数
    quaternion.q0 = 1.0f;
    quaternion.q1 = 0.0f;
    quaternion.q2 = 0.0f;
    quaternion.q3 = 0.0f;
    
    // 清零欧拉角
    euler_angles.pitch = 0.0f;
    euler_angles.roll = 0.0f;
    euler_angles.yaw = 0.0f;
    
    // 初始化传感器数据累加缓冲区
    memset(&raw_data_summed, 0, sizeof(SCH1_raw_data));
    sample_count = 0;
    
    // 初始化校准变量
    calibration_count = 0;
    system_ready = 0;
    gyro_sum_x = 0.0f;
    gyro_sum_y = 0.0f;
    gyro_sum_z = 0.0f;
    
    // 初始化航向角处理变量
    yaw_offset = 0.0f;
    direction_change = 0;
    last_yaw = 0.0f;
    
    // 设置PI控制器参数
    param_kp = 0.0f;  // 可以根据实际情况调整
    param_ki = 0.0f;  // 可以根据实际情况调整
    
    // 初始化SCH16TK10传感器
    printf("正在初始化SCH16TK10传感器...\r\n");
    
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
        return AHRS_STATUS_ERROR;
    }
    
    printf("SCH16TK10传感器初始化成功！\r\n");
    
    // 读取并验证实际灵敏度配置
    uint16_t actual_sens_rate1, actual_sens_rate2, actual_dec_rate2;
    if (SCH1_getRateSensDec(&actual_sens_rate1, &actual_sens_rate2, &actual_dec_rate2) == SCH1_OK)
    {
        // 保存到调试结构体
        g_ahrs_debug.actual_sens_rate1 = actual_sens_rate1;
        g_ahrs_debug.actual_sens_rate2 = actual_sens_rate2;
        g_ahrs_debug.actual_dec_rate2 = actual_dec_rate2;
        g_ahrs_debug.config_sens_rate1 = SENSITIVITY_RATE1;
        g_ahrs_debug.expected_sensitivity = SENSITIVITY_RATE1 * (float)AVG_FACTOR;
        
        printf("陀螺仪实际配置 - 灵敏度Rate1: %d LSB/dps, Rate2: %d LSB/dps, 抽取率: %d\r\n", 
               actual_sens_rate1, actual_sens_rate2, actual_dec_rate2);
        printf("代码中使用的灵敏度: %.0f LSB/dps\r\n", SENSITIVITY_RATE1);
        printf("期望转换因子: %.0f (SENS * AVG_FACTOR)\r\n", g_ahrs_debug.expected_sensitivity);
        
        if(actual_sens_rate1 != (uint16_t)SENSITIVITY_RATE1)
        {
            printf("警告：实际灵敏度与代码配置不匹配！\r\n");
        }
    }
    
    // 测试SPI通信稳定性
    if (SCH1_testSPIStability())
    {
        printf("SPI通信测试通过\r\n");
    }
    else
    {
        printf("SPI通信测试失败，但继续运行\r\n");
    }
    
    ahrs_system.initialized = 1;
    
    printf("AHRS互补滤波器初始化完成\r\n");
    printf("正在进行陀螺仪零偏校准，请保持设备静止...\r\n");
    
    return AHRS_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     AHRS系统更新（在定时中断中调用，建议1ms周期）
// 参数说明     raw_data        传感器原始数据
// 返回参数     更新状态
// 备注信息     实现AVG_FACTOR次采样平均，减少噪声
//-------------------------------------------------------------------------------------------------------------------
ahrs_status_enum ahrs_complementary_update(SCH1_raw_data *raw_data)
{
    if(!ahrs_system.initialized)
        return AHRS_STATUS_NOT_INIT;
    
    // 检查数据帧错误
    if(raw_data->frame_error)
        return AHRS_STATUS_ERROR;
    
    // 累加原始数据（用于多次采样平均降噪）
    raw_data_summed.Rate1_raw[AXIS_X] += raw_data->Rate1_raw[AXIS_X];
    raw_data_summed.Rate1_raw[AXIS_Y] += raw_data->Rate1_raw[AXIS_Y];
    raw_data_summed.Rate1_raw[AXIS_Z] += raw_data->Rate1_raw[AXIS_Z];
    
    raw_data_summed.Acc1_raw[AXIS_X] += raw_data->Acc1_raw[AXIS_X];
    raw_data_summed.Acc1_raw[AXIS_Y] += raw_data->Acc1_raw[AXIS_Y];
    raw_data_summed.Acc1_raw[AXIS_Z] += raw_data->Acc1_raw[AXIS_Z];
    
    raw_data_summed.Temp_raw += raw_data->Temp_raw;
    
    sample_count++;
    
    // 当累加次数达到AVG_FACTOR时，才进行数据转换和姿态更新
    if(sample_count >= AVG_FACTOR)
    {
        // 转换为物理量（SCH1_convert_data会自动除以AVG_FACTOR进行平均）
        SCH1_result sensor_data;
        SCH1_convert_data(&raw_data_summed, &sensor_data);
        
        // 校准阶段
        if(!system_ready)
        {
            calibration_count++;
            
            // 累积陀螺仪数据用于零偏计算（从401到1400，共1000次）
            if(calibration_count > GYRO_CALIB_START && calibration_count <= GYRO_CALIB_END)
            {
                gyro_sum_x += sensor_data.Rate1[AXIS_X];
                gyro_sum_y += sensor_data.Rate1[AXIS_Y];
                gyro_sum_z += sensor_data.Rate1[AXIS_Z];
            }
            
            // 计算零偏平均值
            if(calibration_count == GYRO_CALIB_FINISH)
            {
                gyro_offset.x = gyro_sum_x / GYRO_CALIB_SAMPLES;
                gyro_offset.y = gyro_sum_y / GYRO_CALIB_SAMPLES;
                gyro_offset.z = gyro_sum_z / GYRO_CALIB_SAMPLES;
                
                printf("陀螺仪零偏校准完成:\r\n");
                printf("  X轴偏移: %.6f dps\r\n", gyro_offset.x);
                printf("  Y轴偏移: %.6f dps\r\n", gyro_offset.y);
                printf("  Z轴偏移: %.6f dps\r\n", gyro_offset.z);
            }
            
            // 系统准备就绪
            if(calibration_count > GYRO_READY_COUNT)
            {
                system_ready = 1;
                
                // 初始化1D Yaw积分器：对齐到四元数算出的yaw
                // 在校准结束后，四元数应该已经有了初始姿态
                yaw_gyro = euler_angles.yaw;
                
                printf("AHRS系统准备就绪，开始姿态解算\r\n");
                printf("1D Yaw积分器已初始化为: %.2f度\r\n", yaw_gyro);
            }
        }
        else
        {
            // 正常工作阶段：获取传感器数据并更新姿态
            ahrs_get_sensor_values(&sensor_data);
            
            // ========== 1D Yaw 积分器更新 ==========
            // imu_data.gyro_z 已经在 ahrs_get_sensor_values 中被赋值（单位：rad/s）
            // 积分到度：gyro_z (rad/s) * (180/π) * dt
            // 注意：实测发现需要将 DELTA_T 乘以 2，可能是因为实际更新周期为 4ms
            yaw_gyro += imu_data.gyro_z * (180.0f / AHRS_PI) * DELTA_T * 2.0f;
            
            // 归一化到 [-180, 180] 度
            if (yaw_gyro > 180.0f)
                yaw_gyro -= 360.0f;
            else if (yaw_gyro < -180.0f)
                yaw_gyro += 360.0f;
            // ========================================
            
            // 更新调试信息
            g_ahrs_debug.update_count++;
            
            // 记录原始LSB数据
            g_ahrs_debug.raw_gyro_x = raw_data_summed.Rate1_raw[AXIS_X];
            g_ahrs_debug.raw_gyro_y = raw_data_summed.Rate1_raw[AXIS_Y];
            g_ahrs_debug.raw_gyro_z = raw_data_summed.Rate1_raw[AXIS_Z];
            
            // 记录转换后的数据(dps)
            g_ahrs_debug.gyro_x_dps = sensor_data.Rate1[AXIS_X];
            g_ahrs_debug.gyro_y_dps = sensor_data.Rate1[AXIS_Y];
            g_ahrs_debug.gyro_z_dps = sensor_data.Rate1[AXIS_Z];
            
            // 记录转换后的数据(rad/s)
            g_ahrs_debug.gyro_x_rads = imu_data.gyro_x;
            g_ahrs_debug.gyro_y_rads = imu_data.gyro_y;
            g_ahrs_debug.gyro_z_rads = imu_data.gyro_z;
            
            // 计算实际转换因子（从LSB到dps）
            if(g_ahrs_debug.gyro_z_dps != 0)
            {
                g_ahrs_debug.actual_conversion_factor = (float)g_ahrs_debug.raw_gyro_z / g_ahrs_debug.gyro_z_dps;
            }
            
            // 执行AHRS更新
            ahrs_update_quaternion(imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z,
                                  imu_data.acc_x, imu_data.acc_y, imu_data.acc_z);
            
            // 转换为欧拉角
            ahrs_quaternion_to_euler();
            
            // 记录姿态角
            g_ahrs_debug.pitch_deg = euler_angles.pitch;
            g_ahrs_debug.roll_deg = euler_angles.roll;
            g_ahrs_debug.yaw_deg = euler_angles.yaw;
            g_ahrs_debug.yaw_accumulated_deg = euler_angles.yaw_accumulated;
            g_ahrs_debug.yaw_gyro_deg = yaw_gyro;  // 记录1D Yaw积分器的值
        }
        
        // 重置采样计数和累加缓冲区
        sample_count = 0;
        memset(&raw_data_summed, 0, sizeof(SCH1_raw_data));
    }
    
    return system_ready ? AHRS_STATUS_OK : AHRS_STATUS_CALIBRATING;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取当前欧拉角
// 参数说明     euler           欧拉角输出指针
// 返回参数     状态
//-------------------------------------------------------------------------------------------------------------------
ahrs_status_enum ahrs_get_euler_angles(ahrs_euler_angles_t *euler)
{
    if(!system_ready || euler == NULL)
        return AHRS_STATUS_ERROR;
    
    *euler = euler_angles;
    return AHRS_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取当前四元数
// 参数说明     quat            四元数输出指针
// 返回参数     状态
//-------------------------------------------------------------------------------------------------------------------
ahrs_status_enum ahrs_get_quaternion(ahrs_quaternion_t *quat)
{
    if(!system_ready || quat == NULL)
        return AHRS_STATUS_ERROR;
    
    *quat = quaternion;
    return AHRS_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取陀螺仪零偏
// 参数说明     offset          零偏输出指针
// 返回参数     状态
//-------------------------------------------------------------------------------------------------------------------
ahrs_status_enum ahrs_get_gyro_offset(ahrs_gyro_offset_t *offset)
{
    if(!system_ready || offset == NULL)
        return AHRS_STATUS_ERROR;
    
    *offset = gyro_offset;
    return AHRS_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     重置航向角零点
// 参数说明     无
// 返回参数     状态
//-------------------------------------------------------------------------------------------------------------------
ahrs_status_enum ahrs_reset_yaw(void)
{
    yaw_offset = 0.0f;
    direction_change = 0;
    last_yaw = euler_angles.yaw;
    
    // 同时重置1D Yaw积分器
    yaw_gyro = 0.0f;
    
    printf("航向角已重置（包括1D Yaw积分器）\r\n");
    return AHRS_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取纯陀螺积分的航向角（1D Yaw积分器）
// 参数说明     yaw_deg         航向角输出指针（度）
// 返回参数     状态
// 备注信息     此航向角仅由陀螺仪Z轴积分得到，用于平面导航和转向控制
//-------------------------------------------------------------------------------------------------------------------
ahrs_status_enum ahrs_get_yaw_gyro(float *yaw_deg)
{
    if(!system_ready || yaw_deg == NULL)
        return AHRS_STATUS_ERROR;
    
    *yaw_deg = yaw_gyro;
    return AHRS_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置PI控制器参数
// 参数说明     kp              比例增益
//             ki              积分增益
// 返回参数     状态
//-------------------------------------------------------------------------------------------------------------------
ahrs_status_enum ahrs_set_pi_params(float kp, float ki)
{
    param_kp = kp;
    param_ki = ki;
    
    printf("PI控制器参数已更新: Kp=%.3f, Ki=%.3f\r\n", kp, ki);
    return AHRS_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检查系统是否准备就绪
// 参数说明     无
// 返回参数     1-就绪 0-未就绪
//-------------------------------------------------------------------------------------------------------------------
uint8 ahrs_is_ready(void)
{
    return system_ready;
}

//=================================================内部函数实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     快速平方根倒数算法（Quake III算法）
// 参数说明     x               输入值
// 返回参数     1/sqrt(x)
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
// 函数简介     获取并处理传感器数据
// 参数说明     sensor_data     原始传感器数据
// 返回参数     无
//-------------------------------------------------------------------------------------------------------------------
static void ahrs_get_sensor_values(SCH1_result *sensor_data)
{
    // 加速度计数据处理（低通滤波 + 单位转换）
    imu_data.acc_x = ALPHA * (sensor_data->Acc1[AXIS_X] / G_TO_M_S2) + 
                     (1.0f - ALPHA) * imu_data.acc_x;
    imu_data.acc_y = ALPHA * (sensor_data->Acc1[AXIS_Y] / G_TO_M_S2) + 
                     (1.0f - ALPHA) * imu_data.acc_y;
    imu_data.acc_z = ALPHA * (sensor_data->Acc1[AXIS_Z] / G_TO_M_S2) + 
                     (1.0f - ALPHA) * imu_data.acc_z;
    
    // 陀螺仪数据处理（零偏补偿）
    imu_data.gyro_x = (sensor_data->Rate1[AXIS_X] - gyro_offset.x) * DEG_TO_RAD;
    imu_data.gyro_y = (sensor_data->Rate1[AXIS_Y] - gyro_offset.y) * DEG_TO_RAD;
    imu_data.gyro_z = (sensor_data->Rate1[AXIS_Z] - gyro_offset.z) * DEG_TO_RAD;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     AHRS四元数更新（含PI控制器漂移补偿，但感觉越不越飘，弃用）
// 参数说明     gx, gy, gz      陀螺仪数据 (rad/s)
//             ax, ay, az      加速度计数据 (g)
// 返回参数     无
//-------------------------------------------------------------------------------------------------------------------
static void ahrs_update_quaternion(float gx, float gy, float gz, float ax, float ay, float az)
{
    float half_t = 0.5f * DELTA_T;
    float vx, vy, vz;
    float ex, ey, ez;
    
    // 当前四元数值
    float q0 = quaternion.q0;
    float q1 = quaternion.q1;
    float q2 = quaternion.q2;
    float q3 = quaternion.q3;
    
    // 预计算常用项
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q1q1 = q1 * q1;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;
    
    // 归一化加速度计数据
    float norm = fast_inv_sqrt(ax * ax + ay * ay + az * az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;
    
    // 估计重力方向（从四元数计算）
    vx = 2.0f * (q1q3 - q0q2);
    vy = 2.0f * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    
    // 误差：加速度计测量值与估计值的叉积
    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;
    
    // PI控制器积分误差
    pi_controller.integral_ex += half_t * ex;
    pi_controller.integral_ey += half_t * ey;
    pi_controller.integral_ez += half_t * ez;
    
    // 应用PI控制器补偿陀螺仪漂移
    gx = gx + param_kp * ex + param_ki * pi_controller.integral_ex;
    gy = gy + param_kp * ey + param_ki * pi_controller.integral_ey;
    gz = gz + param_kp * ez + param_ki * pi_controller.integral_ez;
    
    // 四元数一阶更新
    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * half_t;
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * half_t;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * half_t;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * half_t;
    
    // 二阶修正
    float delta_2 = (2.0f * half_t * gx) * (2.0f * half_t * gx) +
                    (2.0f * half_t * gy) * (2.0f * half_t * gy) +
                    (2.0f * half_t * gz) * (2.0f * half_t * gz);
    
    q0 = (1.0f - delta_2 / 8.0f) * q0 + (-q1 * gx - q2 * gy - q3 * gz) * half_t;
    q1 = (1.0f - delta_2 / 8.0f) * q1 + (q0 * gx + q2 * gz - q3 * gy) * half_t;
    q2 = (1.0f - delta_2 / 8.0f) * q2 + (q0 * gy - q1 * gz + q3 * gx) * half_t;
    q3 = (1.0f - delta_2 / 8.0f) * q3 + (q0 * gz + q1 * gy - q2 * gx) * half_t;
    
    // 四元数归一化
    norm = fast_inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    quaternion.q0 = q0 * norm;
    quaternion.q1 = q1 * norm;
    quaternion.q2 = q2 * norm;
    quaternion.q3 = q3 * norm;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     四元数转欧拉角（含航向角累积处理）
// 参数说明     无
// 返回参数     无
//-------------------------------------------------------------------------------------------------------------------
static void ahrs_quaternion_to_euler(void)
{
    float q0 = quaternion.q0;
    float q1 = quaternion.q1;
    float q2 = quaternion.q2;
    float q3 = quaternion.q3;
    
    // 计算俯仰角 (Pitch)
    euler_angles.pitch = asin(-2.0f * q1 * q3 + 2.0f * q0 * q2) * 180.0f / AHRS_PI;
    
    // 计算滚转角 (Roll)
    euler_angles.roll = atan2(2.0f * q2 * q3 + 2.0f * q0 * q1, 
                              -2.0f * q1 * q1 - 2.0f * q2 * q2 + 1.0f) * 180.0f / AHRS_PI;
    
    // 计算偏航角 (Yaw)
    euler_angles.yaw = atan2(2.0f * q1 * q2 + 2.0f * q0 * q3, 
                             -2.0f * q2 * q2 - 2.0f * q3 * q3 + 1.0f) * 180.0f / AHRS_PI;
    
    // 角度范围限制到 [-180, 180]
    if(euler_angles.yaw >= 180.0f)
        euler_angles.yaw -= 360.0f;
    else if(euler_angles.yaw <= -180.0f)
        euler_angles.yaw += 360.0f;
    
    // 航向角累积处理（处理跨越±180°的情况）
    if((euler_angles.yaw - last_yaw) < -350.0f)
        direction_change++;
    else if((euler_angles.yaw - last_yaw) > 350.0f)
        direction_change--;
    
    // 计算累积航向角
    euler_angles.yaw_accumulated = -(360.0f * direction_change + euler_angles.yaw - yaw_offset);
    
    last_yaw = euler_angles.yaw;
}

