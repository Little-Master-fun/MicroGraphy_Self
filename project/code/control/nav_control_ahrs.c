/*********************************************************************************************************************
* 文件名称          nav_control_ahrs.c
* 功能说明          基于航向角跟踪的导航控制系统实现文件
* 作者              AI Assistant
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本
* 2025-01-XX        AI Assistant       1.0v
* 
* 文件作用说明：
* 本文件实现类似dog项目的航向角跟踪导航系统
* 不使用坐标位置，仅通过跟踪路径上的航向角来实现导航
********************************************************************************************************************/

#include "nav_control_ahrs.h"
#include "imu_ahrs_complementary.h"
#include "driver_encoder.h"
#include "motor_control.h"
#include "dual_core_comm.h"
#include "zf_common_headfile.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

//=================================================全局变量定义================================================
nav_ahrs_controller_t nav_ahrs = {0};

//=================================================内部函数声明================================================
static nav_ahrs_status_enum nav_ahrs_update_state(void);
static nav_ahrs_status_enum nav_ahrs_find_lookahead_point(void);
static float nav_ahrs_calculate_curvature(uint16 idx);
static nav_ahrs_status_enum nav_ahrs_angle_pid_control(void);
static float nav_ahrs_angle_difference(float target, float current);

//=================================================外部接口实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化AHRS导航系统
//-------------------------------------------------------------------------------------------------------------------
nav_ahrs_status_enum nav_ahrs_init(void)
{
    // 清零系统结构体
    memset(&nav_ahrs, 0, sizeof(nav_ahrs_controller_t));
    
    // 初始化转弯PID参数（大误差，快速响应）
    nav_ahrs.pid_turn.kp = 0.03f;
    nav_ahrs.pid_turn.ki = 0.005f;
    nav_ahrs.pid_turn.kd = 0.003f;
    nav_ahrs.pid_turn.integral_limit = 50.0f;
    
    // 初始化直线PID参数（小误差，稳定精确）
    nav_ahrs.pid_straight.kp = 0.1f;
    nav_ahrs.pid_straight.ki = 0.002f;
    nav_ahrs.pid_straight.kd = 0.3f;
    nav_ahrs.pid_straight.integral_limit = 20.0f;
    
    // 初始化速度参数
    nav_ahrs.base_speed = 2.5f;  // 基础速度 (m/s)
    
    // 初始化前瞻参数
    nav_ahrs.lookahead_distance = NAV_AHRS_DISTANCE_PER_POINT * 5;  // 默认前瞻5个点 (mm)
    
    // 系统状态
    nav_ahrs.mode = NAV_AHRS_MODE_IDLE;
    nav_ahrs.status = NAV_AHRS_STATUS_OK;
    nav_ahrs.initialized = 1;
    nav_ahrs.path_loaded = 0;
    
    printf("[NAV_AHRS] 导航系统初始化完成\n");
    printf("[NAV_AHRS] 转弯PID: Kp=%.1f, Ki=%.1f, Kd=%.1f\n", 
           nav_ahrs.pid_turn.kp, nav_ahrs.pid_turn.ki, nav_ahrs.pid_turn.kd);
    printf("[NAV_AHRS] 直线PID: Kp=%.1f, Ki=%.1f, Kd=%.1f\n", 
           nav_ahrs.pid_straight.kp, nav_ahrs.pid_straight.ki, nav_ahrs.pid_straight.kd);
    
    return NAV_AHRS_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导航系统更新（周期调用）
// 参数说明     dt              时间间隔 (s)
// 返回参数     状态
//-------------------------------------------------------------------------------------------------------------------
nav_ahrs_status_enum nav_ahrs_update(float dt)
{
    
    if (!nav_ahrs.initialized) {
        return NAV_AHRS_STATUS_NOT_INIT;
    }
    
    if (!nav_ahrs.path_loaded) {
        return NAV_AHRS_STATUS_ERROR;
    }
    
    if (nav_ahrs.mode != NAV_AHRS_MODE_REPLAY) {
        // 非回放模式，停车
        nav_ahrs.left_speed = 0.0f;
        nav_ahrs.right_speed = 0.0f;
        return NAV_AHRS_STATUS_OK;
    }
    
    // 1. 更新当前状态（里程、航向角）
    nav_ahrs_status_enum status = nav_ahrs_update_state();
    if (status != NAV_AHRS_STATUS_OK) {
        // 状态更新失败（例如yaw为NaN，正在校准），停止导航
        nav_ahrs.left_speed = 0.0f;
        nav_ahrs.right_speed = 0.0f;
        printf("[NAV_AHRS] 状态更新失败，停止导航（可能正在校准偏置）\n");
        return status;
    }
    
    // 2. 查找前瞻点
    nav_ahrs_find_lookahead_point();
    
    // 3. 计算路径曲率
    nav_ahrs.curvature = nav_ahrs_calculate_curvature(nav_ahrs.lookahead_index);
    
    // 4. 角度PID控制
    nav_ahrs_angle_pid_control();
    
    // 5. 计算的目标速度会通过共享内存发送给CM7_0
    // CM7_0会调用motor_set_target_speed(nav_ahrs.left_speed, nav_ahrs.right_speed);
    
    // 6. 检查是否完成一圈
    if (nav_ahrs.path.current_index >= nav_ahrs.path.total_points - 1) {
        if (nav_ahrs.path.loop_mode) {
            // 循环模式，重置到起点
            nav_ahrs.path.current_index = 0;
            nav_ahrs.current_distance = 0.0f;
            // 注意：双核架构中，编码器重置由CM7_0负责
            printf("[NAV_AHRS] 完成一圈，重新开始\n");
        } else {
            // 非循环模式，停车
            nav_ahrs.mode = NAV_AHRS_MODE_IDLE;
            nav_ahrs.left_speed = 0.0f;
            nav_ahrs.right_speed = 0.0f;
            // 速度会通过共享内存发送给CM7_0，CM7_0会停止电机
            printf("[NAV_AHRS] 路径完成\n");
            return NAV_AHRS_STATUS_PATH_END;
        }
    }
    
    return NAV_AHRS_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取电机控制输出（仅用于调试显示）
//-------------------------------------------------------------------------------------------------------------------
nav_ahrs_status_enum nav_ahrs_get_motor_output(int32 *left_pwm, int32 *right_pwm)
{
    // 返回当前速度值（用于调试显示）
    if (left_pwm != NULL && right_pwm != NULL) {
        *left_pwm = (int32)(nav_ahrs.left_speed * 1000.0f);  // 转换为 mm/s 显示
        *right_pwm = (int32)(nav_ahrs.right_speed * 1000.0f);
    }
    
    return NAV_AHRS_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置导航模式
//-------------------------------------------------------------------------------------------------------------------
nav_ahrs_status_enum nav_ahrs_set_mode(nav_ahrs_mode_enum mode)
{
    // 如果要进入回放模式，先检查yaw是否有效
    if (mode == NAV_AHRS_MODE_REPLAY) {
        // 同步DCache，确保读取到最新数据
        SCB_CleanInvalidateDCache_by_Addr((uint32_t*)&shared_core0_data, sizeof(core0_to_core1_data_t));
        
        // 检查yaw是否为NaN
        if (isnan(shared_core0_data.yaw)) {
            printf("[NAV_AHRS] 无法启动导航：yaw为NaN，AHRS正在校准偏置，请等待校准完成\n");
            return NAV_AHRS_STATUS_ERROR;
        }
        
        printf("[NAV_AHRS] 开始导航回放\n");
    }
    
    nav_ahrs.mode = mode;
    
    const char* mode_names[] = {"空闲", "回放", "暂停"};
    printf("[NAV_AHRS] 模式切换为: %s\n", mode_names[mode]);
    
    return NAV_AHRS_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置基础速度
//-------------------------------------------------------------------------------------------------------------------
nav_ahrs_status_enum nav_ahrs_set_speed(float speed)
{
    nav_ahrs.base_speed = speed;
    printf("[NAV_AHRS] 基础速度设置为: %.1f\n", speed);
    return NAV_AHRS_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     重置导航系统
//-------------------------------------------------------------------------------------------------------------------
nav_ahrs_status_enum nav_ahrs_reset(void)
{
    nav_ahrs.current_distance = 0.0f;
    nav_ahrs.left_distance = 0.0f;
    nav_ahrs.right_distance = 0.0f;
    nav_ahrs.path.current_index = 0;
    nav_ahrs.lookahead_index = 0;
    
    nav_ahrs.pid_turn.integral = 0.0f;
    nav_ahrs.pid_turn.error_last = 0.0f;
    nav_ahrs.pid_straight.integral = 0.0f;
    nav_ahrs.pid_straight.error_last = 0.0f;
    
    nav_ahrs.left_speed = 0.0f;
    nav_ahrs.right_speed = 0.0f;
    
    // 双核架构：编码器重置和电机停止由CM7_0负责
    // CM7_0会通过共享内存接收到速度为0的指令并停止电机
    
    printf("[NAV_AHRS] 导航系统已重置\n");
    return NAV_AHRS_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取导航状态信息
//-------------------------------------------------------------------------------------------------------------------
nav_ahrs_status_enum nav_ahrs_get_status(float *yaw, float *target_yaw, 
                                          float *error, float *curvature)
{
    if (yaw) *yaw = nav_ahrs.current_yaw;
    if (target_yaw) *target_yaw = nav_ahrs.target_yaw;
    if (error) *error = nav_ahrs.angle_error;
    if (curvature) *curvature = nav_ahrs.curvature;
    
    return NAV_AHRS_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置PID参数
//-------------------------------------------------------------------------------------------------------------------
nav_ahrs_status_enum nav_ahrs_set_pid(uint8 is_turn, float kp, float ki, float kd)
{
    if (is_turn) {
        nav_ahrs.pid_turn.kp = kp;
        nav_ahrs.pid_turn.ki = ki;
        nav_ahrs.pid_turn.kd = kd;
        printf("[NAV_AHRS] 转弯PID: Kp=%.1f, Ki=%.1f, Kd=%.1f\n", kp, ki, kd);
    } else {
        nav_ahrs.pid_straight.kp = kp;
        nav_ahrs.pid_straight.ki = ki;
        nav_ahrs.pid_straight.kd = kd;
        printf("[NAV_AHRS] 直线PID: Kp=%.1f, Ki=%.1f, Kd=%.1f\n", kp, ki, kd);
    }
    
    return NAV_AHRS_STATUS_OK;
}

//=================================================内部函数实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新当前状态（从共享内存读取CM7_0的数据）
//-------------------------------------------------------------------------------------------------------------------
static nav_ahrs_status_enum nav_ahrs_update_state(void)
{
    // 1. 同步DCache，确保读取到RAM中的最新数据
    SCB_CleanInvalidateDCache_by_Addr((uint32_t*)&shared_core0_data, sizeof(core0_to_core1_data_t));
    
    // 2. 检查数据有效性
    if (!shared_core0_data.data_valid) {
        return NAV_AHRS_STATUS_ERROR;
    }
    
    // 3. 检查yaw是否为NaN - 如果是NaN说明正在静止记录偏置（校准中）
    if (isnan(shared_core0_data.yaw)) {
        // yaw为NaN，AHRS正在校准陀螺仪偏置，导航不启动
        static uint8 nan_warning_printed = 0;
        if (!nan_warning_printed) {
            printf("[NAV_AHRS] yaw为NaN，AHRS正在校准偏置，等待校准完成...\n");
            nan_warning_printed = 1;
        }
        return NAV_AHRS_STATUS_ERROR;
    }
    
    // 4. 直接从共享内存获取累积距离（CM7_0的encoder_update()已计算）
    nav_ahrs.current_distance = shared_core0_data.distance;  // 单位: mm
    
    // 5. 获取当前航向角 - 从共享内存
    nav_ahrs.current_yaw = shared_core0_data.yaw;  // 使用CM7_0计算的航向角
    
    // 6. 更新当前路径点索引
    for (uint16 i = nav_ahrs.path.current_index; i < nav_ahrs.path.total_points; i++) {
        if (nav_ahrs.path.points[i].distance > nav_ahrs.current_distance) {
            nav_ahrs.path.current_index = i;
            break;
        }
    }
    
    return NAV_AHRS_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     查找前瞻点
//-------------------------------------------------------------------------------------------------------------------
static nav_ahrs_status_enum nav_ahrs_find_lookahead_point(void)
{
    // 动态前瞻距离（根据速度和曲率调整）
    float speed_factor = nav_ahrs.base_speed / 2.5f;  // 速度因子（基准速度 2.5 m/s）
    float curvature_factor = 1.0f / (1.0f + fabs(nav_ahrs.curvature) * 0.1f);  // 曲率因子
    
    float lookahead_dist = nav_ahrs.lookahead_distance * speed_factor * curvature_factor;
    float target_distance = nav_ahrs.current_distance + lookahead_dist;
    
    // 在路径点中查找前瞻点
    nav_ahrs.lookahead_index = nav_ahrs.path.current_index;
    
    for (uint16 i = nav_ahrs.path.current_index; i < nav_ahrs.path.total_points; i++) {
        if (nav_ahrs.path.points[i].distance >= target_distance) {
            nav_ahrs.lookahead_index = i;
            break;
        }
    }
    
    // 防止索引越界
    if (nav_ahrs.lookahead_index >= nav_ahrs.path.total_points) {
        nav_ahrs.lookahead_index = nav_ahrs.path.total_points - 1;
    }
    
    // 获取目标航向角
    nav_ahrs.target_yaw = nav_ahrs.path.points[nav_ahrs.lookahead_index].yaw;
    
    return NAV_AHRS_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算路径曲率
// 参数说明     idx             当前点索引
// 返回参数     曲率值
//-------------------------------------------------------------------------------------------------------------------
static float nav_ahrs_calculate_curvature(uint16 idx)
{
    // 使用三个点计算曲率
    if (idx < 2 || idx >= nav_ahrs.path.total_points - 2) {
        return 0.0f;
    }
    
    float theta1 = nav_ahrs.path.points[idx - 2].yaw;
    float theta3 = nav_ahrs.path.points[idx + 2].yaw;
    
    float d12 = nav_ahrs.path.points[idx].distance - nav_ahrs.path.points[idx - 2].distance;
    float d23 = nav_ahrs.path.points[idx + 2].distance - nav_ahrs.path.points[idx].distance;
    
    if (d12 < 0.001f || d23 < 0.001f) {
        return 0.0f;
    }
    
    // 计算方向向量
    float T1_x = cosf(theta1 * M_PI / 180.0f);
    float T1_y = sinf(theta1 * M_PI / 180.0f);
    float T3_x = cosf(theta3 * M_PI / 180.0f);
    float T3_y = sinf(theta3 * M_PI / 180.0f);
    
    // 方向向量差分
    float delta_Tx = T3_x - T1_x;
    float delta_Ty = T3_y - T1_y;
    float total_distance = (d12 + d23) / 2.0f;
    
    // 曲率
    float delta_T_norm = sqrtf(delta_Tx * delta_Tx + delta_Ty * delta_Ty);
    float angle_diff = theta3 - theta1;
    if (angle_diff != 0) {
        delta_T_norm *= (angle_diff / fabs(angle_diff));
    }
    
    float curvature = delta_T_norm / total_distance * 1000.0f;
    
    // 曲率限幅
    if (curvature > 65.0f) curvature = 65.0f;
    if (curvature < -65.0f) curvature = -65.0f;
    
    return curvature;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     角度PID控制
//-------------------------------------------------------------------------------------------------------------------
static nav_ahrs_status_enum nav_ahrs_angle_pid_control(void)
{
    // 1. 计算角度误差
    nav_ahrs.angle_error = nav_ahrs_angle_difference(nav_ahrs.target_yaw, nav_ahrs.current_yaw);
    
    // 2. 根据误差大小选择PID控制器
    nav_ahrs_pid_t *pid;
    if (fabs(nav_ahrs.angle_error) > 20.0f) {
        // 大误差，使用转弯PID
        pid = &nav_ahrs.pid_turn;
    } else {
        // 小误差，使用直线PID
        pid = &nav_ahrs.pid_straight;
    }
    
    // 3. PID计算
    pid->error = nav_ahrs.angle_error;
    
    // 积分项
    pid->integral += pid->error;
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
    
    // PID输出
    pid->output = pid->kp * pid->error + 
                  pid->ki * pid->integral +
                  pid->kd * (pid->error - pid->error_last);
    
    pid->error_last = pid->error;
    
    // 4. 计算左右轮速度
    float direction_adjust = pid->output * 0.01f;  // PID输出缩放到速度调整量 (m/s)
    
    nav_ahrs.left_speed = nav_ahrs.base_speed - direction_adjust;
    nav_ahrs.right_speed = nav_ahrs.base_speed + direction_adjust;
    
    // 速度限幅 (m/s)
    float max_speed = 3.0f;  // 最大速度 3 m/s
    if (nav_ahrs.left_speed > max_speed) nav_ahrs.left_speed = max_speed;
    if (nav_ahrs.left_speed < -max_speed) nav_ahrs.left_speed = -max_speed;
    if (nav_ahrs.right_speed > max_speed) nav_ahrs.right_speed = max_speed;
    if (nav_ahrs.right_speed < -max_speed) nav_ahrs.right_speed = -max_speed;
    
    return NAV_AHRS_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算角度差（考虑跨越±180°）
//-------------------------------------------------------------------------------------------------------------------
static float nav_ahrs_angle_difference(float target, float current)
{
    float diff = target - current;
    
    // 处理跨越±180°的情况
    if (diff > 180.0f) {
        diff -= 360.0f;
    } else if (diff < -180.0f) {
        diff += 360.0f;
    }
    
    return diff;
}

