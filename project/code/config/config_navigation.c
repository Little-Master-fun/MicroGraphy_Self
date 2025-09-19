/*********************************************************************************************************************
* 文件名称          config_navigation.c
* 功能说明          导航系统配置参数实现文件
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本              备注
* 2025-09-18        LittleMaster        v1.0              创建导航系统配置实现
*
* 文件作用说明：
* 本文件实现导航系统配置参数的管理功能，包括参数初始化、保存、加载和访问接口
* 提供统一的配置管理，避免模块间的参数依赖问题
*
* 主要功能：
* 1. 配置参数的初始化和默认值设置
* 2. 配置参数的Flash存储和读取
* 3. 运行时配置参数的获取和设置
* 4. 配置参数的有效性验证
* 5. 向后兼容性支持
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "config_navigation.h"
//#include "flash_road.h"

//=================================================全局变量定义================================================
nav_config_struct nav_config = {0};            // 导航系统配置结构体
nav_data_struct nav_data = {0};                // 导航系统数据结构体

// 共享数据数组
float nav_errors_coords[NAV_COORD_RECORD] = {0};       // 错误坐标数组
int nav_mileage_list[NAV_COORD_RECORD] = {0};          // 里程列表数组
int32 nav_yaw_buffer[4000] = {0};                     // 偏航角缓冲数组

//=================================================内部函数声明================================================
static uint8 nav_config_validate(void);
static void nav_config_apply_limits(void);

//=================================================主要接口函数================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导航配置系统初始化
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_config_init(void)
{
    // 首先加载默认配置
    nav_config_load_default();
    
    // 尝试从Flash加载保存的配置
    if (nav_config_load() != 0)
    {
        // Flash加载失败，使用默认配置
        nav_config_load_default();
    }
    
    // 验证配置参数有效性
    if (nav_config_validate() != 0)
    {
        // 配置无效，恢复默认配置
        nav_config_load_default();
    }
    
    // 应用参数限制
    nav_config_apply_limits();
    
    // 标记配置已加载
    nav_config.config_loaded = 1;
    nav_config.config_modified = 0;
    
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     加载默认配置
//-------------------------------------------------------------------------------------------------------------------
void nav_config_load_default(void)
{
    // 路径点坐标默认值
    nav_config.path_points_x[0] = NAV_DEFAULT_X1;
    nav_config.path_points_y[0] = NAV_DEFAULT_Y1;
    nav_config.path_points_x[1] = NAV_DEFAULT_X2;
    nav_config.path_points_y[1] = NAV_DEFAULT_Y2;
    nav_config.path_points_x[2] = NAV_DEFAULT_X3;
    nav_config.path_points_y[2] = NAV_DEFAULT_Y3;
    nav_config.path_points_x[3] = NAV_DEFAULT_X4;
    nav_config.path_points_y[3] = NAV_DEFAULT_Y4;
    nav_config.path_points_x[4] = NAV_DEFAULT_X5;
    nav_config.path_points_y[4] = NAV_DEFAULT_Y5;
    nav_config.path_points_x[5] = NAV_DEFAULT_X6;
    nav_config.path_points_y[5] = NAV_DEFAULT_Y6;
    
    // 控制参数默认值
    nav_config.control_radius[0] = NAV_DEFAULT_R1;
    nav_config.control_radius[1] = NAV_DEFAULT_R2;
    
    // 导航PID参数默认值
    nav_config.nav_pid_kp = NAV_DEFAULT_PID_KP;
    nav_config.nav_pid_ki = NAV_DEFAULT_PID_KI;
    nav_config.nav_pid_kd = NAV_DEFAULT_PID_KD;
    nav_config.nav_pid_speed = NAV_DEFAULT_PID_SPEED;
    
    // 电机控制PID参数默认值 (使用实际调试参数)
    // 左电机速度环PID (PIDSL - 左轮速度，继承PIDS参数)
    nav_config.motor_control.left_speed_kp = LEFT_WHEEL_SPEED_PID_KP;  // 30.0f
    nav_config.motor_control.left_speed_ki = LEFT_WHEEL_SPEED_PID_KI;  // 2.4f
    nav_config.motor_control.left_speed_kd = LEFT_WHEEL_SPEED_PID_KD;  // 0.0f
    
    // 左电机位置环PID (暂时保持原值)
    nav_config.motor_control.left_position_kp = 2.0f;
    nav_config.motor_control.left_position_ki = 0.1f;
    nav_config.motor_control.left_position_kd = 0.05f;
    
    // 右电机速度环PID (PIDS - 速度控制)
    nav_config.motor_control.right_speed_kp = MOTOR_SPEED_PID_KP;      // 30.0f
    nav_config.motor_control.right_speed_ki = MOTOR_SPEED_PID_KI;      // 2.4f
    nav_config.motor_control.right_speed_kd = MOTOR_SPEED_PID_KD;      // 0.0f
    
    // 右电机位置环PID (暂时保持原值)
    nav_config.motor_control.right_position_kp = 2.0f;
    nav_config.motor_control.right_position_ki = 0.1f;
    nav_config.motor_control.right_position_kd = 0.05f;
    
    // 电机控制参数默认值
    nav_config.motor_control.max_speed = 2.0f;          // 最大速度 2m/s
    nav_config.motor_control.max_acceleration = 5.0f;   // 最大加速度 5m/s?
    nav_config.motor_control.wheelbase = 150.0f;        // 轮距 150mm
    nav_config.motor_control.control_frequency = 1000;  // 控制频率 1KHz
    
    // 系统运行参数默认值
    nav_config.target_speed = NAV_DEFAULT_TARGET_SPEED;
    nav_config.fuya_enable = NAV_FUYA_DEFAULT;
    nav_config.go_flag = 0;
    nav_config.go_state = 0;
    
    // 系统状态
    nav_config.config_loaded = 0;
    nav_config.config_modified = 1;  // 标记为已修改，需要保存
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     保存当前配置到Flash
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_config_save(void)
{
    // 这里可以使用Flash存储系统保存配置
    // 暂时返回成功，实际实现需要根据Flash存储接口
    
    // 标记配置已保存
    nav_config.config_modified = 0;
    
    return 0;  // 成功
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     从Flash加载配置
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_config_load(void)
{
    // 这里可以使用Flash存储系统加载配置
    // 暂时返回失败，实际实现需要根据Flash存储接口
    
    return 1;  // 失败，使用默认配置
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取路径点坐标
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_config_get_path_point(uint8 point_index, float *x, float *y)
{
    if (point_index >= 6 || x == NULL || y == NULL)
    {
        return 1;  // 参数错误
    }
    
    *x = nav_config.path_points_x[point_index];
    *y = nav_config.path_points_y[point_index];
    
    return 0;  // 成功
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置路径点坐标
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_config_set_path_point(uint8 point_index, float x, float y)
{
    if (point_index >= 6)
    {
        return 1;  // 参数错误
    }
    
    nav_config.path_points_x[point_index] = x;
    nav_config.path_points_y[point_index] = y;
    nav_config.config_modified = 1;  // 标记配置已修改
    
    return 0;  // 成功
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取控制半径参数
//-------------------------------------------------------------------------------------------------------------------
int nav_config_get_control_radius(uint8 radius_index)
{
    if (radius_index >= 2)
    {
        return 0;  // 参数错误，返回0
    }
    
    return nav_config.control_radius[radius_index];
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置控制半径参数
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_config_set_control_radius(uint8 radius_index, int radius)
{
    if (radius_index >= 2)
    {
        return 1;  // 参数错误
    }
    
    nav_config.control_radius[radius_index] = radius;
    nav_config.config_modified = 1;  // 标记配置已修改
    
    return 0;  // 成功
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取PID参数
//-------------------------------------------------------------------------------------------------------------------
void nav_config_get_pid(float *kp, float *ki, float *kd, int *speed)
{
    if (kp != NULL) *kp = nav_config.nav_pid_kp;
    if (ki != NULL) *ki = nav_config.nav_pid_ki;
    if (kd != NULL) *kd = nav_config.nav_pid_kd;
    if (speed != NULL) *speed = nav_config.nav_pid_speed;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置PID参数
//-------------------------------------------------------------------------------------------------------------------
void nav_config_set_pid(float kp, float ki, float kd, int speed)
{
    nav_config.nav_pid_kp = kp;
    nav_config.nav_pid_ki = ki;
    nav_config.nav_pid_kd = kd;
    nav_config.nav_pid_speed = speed;
    nav_config.config_modified = 1;  // 标记配置已修改
}

//=================================================内部函数实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     验证配置参数有效性
//-------------------------------------------------------------------------------------------------------------------
static uint8 nav_config_validate(void)
{
    // 检查路径点坐标范围（示例：-1000 到 1000）
    for (int i = 0; i < 6; i++)
    {
        if (nav_config.path_points_x[i] < -1000.0f || nav_config.path_points_x[i] > 1000.0f ||
            nav_config.path_points_y[i] < -1000.0f || nav_config.path_points_y[i] > 1000.0f)
        {
            return 1;  // 坐标超出范围
        }
    }
    
    // 检查控制半径范围（示例：1 到 100）
    for (int i = 0; i < 2; i++)
    {
        if (nav_config.control_radius[i] < 1 || nav_config.control_radius[i] > 100)
        {
            return 1;  // 半径超出范围
        }
    }
    
    // 检查导航PID参数范围
    if (nav_config.nav_pid_kp < 0.0f || nav_config.nav_pid_kp > 100.0f ||
        nav_config.nav_pid_ki < 0.0f || nav_config.nav_pid_ki > 100.0f ||
        nav_config.nav_pid_kd < 0.0f || nav_config.nav_pid_kd > 100.0f)
    {
        return 1;  // PID参数超出范围
    }
    
    // 检查速度参数范围（示例：0 到 500）
    if (nav_config.nav_pid_speed < 0 || nav_config.nav_pid_speed > 500 ||
        nav_config.target_speed < 0 || nav_config.target_speed > 500)
    {
        return 1;  // 速度参数超出范围
    }
    
    return 0;  // 参数有效
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     应用参数限制
//-------------------------------------------------------------------------------------------------------------------
static void nav_config_apply_limits(void)
{
    // 限制路径点坐标范围
    for (int i = 0; i < 6; i++)
    {
        if (nav_config.path_points_x[i] < -1000.0f) nav_config.path_points_x[i] = -1000.0f;
        if (nav_config.path_points_x[i] > 1000.0f) nav_config.path_points_x[i] = 1000.0f;
        if (nav_config.path_points_y[i] < -1000.0f) nav_config.path_points_y[i] = -1000.0f;
        if (nav_config.path_points_y[i] > 1000.0f) nav_config.path_points_y[i] = 1000.0f;
    }
    
    // 限制控制半径范围
    for (int i = 0; i < 2; i++)
    {
        if (nav_config.control_radius[i] < 1) nav_config.control_radius[i] = 1;
        if (nav_config.control_radius[i] > 100) nav_config.control_radius[i] = 100;
    }
    
    // 限制导航PID参数范围
    if (nav_config.nav_pid_kp < 0.0f) nav_config.nav_pid_kp = 0.0f;
    if (nav_config.nav_pid_kp > 100.0f) nav_config.nav_pid_kp = 100.0f;
    if (nav_config.nav_pid_ki < 0.0f) nav_config.nav_pid_ki = 0.0f;
    if (nav_config.nav_pid_ki > 100.0f) nav_config.nav_pid_ki = 100.0f;
    if (nav_config.nav_pid_kd < 0.0f) nav_config.nav_pid_kd = 0.0f;
    if (nav_config.nav_pid_kd > 100.0f) nav_config.nav_pid_kd = 100.0f;
    
    // 限制速度参数范围
    if (nav_config.nav_pid_speed < 0) nav_config.nav_pid_speed = 0;
    if (nav_config.nav_pid_speed > 500) nav_config.nav_pid_speed = 500;
    if (nav_config.target_speed < 0) nav_config.target_speed = 0;
    if (nav_config.target_speed > 500) nav_config.target_speed = 500;
}

//=================================================导航数据管理函数================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导航数据系统初始化
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_data_init(void)
{
    // 初始化数据结构体
    memset(&nav_data, 0, sizeof(nav_data_struct));
    
    // 初始化数组
    memset(nav_errors_coords, 0, sizeof(nav_errors_coords));
    memset(nav_mileage_list, 0, sizeof(nav_mileage_list));
    memset(nav_yaw_buffer, 0, sizeof(nav_yaw_buffer));
    
    // 设置初始值
    nav_data.max_error_point_count = NAV_COORD_RECORD;
    
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取导航数据指针
//-------------------------------------------------------------------------------------------------------------------
nav_data_struct* nav_data_get_pointer(void)
{
    return &nav_data;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     重置导航数据
//-------------------------------------------------------------------------------------------------------------------
void nav_data_reset(void)
{
    // 保留最大错误点数量
    int max_points = nav_data.max_error_point_count;
    
    // 清零数据结构体
    memset(&nav_data, 0, sizeof(nav_data_struct));
    
    // 恢复最大错误点数量
    nav_data.max_error_point_count = max_points;
    
    // 清零数组
    memset(nav_errors_coords, 0, sizeof(nav_errors_coords));
    memset(nav_mileage_list, 0, sizeof(nav_mileage_list));
    // 注意：yaw_buffer 不清零，可能需要保留历史数据
}

//=================================================电机控制PID参数访问函数================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取电机控制PID参数
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_config_get_motor_pid(uint8 motor_id, uint8 pid_type, float *kp, float *ki, float *kd)
{
    if (kp == NULL || ki == NULL || kd == NULL)
    {
        return 1;
    }
    
    if (motor_id == 0)  // 左电机
    {
        if (pid_type == 0)  // 速度环
        {
            *kp = nav_config.motor_control.left_speed_kp;
            *ki = nav_config.motor_control.left_speed_ki;
            *kd = nav_config.motor_control.left_speed_kd;
        }
        else if (pid_type == 1)  // 位置环
        {
            *kp = nav_config.motor_control.left_position_kp;
            *ki = nav_config.motor_control.left_position_ki;
            *kd = nav_config.motor_control.left_position_kd;
        }
        else
        {
            return 1;
        }
    }
    else if (motor_id == 1)  // 右电机
    {
        if (pid_type == 0)  // 速度环
        {
            *kp = nav_config.motor_control.right_speed_kp;
            *ki = nav_config.motor_control.right_speed_ki;
            *kd = nav_config.motor_control.right_speed_kd;
        }
        else if (pid_type == 1)  // 位置环
        {
            *kp = nav_config.motor_control.right_position_kp;
            *ki = nav_config.motor_control.right_position_ki;
            *kd = nav_config.motor_control.right_position_kd;
        }
        else
        {
            return 1;
        }
    }
    else
    {
        return 1;
    }
    
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置电机控制PID参数
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_config_set_motor_pid(uint8 motor_id, uint8 pid_type, float kp, float ki, float kd)
{
    if (motor_id == 0)  // 左电机
    {
        if (pid_type == 0)  // 速度环
        {
            nav_config.motor_control.left_speed_kp = kp;
            nav_config.motor_control.left_speed_ki = ki;
            nav_config.motor_control.left_speed_kd = kd;
        }
        else if (pid_type == 1)  // 位置环
        {
            nav_config.motor_control.left_position_kp = kp;
            nav_config.motor_control.left_position_ki = ki;
            nav_config.motor_control.left_position_kd = kd;
        }
        else
        {
            return 1;
        }
    }
    else if (motor_id == 1)  // 右电机
    {
        if (pid_type == 0)  // 速度环
        {
            nav_config.motor_control.right_speed_kp = kp;
            nav_config.motor_control.right_speed_ki = ki;
            nav_config.motor_control.right_speed_kd = kd;
        }
        else if (pid_type == 1)  // 位置环
        {
            nav_config.motor_control.right_position_kp = kp;
            nav_config.motor_control.right_position_ki = ki;
            nav_config.motor_control.right_position_kd = kd;
        }
        else
        {
            return 1;
        }
    }
    else
    {
        return 1;
    }
    
    // 标记配置已修改
    nav_config.config_modified = 1;
    
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取电机控制参数
//-------------------------------------------------------------------------------------------------------------------
void nav_config_get_motor_params(float *max_speed, float *max_acceleration, float *wheelbase, int *control_frequency)
{
    if (max_speed != NULL)
    {
        *max_speed = nav_config.motor_control.max_speed;
    }
    
    if (max_acceleration != NULL)
    {
        *max_acceleration = nav_config.motor_control.max_acceleration;
    }
    
    if (wheelbase != NULL)
    {
        *wheelbase = nav_config.motor_control.wheelbase;
    }
    
    if (control_frequency != NULL)
    {
        *control_frequency = nav_config.motor_control.control_frequency;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置电机控制参数
//-------------------------------------------------------------------------------------------------------------------
void nav_config_set_motor_params(float max_speed, float max_accel, float wheelbase, int control_frequency)
{
    nav_config.motor_control.max_speed = max_speed;
    nav_config.motor_control.max_acceleration = max_accel;
    nav_config.motor_control.wheelbase = wheelbase;
    nav_config.motor_control.control_frequency = control_frequency;
    
    // 标记配置已修改
    nav_config.config_modified = 1;
}

nav_config_struct* nav_config_get(void)
{
    return &nav_config;
}

nav_data_struct* nav_data_get(void)
{
    return &nav_data;
}
