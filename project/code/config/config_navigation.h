/*********************************************************************************************************************
* 文件名称          config_navigation.h
* 功能说明          导航系统配置参数文件
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本              备注
* 2025-09-18        LittleMaster        v1.0              创建导航系统配置文件
*
* 文件作用说明：
* 本文件集中管理导航系统的所有可配置参数，包括路径点坐标、控制参数、系统设置等
* 通过统一的配置管理，避免模块间的循环依赖，提高代码的可维护性
*
* 配置分类：
* 1. 路径点坐标配置：X1-X6, Y1-Y6 路径关键点坐标
* 2. 控制参数配置：R1, R2 控制半径参数
* 3. PID参数配置：Kp, Ki, Kd 控制器参数
* 4. 系统运行配置：速度、里程、角度等系统参数
* 5. Flash存储配置：存储页面、数据范围等配置
* 6. UI显示配置：显示模式、按键响应等配置
*
* 使用方式：
* 1. 在需要使用配置的文件中包含本头文件
* 2. 通过宏定义或全局变量访问配置参数
* 3. 运行时可通过UI界面修改可变参数
* 4. 系统重启后自动加载默认配置
********************************************************************************************************************/

#ifndef _CONFIG_NAVIGATION_H_
#define _CONFIG_NAVIGATION_H_

#include "zf_common_typedef.h"

//=================================================路径点坐标配置================================================
// 默认路径点坐标（可通过UI界面调整）
#define NAV_DEFAULT_X1              (0.0f)      // 路径点1 X坐标
#define NAV_DEFAULT_Y1              (10.0f)     // 路径点1 Y坐标
#define NAV_DEFAULT_X2              (0.0f)      // 路径点2 X坐标
#define NAV_DEFAULT_Y2              (10.0f)     // 路径点2 Y坐标
#define NAV_DEFAULT_X3              (0.0f)      // 路径点3 X坐标
#define NAV_DEFAULT_Y3              (20.0f)     // 路径点3 Y坐标
#define NAV_DEFAULT_X4              (0.0f)      // 路径点4 X坐标
#define NAV_DEFAULT_Y4              (20.0f)     // 路径点4 Y坐标
#define NAV_DEFAULT_X5              (0.0f)      // 路径点5 X坐标
#define NAV_DEFAULT_Y5              (20.0f)     // 路径点5 Y坐标
#define NAV_DEFAULT_X6              (0.0f)      // 路径点6 X坐标
#define NAV_DEFAULT_Y6              (20.0f)     // 路径点6 Y坐标

//=================================================控制参数配置================================================
#define NAV_DEFAULT_R1              (30)        // 控制半径1
#define NAV_DEFAULT_R2              (30)        // 控制半径2

//=================================================PID参数配置================================================
// 主要路径跟踪PID参数 (PIDG - 导航控制)
#define NAV_DEFAULT_PID_KP          (24.3f)     // PID比例系数 (主要路径跟踪)
#define NAV_DEFAULT_PID_KI          (0.0f)      // PID积分系数
#define NAV_DEFAULT_PID_KD          (29.5f)     // PID微分系数
#define NAV_DEFAULT_PID_SPEED       (100)       // PID默认速度

// 电机速度环PID参数 (PIDS - 速度控制)
#define MOTOR_SPEED_PID_KP          (30.0f)     // 速度环比例系数
#define MOTOR_SPEED_PID_KI          (2.4f)      // 速度环积分系数
#define MOTOR_SPEED_PID_KD          (0.0f)      // 速度环微分系数

// 转向控制PID参数 (PIDD - 方向控制)
#define DIRECTION_PID_KP            (12.9f)     // 方向控制比例系数
#define DIRECTION_PID_KI            (0.0f)      // 方向控制积分系数
#define DIRECTION_PID_KD            (13.6f)     // 方向控制微分系数

// 直线行驶PID参数 (PID1 - 直线方向)
#define STRAIGHT_LINE_PID_KP        (10.0f)     // 直线方向比例系数
#define STRAIGHT_LINE_PID_KI        (0.0f)      // 直线方向积分系数
#define STRAIGHT_LINE_PID_KD        (10.0f)     // 直线方向微分系数

// 左轮速度PID参数 (PIDSL - 左轮速度，继承PIDS)
#define LEFT_WHEEL_SPEED_PID_KP     (30.0f)     // 左轮速度比例系数 (继承PIDS)
#define LEFT_WHEEL_SPEED_PID_KI     (2.4f)      // 左轮速度积分系数 (继承PIDS)
#define LEFT_WHEEL_SPEED_PID_KD     (0.0f)      // 左轮速度微分系数 (继承PIDS)

//=================================================系统运行配置================================================
#define NAV_DEFAULT_TARGET_SPEED    (0)         // 默认目标速度
#define NAV_FUYA_DEFAULT            (0)         // 辅助功能默认状态

//=================================================Flash存储配置================================================
#define NAV_FLASH_MAX_SIZE          (500)       // Flash存储的最大页面
#define NAV_COORD_RECORD            (25000)     // 坐标记录数量
#define NAV_FLASH_END_PAGE          (1)         // Flash中止页面
#define NAV_FLASH_START_PAGE        (40)        // Flash起始页面
#define NAV_SET_MILEAGE             (400)       // 里程脉冲当量：400脉冲/厘米

//=================================================数学常数配置================================================
#ifndef M_PI
#define M_PI                        (3.1415926535897932384626433832795f)
#endif
#define NAV_DEG_TO_RAD(angle)       ((angle) * M_PI / 180.0f)
#define NAV_RAD_TO_DEG(angle)       ((angle) * 180.0f / M_PI)

//=================================================UI界面配置================================================
#define NAV_UI_REFRESH_RATE         (50)        // UI刷新率 (ms)
#define NAV_KEY_DEBOUNCE_TIME       (20)        // 按键防抖时间 (ms)

//=================================================数据结构定义================================================
// 导航系统配置结构体
typedef struct
{
    // 路径点坐标
    float path_points_x[6];                     // X坐标数组 [X1-X6]
    float path_points_y[6];                     // Y坐标数组 [Y1-Y6]
    
    // 控制参数
    int control_radius[2];                      // 控制半径 [R1, R2]
    
    // 导航PID参数
    float nav_pid_kp;                           // 导航PID比例系数
    float nav_pid_ki;                           // 导航PID积分系数
    float nav_pid_kd;                           // 导航PID微分系数
    int nav_pid_speed;                          // 导航PID速度
    
    // 电机控制PID参数
    struct {
        // 左电机速度环PID
        float left_speed_kp;                    // 左电机速度环比例系数
        float left_speed_ki;                    // 左电机速度环积分系数
        float left_speed_kd;                    // 左电机速度环微分系数
        
        // 左电机位置环PID
        float left_position_kp;                 // 左电机位置环比例系数
        float left_position_ki;                 // 左电机位置环积分系数
        float left_position_kd;                 // 左电机位置环微分系数
        
        // 右电机速度环PID
        float right_speed_kp;                   // 右电机速度环比例系数
        float right_speed_ki;                   // 右电机速度环积分系数
        float right_speed_kd;                   // 右电机速度环微分系数
        
        // 右电机位置环PID
        float right_position_kp;                // 右电机位置环比例系数
        float right_position_ki;                // 右电机位置环积分系数
        float right_position_kd;                // 右电机位置环微分系数
        
        // 电机控制参数
        float max_speed;                        // 最大速度限制 (m/s)
        float max_acceleration;                 // 最大加速度限制 (m/s?)
        float wheelbase;                        // 轮距 (mm)
        int control_frequency;                  // 控制频率 (Hz)
        
    } motor_control;
    
    // 系统运行参数
    int target_speed;                           // 目标速度
    uint8 fuya_enable;                          // 辅助功能使能
    uint8 go_flag;                              // 发车标志
    uint8 go_state;                             // 运行状态
    
    // 系统状态
    uint8 config_loaded;                        // 配置已加载标志
    uint8 config_modified;                      // 配置已修改标志
    
} nav_config_struct;

// 导航系统数据结构体
typedef struct
{
    // 编码器数据
    int encoder_sum;                            // 编码器总和
    int encoder_left;                           // 左编码器
    
    // 路径数据
    int max_error_point_count;                  // 最大错误点数量
    int actual_error_point_count;               // 实际错误点数量
    int current_error_point_index;              // 当前错误点索引
    
    // 里程数据
    int total_mileage;                          // 总里程
    int last_mileage;                           // 上次里程
    
    // 角度和方向数据
    float last_yaw_angle;                       // 上次偏航角
    float current_yaw_angle;                    // 当前偏航角
    float error_direction;                      // 错误方向
    int error_angle_direction;                  // 错误角度方向
    
    // 曲率数据
    float curvature;                            // 当前曲率
    float curvature_straight;                   // 直线曲率
    
    // 状态标志
    uint8 error_make_flag;                      // 错误生成标志
    uint8 curvature_threshold_flag;             // 曲率阈值标志
    uint8 straight_angle_flag;                  // 直角标志
    uint8 last_straight_flag;                   // 上次直角标志
    
    // 状态机数据
    int positive_reset_state;                   // 正曲率清零状态机
    int negative_reset_state;                   // 负曲率清零状态机
    int continuous_straight_flag;               // 连续直角标志
    
    // 阈值状态
    bool was_high_threshold;                    // 是否曾经高于阈值
    bool was_low_threshold;                     // 是否曾经低于阈值
    bool high_to_mid_reset;                     // 高到中的重置标志
    bool mid_to_low_cancel;                     // 中到低的取消标志
    bool low_to_mid_reset;                      // 低到中的重置标志
    bool mid_to_high_cancel;                    // 中到高的取消标志
    
    // 时间计数
    uint8 time_counter_1;                       // 时间计数器1
    uint8 time_counter_2;                       // 时间计数器2
    
    // 计算缓冲
    int calculation_buffer;                     // 计算缓冲
    
} nav_data_struct;

//=================================================全局变量声明================================================
extern nav_config_struct nav_config;           // 导航系统配置
extern nav_data_struct nav_data;               // 导航系统数据

// 共享数据数组
extern float nav_errors_coords[NAV_COORD_RECORD];       // 错误坐标数组
extern int nav_mileage_list[NAV_COORD_RECORD];          // 里程列表数组
extern int32 nav_yaw_buffer[4000];                     // 偏航角缓冲数组

//=================================================函数声明================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导航配置系统初始化
// 参数说明     void
// 返回参数     uint8                          初始化结果 (0=成功, 1=失败)
// 使用示例     nav_config_init();
// 备注信息     加载默认配置参数，初始化配置系统
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_config_init(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     加载默认配置
// 参数说明     void
// 返回参数     void
// 使用示例     nav_config_load_default();
// 备注信息     恢复所有参数到默认值
//-------------------------------------------------------------------------------------------------------------------
void nav_config_load_default(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     保存当前配置到Flash
// 参数说明     void
// 返回参数     uint8                          保存结果 (0=成功, 1=失败)
// 使用示例     nav_config_save();
// 备注信息     将当前配置参数保存到Flash存储
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_config_save(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     从Flash加载配置
// 参数说明     void
// 返回参数     uint8                          加载结果 (0=成功, 1=失败)
// 使用示例     nav_config_load();
// 备注信息     从Flash存储加载配置参数
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_config_load(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取路径点坐标
// 参数说明     point_index                    路径点索引 (0-5 对应 X1-X6)
// 参数说明     x                              X坐标指针
// 参数说明     y                              Y坐标指针
// 返回参数     uint8                          获取结果 (0=成功, 1=失败)
// 使用示例     nav_config_get_path_point(0, &x1, &y1);
// 备注信息     获取指定路径点的坐标值
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_config_get_path_point(uint8 point_index, float *x, float *y);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置路径点坐标
// 参数说明     point_index                    路径点索引 (0-5 对应 X1-X6)
// 参数说明     x                              X坐标值
// 参数说明     y                              Y坐标值
// 返回参数     uint8                          设置结果 (0=成功, 1=失败)
// 使用示例     nav_config_set_path_point(0, 10.5f, 20.3f);
// 备注信息     设置指定路径点的坐标值
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_config_set_path_point(uint8 point_index, float x, float y);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取控制半径参数
// 参数说明     radius_index                   半径索引 (0=R1, 1=R2)
// 返回参数     int                            半径值
// 使用示例     r1 = nav_config_get_control_radius(0);
// 备注信息     获取指定控制半径参数
//-------------------------------------------------------------------------------------------------------------------
int nav_config_get_control_radius(uint8 radius_index);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置控制半径参数
// 参数说明     radius_index                   半径索引 (0=R1, 1=R2)
// 参数说明     radius                         半径值
// 返回参数     uint8                          设置结果 (0=成功, 1=失败)
// 使用示例     nav_config_set_control_radius(0, 35);
// 备注信息     设置指定控制半径参数
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_config_set_control_radius(uint8 radius_index, int radius);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取PID参数
// 参数说明     kp                             比例系数指针
// 参数说明     ki                             积分系数指针
// 参数说明     kd                             微分系数指针
// 参数说明     speed                          速度指针
// 返回参数     void
// 使用示例     nav_config_get_pid(&kp, &ki, &kd, &speed);
// 备注信息     获取PID控制器参数
//-------------------------------------------------------------------------------------------------------------------
void nav_config_get_pid(float *kp, float *ki, float *kd, int *speed);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置PID参数
// 参数说明     kp                             比例系数
// 参数说明     ki                             积分系数
// 参数说明     kd                             微分系数
// 参数说明     speed                          速度
// 返回参数     void
// 使用示例     nav_config_set_pid(1.2f, 0.1f, 0.05f, 120);
// 备注信息     设置PID控制器参数
//-------------------------------------------------------------------------------------------------------------------
void nav_config_set_pid(float kp, float ki, float kd, int speed);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取电机控制PID参数
// 参数说明     motor_id                       电机ID (0=左电机, 1=右电机)
// 参数说明     pid_type                       PID类型 (0=速度环, 1=位置环)
// 参数说明     kp                             比例系数指针
// 参数说明     ki                             积分系数指针
// 参数说明     kd                             微分系数指针
// 返回参数     uint8                          获取结果 (0=成功, 1=失败)
// 使用示例     nav_config_get_motor_pid(0, 0, &kp, &ki, &kd);
// 备注信息     获取指定电机的PID控制器参数
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_config_get_motor_pid(uint8 motor_id, uint8 pid_type, float *kp, float *ki, float *kd);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置电机控制PID参数
// 参数说明     motor_id                       电机ID (0=左电机, 1=右电机)
// 参数说明     pid_type                       PID类型 (0=速度环, 1=位置环)
// 参数说明     kp                             比例系数
// 参数说明     ki                             积分系数
// 参数说明     kd                             微分系数
// 返回参数     uint8                          设置结果 (0=成功, 1=失败)
// 使用示例     nav_config_set_motor_pid(0, 0, 8.0f, 0.5f, 0.1f);
// 备注信息     设置指定电机的PID控制器参数
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_config_set_motor_pid(uint8 motor_id, uint8 pid_type, float kp, float ki, float kd);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取电机控制参数
// 参数说明     max_speed                      最大速度指针 (m/s)
// 参数说明     max_acceleration               最大加速度指针 (m/s?)
// 参数说明     wheelbase                      轮距指针 (mm)
// 参数说明     control_frequency              控制频率指针 (Hz)
// 返回参数     void
// 使用示例     nav_config_get_motor_params(&max_speed, &max_acc, &wheelbase, &freq);
// 备注信息     获取电机控制系统参数
//-------------------------------------------------------------------------------------------------------------------
void nav_config_get_motor_params(float *max_speed, float *max_acceleration, float *wheelbase, int *control_frequency);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置电机控制参数
// 参数说明     max_speed                      最大速度 (m/s)
// 参数说明     max_acceleration               最大加速度 (m/s?)
// 参数说明     wheelbase                      轮距 (mm)
// 参数说明     control_frequency              控制频率 (Hz)
// 返回参数     void
// 使用示例     nav_config_set_motor_params(2.0f, 5.0f, 150.0f, 1000);
// 备注信息     设置电机控制系统参数
//-------------------------------------------------------------------------------------------------------------------
void nav_config_set_motor_params(float max_speed, float max_acceleration, float wheelbase, int control_frequency);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导航数据系统初始化
// 参数说明     void
// 返回参数     uint8                          初始化结果 (0=成功, 1=失败)
// 使用示例     nav_data_init();
// 备注信息     初始化导航数据结构体和数组
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_data_init(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取导航数据指针
// 参数说明     void
// 返回参数     nav_data_struct*               导航数据结构体指针
// 使用示例     nav_data_struct *data = nav_data_get_pointer();
// 备注信息     获取导航数据结构体的指针，用于直接访问
//-------------------------------------------------------------------------------------------------------------------
nav_data_struct* nav_data_get_pointer(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     重置导航数据
// 参数说明     void
// 返回参数     void
// 使用示例     nav_data_reset();
// 备注信息     将所有导航数据重置为初始值
//-------------------------------------------------------------------------------------------------------------------
void nav_data_reset(void);

// 便捷访问宏定义（向后兼容）

// 配置参数访问宏
#define X1  (nav_config.path_points_x[0])
#define Y1  (nav_config.path_points_y[0])
#define X2  (nav_config.path_points_x[1])
#define Y2  (nav_config.path_points_y[1])
#define X3  (nav_config.path_points_x[2])
#define Y3  (nav_config.path_points_y[2])
#define X4  (nav_config.path_points_x[3])
#define Y4  (nav_config.path_points_y[3])
#define X5  (nav_config.path_points_x[4])
#define Y5  (nav_config.path_points_y[4])
#define X6  (nav_config.path_points_x[5])
#define Y6  (nav_config.path_points_y[5])

#define R1  (nav_config.control_radius[0])
#define R2  (nav_config.control_radius[1])

// 导航PID控制参数（兼容性宏）
#define PIDG                            (nav_config.nav_pid_kp)     // PID比例系数（兼容性宏）
#define pid_kp                          (nav_config.nav_pid_kp)
#define pid_ki                          (nav_config.nav_pid_ki)
#define pid_kd                          (nav_config.nav_pid_kd)
#define pid_speed                       (nav_config.nav_pid_speed)

// 电机控制PID参数访问宏
#define motor_left_speed_kp             (nav_config.motor_control.left_speed_kp)
#define motor_left_speed_ki             (nav_config.motor_control.left_speed_ki)
#define motor_left_speed_kd             (nav_config.motor_control.left_speed_kd)
#define motor_left_position_kp          (nav_config.motor_control.left_position_kp)
#define motor_left_position_ki          (nav_config.motor_control.left_position_ki)
#define motor_left_position_kd          (nav_config.motor_control.left_position_kd)

#define motor_right_speed_kp            (nav_config.motor_control.right_speed_kp)
#define motor_right_speed_ki            (nav_config.motor_control.right_speed_ki)
#define motor_right_speed_kd            (nav_config.motor_control.right_speed_kd)
#define motor_right_position_kp         (nav_config.motor_control.right_position_kp)
#define motor_right_position_ki         (nav_config.motor_control.right_position_ki)
#define motor_right_position_kd         (nav_config.motor_control.right_position_kd)

#define motor_max_speed                 (nav_config.motor_control.max_speed)
#define motor_max_acceleration          (nav_config.motor_control.max_acceleration)
#define motor_wheelbase                 (nav_config.motor_control.wheelbase)
#define motor_control_frequency         (nav_config.motor_control.control_frequency)

#define finaltarget_speed   (nav_config.target_speed)
#define fuya                (nav_config.fuya_enable)
#define go_index            (nav_config.go_flag)
#define go_go_go            (nav_config.go_state)

// 数据变量访问宏（这名字跟取着玩一样）
#define encoder_sum_nav                 (nav_data.encoder_sum)
#define encoder_left_nav                (nav_data.encoder_left)
#define max_error_point_mem             (nav_data.max_error_point_count)
#define actual_error_point              (nav_data.actual_error_point_count)
#define point_error_index               (nav_data.current_error_point_index)
#define Mileage_All_sum                 (nav_data.total_mileage)
#define Mileage_All_sum_last            (nav_data.last_mileage)
#define Last_Nag_yaw                    (nav_data.last_yaw_angle)
#define rt_yaw                          (nav_data.current_yaw_angle)
#define error_dir                       (nav_data.error_direction)
#define error_angle_dir                 (nav_data.error_angle_direction)
#define qulv                            (nav_data.curvature)
#define qulv_zhijiao                    (nav_data.curvature_straight)
#define error_make_flag                 (nav_data.error_make_flag)
#define curvature_threshold_counter  (nav_data.curvature_threshold_flag)
#define opopop                          (nav_data.straight_angle_flag)
#define lastopopop                      (nav_data.last_straight_flag)
#define zheng_reset_state               (nav_data.positive_reset_state)
#define fu_reset_state                  (nav_data.negative_reset_state)
#define lianxuzhijiao                   (nav_data.continuous_straight_flag)
#define was_high                        (nav_data.was_high_threshold)
#define was_low                         (nav_data.was_low_threshold)
#define high_to_mid_reset               (nav_data.high_to_mid_reset)
#define mid_to_low_cancel               (nav_data.mid_to_low_cancel)
#define low_to_mid_reset                (nav_data.low_to_mid_reset)
#define mid_to_high_cancel              (nav_data.mid_to_high_cancel)
#define zhetime                         (nav_data.time_counter_1)
#define zhitime                         (nav_data.time_counter_2)
#define cnmb                            (nav_data.calculation_buffer)

// 数组访问宏
#define errors_coords                   nav_errors_coords
#define Mileage_All_sum_list            nav_mileage_list
#define data_yaw_buffer                 nav_yaw_buffer

// 不规范变量名映射（认真的吗）
// #define calc_buffer_aa                  (nav_data.calculation_buffer)
// #define const_bx                        6
// #define yugvbjvutyjvbihihib             (nav_data.curvature_threshold_flag)
// #define vbhjnmkl                        (nav_data.straight_angle_flag)
// #define hgbnm                           (nav_data.last_straight_flag)
// #define ghui                            (nav_data.positive_reset_state)
// #define tyu                             (nav_data.negative_reset_state)
// #define vbn                             (nav_data.continuous_straight_flag)
// #define iiiop                           (nav_data.time_counter_1)
// #define opopop_zhijiao                  (nav_data.curvature_straight)
// #define tyu_zhijiao                     (nav_data.time_counter_2)
// #define vbn_zhijiao                     (nav_data.was_high_threshold)
// #define ghui_zhijiao                    (nav_data.was_low_threshold)
// #define qulvzhe                         (nav_data.curvature)
// #define Qulv                            (nav_data.curvature)

//================================================= 全局访问接口 =================================================
nav_config_struct* nav_config_get(void);
nav_data_struct* nav_data_get(void);


#endif // _CONFIG_NAVIGATION_H_
