/*********************************************************************************************************************
* �ļ�����          nav_control.h
* ����˵��          ��������ϵͳͷ�ļ�
* ����              LittleMaster
* �汾��Ϣ          v1.0
* �޸ļ�¼
* ����              ����                ��ע
* 2025-09-19        LittleMaster        ������������ϵͳ��֧�ָ��ٳ����뾫ȷ����
*
* �ļ�����˵����
* ���ļ������������ĵ�������ϵͳ��������
* 1. �ഫ�����ںϣ������� + SCH16TK10 IMU��
* 2. Pure Pursuit·�������㷨
* 3. ��̬�ٶȿ��ƺ͹滮
* 4. �����뾫�ȱ�֤����
* 5. ��ȫ��غ��ݴ���
*
********************************************************************************************************************/

#ifndef _NAV_CONTROL_H_
#define _NAV_CONTROL_H_

#include "zf_common_typedef.h"
#include "driver_encoder.h"
#include "driver_sch16tk10.h"
#include "driver_flash_road.h"
#include "motor_control.h"
#include <math.h>

//=================================================���ò�������================================================
// ����ϵͳ��������
#define NAV_MAX_SPEED               3.8f        // ����ٶ� m/s
#define NAV_MIN_SPEED               3.5f        // ��С�ٶ� m/s
#define NAV_DEFAULT_SPEED           2.0f        // Ĭ���ٶ� m/s

// Pure Pursuit�㷨����
#define NAV_MIN_LOOKAHEAD           0.5f        // ��Сǰհ���� m
#define NAV_MAX_LOOKAHEAD           2.0f        // ���ǰհ���� m
#define NAV_LOOKAHEAD_FACTOR        0.6f        // �ٶ�-ǰհϵ��
#define NAV_WHEELBASE               0.26f       // �־� m

// ���ȿ��Ʋ���
#define NAV_WAYPOINT_TOLERANCE      0.15f       // ·�����ݲ� m
#define NAV_HEADING_TOLERANCE       0.087f      // �����ݲ� rad (5��)
#define NAV_POSITION_TOLERANCE      0.1f        // λ���ݲ� m
#define NAV_PATH_DEVIATION_MAX      0.5f        // ���·��ƫ����� m

// �ٶȿ��Ʋ���
#define NAV_MAX_ACCELERATION        2.0f        // �����ٶ� 
#define NAV_MAX_DECELERATION        3.0f        // �����ٶ� 
#define NAV_MAX_JERK                5.0f        // ���Ӽ��ٶ� 

// �������ںϲ���
#define NAV_ENCODER_WEIGHT          0.7f        // ������Ȩ��
#define NAV_IMU_WEIGHT              0.3f        // IMUȨ��
#define NAV_FUSION_RATE             100         // �ں�Ƶ�� Hz
#define NAV_CORRECTION_RATE         10          // У��Ƶ�� Hz

// ϵͳ���Ʋ���
#define NAV_CONTROL_PERIOD_MS       10          // �������� ms (100Hz)
#define NAV_MAX_WAYPOINTS           50          // ���·��������
#define NAV_SAFETY_CHECK_INTERVAL   5           // ��ȫ����� (�������ڱ���)

//=================================================ö�����Ͷ���================================================
// ����ϵͳ״̬ö��
typedef enum {
    NAV_STATUS_IDLE             = 0,        // ����״̬
    NAV_STATUS_INITIALIZING     = 1,        // ��ʼ����
    NAV_STATUS_PATH_LOADING     = 2,        // ·��������
    NAV_STATUS_NAVIGATING       = 3,        // ������
    NAV_STATUS_PAUSED           = 4,        // ��ͣ
    NAV_STATUS_COMPLETED        = 5,        // ���
    NAV_STATUS_ERROR            = 6,        // ����״̬
    NAV_STATUS_EMERGENCY_STOP   = 7         // ����ͣ��
} nav_status_enum;

// ����ģʽö��
typedef enum {
    NAV_MODE_PURE_PURSUIT       = 0,        // Pure Pursuitģʽ
    NAV_MODE_PRECISE            = 1,        // ��ȷģʽ
    NAV_MODE_HIGH_SPEED         = 2,        // ����ģʽ
    NAV_MODE_SAFE               = 3         // ��ȫģʽ
} nav_mode_enum;

// ������״̬ö��
typedef enum {
    NAV_SENSOR_OK               = 0,        // ����������
    NAV_SENSOR_WARNING          = 1,        // ����������
    NAV_SENSOR_ERROR            = 2,        // ����������
    NAV_SENSOR_OFFLINE          = 3         // ����������
} nav_sensor_status_enum;

//=================================================���ݽṹ����================================================
// 2Dλ�ýṹ
typedef struct {
    float x;                                // X���� m
    float y;                                // Y���� m
} nav_position_t;

// 2D�ٶȽṹ
typedef struct {
    float vx;                               // X�����ٶ� m/s
    float vy;                               // Y�����ٶ� m/s
    float linear;                           // ���ٶ� m/s
    float angular;                          // ���ٶ� rad/s
} nav_velocity_t;

// ��������̬�ṹ
typedef struct {
    nav_position_t position;                // λ��
    float heading;                          // ����� rad
    nav_velocity_t velocity;                // �ٶ�
    uint32 timestamp_ms;                    // ʱ��� ms
} nav_pose_t;

// ·����ṹ
typedef struct {
    nav_position_t position;                // λ��
    float target_speed;                     // Ŀ���ٶ� m/s
    float tolerance;                        // �����ݲ� m
    uint8 point_type;                       // ·��������
} nav_waypoint_t;

// ·���ṹ
typedef struct {
    nav_waypoint_t waypoints[NAV_MAX_WAYPOINTS];  // ·��������
    uint16 waypoint_count;                  // ·��������
    uint16 current_waypoint;                // ��ǰĿ��·��������
    float total_length;                     // ·���ܳ��� m
    char path_name[32];                     // ·������
} nav_path_t;

// �������ں�״̬�ṹ
typedef struct {
    // ����������
    nav_pose_t encoder_pose;                // ������λ��
    nav_sensor_status_enum encoder_status;  // ������״̬
    
    // IMU����
    float imu_angular_velocity[3];          // IMU���ٶ� [x,y,z] rad/s
    float imu_acceleration[3];              // IMU���ٶ� [x,y,z] m/s?
    float imu_temperature;                  // IMU�¶� ��C
    nav_sensor_status_enum imu_status;      // IMU״̬
    
    // �ںϽ��
    nav_pose_t fused_pose;                  // �ںϺ�λ��
    float confidence;                       // �ں����Ŷ� 0-1
    uint32 last_update_ms;                  // ������ʱ��
} nav_sensor_fusion_t;

// Pure Pursuit���ƽṹ
typedef struct {
    nav_position_t lookahead_point;         // ǰհ��
    float lookahead_distance;               // ǰհ���� m
    float curvature;                        // ���� 1/m
    float steering_angle;                   // ת��� rad
    float cross_track_error;                // ����ƫ�� m
    uint16 target_waypoint_index;           // Ŀ��·��������
} nav_pure_pursuit_t;

// �ٶȿ��ƽṹ
typedef struct {
    float target_speed;                     // Ŀ���ٶ� m/s
    float current_speed;                    // ��ǰ�ٶ� m/s
    float left_wheel_speed;                 // �����ٶ� m/s
    float right_wheel_speed;                // �����ٶ� m/s
    float acceleration;                     // ��ǰ���ٶ� m/s?
    float max_speed_for_curvature;          // �������Ƶ�����ٶ� m/s
} nav_speed_control_t;

// ��ȫ��ؽṹ
typedef struct {
    uint8 emergency_stop_triggered;         // ����ͣ����־
    float path_deviation;                   // ·��ƫ����� m
    uint32 sensor_timeout_count;            // ��������ʱ����
    uint32 control_loop_overrun_count;      // ����ѭ����ʱ����
    uint32 last_safety_check_ms;            // ���ȫ���ʱ��
    uint8 sensor_failure_flags;             // ���������ϱ�־λ
} nav_safety_monitor_t;

// ����ϵͳͳ����Ϣ
typedef struct {
    float total_distance_traveled;          // ����ʻ���� m
    float average_speed;                    // ƽ���ٶ� m/s
    float max_speed_reached;                // �ﵽ������ٶ� m/s
    uint32 navigation_start_time_ms;        // ������ʼʱ��
    uint32 total_navigation_time_ms;        // �ܵ���ʱ��
    uint16 waypoints_reached;               // �ѵ���·������
    float max_cross_track_error;            // ��������� m
    float average_cross_track_error;        // ƽ��������� m
} nav_statistics_t;

// ����ϵͳ���ṹ
typedef struct {
    // ϵͳ״̬
    nav_status_enum status;                 // ����״̬
    nav_mode_enum mode;                     // ����ģʽ
    uint8 initialized;                      // ��ʼ����־
    
    // ·����Ϣ
    nav_path_t current_path;                // ��ǰ·��
    
    // �������ں�
    nav_sensor_fusion_t sensor_fusion;      // �������ں�
    
    // �����㷨
    nav_pure_pursuit_t pure_pursuit;        // Pure Pursuit����
    nav_speed_control_t speed_control;      // �ٶȿ���
    
    // ��ȫ���
    nav_safety_monitor_t safety_monitor;    // ��ȫ���
    
    // ͳ����Ϣ
    nav_statistics_t statistics;            // ͳ����Ϣ
    
    // ʱ�����
    uint32 last_control_update_ms;          // �����Ƹ���ʱ��
    uint32 control_loop_count;              // ����ѭ������
    
} nav_system_t;

//=================================================ȫ�ֱ�������================================================
extern nav_system_t nav_system;

//=================================================��������================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ����ϵͳ��ʼ��
// ����˵��     void
// ���ز���     uint8               ��ʼ����� (1=�ɹ�, 0=ʧ��)
// ʹ��ʾ��     nav_init();
// ��ע��Ϣ     ��ʼ������ϵͳ�����ô������Ϳ��Ʋ���
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_init(void);

//-------------------------------------------------------------------------------------------------------------------
// �������     ���ص���·��
// ����˵��     path_name           ·������
// ���ز���     uint8               ���ؽ�� (1=�ɹ�, 0=ʧ��)
// ʹ��ʾ��     nav_load_path("��·��");
// ��ע��Ϣ     ��Flash����ָ��·��������ϵͳ
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_load_path(const char *path_name);

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ʼ����
// ����˵��     mode                ����ģʽ
// ���ز���     uint8               ������� (1=�ɹ�, 0=ʧ��)
// ʹ��ʾ��     nav_start(NAV_MODE_HIGH_SPEED);
// ��ע��Ϣ     ��������ϵͳ����ʼ·������
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_start(nav_mode_enum mode);

//-------------------------------------------------------------------------------------------------------------------
// �������     ֹͣ����
// ����˵��     emergency           �Ƿ����ͣ�� (1=��, 0=��)
// ���ز���     uint8               ֹͣ��� (1=�ɹ�, 0=ʧ��)
// ʹ��ʾ��     nav_stop(0);
// ��ע��Ϣ     ֹͣ����ϵͳ����ѡ�����ͣ��ģʽ
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_stop(uint8 emergency);

//-------------------------------------------------------------------------------------------------------------------
// �������     ����ϵͳ��ѭ������
// ����˵��     void
// ���ز���     uint8               ���½�� (1=�ɹ�, 0=ʧ��)
// ʹ��ʾ��     nav_update(); // ����ѭ������100HzƵ�ʵ���
// ��ע��Ϣ     ����ϵͳ�ĺ��ĸ��º�������Ҫ���ڵ���
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_update(void);

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ͣ/�ָ�����
// ����˵��     pause               ��ͣ��־ (1=��ͣ, 0=�ָ�)
// ���ز���     uint8               ������� (1=�ɹ�, 0=ʧ��)
// ʹ��ʾ��     nav_pause(1);
// ��ע��Ϣ     ��ͣ��ָ���������ͣʱ���ֵ�ǰλ��
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_pause(uint8 pause);

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ����ϵͳ״̬
// ����˵��     void
// ���ز���     nav_status_enum     ��ǰ����״̬
// ʹ��ʾ��     status = nav_get_status();
// ��ע��Ϣ     ��ȡ��ǰ����ϵͳ������״̬
//-------------------------------------------------------------------------------------------------------------------
nav_status_enum nav_get_status(void);

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ��ǰλ��
// ����˵��     position            λ�����ָ��
// ���ز���     uint8               ��ȡ��� (1=�ɹ�, 0=ʧ��)
// ʹ��ʾ��     nav_get_current_position(&pos);
// ��ע��Ϣ     ��ȡ��ǰС�����ں�λ����Ϣ
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_get_current_position(nav_position_t *position);

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ����ͳ����Ϣ
// ����˵��     stats               ͳ����Ϣ���ָ��
// ���ز���     uint8               ��ȡ��� (1=�ɹ�, 0=ʧ��)
// ʹ��ʾ��     nav_get_statistics(&stats);
// ��ע��Ϣ     ��ȡ�������̵�ͳ����Ϣ
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_get_statistics(nav_statistics_t *stats);

//-------------------------------------------------------------------------------------------------------------------
// �������     ���õ�������
// ����˵��     max_speed           ����ٶ� m/s
// ����˵��     lookahead_factor    ǰհ����ϵ��
// ���ز���     uint8               ���ý�� (1=�ɹ�, 0=ʧ��)
// ʹ��ʾ��     nav_set_parameters(3.5f, 0.6f);
// ��ע��Ϣ     ��̬��������ϵͳ����
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_set_parameters(float max_speed, float lookahead_factor);

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ·���������
// ����˵��     void
// ���ز���     float               ����ƫ�� m
// ʹ��ʾ��     error = nav_get_cross_track_error();
// ��ע��Ϣ     ��ȡ��ǰ��·������������
//-------------------------------------------------------------------------------------------------------------------
float nav_get_cross_track_error(void);

//-------------------------------------------------------------------------------------------------------------------
// �������     ����Ƿ񵽴�Ŀ��
// ����˵��     void
// ���ز���     uint8               ����״̬ (1=�ѵ���, 0=δ����)
// ʹ��ʾ��     arrived = nav_is_target_reached();
// ��ע��Ϣ     ����Ƿ��ѵ���·���յ�
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_is_target_reached(void);

#endif // _NAV_CONTROL_H_

