/*********************************************************************************************************************
* �ļ�����          nav_control_ahrs.h
* ����˵��          ���ں���Ǹ��ٵĵ�������ϵͳͷ�ļ�
* ����              AI Assistant
* �汾��Ϣ          v1.0
* �޸ļ�¼
* ����              ����                �汾
* 2025-01-XX        AI Assistant       1.0v
* 
* �ļ�����˵����
* ���ļ�ʵ������dog��Ŀ�ĺ���Ǹ��ٵ���ϵͳ
* ����˼·����¼�켣��ÿ����ĺ���ǣ��ط�ʱ����Ŀ�꺽���
* 
* ��Ҫ���ԣ�
* 1. Ԥ����������·����1m��1m��
* 2. ������̵�ǰհ�����
* 3. ��̬���ʼ���
* 4. ˫PID���Ʋ��ԣ�ֱ��/ת�䣩
********************************************************************************************************************/

#ifndef _NAV_CONTROL_AHRS_H_
#define _NAV_CONTROL_AHRS_H_

#include "zf_common_typedef.h"

// ����M_PI�ظ�����
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

//=================================================���ò���================================================
#define NAV_AHRS_MAX_POINTS         2000        // ���·������
#define NAV_AHRS_DISTANCE_PER_POINT 4.0f        // ÿ����ľ����� (mm)
#define NAV_AHRS_WHEELBASE          0.134f      // �־� (m)
#define NAV_AHRS_WHEEL_DIAMETER     0.065f      // �־� (m)

//=================================================ö�ٶ���================================================
// ����״̬ö��
typedef enum {
    NAV_AHRS_STATUS_OK = 0,
    NAV_AHRS_STATUS_ERROR,
    NAV_AHRS_STATUS_NOT_INIT,
    NAV_AHRS_STATUS_PATH_END
} nav_ahrs_status_enum;

// ����ģʽö��
typedef enum {
    NAV_AHRS_MODE_IDLE = 0,     // ����
    NAV_AHRS_MODE_REPLAY,       // �ط�ģʽ
    NAV_AHRS_MODE_PAUSE         // ��ͣ
} nav_ahrs_mode_enum;

//=================================================���ݽṹ����================================================
// ·����ṹ��
typedef struct {
    float yaw;              // �õ�ĺ���� (��)
    float distance;         // �õ���ۻ����� (mm)
} nav_ahrs_waypoint_t;

// ·�����ݽṹ
typedef struct {
    nav_ahrs_waypoint_t points[NAV_AHRS_MAX_POINTS];   // ·��������
    uint16 total_points;                                 // ��·������
    uint16 current_index;                                // ��ǰ���ٵ�����
    uint8 loop_mode;                                     // ѭ��ģʽ��־
} nav_ahrs_path_t;

// PID�������ṹ��
typedef struct {
    float kp;               // ����ϵ��
    float ki;               // ����ϵ��
    float kd;               // ΢��ϵ��
    float error;            // ��ǰ���
    float error_last;       // �ϴ����
    float integral;         // �����ۻ�
    float output;           // ���ֵ
    float integral_limit;   // �����޷�
} nav_ahrs_pid_t;

// �����������ṹ��
typedef struct {
    // ·������
    nav_ahrs_path_t path;
    
    // ��ǰ״̬
    float current_yaw;          // ��ǰ����� (��)
    float current_distance;     // ��ǰ�ۻ����� (mm)
    float left_distance;        // �����ۻ����� (mm)
    float right_distance;       // �����ۻ����� (mm)
    
    // Ŀ��״̬
    float target_yaw;           // Ŀ�꺽��� (��)
    float angle_error;          // �Ƕ���� (��)
    
    // ǰհ����
    uint16 lookahead_index;     // ǰհ������
    float lookahead_distance;   // ǰհ���� (mm)
    float curvature;            // ��ǰ·������
    
    // PID������
    nav_ahrs_pid_t pid_turn;    // ת��PID
    nav_ahrs_pid_t pid_straight;// ֱ��PID
    
    // �ٶȿ���
    float base_speed;           // �����ٶ� (m/s)
    float left_speed;           // ����Ŀ���ٶ� (m/s)
    float right_speed;          // ����Ŀ���ٶ� (m/s)
    
    // ϵͳ״̬
    nav_ahrs_mode_enum mode;
    nav_ahrs_status_enum status;
    uint8 initialized;
    uint8 path_loaded;
    
} nav_ahrs_controller_t;

//=================================================ȫ�ֱ�������================================================
extern nav_ahrs_controller_t nav_ahrs;

//=================================================��������================================================

/**
 * @brief  ��ʼ��AHRS����ϵͳ
 * @retval ״̬
 */
nav_ahrs_status_enum nav_ahrs_init(void);

/**
 * @brief  ����ϵͳ���£����ڵ��ã�
 * @param  dt       ʱ���� (s)
 * @retval ״̬
 * @note   ������5-10ms��ʱ���е���
 */
nav_ahrs_status_enum nav_ahrs_update(float dt);

/**
 * @brief  ��ȡ����������
 * @param  left_pwm     ����PWM���
 * @param  right_pwm    ����PWM���
 * @retval ״̬
 */
nav_ahrs_status_enum nav_ahrs_get_motor_output(int32 *left_pwm, int32 *right_pwm);

/**
 * @brief  ���õ���ģʽ
 * @param  mode     ����ģʽ
 * @retval ״̬
 */
nav_ahrs_status_enum nav_ahrs_set_mode(nav_ahrs_mode_enum mode);

/**
 * @brief  ���û����ٶ�
 * @param  speed    �ٶ� (m/s)
 * @retval ״̬
 */
nav_ahrs_status_enum nav_ahrs_set_speed(float speed);

/**
 * @brief  ���õ���ϵͳ
 * @retval ״̬
 */
nav_ahrs_status_enum nav_ahrs_reset(void);

/**
 * @brief  ��ȡ����״̬��Ϣ
 * @param  yaw          ��ǰ�����
 * @param  target_yaw   Ŀ�꺽���
 * @param  error        �Ƕ����
 * @param  curvature    ·������
 * @retval ״̬
 */
nav_ahrs_status_enum nav_ahrs_get_status(float *yaw, float *target_yaw, 
                                          float *error, float *curvature);

/**
 * @brief  ����PID����
 * @param  is_turn      1-ת��PID��0-ֱ��PID
 * @param  kp           ����ϵ��
 * @param  ki           ����ϵ��
 * @param  kd           ΢��ϵ��
 * @retval ״̬
 */
nav_ahrs_status_enum nav_ahrs_set_pid(uint8 is_turn, float kp, float ki, float kd);

#endif // _NAV_CONTROL_AHRS_H_

