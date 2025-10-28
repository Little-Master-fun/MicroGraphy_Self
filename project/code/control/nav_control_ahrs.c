/*********************************************************************************************************************
* �ļ�����          nav_control_ahrs.c
* ����˵��          ���ں���Ǹ��ٵĵ�������ϵͳʵ���ļ�
* ����              AI Assistant
* �汾��Ϣ          v1.0
* �޸ļ�¼
* ����              ����                �汾
* 2025-01-XX        AI Assistant       1.0v
* 
* �ļ�����˵����
* ���ļ�ʵ������dog��Ŀ�ĺ���Ǹ��ٵ���ϵͳ
* ��ʹ������λ�ã���ͨ������·���ϵĺ������ʵ�ֵ���
********************************************************************************************************************/

#include "nav_control_ahrs.h"
#include "imu_ahrs_complementary.h"
#include "driver_encoder.h"
#include "motor_control.h"
#include "zf_common_headfile.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

//=================================================ȫ�ֱ�������================================================
nav_ahrs_controller_t nav_ahrs = {0};

//=================================================�ڲ���������================================================
static nav_ahrs_status_enum nav_ahrs_update_state(void);
static nav_ahrs_status_enum nav_ahrs_find_lookahead_point(void);
static float nav_ahrs_calculate_curvature(uint16 idx);
static nav_ahrs_status_enum nav_ahrs_angle_pid_control(void);
static float nav_ahrs_normalize_angle(float angle);
static float nav_ahrs_angle_difference(float target, float current);

//=================================================�ⲿ�ӿ�ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ʼ��AHRS����ϵͳ
//-------------------------------------------------------------------------------------------------------------------
nav_ahrs_status_enum nav_ahrs_init(void)
{
    // ����ϵͳ�ṹ��
    memset(&nav_ahrs, 0, sizeof(nav_ahrs_controller_t));
    
    // ��ʼ��ת��PID����������������Ӧ��
    nav_ahrs.pid_turn.kp = 15.0f;
    nav_ahrs.pid_turn.ki = 0.5f;
    nav_ahrs.pid_turn.kd = 5.0f;
    nav_ahrs.pid_turn.integral_limit = 50.0f;
    
    // ��ʼ��ֱ��PID������С���ȶ���ȷ��
    nav_ahrs.pid_straight.kp = 8.0f;
    nav_ahrs.pid_straight.ki = 0.2f;
    nav_ahrs.pid_straight.kd = 3.0f;
    nav_ahrs.pid_straight.integral_limit = 20.0f;
    
    // ��ʼ���ٶȲ���
    nav_ahrs.base_speed = 2.5f;  // �����ٶ� (m/s)
    
    // ��ʼ��ǰհ����
    nav_ahrs.lookahead_distance = NAV_AHRS_DISTANCE_PER_POINT * 5;  // Ĭ��ǰհ5���� (mm)
    
    // ϵͳ״̬
    nav_ahrs.mode = NAV_AHRS_MODE_IDLE;
    nav_ahrs.status = NAV_AHRS_STATUS_OK;
    nav_ahrs.initialized = 1;
    nav_ahrs.path_loaded = 0;
    
    printf("[NAV_AHRS] ����ϵͳ��ʼ�����\n");
    printf("[NAV_AHRS] ת��PID: Kp=%.1f, Ki=%.1f, Kd=%.1f\n", 
           nav_ahrs.pid_turn.kp, nav_ahrs.pid_turn.ki, nav_ahrs.pid_turn.kd);
    printf("[NAV_AHRS] ֱ��PID: Kp=%.1f, Ki=%.1f, Kd=%.1f\n", 
           nav_ahrs.pid_straight.kp, nav_ahrs.pid_straight.ki, nav_ahrs.pid_straight.kd);
    
    return NAV_AHRS_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����ϵͳ���£����ڵ��ã�
// ����˵��     dt              ʱ���� (s)
// ���ز���     ״̬
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
        // �ǻط�ģʽ��ͣ��
        nav_ahrs.left_speed = 0.0f;
        nav_ahrs.right_speed = 0.0f;
        return NAV_AHRS_STATUS_OK;
    }
    
    // 1. ���µ�ǰ״̬����̡�����ǣ�
    nav_ahrs_update_state();
    
    // 2. ����ǰհ��
    nav_ahrs_find_lookahead_point();
    
    // 3. ����·������
    nav_ahrs.curvature = nav_ahrs_calculate_curvature(nav_ahrs.lookahead_index);
    
    // 4. �Ƕ�PID����
    nav_ahrs_angle_pid_control();
    
    // 5. ���õ��Ŀ���ٶ�
    motor_set_target_speed(nav_ahrs.left_speed, nav_ahrs.right_speed);
    
    // 6. ����Ƿ����һȦ
    if (nav_ahrs.path.current_index >= nav_ahrs.path.total_points - 1) {
        if (nav_ahrs.path.loop_mode) {
            // ѭ��ģʽ�����õ����
            nav_ahrs.path.current_index = 0;
            nav_ahrs.current_distance = 0.0f;
            encoder_reset(ENCODER_ID_BOTH);
            printf("[NAV_AHRS] ���һȦ�����¿�ʼ\n");
        } else {
            // ��ѭ��ģʽ��ͣ��
            nav_ahrs.mode = NAV_AHRS_MODE_IDLE;
            nav_ahrs.left_speed = 0.0f;
            nav_ahrs.right_speed = 0.0f;
            motor_set_target_speed(0.0f, 0.0f);
            printf("[NAV_AHRS] ·�����\n");
            return NAV_AHRS_STATUS_PATH_END;
        }
    }
    
    return NAV_AHRS_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ�����������������ڵ�����ʾ��
//-------------------------------------------------------------------------------------------------------------------
nav_ahrs_status_enum nav_ahrs_get_motor_output(int32 *left_pwm, int32 *right_pwm)
{
    // ���ص�ǰ�ٶ�ֵ�����ڵ�����ʾ��
    if (left_pwm != NULL && right_pwm != NULL) {
        *left_pwm = (int32)(nav_ahrs.left_speed * 1000.0f);  // ת��Ϊ mm/s ��ʾ
        *right_pwm = (int32)(nav_ahrs.right_speed * 1000.0f);
    }
    
    return NAV_AHRS_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ���õ���ģʽ
//-------------------------------------------------------------------------------------------------------------------
nav_ahrs_status_enum nav_ahrs_set_mode(nav_ahrs_mode_enum mode)
{
    nav_ahrs.mode = mode;
    
    const char* mode_names[] = {"����", "�ط�", "��ͣ"};
    printf("[NAV_AHRS] ģʽ�л�Ϊ: %s\n", mode_names[mode]);
    
    if (mode == NAV_AHRS_MODE_REPLAY) {
        printf("[NAV_AHRS] ��ʼ�����ط�\n");
    }
    
    return NAV_AHRS_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ���û����ٶ�
//-------------------------------------------------------------------------------------------------------------------
nav_ahrs_status_enum nav_ahrs_set_speed(float speed)
{
    nav_ahrs.base_speed = speed;
    printf("[NAV_AHRS] �����ٶ�����Ϊ: %.1f\n", speed);
    return NAV_AHRS_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ���õ���ϵͳ
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
    
    // ���ñ�����
    encoder_reset(ENCODER_ID_BOTH);
    
    // ֹͣ���
    motor_set_target_speed(0.0f, 0.0f);
    
    printf("[NAV_AHRS] ����ϵͳ������\n");
    return NAV_AHRS_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ����״̬��Ϣ
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
// �������     ����PID����
//-------------------------------------------------------------------------------------------------------------------
nav_ahrs_status_enum nav_ahrs_set_pid(uint8 is_turn, float kp, float ki, float kd)
{
    if (is_turn) {
        nav_ahrs.pid_turn.kp = kp;
        nav_ahrs.pid_turn.ki = ki;
        nav_ahrs.pid_turn.kd = kd;
        printf("[NAV_AHRS] ת��PID: Kp=%.1f, Ki=%.1f, Kd=%.1f\n", kp, ki, kd);
    } else {
        nav_ahrs.pid_straight.kp = kp;
        nav_ahrs.pid_straight.ki = ki;
        nav_ahrs.pid_straight.kd = kd;
        printf("[NAV_AHRS] ֱ��PID: Kp=%.1f, Ki=%.1f, Kd=%.1f\n", kp, ki, kd);
    }
    
    return NAV_AHRS_STATUS_OK;
}

//=================================================�ڲ�����ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ���µ�ǰ״̬
//-------------------------------------------------------------------------------------------------------------------
static nav_ahrs_status_enum nav_ahrs_update_state(void)
{
    // 1. ���±���������
    encoder_update();
    
    // 2. ��ȡ�������ۻ����� (mm)
    nav_ahrs.left_distance = encoder_get_distance(ENCODER_ID_LEFT);
    nav_ahrs.right_distance = encoder_get_distance(ENCODER_ID_RIGHT);
    
    // 3. �����ۻ����루ȡƽ����
    nav_ahrs.current_distance = (nav_ahrs.left_distance + nav_ahrs.right_distance) / 2.0f;
    
    // 4. ��ȡ��ǰ�����
    ahrs_euler_angles_t euler;
    if (ahrs_get_euler_angles(&euler) == AHRS_STATUS_OK) {
        nav_ahrs.current_yaw = euler.yaw_accumulated;  // ʹ���ۻ������
    }
    
    // 5. ���µ�ǰ·��������
    for (uint16 i = nav_ahrs.path.current_index; i < nav_ahrs.path.total_points; i++) {
        if (nav_ahrs.path.points[i].distance > nav_ahrs.current_distance) {
            nav_ahrs.path.current_index = i;
            break;
        }
    }
    
    return NAV_AHRS_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����ǰհ��
//-------------------------------------------------------------------------------------------------------------------
static nav_ahrs_status_enum nav_ahrs_find_lookahead_point(void)
{
    // ��̬ǰհ���루�����ٶȺ����ʵ�����
    float speed_factor = nav_ahrs.base_speed / 2.5f;  // �ٶ����ӣ���׼�ٶ� 2.5 m/s��
    float curvature_factor = 1.0f / (1.0f + fabs(nav_ahrs.curvature) * 0.1f);  // ��������
    
    float lookahead_dist = nav_ahrs.lookahead_distance * speed_factor * curvature_factor;
    float target_distance = nav_ahrs.current_distance + lookahead_dist;
    
    // ��·�����в���ǰհ��
    nav_ahrs.lookahead_index = nav_ahrs.path.current_index;
    
    for (uint16 i = nav_ahrs.path.current_index; i < nav_ahrs.path.total_points; i++) {
        if (nav_ahrs.path.points[i].distance >= target_distance) {
            nav_ahrs.lookahead_index = i;
            break;
        }
    }
    
    // ��ֹ����Խ��
    if (nav_ahrs.lookahead_index >= nav_ahrs.path.total_points) {
        nav_ahrs.lookahead_index = nav_ahrs.path.total_points - 1;
    }
    
    // ��ȡĿ�꺽���
    nav_ahrs.target_yaw = nav_ahrs.path.points[nav_ahrs.lookahead_index].yaw;
    
    return NAV_AHRS_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����·������
// ����˵��     idx             ��ǰ������
// ���ز���     ����ֵ
//-------------------------------------------------------------------------------------------------------------------
static float nav_ahrs_calculate_curvature(uint16 idx)
{
    // ʹ��������������ʣ�����dog��Ŀ��
    if (idx < 2 || idx >= nav_ahrs.path.total_points - 2) {
        return 0.0f;
    }
    
    float theta1 = nav_ahrs.path.points[idx - 2].yaw;
    float theta2 = nav_ahrs.path.points[idx].yaw;
    float theta3 = nav_ahrs.path.points[idx + 2].yaw;
    
    float d12 = nav_ahrs.path.points[idx].distance - nav_ahrs.path.points[idx - 2].distance;
    float d23 = nav_ahrs.path.points[idx + 2].distance - nav_ahrs.path.points[idx].distance;
    
    if (d12 < 0.001f || d23 < 0.001f) {
        return 0.0f;
    }
    
    // ���㷽������
    float T1_x = cosf(theta1 * M_PI / 180.0f);
    float T1_y = sinf(theta1 * M_PI / 180.0f);
    float T3_x = cosf(theta3 * M_PI / 180.0f);
    float T3_y = sinf(theta3 * M_PI / 180.0f);
    
    // �����������
    float delta_Tx = T3_x - T1_x;
    float delta_Ty = T3_y - T1_y;
    float total_distance = (d12 + d23) / 2.0f;
    
    // ����
    float delta_T_norm = sqrtf(delta_Tx * delta_Tx + delta_Ty * delta_Ty);
    float angle_diff = theta3 - theta1;
    if (angle_diff != 0) {
        delta_T_norm *= (angle_diff / fabs(angle_diff));
    }
    
    float curvature = delta_T_norm / total_distance * 1000.0f;
    
    // �����޷�
    if (curvature > 65.0f) curvature = 65.0f;
    if (curvature < -65.0f) curvature = -65.0f;
    
    return curvature;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     �Ƕ�PID����
//-------------------------------------------------------------------------------------------------------------------
static nav_ahrs_status_enum nav_ahrs_angle_pid_control(void)
{
    // 1. ����Ƕ����
    nav_ahrs.angle_error = nav_ahrs_angle_difference(nav_ahrs.target_yaw, nav_ahrs.current_yaw);
    
    // 2. ��������Сѡ��PID������
    nav_ahrs_pid_t *pid;
    if (fabs(nav_ahrs.angle_error) > 2.0f) {
        // ����ʹ��ת��PID
        pid = &nav_ahrs.pid_turn;
    } else {
        // С��ʹ��ֱ��PID
        pid = &nav_ahrs.pid_straight;
    }
    
    // 3. PID����
    pid->error = nav_ahrs.angle_error;
    
    // ������
    pid->integral += pid->error;
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
    
    // PID���
    pid->output = pid->kp * pid->error + 
                  pid->ki * pid->integral +
                  pid->kd * (pid->error - pid->error_last);
    
    pid->error_last = pid->error;
    
    // 4. �����������ٶ�
    float direction_adjust = pid->output * 0.01f;  // PID������ŵ��ٶȵ����� (m/s)
    
    nav_ahrs.left_speed = nav_ahrs.base_speed - direction_adjust;
    nav_ahrs.right_speed = nav_ahrs.base_speed + direction_adjust;
    
    // �ٶ��޷� (m/s)
    float max_speed = 3.0f;  // ����ٶ� 3 m/s
    if (nav_ahrs.left_speed > max_speed) nav_ahrs.left_speed = max_speed;
    if (nav_ahrs.left_speed < -max_speed) nav_ahrs.left_speed = -max_speed;
    if (nav_ahrs.right_speed > max_speed) nav_ahrs.right_speed = max_speed;
    if (nav_ahrs.right_speed < -max_speed) nav_ahrs.right_speed = -max_speed;
    
    return NAV_AHRS_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     �Ƕȹ�һ����[-180, 180]
//-------------------------------------------------------------------------------------------------------------------
static float nav_ahrs_normalize_angle(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle <= -180.0f) angle += 360.0f;
    return angle;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����ǶȲ���ǿ�Խ��180�㣩
//-------------------------------------------------------------------------------------------------------------------
static float nav_ahrs_angle_difference(float target, float current)
{
    float diff = target - current;
    
    // �����Խ��180������
    if (diff > 180.0f) {
        diff -= 360.0f;
    } else if (diff < -180.0f) {
        diff += 360.0f;
    }
    
    return diff;
}

