/*********************************************************************************************************************
* �ļ�����          nav_control.c
* ����˵��          ��������ϵͳʵ���ļ�
* ����              LittleMaster
* �汾��Ϣ          v1.0
* �޸ļ�¼
* ����              ����                �汾
* 2025-09-22        LittleMaster       1.0v
* 
* �ļ�����˵����
* �õ�����ʽ�ѱ���������ʱ����������ǻ���ƫ���ǾͲ�ʹ�������ǣ�ֱ��ʹ�ñ�����������λ�ã�����ȴû�п��ǵ��򻬵����⣬һ���ͻ������������
* ���ļ�ʵ�ֵ�������ϵͳ�����й��ܣ�����·�����١��˶�ѧ����Ϳ������
* 
*
* ��Ҫ���ԣ�
* 1. Pure Pursuit·�������㷨
* 2. �û��ٶȿ��� + �㷨ת�����
* 3. ��������̼ƶ�λ
* 4. 1x1�������β���·��
* 5. ʵʱ״̬��غ͵�����Ϣ
********************************************************************************************************************/

#include "nav_control.h"
#include "driver_encoder.h"
#include "motor_control.h"
#include "driver_sch16tk10.h"
#include "zf_common_headfile.h"
#include "zf_driver_timer.h"
#include <stdio.h>
#include <string.h>

//=================================================ȫ�ֱ�������================================================
nav_system_t nav_system = {0};

//=================================================�ڲ���������================================================
static nav_status_enum nav_update_pose(void);
static nav_status_enum nav_pure_pursuit_control(void);
static nav_status_enum nav_calculate_wheel_speeds(float linear_vel, float angular_vel);
static float nav_calculate_distance(nav_point2d_t p1, nav_point2d_t p2);
static float nav_normalize_angle(float angle);
static nav_status_enum nav_find_lookahead_point(nav_point2d_t *lookahead_point);
static nav_status_enum nav_check_target_reached(void);
static float nav_adaptive_speed_calculation(float base_speed);

//=================================================�ⲿ�ӿ�ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ����ϵͳ��ʼ��
//-------------------------------------------------------------------------------------------------------------------
nav_status_enum nav_init(void)
{
    // ����ϵͳ�ṹ��
    memset(&nav_system, 0, sizeof(nav_system_t));
    
    // ��ʼ��������
    if (encoder_init() != ENCODER_STATUS_OK) {
        printf("���󣺱�������ʼ��ʧ��\n");
        nav_system.status = NAV_STATUS_ERROR;
        return NAV_STATUS_ERROR;
    }
    
    // ��ʼ��ϵͳʱ���������
    timer_init(TC_TIME2_CH2, TIMER_MS);
    timer_start(TC_TIME2_CH2);
    
    // ��ʼ�����PID����
    if (motor_pid_init() != MOTOR_PID_OK) {
        printf("���󣺵��PID���Ƴ�ʼ��ʧ��\n");
        nav_system.status = NAV_STATUS_ERROR;
        return NAV_STATUS_ERROR;
    }
    
    // ��ʼ��IMU������
    SCH1_filter filter = {50, 50, 50};        // 50Hz�˲�Ƶ��
    SCH1_sensitivity sens = {100, 100, 4, 4, 4}; // ����������
    SCH1_decimation dec = {1, 1};             // ��ȡ����
    
    if (SCH1_init(filter, sens, dec, false) != SCH1_OK) {
        printf("���棺IMU��������ʼ��ʧ�ܣ���ʹ�ñ�������λ\n");
        // IMUʧ�ܲ�Ӱ�쵼��ϵͳ���У�ֻ�Ƕ�λ���Ƚ���
    }
    
    // ��ʼ��ϵͳ����
    nav_system.speed_config.desired_speed = 0.5f;        // Ĭ��0.5m/s
    nav_system.speed_config.max_speed = 2.0f;            // ���2.0m/s
    nav_system.speed_config.acceleration_limit = 1.0f;   // 1.0m/s?���ٶ�����
    nav_system.speed_config.deceleration_limit = 2.0f;   // 2.0m/s?���ٶ�����
    nav_system.speed_config.adaptive_speed_enable = 1;   // ��������Ӧ�ٶ�
    
    nav_system.mode = NAV_MODE_MANUAL;
    nav_system.status = NAV_STATUS_OK;
    nav_system.system_enabled = 1;
    
    // ���ñ�����
    encoder_reset(ENCODER_ID_BOTH);
    
    printf("����ϵͳ��ʼ�����\n");
    printf("Ĭ���ٶ�: %.1f m/s\n", nav_system.speed_config.desired_speed);
    
    return NAV_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     �����û������ٶ�
//-------------------------------------------------------------------------------------------------------------------
nav_status_enum nav_set_desired_speed(float desired_speed)
{
    if (desired_speed < 0.0f || desired_speed > nav_system.speed_config.max_speed) {
        printf("���������ٶȳ�����Χ (0 ~ %.1f m/s)\n", nav_system.speed_config.max_speed);
        return NAV_STATUS_ERROR;
    }
    
    nav_system.speed_config.desired_speed = desired_speed;
    printf("�����ٶ�������Ϊ: %.1f m/s\n", desired_speed);
    
    return NAV_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����Ԥ����·��
//-------------------------------------------------------------------------------------------------------------------
nav_status_enum nav_load_preset_path(uint8 path_type)
{
    if (path_type == 0) {
        // 1x1��������·�������ܶ�·���㣩
        nav_system.path.total_points = 20;
        nav_system.path.current_target_index = 0;
        nav_system.path.loop_mode = 1;  // ѭ��ģʽ
        nav_system.path.reverse_mode = 0;
        
        // �����ܼ���������·���� (�����ԭ�㣬˳ʱ�뷽��)
        // ��һ���ߣ���(0,0)��(1,0) - ����
        nav_system.path.waypoints[0]  = (nav_waypoint_t){{0.0f, 0.0f}, 0.6f, 0}; // ���
        nav_system.path.waypoints[1]  = (nav_waypoint_t){{0.3f, 0.0f}, 0.8f, 0}; // ��һ�����е�1
        nav_system.path.waypoints[2]  = (nav_waypoint_t){{0.7f, 0.0f}, 0.8f, 0}; // ��һ�����е�2
        nav_system.path.waypoints[3]  = (nav_waypoint_t){{0.9f, 0.0f}, 0.5f, 2}; // ת��ǰ���ٵ�
        nav_system.path.waypoints[4]  = (nav_waypoint_t){{1.0f, 0.0f}, 0.4f, 2}; // ��һ��ת���
        
        // �ڶ����ߣ���(1,0)��(1,1) - ����  
        nav_system.path.waypoints[5]  = (nav_waypoint_t){{1.0f, 0.1f}, 0.5f, 0}; // ת�����ٵ�
        nav_system.path.waypoints[6]  = (nav_waypoint_t){{1.0f, 0.3f}, 0.8f, 0}; // �ڶ������е�1
        nav_system.path.waypoints[7]  = (nav_waypoint_t){{1.0f, 0.7f}, 0.8f, 0}; // �ڶ������е�2
        nav_system.path.waypoints[8]  = (nav_waypoint_t){{1.0f, 0.9f}, 0.5f, 2}; // ת��ǰ���ٵ�
        nav_system.path.waypoints[9]  = (nav_waypoint_t){{1.0f, 1.0f}, 0.4f, 2}; // �ڶ���ת���
        
        // �������ߣ���(1,1)��(0,1) - ����
        nav_system.path.waypoints[10] = (nav_waypoint_t){{0.9f, 1.0f}, 0.5f, 0}; // ת�����ٵ�
        nav_system.path.waypoints[11] = (nav_waypoint_t){{0.7f, 1.0f}, 0.8f, 0}; // ���������е�1
        nav_system.path.waypoints[12] = (nav_waypoint_t){{0.3f, 1.0f}, 0.8f, 0}; // ���������е�2
        nav_system.path.waypoints[13] = (nav_waypoint_t){{0.1f, 1.0f}, 0.5f, 2}; // ת��ǰ���ٵ�
        nav_system.path.waypoints[14] = (nav_waypoint_t){{0.0f, 1.0f}, 0.4f, 2}; // ������ת���
        
        // �������ߣ���(0,1)��(0,0) - ����
        nav_system.path.waypoints[15] = (nav_waypoint_t){{0.0f, 0.9f}, 0.5f, 0}; // ת�����ٵ�
        nav_system.path.waypoints[16] = (nav_waypoint_t){{0.0f, 0.7f}, 0.8f, 0}; // ���������е�1
        nav_system.path.waypoints[17] = (nav_waypoint_t){{0.0f, 0.3f}, 0.8f, 0}; // ���������е�2
        nav_system.path.waypoints[18] = (nav_waypoint_t){{0.0f, 0.1f}, 0.5f, 2}; // ת��ǰ���ٵ�
        nav_system.path.waypoints[19] = (nav_waypoint_t){{0.0f, 0.0f}, 0.3f, 1}; // �ص����(ͣ����)
        
        nav_system.status = NAV_STATUS_OK;
        
        printf("�Ѽ���1x1��������·������%d��·����\n", nav_system.path.total_points);
        for (int i = 0; i < nav_system.path.total_points; i++) {
            printf("  ��%d: (%.1f, %.1f) �ٶ�=%.1f\n", i, 
                   nav_system.path.waypoints[i].position.x,
                   nav_system.path.waypoints[i].position.y,
                   nav_system.path.waypoints[i].target_speed);
        }
        
        return NAV_STATUS_OK;
    }
    
    printf("����δ֪��·������ %d\n", path_type);
    return NAV_STATUS_ERROR;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����ϵͳ�����º���
//-------------------------------------------------------------------------------------------------------------------
nav_status_enum nav_update(void)
{
    if (!nav_system.system_enabled || nav_system.status == NAV_STATUS_ERROR) {
        return nav_system.status;
    }
    
    nav_system.update_count++;
    nav_system.last_update_time = timer_get(TC_TIME2_CH2);  // ��ȡ��ǰʱ���(ms)
    
    // 1. ���»�����λ�˹���
    nav_status_enum pose_status = nav_update_pose();
    if (pose_status != NAV_STATUS_OK) {
        return pose_status;
    }
    
    // 2. ����Ƿ���·��
    if (nav_system.path.total_points == 0) {
        nav_system.status = NAV_STATUS_NO_PATH;
        motor_set_target_speed(0.0f, 0.0f);  // ͣ��
        return NAV_STATUS_NO_PATH;
    }
    
    // 3. ���ݵ���ģʽִ�п���
    switch (nav_system.mode) {
        case NAV_MODE_AUTO:
            // ����Ƿ񵽴�Ŀ���
            nav_check_target_reached();
            
            // ִ��Pure Pursuit·������
            nav_pure_pursuit_control();
            break;
            
        case NAV_MODE_MANUAL:
            // �ֶ�ģʽ���û����ƣ�ͣ��
            motor_set_target_speed(0.0f, 0.0f);
            break;
            
        case NAV_MODE_PAUSE:
            // ��ͣģʽ�����ֵ�ǰλ��
            motor_set_target_speed(0.0f, 0.0f);
            break;
            
        case NAV_MODE_EMERGENCY:
            // ����ģʽ������ͣ��
            motor_set_target_speed(0.0f, 0.0f);
            nav_system.status = NAV_STATUS_ERROR;
            break;
    }
    
    return nav_system.status;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ���õ���ģʽ
//-------------------------------------------------------------------------------------------------------------------
nav_status_enum nav_set_mode(nav_mode_enum mode)
{
    nav_system.mode = mode;
    
    const char* mode_names[] = {"�ֶ�", "�Զ�", "��ͣ", "����"};
    printf("����ģʽ���л�Ϊ: %s\n", mode_names[mode]);
    
    // ģʽ�л�ʱ�ĳ�ʼ������
    if (mode == NAV_MODE_AUTO && nav_system.path.total_points > 0) {
        nav_system.status = NAV_STATUS_OK;
        printf("��ʼ�Զ�·������\n");
    }
    
    return NAV_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ����ϵͳ״̬
//-------------------------------------------------------------------------------------------------------------------
nav_status_enum nav_get_system_status(nav_system_t *system_info)
{
    if (system_info == NULL) {
        return NAV_STATUS_ERROR;
    }
    
    memcpy(system_info, &nav_system, sizeof(nav_system_t));
    return NAV_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ���õ���ϵͳ
//-------------------------------------------------------------------------------------------------------------------
nav_status_enum nav_reset(void)
{
    // ����λ�ù���
    nav_system.current_pose.x = 0.0f;
    nav_system.current_pose.y = 0.0f;
    nav_system.current_pose.theta = 0.0f;
    nav_system.current_pose.linear_velocity = 0.0f;
    nav_system.current_pose.angular_velocity = 0.0f;
    
    // ����·������
    nav_system.path.current_target_index = 0;
    
    // ���ñ�����
    encoder_reset(ENCODER_ID_BOTH);
    
    // ͣ��
    motor_set_target_speed(0.0f, 0.0f);
    
    nav_system.status = NAV_STATUS_OK;
    printf("����ϵͳ������\n");
    
    return NAV_STATUS_OK;
}

//=================================================�ڲ�����ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ����С��λ�˹���
//-------------------------------------------------------------------------------------------------------------------
static nav_status_enum nav_update_pose(void)
{
    static float last_left_distance = 0.0f;
    static float last_right_distance = 0.0f;
    
    // ���±���������
    encoder_update();
    
    // ��ȡ�������ٶȺ;���
    float left_speed = encoder_get_speed(ENCODER_ID_LEFT);
    float right_speed = encoder_get_speed(ENCODER_ID_RIGHT);
    float left_distance = encoder_get_distance(ENCODER_ID_LEFT) / 1000.0f;   // ת��Ϊ��
    float right_distance = encoder_get_distance(ENCODER_ID_RIGHT) / 1000.0f; // ת��Ϊ��
    
    // �����������
    float delta_left = left_distance - last_left_distance;
    float delta_right = right_distance - last_right_distance;
    last_left_distance = left_distance;
    last_right_distance = right_distance;
    
    // �����˶�ѧ���⣺����λ������
    float delta_s = (delta_left + delta_right) / 2.0f;           // ��λ��
    float delta_theta = (delta_right - delta_left) / NAV_WHEELBASE; // ��λ��
    
    // ����λ��
    nav_system.current_pose.theta += delta_theta;
    nav_system.current_pose.theta = nav_normalize_angle(nav_system.current_pose.theta);
    
    nav_system.current_pose.x += delta_s * cosf(nav_system.current_pose.theta);
    nav_system.current_pose.y += delta_s * sinf(nav_system.current_pose.theta);
    
    // �����ٶ�
    nav_system.current_pose.linear_velocity = (left_speed + right_speed) / 2.0f;
    nav_system.current_pose.angular_velocity = (right_speed - left_speed) / NAV_WHEELBASE;
    
    return NAV_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     Pure Pursuit·�����ٿ���
//-------------------------------------------------------------------------------------------------------------------
static nav_status_enum nav_pure_pursuit_control(void)
{
    nav_point2d_t lookahead_point;
    
    // ����ǰհ��
    if (nav_find_lookahead_point(&lookahead_point) != NAV_STATUS_OK) {
        return NAV_STATUS_ERROR;
    }
    
    // ���㵽ǰհ��ľ���ͽǶ�
    float dx = lookahead_point.x - nav_system.current_pose.x;
    float dy = lookahead_point.y - nav_system.current_pose.y;
    float target_angle = atan2f(dy, dx);
    
    // ����Ƕ����
    float angle_error = nav_normalize_angle(target_angle - nav_system.current_pose.theta);
    
    // Pure Pursuit�㷨��������
    float lookahead_distance = sqrtf(dx*dx + dy*dy);
    float curvature = 2.0f * sinf(angle_error) / lookahead_distance;
    
    // ����Ŀ���ٶȣ������û��趨 + ����Ӧ������
    float target_linear_speed = nav_adaptive_speed_calculation(nav_system.speed_config.desired_speed);
    
    // ������ٶ�
    float target_angular_speed = curvature * target_linear_speed;
    
    // ���ٶ�����
    if (target_angular_speed > NAV_MAX_ANGULAR_VEL) {
        target_angular_speed = NAV_MAX_ANGULAR_VEL;
    } else if (target_angular_speed < -NAV_MAX_ANGULAR_VEL) {
        target_angular_speed = -NAV_MAX_ANGULAR_VEL;
    }
    
    // ���¿������
    nav_system.output.linear_velocity = target_linear_speed;
    nav_system.output.angular_velocity = target_angular_speed;
    nav_system.output.cross_track_error = lookahead_distance * sinf(angle_error);
    nav_point2d_t current_pos = {nav_system.current_pose.x, nav_system.current_pose.y};
    nav_system.output.distance_to_target = nav_calculate_distance(
        current_pos, 
        nav_system.path.waypoints[nav_system.path.current_target_index].position
    );
    
    // �˶�ѧ��⣺�����������ٶ�
    nav_calculate_wheel_speeds(target_linear_speed, target_angular_speed);
    
    return NAV_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     �˶�ѧ��⣺�����������ٶ�
//-------------------------------------------------------------------------------------------------------------------
static nav_status_enum nav_calculate_wheel_speeds(float linear_vel, float angular_vel)
{
    // �����˶�ѧ���
    float left_speed = linear_vel - angular_vel * NAV_WHEELBASE / 2.0f;
    float right_speed = linear_vel + angular_vel * NAV_WHEELBASE / 2.0f;
    
    // ��������ṹ��
    nav_system.output.left_wheel_speed = left_speed;
    nav_system.output.right_wheel_speed = right_speed;
    
    // ���͵��������
    motor_set_target_speed(left_speed, right_speed);
    
    return NAV_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����ǰհ��
//-------------------------------------------------------------------------------------------------------------------
static nav_status_enum nav_find_lookahead_point(nav_point2d_t *lookahead_point)
{
    if (nav_system.path.current_target_index >= nav_system.path.total_points) {
        return NAV_STATUS_ERROR;
    }
    
    // �򻯰汾��ֱ��ʹ�õ�ǰĿ�����Ϊǰհ��
    nav_waypoint_t *current_target = &nav_system.path.waypoints[nav_system.path.current_target_index];
    lookahead_point->x = current_target->position.x;
    lookahead_point->y = current_target->position.y;
    
    return NAV_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ���Ŀ����Ƿ񵽴�
//-------------------------------------------------------------------------------------------------------------------
static nav_status_enum nav_check_target_reached(void)
{
    if (nav_system.path.current_target_index >= nav_system.path.total_points) {
        return NAV_STATUS_PATH_COMPLETE;
    }
    
    nav_waypoint_t *current_target = &nav_system.path.waypoints[nav_system.path.current_target_index];
    
    nav_point2d_t current_pos = {nav_system.current_pose.x, nav_system.current_pose.y};
    float distance = nav_calculate_distance(current_pos, current_target->position);
    
    if (distance < NAV_TARGET_TOLERANCE) {
        printf("����·���� %d: (%.2f, %.2f)\n", 
               nav_system.path.current_target_index,
               current_target->position.x, 
               current_target->position.y);
        
        nav_system.path.current_target_index++;
        
        // ����Ƿ����·��
        if (nav_system.path.current_target_index >= nav_system.path.total_points) {
            if (nav_system.path.loop_mode) {
                nav_system.path.current_target_index = 0;  // ѭ��ģʽ���¿�ʼ
                printf("·��ѭ�������¿�ʼ\n");
            } else {
                nav_system.status = NAV_STATUS_PATH_COMPLETE;
                motor_set_target_speed(0.0f, 0.0f);  // ͣ��
                printf("·��������ɣ�\n");
            }
        }
    }
    
    return NAV_STATUS_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����Ӧ�ٶȼ���
//-------------------------------------------------------------------------------------------------------------------
static float nav_adaptive_speed_calculation(float base_speed)
{
    if (!nav_system.speed_config.adaptive_speed_enable) {
        return base_speed;
    }
    
    float adaptive_speed = base_speed;
    
    // ���ݺ����������ٶ�
    float lateral_error = fabsf(nav_system.output.cross_track_error);
    if (lateral_error > 0.1f) {
        adaptive_speed *= 0.8f;  // ƫ��·��ʱ����
    }
    
    // ���ݽ��ٶȵ����ٶ�
    float angular_vel = fabsf(nav_system.output.angular_velocity);
    if (angular_vel > 1.0f) {
        adaptive_speed *= 0.7f;  // ת��ʱ����
    }
    
    // �ӽ�Ŀ���ʱ����
    if (nav_system.output.distance_to_target < 0.3f) {
        adaptive_speed *= 0.6f;
    }
    
    return adaptive_speed;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ������������
//-------------------------------------------------------------------------------------------------------------------
static float nav_calculate_distance(nav_point2d_t p1, nav_point2d_t p2)
{
    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    return sqrtf(dx*dx + dy*dy);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     �Ƕȹ�һ����(-��, ��]
//-------------------------------------------------------------------------------------------------------------------
static float nav_normalize_angle(float angle)
{
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle <= -M_PI) angle += 2.0f * M_PI;
    return angle;
}
