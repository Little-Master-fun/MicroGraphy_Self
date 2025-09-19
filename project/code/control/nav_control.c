/*********************************************************************************************************************
* �ļ�����          nav_control.c
* ����˵��          ��������ϵͳʵ���ļ�
* ����              LittleMaster
* �汾��Ϣ          v1.0
* �޸ļ�¼
* ����              ����                ��ע
* 2025-09-19        LittleMaster        ������������ϵͳ��֧�ָ��ٳ����뾫ȷ����
*
* �ļ�����˵����
* ���ļ�ʵ���������ĵ�������ϵͳ��������
* 1. �ഫ�����ںϣ������� + SCH16TK10 IMU��
* 2. Pure Pursuit·�������㷨
* 3. ��̬�ٶȿ��ƺ͹滮
* 4. �����뾫�ȱ�֤����
* 5. ��ȫ��غ��ݴ���
*
********************************************************************************************************************/

#include "nav_control.h"
#include "zf_common_headfile.h"
#include "zf_driver_timer.h"  

//=================================================ȫ�ֱ�������================================================
nav_system_t nav_system = {                        // ����ϵͳ���ṹ
    .status = NAV_STATUS_IDLE,
    .mode = NAV_MODE_PURE_PURSUIT,
    .initialized = 0
};

//=================================================�ڲ���̬��������================================================
// ��ѧ���ߺ���
static float nav_distance_2d(const nav_position_t *p1, const nav_position_t *p2);
static float nav_angle_normalize(float angle);
// static float nav_angle_difference(float angle1, float angle2); 
static float nav_clamp(float value, float min_val, float max_val);

// �������ں���غ���
static uint8 nav_update_sensor_fusion(void);
static uint8 nav_read_encoder_data(void);
static uint8 nav_read_imu_data(void);
static void nav_fuse_sensor_data(void);
static uint8 nav_check_sensor_health(void);

// Pure Pursuit�㷨��غ���
static uint8 nav_update_pure_pursuit(void);
static nav_position_t nav_find_lookahead_point(void);
static float nav_calculate_curvature(const nav_position_t *lookahead_point);
// static uint16 nav_find_closest_waypoint(void); 
static float nav_calculate_cross_track_error(void);

// �ٶȿ�����غ���
static uint8 nav_update_speed_control(void);
static float nav_calculate_target_speed(void);
static void nav_calculate_wheel_speeds(float target_speed, float curvature);
static float nav_smooth_speed_transition(float target_speed, float current_speed);

// ��ȫ�����غ���
static uint8 nav_update_safety_monitor(void);
static uint8 nav_check_path_deviation(void);
static uint8 nav_check_sensor_timeout(void);
static void nav_trigger_emergency_stop(void);

// ͳ����Ϣ���º���
static void nav_update_statistics(void);

// ϵͳ������
static uint32 nav_get_system_time_ms(void);
static void nav_reset_system_state(void);

//=================================================��ѧ�͹��ߺ���ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ������������
//-------------------------------------------------------------------------------------------------------------------
static float nav_distance_2d(const nav_position_t *p1, const nav_position_t *p2)
{
    if (p1 == NULL || p2 == NULL) return 0.0f;
    
    float dx = p2->x - p1->x;
    float dy = p2->y - p1->y;
    return sqrtf(dx * dx + dy * dy);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     �Ƕȹ�һ����[-��, ��]
//-------------------------------------------------------------------------------------------------------------------
static float nav_angle_normalize(float angle)
{
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

// ��ʱע��δʹ�õĺ���
// static float nav_angle_difference(float angle1, float angle2)
// {
//     return nav_angle_normalize(angle1 - angle2);
// }

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ֵ�޷�
//-------------------------------------------------------------------------------------------------------------------
static float nav_clamp(float value, float min_val, float max_val)
{
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

//=================================================�������ںϹ���ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ���´������ں�����
//-------------------------------------------------------------------------------------------------------------------
static uint8 nav_update_sensor_fusion(void)
{
    uint8 result = 1;
    
    // ��ȡ����������
    if (!nav_read_encoder_data()) {
        nav_system.sensor_fusion.encoder_status = NAV_SENSOR_ERROR;
        result = 0;
    }
    
    // ��ȡIMU����
    if (!nav_read_imu_data()) {
        nav_system.sensor_fusion.imu_status = NAV_SENSOR_ERROR;
        result = 0;
    }
    
    // ��鴫��������״̬
    if (!nav_check_sensor_health()) {
        result = 0;
    }
    
    // ִ�д����������ں�
    if (result) {
        nav_fuse_sensor_data();
        nav_system.sensor_fusion.last_update_ms = nav_get_system_time_ms();
    }
    
    return result;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ����������
//-------------------------------------------------------------------------------------------------------------------
static uint8 nav_read_encoder_data(void)
{
    if (encoder_update() != ENCODER_STATUS_OK) {
        return 0;
    }
    
    nav_sensor_fusion_t *fusion = &nav_system.sensor_fusion;
    
    // ���±�����λ������
    fusion->encoder_pose.velocity.linear = encoder_system.linear_velocity;
    fusion->encoder_pose.velocity.angular = encoder_system.angular_velocity;
    fusion->encoder_pose.heading = encoder_system.heading_angle;
    
    // �򻯵�λ�û��� (ʵ��Ӧ������Ҫ����ȷ���˶�ѧģ��)
    static uint32 last_update_ms = 0;
    uint32 current_time_ms = nav_get_system_time_ms();
    
    if (last_update_ms != 0) {
        float dt = (current_time_ms - last_update_ms) / 1000.0f;
        
        // ����λ�� (��������˶�ѧ)
        float distance = fusion->encoder_pose.velocity.linear * dt;
        fusion->encoder_pose.position.x += distance * cosf(fusion->encoder_pose.heading);
        fusion->encoder_pose.position.y += distance * sinf(fusion->encoder_pose.heading);
    }
    
    last_update_ms = current_time_ms;
    fusion->encoder_pose.timestamp_ms = current_time_ms;
    fusion->encoder_status = NAV_SENSOR_OK;
    
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡIMU����
//-------------------------------------------------------------------------------------------------------------------
static uint8 nav_read_imu_data(void)
{
    static SCH1_raw_data imu_raw_data;
    static SCH1_result imu_result;
    
    // ��ȡIMUԭʼ����
    SCH1_getData(&imu_raw_data);
    
    // �������֡����
    if (imu_raw_data.frame_error) {
        return 0;
    }
    
    // ת��Ϊ����λ
    SCH1_convert_data(&imu_raw_data, &imu_result);
    
    nav_sensor_fusion_t *fusion = &nav_system.sensor_fusion;
    
    // �洢IMU����
    fusion->imu_angular_velocity[0] = imu_result.Rate1[AXIS_X];
    fusion->imu_angular_velocity[1] = imu_result.Rate1[AXIS_Y];
    fusion->imu_angular_velocity[2] = imu_result.Rate1[AXIS_Z];
    
    fusion->imu_acceleration[0] = imu_result.Acc1[AXIS_X];
    fusion->imu_acceleration[1] = imu_result.Acc1[AXIS_Y];
    fusion->imu_acceleration[2] = imu_result.Acc1[AXIS_Z];
    
    fusion->imu_temperature = imu_result.Temp;
    fusion->imu_status = NAV_SENSOR_OK;
    
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     �ںϴ���������
//-------------------------------------------------------------------------------------------------------------------
static void nav_fuse_sensor_data(void)
{
    nav_sensor_fusion_t *fusion = &nav_system.sensor_fusion;
    
    // �ں�λ����Ϣ (��Ҫʹ�ñ�����)
    fusion->fused_pose.position = fusion->encoder_pose.position;
    
    // �ںϽ��ٶ� (������ + IMU�����˲�)
    if (fusion->encoder_status == NAV_SENSOR_OK && fusion->imu_status == NAV_SENSOR_OK) {
        // �����˲��ںϽ��ٶ�
        float encoder_angular_vel = fusion->encoder_pose.velocity.angular;
        float imu_angular_vel = fusion->imu_angular_velocity[2]; // Z����ٶ�
        
        // ��Ȩ�ں�
        fusion->fused_pose.velocity.angular = 
            NAV_ENCODER_WEIGHT * encoder_angular_vel + 
            NAV_IMU_WEIGHT * imu_angular_vel;
            
        fusion->confidence = 0.9f; // �����Ŷ�
    }
    else if (fusion->encoder_status == NAV_SENSOR_OK) {
        // ��ʹ�ñ�����
        fusion->fused_pose.velocity.angular = fusion->encoder_pose.velocity.angular;
        fusion->confidence = 0.7f; // �е����Ŷ�
    }
    else {
        fusion->confidence = 0.1f; // �����Ŷ�
    }
    
    // ���º���� (���ֽ��ٶ�)
    static uint32 last_fusion_time_ms = 0;
    uint32 current_time_ms = nav_get_system_time_ms();
    
    if (last_fusion_time_ms != 0) {
        float dt = (current_time_ms - last_fusion_time_ms) / 1000.0f;
        fusion->fused_pose.heading += fusion->fused_pose.velocity.angular * dt;
        fusion->fused_pose.heading = nav_angle_normalize(fusion->fused_pose.heading);
    }
    
    last_fusion_time_ms = current_time_ms;
    
    // �������ٶ�
    fusion->fused_pose.velocity.linear = fusion->encoder_pose.velocity.linear;
    fusion->fused_pose.timestamp_ms = current_time_ms;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��鴫��������״̬
//-------------------------------------------------------------------------------------------------------------------
static uint8 nav_check_sensor_health(void)
{
    nav_sensor_fusion_t *fusion = &nav_system.sensor_fusion;
    uint32 current_time_ms = nav_get_system_time_ms();
    
    // ��鴫�������ݸ���ʱ��
    if (current_time_ms - fusion->last_update_ms > 100) { // 100ms��ʱ
        return 0;
    }
    
    // �����������ݺ�����
    if (fabsf(fusion->encoder_pose.velocity.linear) > NAV_MAX_SPEED + 1.0f) {
        fusion->encoder_status = NAV_SENSOR_WARNING;
    }
    
    // ���IMU���ݺ�����
    if (fabsf(fusion->imu_angular_velocity[2]) > 10.0f) { // 10 rad/s �����ٶ�
        fusion->imu_status = NAV_SENSOR_WARNING;
    }
    
    return (fusion->encoder_status != NAV_SENSOR_ERROR && 
            fusion->imu_status != NAV_SENSOR_ERROR);
}

//=================================================Pure Pursuit�㷨ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ����Pure Pursuit����
//-------------------------------------------------------------------------------------------------------------------
static uint8 nav_update_pure_pursuit(void)
{
    nav_pure_pursuit_t *pp = &nav_system.pure_pursuit;
    
    // ����ǰհ����
    float current_speed = nav_system.sensor_fusion.fused_pose.velocity.linear;
    pp->lookahead_distance = NAV_MIN_LOOKAHEAD + 
                             NAV_LOOKAHEAD_FACTOR * current_speed;
    pp->lookahead_distance = nav_clamp(pp->lookahead_distance, 
                                       NAV_MIN_LOOKAHEAD, NAV_MAX_LOOKAHEAD);
    
    // �ҵ�ǰհ��
    pp->lookahead_point = nav_find_lookahead_point();
    
    // �������ƫ��
    pp->cross_track_error = nav_calculate_cross_track_error();
    
    // ��������
    pp->curvature = nav_calculate_curvature(&pp->lookahead_point);
    
    // ����ת���
    pp->steering_angle = atanf(NAV_WHEELBASE * pp->curvature);
    pp->steering_angle = nav_clamp(pp->steering_angle, -M_PI/4, M_PI/4); // ��45������
    
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����ǰհ��
//-------------------------------------------------------------------------------------------------------------------
static nav_position_t nav_find_lookahead_point(void)
{
    nav_position_t lookahead_point = {0};
    nav_path_t *path = &nav_system.current_path;
    nav_position_t robot_pos = nav_system.sensor_fusion.fused_pose.position;
    float lookahead_dist = nav_system.pure_pursuit.lookahead_distance;
    
    if (path->waypoint_count == 0) {
        return lookahead_point;
    }
    
    // �ӵ�ǰĿ��·���㿪ʼ����
    uint16 start_index = path->current_waypoint;
    if (start_index >= path->waypoint_count) {
        start_index = path->waypoint_count - 1;
    }
    
    // ��·����Ѱ�Ҿ��������lookahead_dist�ĵ�
    for (uint16 i = start_index; i < path->waypoint_count - 1; i++) {
        nav_position_t p1 = path->waypoints[i].position;
        nav_position_t p2 = path->waypoints[i + 1].position;
        
        // ��������˵��߶ε�ͶӰ��
        float dx = p2.x - p1.x;
        float dy = p2.y - p1.y;
        float segment_length = sqrtf(dx * dx + dy * dy);
        
        if (segment_length < 1e-6f) continue; // �������
        
        // ��һ���߶η�������
        dx /= segment_length;
        dy /= segment_length;
        
        // �����˵�p1������
        float rx = robot_pos.x - p1.x;
        float ry = robot_pos.y - p1.y;
        
        // ͶӰ����
        float projection = rx * dx + ry * dy;
        projection = nav_clamp(projection, 0.0f, segment_length);
        
        // ͶӰ��
        nav_position_t proj_point;
        proj_point.x = p1.x + projection * dx;
        proj_point.y = p1.y + projection * dy;
        
        // ����Ƿ��ҵ�ǰհ��
        float dist_to_proj = nav_distance_2d(&robot_pos, &proj_point);
        
        if (dist_to_proj <= lookahead_dist) {
            // �����߶μ���Ѱ��ǰհ��
            float remaining_dist = lookahead_dist - dist_to_proj;
            float remaining_on_segment = segment_length - projection;
            
            if (remaining_dist <= remaining_on_segment) {
                // ǰհ���ڵ�ǰ�߶���
                lookahead_point.x = proj_point.x + remaining_dist * dx;
                lookahead_point.y = proj_point.y + remaining_dist * dy;
                nav_system.pure_pursuit.target_waypoint_index = i;
                return lookahead_point;
            }
        }
    }
    
    // ���û�ҵ����������һ��·����
    if (path->waypoint_count > 0) {
        lookahead_point = path->waypoints[path->waypoint_count - 1].position;
        nav_system.pure_pursuit.target_waypoint_index = path->waypoint_count - 1;
    }
    
    return lookahead_point;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��������
//-------------------------------------------------------------------------------------------------------------------
static float nav_calculate_curvature(const nav_position_t *lookahead_point)
{
    nav_position_t robot_pos = nav_system.sensor_fusion.fused_pose.position;
    float robot_heading = nav_system.sensor_fusion.fused_pose.heading;
    
    // ���㵽ǰհ������λ��
    float dx = lookahead_point->x - robot_pos.x;
    float dy = lookahead_point->y - robot_pos.y;
    
    // ת��������������ϵ
    float cos_h = cosf(robot_heading);
    float sin_h = sinf(robot_heading);
    
    // float local_x = dx * cos_h + dy * sin_h;  // ��ʱδʹ��
    float local_y = -dx * sin_h + dy * cos_h;
    
    float lookahead_dist = nav_system.pure_pursuit.lookahead_distance;
    
    // ��ֹ����
    if (lookahead_dist < 1e-3f) {
        return 0.0f;
    }
    
    // Pure Pursuit���ʹ�ʽ
    float curvature = 2.0f * local_y / (lookahead_dist * lookahead_dist);
    
    // ��������
    float max_curvature = 1.0f / 0.5f; // ��Сת��뾶0.5m
    return nav_clamp(curvature, -max_curvature, max_curvature);
}

// ��ʱע��δʹ�õĺ���
// static uint16 nav_find_closest_waypoint(void)
// {
//     nav_path_t *path = &nav_system.current_path;
//     nav_position_t robot_pos = nav_system.sensor_fusion.fused_pose.position;
//     
//     if (path->waypoint_count == 0) {
//         return 0;
//     }
//     
//     uint16 closest_index = 0;
//     float min_distance = nav_distance_2d(&robot_pos, &path->waypoints[0].position);
//     
//     for (uint16 i = 1; i < path->waypoint_count; i++) {
//         float distance = nav_distance_2d(&robot_pos, &path->waypoints[i].position);
//         if (distance < min_distance) {
//             min_distance = distance;
//             closest_index = i;
//         }
//     }
//     
//     return closest_index;
// }

//-------------------------------------------------------------------------------------------------------------------
// �������     �������ƫ��
//-------------------------------------------------------------------------------------------------------------------
static float nav_calculate_cross_track_error(void)
{
    nav_path_t *path = &nav_system.current_path;
    nav_position_t robot_pos = nav_system.sensor_fusion.fused_pose.position;
    
    if (path->waypoint_count < 2) {
        return 0.0f;
    }
    
    // �ҵ���ǰĿ���߶�
    uint16 target_index = nav_system.pure_pursuit.target_waypoint_index;
    if (target_index >= path->waypoint_count - 1) {
        target_index = path->waypoint_count - 2;
    }
    
    nav_position_t p1 = path->waypoints[target_index].position;
    nav_position_t p2 = path->waypoints[target_index + 1].position;
    
    // ����㵽�߶εľ���
    float A = p2.y - p1.y;
    float B = p1.x - p2.x;
    float C = p2.x * p1.y - p1.x * p2.y;
    
    float denominator = sqrtf(A * A + B * B);
    if (denominator < 1e-6f) {
        return 0.0f;
    }
    
    float cross_track_error = (A * robot_pos.x + B * robot_pos.y + C) / denominator;
    
    return cross_track_error;
}

//=================================================�ٶȿ��ƹ���ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     �����ٶȿ���
//-------------------------------------------------------------------------------------------------------------------
static uint8 nav_update_speed_control(void)
{
    nav_speed_control_t *speed_ctrl = &nav_system.speed_control;
    
    // ����Ŀ���ٶ�
    speed_ctrl->target_speed = nav_calculate_target_speed();
    
    // ƽ���ٶȹ���
    speed_ctrl->target_speed = nav_smooth_speed_transition(
        speed_ctrl->target_speed, 
        speed_ctrl->current_speed
    );
    
    // ���µ�ǰ�ٶ�
    speed_ctrl->current_speed = nav_system.sensor_fusion.fused_pose.velocity.linear;
    
    // ������ٶ�
    static float last_speed = 0.0f;
    static uint32 last_time_ms = 0;
    uint32 current_time_ms = nav_get_system_time_ms();
    
    if (last_time_ms != 0) {
        float dt = (current_time_ms - last_time_ms) / 1000.0f;
        if (dt > 1e-6f) {
            speed_ctrl->acceleration = (speed_ctrl->current_speed - last_speed) / dt;
        }
    }
    
    last_speed = speed_ctrl->current_speed;
    last_time_ms = current_time_ms;
    
    // �����������ٶ�
    nav_calculate_wheel_speeds(speed_ctrl->target_speed, 
                               nav_system.pure_pursuit.curvature);
    
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����Ŀ���ٶ�
//-------------------------------------------------------------------------------------------------------------------
static float nav_calculate_target_speed(void)
{
    nav_path_t *path = &nav_system.current_path;
    
    if (path->waypoint_count == 0) {
        return 0.0f;
    }
    
    // ����Ŀ���ٶ�
    uint16 target_index = nav_system.pure_pursuit.target_waypoint_index;
    if (target_index >= path->waypoint_count) {
        target_index = path->waypoint_count - 1;
    }
    
    float base_speed = path->waypoints[target_index].target_speed;
    
    // �������ʵ����ٶ�
    float curvature = fabsf(nav_system.pure_pursuit.curvature);
    float max_speed_for_curvature = NAV_MAX_SPEED;
    
    if (curvature > 1e-3f) {
        // �������������Ƽ�������ٶ�
        float max_lateral_acceleration = 2.0f; // 2 m/s? ��������ٶ�
        max_speed_for_curvature = sqrtf(max_lateral_acceleration / curvature);
    }
    
    nav_system.speed_control.max_speed_for_curvature = max_speed_for_curvature;
    
    // ���ݺ���ƫ������ٶ�
    float cross_track_error = fabsf(nav_system.pure_pursuit.cross_track_error);
    float speed_reduction_factor = 1.0f;
    
    if (cross_track_error > NAV_POSITION_TOLERANCE) {
        speed_reduction_factor = NAV_POSITION_TOLERANCE / cross_track_error;
        speed_reduction_factor = nav_clamp(speed_reduction_factor, 0.3f, 1.0f);
    }
    
    // Ӧ����������
    float target_speed = base_speed * speed_reduction_factor;
    target_speed = nav_clamp(target_speed, NAV_MIN_SPEED, 
                            fminf(max_speed_for_curvature, NAV_MAX_SPEED));
    
    // ����Ƿ�ӽ��յ�
    nav_position_t robot_pos = nav_system.sensor_fusion.fused_pose.position;
    nav_position_t final_waypoint = path->waypoints[path->waypoint_count - 1].position;
    float distance_to_end = nav_distance_2d(&robot_pos, &final_waypoint);
    
    if (distance_to_end < 1.0f) { // �����յ�1���ڿ�ʼ����
        float decel_factor = distance_to_end / 1.0f;
        target_speed *= decel_factor;
        target_speed = nav_clamp(target_speed, NAV_MIN_SPEED, target_speed);
    }
    
    return target_speed;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     �����������ٶ�
//-------------------------------------------------------------------------------------------------------------------
static void nav_calculate_wheel_speeds(float target_speed, float curvature)
{
    nav_speed_control_t *speed_ctrl = &nav_system.speed_control;
    
    // ��������˶�ѧ
    float angular_velocity = target_speed * curvature;
    float wheel_speed_diff = angular_velocity * NAV_WHEELBASE / 2.0f;
    
    speed_ctrl->left_wheel_speed = target_speed - wheel_speed_diff;
    speed_ctrl->right_wheel_speed = target_speed + wheel_speed_diff;
    
    // �ٶ�����
    speed_ctrl->left_wheel_speed = nav_clamp(speed_ctrl->left_wheel_speed, 
                                            -NAV_MAX_SPEED, NAV_MAX_SPEED);
    speed_ctrl->right_wheel_speed = nav_clamp(speed_ctrl->right_wheel_speed, 
                                             -NAV_MAX_SPEED, NAV_MAX_SPEED);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ƽ���ٶȹ���
//-------------------------------------------------------------------------------------------------------------------
static float nav_smooth_speed_transition(float target_speed, float current_speed)
{
    float speed_error = target_speed - current_speed;
    float max_delta_speed = NAV_MAX_ACCELERATION * (NAV_CONTROL_PERIOD_MS / 1000.0f);
    
    if (speed_error > max_delta_speed) {
        return current_speed + max_delta_speed;
    } else if (speed_error < -max_delta_speed) {
        return current_speed - max_delta_speed;
    }
    
    return target_speed;
}

//=================================================��ȫ��ع���ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ���°�ȫ���
//-------------------------------------------------------------------------------------------------------------------
static uint8 nav_update_safety_monitor(void)
{
    nav_safety_monitor_t *safety = &nav_system.safety_monitor;
    uint32 current_time_ms = nav_get_system_time_ms();
    
    // ���ڰ�ȫ���
    if (current_time_ms - safety->last_safety_check_ms >= 
        NAV_SAFETY_CHECK_INTERVAL * NAV_CONTROL_PERIOD_MS) {
        
        // ���·��ƫ��
        if (!nav_check_path_deviation()) {
            safety->path_deviation = nav_system.pure_pursuit.cross_track_error;
            if (fabsf(safety->path_deviation) > NAV_PATH_DEVIATION_MAX) {
                printf("���棺·��ƫ����� %.2fm\n", safety->path_deviation);
                return 0;
            }
        }
        
        // ��鴫������ʱ
        if (!nav_check_sensor_timeout()) {
            safety->sensor_timeout_count++;
            if (safety->sensor_timeout_count > 10) { // ����10�γ�ʱ
                printf("���󣺴�������ʱ\n");
                nav_trigger_emergency_stop();
                return 0;
            }
        } else {
            safety->sensor_timeout_count = 0; // ���ü���
        }
        
        safety->last_safety_check_ms = current_time_ms;
    }
    
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ���·��ƫ��
//-------------------------------------------------------------------------------------------------------------------
static uint8 nav_check_path_deviation(void)
{
    float cross_track_error = fabsf(nav_system.pure_pursuit.cross_track_error);
    return (cross_track_error <= NAV_PATH_DEVIATION_MAX);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��鴫������ʱ
//-------------------------------------------------------------------------------------------------------------------
static uint8 nav_check_sensor_timeout(void)
{
    uint32 current_time_ms = nav_get_system_time_ms();
    uint32 last_sensor_update = nav_system.sensor_fusion.last_update_ms;
    
    return (current_time_ms - last_sensor_update <= 100); // 100ms��ʱ
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��������ͣ��
//-------------------------------------------------------------------------------------------------------------------
static void nav_trigger_emergency_stop(void)
{
    nav_system.safety_monitor.emergency_stop_triggered = 1;
    nav_system.status = NAV_STATUS_EMERGENCY_STOP;
    
    // ����ֹͣ���
    motor_left_speed_control(0.0f, nav_system.sensor_fusion.fused_pose.velocity.linear);
    motor_right_speed_control(0.0f, nav_system.sensor_fusion.fused_pose.velocity.linear);
    
    printf("����ͣ��������\n");
}

//=================================================ͳ����Ϣ����ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ����ͳ����Ϣ
//-------------------------------------------------------------------------------------------------------------------
static void nav_update_statistics(void)
{
    nav_statistics_t *stats = &nav_system.statistics;
    uint32 current_time_ms = nav_get_system_time_ms();
    
    // ������ʻ����
    static nav_position_t last_position = {0};
    static uint8 position_initialized = 0;
    
    nav_position_t current_position = nav_system.sensor_fusion.fused_pose.position;
    
    if (position_initialized) {
        float distance_increment = nav_distance_2d(&last_position, &current_position);
        stats->total_distance_traveled += distance_increment;
    } else {
        position_initialized = 1;
    }
    
    last_position = current_position;
    
    // �����ٶ�ͳ��
    float current_speed = nav_system.sensor_fusion.fused_pose.velocity.linear;
    if (current_speed > stats->max_speed_reached) {
        stats->max_speed_reached = current_speed;
    }
    
    // ���µ���ʱ��
    if (stats->navigation_start_time_ms != 0) {
        stats->total_navigation_time_ms = current_time_ms - stats->navigation_start_time_ms;
        
        if (stats->total_navigation_time_ms > 0) {
            stats->average_speed = stats->total_distance_traveled / 
                                  (stats->total_navigation_time_ms / 1000.0f);
        }
    }
    
    // ���º������ͳ��
    float cross_track_error = fabsf(nav_system.pure_pursuit.cross_track_error);
    if (cross_track_error > stats->max_cross_track_error) {
        stats->max_cross_track_error = cross_track_error;
    }
    
    // ����ƽ���������򻯰汾��
    static float error_sum = 0.0f;
    static uint32 error_count = 0;
    
    error_sum += cross_track_error;
    error_count++;
    stats->average_cross_track_error = error_sum / error_count;
}

//=================================================ϵͳ������ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡϵͳʱ��
//-------------------------------------------------------------------------------------------------------------------
static uint32 nav_get_system_time_ms(void)
{
    // ʹ��ϵͳ��ʱ����ȡ����ʱ��
    static uint8 timer_initialized = 0;
    if (!timer_initialized) {
        timer_init(TC_TIME2_CH0, TIMER_MS);  // ��ʼ����ʱ��Ϊ����ģʽ
        timer_start(TC_TIME2_CH0);           // ������ʱ��
        timer_initialized = 1;
    }
    return timer_get(TC_TIME2_CH0);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����ϵͳ״̬
//-------------------------------------------------------------------------------------------------------------------
static void nav_reset_system_state(void)
{
    memset(&nav_system, 0, sizeof(nav_system_t));
    nav_system.status = NAV_STATUS_IDLE;
    nav_system.mode = NAV_MODE_PURE_PURSUIT;
}

//=================================================��Ҫ�ӿں���ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ����ϵͳ��ʼ��
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_init(void)
{
    printf("��ʼ����������ϵͳ...\n");
    
    // ����ϵͳ״̬
    nav_reset_system_state();
    
    nav_system.status = NAV_STATUS_INITIALIZING;
    
    // ��ʼ��������ϵͳ
    if (encoder_init() != ENCODER_STATUS_OK) {
        printf("��������ʼ��ʧ��\n");
        nav_system.status = NAV_STATUS_ERROR;
        return 0;
    }
    
    // ��ʼ��IMUϵͳ
    SCH1_filter sFilter = {.Rate12 = 30, .Acc12 = 30, .Acc3 = 30};
    SCH1_sensitivity sSensitivity = {.Rate1 = 1600, .Rate2 = 1600, .Acc1 = 3200, .Acc2 = 3200, .Acc3 = 3200};
    SCH1_decimation sDecimation = {.Rate2 = 1, .Acc2 = 1};
    
    if (SCH1_init(sFilter, sSensitivity, sDecimation, false) != SCH1_OK) {
        printf("IMU��ʼ��ʧ��\n");
        nav_system.status = NAV_STATUS_ERROR;
        return 0;
    }
    
    // ��ʼ���������ϵͳ
    if (motor_pid_init() != MOTOR_PID_OK) {
        printf("�������ϵͳ��ʼ��ʧ��\n");
        nav_system.status = NAV_STATUS_ERROR;
        return 0;
    }
    
    // ��ʼ��Flash·���洢ϵͳ
    if (flash_road_init() != FLASH_ROAD_RESULT_OK) {
        printf("Flash·���洢ϵͳ��ʼ��ʧ��\n");
        nav_system.status = NAV_STATUS_ERROR;
        return 0;
    }
    
    nav_system.initialized = 1;
    nav_system.status = NAV_STATUS_IDLE;
    
    printf("��������ϵͳ��ʼ�����\n");
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ���ص���·��
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_load_path(const char *path_name)
{
    if (!nav_system.initialized || path_name == NULL) {
        return 0;
    }
    
    nav_system.status = NAV_STATUS_PATH_LOADING;
    
    // ��ʱ��·������ʵ��
    // ��ʵ��Ӧ������Ҫʵ�ִ�Flash����·���Ĺ���
    nav_path_t *nav_path = &nav_system.current_path;
    nav_path->waypoint_count = 4; // ʾ��·��
    nav_path->current_waypoint = 0;
    nav_path->total_length = 10.0f;
    strncpy(nav_path->path_name, path_name, sizeof(nav_path->path_name) - 1);
    
    // ����ʾ��·����
    nav_path->waypoints[0] = (nav_waypoint_t){{0.0f, 0.0f}, 1.0f, 0.1f, 0};
    nav_path->waypoints[1] = (nav_waypoint_t){{2.0f, 0.0f}, 2.0f, 0.1f, 0};
    nav_path->waypoints[2] = (nav_waypoint_t){{2.0f, 2.0f}, 1.5f, 0.1f, 0};
    nav_path->waypoints[3] = (nav_waypoint_t){{0.0f, 2.0f}, 1.0f, 0.1f, 0};
    
    printf("·�����سɹ�: %s (ʾ��·��)\n", path_name);
    
    nav_system.status = NAV_STATUS_IDLE;
    
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ʼ����
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_start(nav_mode_enum mode)
{
    if (!nav_system.initialized || nav_system.current_path.waypoint_count == 0) {
        return 0;
    }
    
    nav_system.mode = mode;
    nav_system.status = NAV_STATUS_NAVIGATING;
    
    // ����ͳ����Ϣ
    memset(&nav_system.statistics, 0, sizeof(nav_statistics_t));
    nav_system.statistics.navigation_start_time_ms = nav_get_system_time_ms();
    
    // ����·������״̬
    nav_system.current_path.current_waypoint = 0;
    
    printf("��ʼ����: %s (ģʽ: %d)\n", 
           nav_system.current_path.path_name, mode);
    
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ֹͣ����
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_stop(uint8 emergency)
{
    if (emergency) {
        nav_trigger_emergency_stop();
    } else {
        nav_system.status = NAV_STATUS_IDLE;
        
        // ƽ��ͣ��
        motor_left_speed_control(0.0f, nav_system.sensor_fusion.fused_pose.velocity.linear);
        motor_right_speed_control(0.0f, nav_system.sensor_fusion.fused_pose.velocity.linear);
    }
    
    printf("����ֹͣ (����ͣ��: %s)\n", emergency ? "��" : "��");
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����ϵͳ��ѭ������
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_update(void)
{
    if (!nav_system.initialized) {
        return 0;
    }
    
    uint32 current_time_ms = nav_get_system_time_ms();
    nav_system.control_loop_count++;
    
    // ����������
    if (current_time_ms - nav_system.last_control_update_ms < NAV_CONTROL_PERIOD_MS) {
        return 1; // ��δ������ʱ��
    }
    
    nav_system.last_control_update_ms = current_time_ms;
    
    // ״̬������
    switch (nav_system.status) {
        case NAV_STATUS_NAVIGATING:
            // ���´������ں�
            if (!nav_update_sensor_fusion()) {
                printf("�������ںϸ���ʧ��\n");
                break;
            }
            
            // ����Pure Pursuit����
            if (!nav_update_pure_pursuit()) {
                printf("Pure Pursuit����ʧ��\n");
                break;
            }
            
            // �����ٶȿ���
            if (!nav_update_speed_control()) {
                printf("�ٶȿ��Ƹ���ʧ��\n");
                break;
            }
            
            // ��ȫ���
            if (!nav_update_safety_monitor()) {
                printf("��ȫ��ؼ��ʧ��\n");
                nav_stop(1); // ����ͣ��
                break;
            }
            
            // Ӧ�ÿ���ָ��
            motor_left_speed_control(nav_system.speed_control.left_wheel_speed, 
                                    nav_system.sensor_fusion.fused_pose.velocity.linear);
            motor_right_speed_control(nav_system.speed_control.right_wheel_speed, 
                                     nav_system.sensor_fusion.fused_pose.velocity.linear);
            
            // ����Ƿ񵽴��յ�
            if (nav_is_target_reached()) {
                nav_system.status = NAV_STATUS_COMPLETED;
                printf("������ɣ�\n");
            }
            
            // ����ͳ����Ϣ
            nav_update_statistics();
            break;
            
        case NAV_STATUS_PAUSED:
            // ��ͣ״̬�����ֵ�ǰλ��
            motor_left_speed_control(0.0f, nav_system.sensor_fusion.fused_pose.velocity.linear);
            motor_right_speed_control(0.0f, nav_system.sensor_fusion.fused_pose.velocity.linear);
            nav_update_sensor_fusion(); // �������´���������
            break;
            
        case NAV_STATUS_EMERGENCY_STOP:
            // ����ͣ��״̬
            motor_left_speed_control(0.0f, nav_system.sensor_fusion.fused_pose.velocity.linear);
            motor_right_speed_control(0.0f, nav_system.sensor_fusion.fused_pose.velocity.linear);
            break;
            
        default:
            // ����״̬����Ҫ���Ƹ���
            break;
    }
    
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ͣ/�ָ�����
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_pause(uint8 pause)
{
    if (!nav_system.initialized) {
        return 0;
    }
    
    if (pause) {
        if (nav_system.status == NAV_STATUS_NAVIGATING) {
            nav_system.status = NAV_STATUS_PAUSED;
            printf("��������ͣ\n");
        }
    } else {
        if (nav_system.status == NAV_STATUS_PAUSED) {
            nav_system.status = NAV_STATUS_NAVIGATING;
            printf("�����ѻָ�\n");
        }
    }
    
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ����ϵͳ״̬
//-------------------------------------------------------------------------------------------------------------------
nav_status_enum nav_get_status(void)
{
    return nav_system.status;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ��ǰλ��
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_get_current_position(nav_position_t *position)
{
    if (position == NULL || !nav_system.initialized) {
        return 0;
    }
    
    *position = nav_system.sensor_fusion.fused_pose.position;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ����ͳ����Ϣ
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_get_statistics(nav_statistics_t *stats)
{
    if (stats == NULL || !nav_system.initialized) {
        return 0;
    }
    
    *stats = nav_system.statistics;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ���õ�������
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_set_parameters(float max_speed, float lookahead_factor)
{
    if (!nav_system.initialized) {
        return 0;
    }
    
    // ������֤������
    if (max_speed > 0.0f && max_speed <= 5.0f) {
        // ��̬��������ٶȣ���Ҫ���¶��峣����ʹ��ȫ�ֱ�����
        printf("����ٶ�����Ϊ: %.2f m/s\n", max_speed);
    }
    
    if (lookahead_factor > 0.0f && lookahead_factor <= 2.0f) {
        // ��̬����ǰհϵ��
        printf("ǰհ����ϵ������Ϊ: %.2f\n", lookahead_factor);
    }
    
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ·���������
//-------------------------------------------------------------------------------------------------------------------
float nav_get_cross_track_error(void)
{
    return nav_system.pure_pursuit.cross_track_error;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����Ƿ񵽴�Ŀ��
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_is_target_reached(void)
{
    nav_path_t *path = &nav_system.current_path;
    
    if (path->waypoint_count == 0) {
        return 1;
    }
    
    nav_position_t robot_pos = nav_system.sensor_fusion.fused_pose.position;
    nav_position_t final_waypoint = path->waypoints[path->waypoint_count - 1].position;
    
    float distance_to_end = nav_distance_2d(&robot_pos, &final_waypoint);
    float tolerance = path->waypoints[path->waypoint_count - 1].tolerance;
    
    return (distance_to_end <= tolerance);
}
