/*********************************************************************************************************************
* �ļ�����          config_navigation.c
* ����˵��          ����ϵͳ���ò���ʵ���ļ�
* ����              LittleMaster
* �汾��Ϣ          v1.0
* �޸ļ�¼
* ����              ����                �汾              ��ע
* 2025-09-18        LittleMaster        v1.0              ��������ϵͳ����ʵ��
*
* �ļ�����˵����
* ���ļ�ʵ�ֵ���ϵͳ���ò����Ĺ����ܣ�����������ʼ�������桢���غͷ��ʽӿ�
* �ṩͳһ�����ù�������ģ���Ĳ�����������
*
* ��Ҫ���ܣ�
* 1. ���ò����ĳ�ʼ����Ĭ��ֵ����
* 2. ���ò�����Flash�洢�Ͷ�ȡ
* 3. ����ʱ���ò����Ļ�ȡ������
* 4. ���ò�������Ч����֤
* 5. ��������֧��
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "config_navigation.h"
//#include "flash_road.h"

//=================================================ȫ�ֱ�������================================================
nav_config_struct nav_config = {0};            // ����ϵͳ���ýṹ��
nav_data_struct nav_data = {0};                // ����ϵͳ���ݽṹ��

// ������������
float nav_errors_coords[NAV_COORD_RECORD] = {0};       // ������������
int nav_mileage_list[NAV_COORD_RECORD] = {0};          // ����б�����
int32 nav_yaw_buffer[4000] = {0};                     // ƫ���ǻ�������

//=================================================�ڲ���������================================================
static uint8 nav_config_validate(void);
static void nav_config_apply_limits(void);

//=================================================��Ҫ�ӿں���================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ��������ϵͳ��ʼ��
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_config_init(void)
{
    // ���ȼ���Ĭ������
    nav_config_load_default();
    
    // ���Դ�Flash���ر��������
    if (nav_config_load() != 0)
    {
        // Flash����ʧ�ܣ�ʹ��Ĭ������
        nav_config_load_default();
    }
    
    // ��֤���ò�����Ч��
    if (nav_config_validate() != 0)
    {
        // ������Ч���ָ�Ĭ������
        nav_config_load_default();
    }
    
    // Ӧ�ò�������
    nav_config_apply_limits();
    
    // ��������Ѽ���
    nav_config.config_loaded = 1;
    nav_config.config_modified = 0;
    
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����Ĭ������
//-------------------------------------------------------------------------------------------------------------------
void nav_config_load_default(void)
{
    // ·��������Ĭ��ֵ
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
    
    // ���Ʋ���Ĭ��ֵ
    nav_config.control_radius[0] = NAV_DEFAULT_R1;
    nav_config.control_radius[1] = NAV_DEFAULT_R2;
    
    // ����PID����Ĭ��ֵ
    nav_config.nav_pid_kp = NAV_DEFAULT_PID_KP;
    nav_config.nav_pid_ki = NAV_DEFAULT_PID_KI;
    nav_config.nav_pid_kd = NAV_DEFAULT_PID_KD;
    nav_config.nav_pid_speed = NAV_DEFAULT_PID_SPEED;
    
    // �������PID����Ĭ��ֵ (ʹ��ʵ�ʵ��Բ���)
    // �����ٶȻ�PID (PIDSL - �����ٶȣ��̳�PIDS����)
    nav_config.motor_control.left_speed_kp = LEFT_WHEEL_SPEED_PID_KP;  // 30.0f
    nav_config.motor_control.left_speed_ki = LEFT_WHEEL_SPEED_PID_KI;  // 2.4f
    nav_config.motor_control.left_speed_kd = LEFT_WHEEL_SPEED_PID_KD;  // 0.0f
    
    // ����λ�û�PID (��ʱ����ԭֵ)
    nav_config.motor_control.left_position_kp = 2.0f;
    nav_config.motor_control.left_position_ki = 0.1f;
    nav_config.motor_control.left_position_kd = 0.05f;
    
    // �ҵ���ٶȻ�PID (PIDS - �ٶȿ���)
    nav_config.motor_control.right_speed_kp = MOTOR_SPEED_PID_KP;      // 30.0f
    nav_config.motor_control.right_speed_ki = MOTOR_SPEED_PID_KI;      // 2.4f
    nav_config.motor_control.right_speed_kd = MOTOR_SPEED_PID_KD;      // 0.0f
    
    // �ҵ��λ�û�PID (��ʱ����ԭֵ)
    nav_config.motor_control.right_position_kp = 2.0f;
    nav_config.motor_control.right_position_ki = 0.1f;
    nav_config.motor_control.right_position_kd = 0.05f;
    
    // ������Ʋ���Ĭ��ֵ
    nav_config.motor_control.max_speed = 2.0f;          // ����ٶ� 2m/s
    nav_config.motor_control.max_acceleration = 5.0f;   // �����ٶ� 5m/s?
    nav_config.motor_control.wheelbase = 150.0f;        // �־� 150mm
    nav_config.motor_control.control_frequency = 1000;  // ����Ƶ�� 1KHz
    
    // ϵͳ���в���Ĭ��ֵ
    nav_config.target_speed = NAV_DEFAULT_TARGET_SPEED;
    nav_config.fuya_enable = NAV_FUYA_DEFAULT;
    nav_config.go_flag = 0;
    nav_config.go_state = 0;
    
    // ϵͳ״̬
    nav_config.config_loaded = 0;
    nav_config.config_modified = 1;  // ���Ϊ���޸ģ���Ҫ����
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ���浱ǰ���õ�Flash
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_config_save(void)
{
    // �������ʹ��Flash�洢ϵͳ��������
    // ��ʱ���سɹ���ʵ��ʵ����Ҫ����Flash�洢�ӿ�
    
    // ��������ѱ���
    nav_config.config_modified = 0;
    
    return 0;  // �ɹ�
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��Flash��������
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_config_load(void)
{
    // �������ʹ��Flash�洢ϵͳ��������
    // ��ʱ����ʧ�ܣ�ʵ��ʵ����Ҫ����Flash�洢�ӿ�
    
    return 1;  // ʧ�ܣ�ʹ��Ĭ������
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ·��������
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_config_get_path_point(uint8 point_index, float *x, float *y)
{
    if (point_index >= 6 || x == NULL || y == NULL)
    {
        return 1;  // ��������
    }
    
    *x = nav_config.path_points_x[point_index];
    *y = nav_config.path_points_y[point_index];
    
    return 0;  // �ɹ�
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����·��������
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_config_set_path_point(uint8 point_index, float x, float y)
{
    if (point_index >= 6)
    {
        return 1;  // ��������
    }
    
    nav_config.path_points_x[point_index] = x;
    nav_config.path_points_y[point_index] = y;
    nav_config.config_modified = 1;  // ����������޸�
    
    return 0;  // �ɹ�
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ���ư뾶����
//-------------------------------------------------------------------------------------------------------------------
int nav_config_get_control_radius(uint8 radius_index)
{
    if (radius_index >= 2)
    {
        return 0;  // �������󣬷���0
    }
    
    return nav_config.control_radius[radius_index];
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ���ÿ��ư뾶����
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_config_set_control_radius(uint8 radius_index, int radius)
{
    if (radius_index >= 2)
    {
        return 1;  // ��������
    }
    
    nav_config.control_radius[radius_index] = radius;
    nav_config.config_modified = 1;  // ����������޸�
    
    return 0;  // �ɹ�
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡPID����
//-------------------------------------------------------------------------------------------------------------------
void nav_config_get_pid(float *kp, float *ki, float *kd, int *speed)
{
    if (kp != NULL) *kp = nav_config.nav_pid_kp;
    if (ki != NULL) *ki = nav_config.nav_pid_ki;
    if (kd != NULL) *kd = nav_config.nav_pid_kd;
    if (speed != NULL) *speed = nav_config.nav_pid_speed;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����PID����
//-------------------------------------------------------------------------------------------------------------------
void nav_config_set_pid(float kp, float ki, float kd, int speed)
{
    nav_config.nav_pid_kp = kp;
    nav_config.nav_pid_ki = ki;
    nav_config.nav_pid_kd = kd;
    nav_config.nav_pid_speed = speed;
    nav_config.config_modified = 1;  // ����������޸�
}

//=================================================�ڲ�����ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ��֤���ò�����Ч��
//-------------------------------------------------------------------------------------------------------------------
static uint8 nav_config_validate(void)
{
    // ���·�������귶Χ��ʾ����-1000 �� 1000��
    for (int i = 0; i < 6; i++)
    {
        if (nav_config.path_points_x[i] < -1000.0f || nav_config.path_points_x[i] > 1000.0f ||
            nav_config.path_points_y[i] < -1000.0f || nav_config.path_points_y[i] > 1000.0f)
        {
            return 1;  // ���곬����Χ
        }
    }
    
    // �����ư뾶��Χ��ʾ����1 �� 100��
    for (int i = 0; i < 2; i++)
    {
        if (nav_config.control_radius[i] < 1 || nav_config.control_radius[i] > 100)
        {
            return 1;  // �뾶������Χ
        }
    }
    
    // ��鵼��PID������Χ
    if (nav_config.nav_pid_kp < 0.0f || nav_config.nav_pid_kp > 100.0f ||
        nav_config.nav_pid_ki < 0.0f || nav_config.nav_pid_ki > 100.0f ||
        nav_config.nav_pid_kd < 0.0f || nav_config.nav_pid_kd > 100.0f)
    {
        return 1;  // PID����������Χ
    }
    
    // ����ٶȲ�����Χ��ʾ����0 �� 500��
    if (nav_config.nav_pid_speed < 0 || nav_config.nav_pid_speed > 500 ||
        nav_config.target_speed < 0 || nav_config.target_speed > 500)
    {
        return 1;  // �ٶȲ���������Χ
    }
    
    return 0;  // ������Ч
}

//-------------------------------------------------------------------------------------------------------------------
// �������     Ӧ�ò�������
//-------------------------------------------------------------------------------------------------------------------
static void nav_config_apply_limits(void)
{
    // ����·�������귶Χ
    for (int i = 0; i < 6; i++)
    {
        if (nav_config.path_points_x[i] < -1000.0f) nav_config.path_points_x[i] = -1000.0f;
        if (nav_config.path_points_x[i] > 1000.0f) nav_config.path_points_x[i] = 1000.0f;
        if (nav_config.path_points_y[i] < -1000.0f) nav_config.path_points_y[i] = -1000.0f;
        if (nav_config.path_points_y[i] > 1000.0f) nav_config.path_points_y[i] = 1000.0f;
    }
    
    // ���ƿ��ư뾶��Χ
    for (int i = 0; i < 2; i++)
    {
        if (nav_config.control_radius[i] < 1) nav_config.control_radius[i] = 1;
        if (nav_config.control_radius[i] > 100) nav_config.control_radius[i] = 100;
    }
    
    // ���Ƶ���PID������Χ
    if (nav_config.nav_pid_kp < 0.0f) nav_config.nav_pid_kp = 0.0f;
    if (nav_config.nav_pid_kp > 100.0f) nav_config.nav_pid_kp = 100.0f;
    if (nav_config.nav_pid_ki < 0.0f) nav_config.nav_pid_ki = 0.0f;
    if (nav_config.nav_pid_ki > 100.0f) nav_config.nav_pid_ki = 100.0f;
    if (nav_config.nav_pid_kd < 0.0f) nav_config.nav_pid_kd = 0.0f;
    if (nav_config.nav_pid_kd > 100.0f) nav_config.nav_pid_kd = 100.0f;
    
    // �����ٶȲ�����Χ
    if (nav_config.nav_pid_speed < 0) nav_config.nav_pid_speed = 0;
    if (nav_config.nav_pid_speed > 500) nav_config.nav_pid_speed = 500;
    if (nav_config.target_speed < 0) nav_config.target_speed = 0;
    if (nav_config.target_speed > 500) nav_config.target_speed = 500;
}

//=================================================�������ݹ�����================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ��������ϵͳ��ʼ��
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_data_init(void)
{
    // ��ʼ�����ݽṹ��
    memset(&nav_data, 0, sizeof(nav_data_struct));
    
    // ��ʼ������
    memset(nav_errors_coords, 0, sizeof(nav_errors_coords));
    memset(nav_mileage_list, 0, sizeof(nav_mileage_list));
    memset(nav_yaw_buffer, 0, sizeof(nav_yaw_buffer));
    
    // ���ó�ʼֵ
    nav_data.max_error_point_count = NAV_COORD_RECORD;
    
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ��������ָ��
//-------------------------------------------------------------------------------------------------------------------
nav_data_struct* nav_data_get_pointer(void)
{
    return &nav_data;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ���õ�������
//-------------------------------------------------------------------------------------------------------------------
void nav_data_reset(void)
{
    // ���������������
    int max_points = nav_data.max_error_point_count;
    
    // �������ݽṹ��
    memset(&nav_data, 0, sizeof(nav_data_struct));
    
    // �ָ������������
    nav_data.max_error_point_count = max_points;
    
    // ��������
    memset(nav_errors_coords, 0, sizeof(nav_errors_coords));
    memset(nav_mileage_list, 0, sizeof(nav_mileage_list));
    // ע�⣺yaw_buffer �����㣬������Ҫ������ʷ����
}

//=================================================�������PID�������ʺ���================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ�������PID����
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_config_get_motor_pid(uint8 motor_id, uint8 pid_type, float *kp, float *ki, float *kd)
{
    if (kp == NULL || ki == NULL || kd == NULL)
    {
        return 1;
    }
    
    if (motor_id == 0)  // ����
    {
        if (pid_type == 0)  // �ٶȻ�
        {
            *kp = nav_config.motor_control.left_speed_kp;
            *ki = nav_config.motor_control.left_speed_ki;
            *kd = nav_config.motor_control.left_speed_kd;
        }
        else if (pid_type == 1)  // λ�û�
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
    else if (motor_id == 1)  // �ҵ��
    {
        if (pid_type == 0)  // �ٶȻ�
        {
            *kp = nav_config.motor_control.right_speed_kp;
            *ki = nav_config.motor_control.right_speed_ki;
            *kd = nav_config.motor_control.right_speed_kd;
        }
        else if (pid_type == 1)  // λ�û�
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
// �������     ���õ������PID����
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_config_set_motor_pid(uint8 motor_id, uint8 pid_type, float kp, float ki, float kd)
{
    if (motor_id == 0)  // ����
    {
        if (pid_type == 0)  // �ٶȻ�
        {
            nav_config.motor_control.left_speed_kp = kp;
            nav_config.motor_control.left_speed_ki = ki;
            nav_config.motor_control.left_speed_kd = kd;
        }
        else if (pid_type == 1)  // λ�û�
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
    else if (motor_id == 1)  // �ҵ��
    {
        if (pid_type == 0)  // �ٶȻ�
        {
            nav_config.motor_control.right_speed_kp = kp;
            nav_config.motor_control.right_speed_ki = ki;
            nav_config.motor_control.right_speed_kd = kd;
        }
        else if (pid_type == 1)  // λ�û�
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
    
    // ����������޸�
    nav_config.config_modified = 1;
    
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ������Ʋ���
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
// �������     ���õ�����Ʋ���
//-------------------------------------------------------------------------------------------------------------------
void nav_config_set_motor_params(float max_speed, float max_accel, float wheelbase, int control_frequency)
{
    nav_config.motor_control.max_speed = max_speed;
    nav_config.motor_control.max_acceleration = max_accel;
    nav_config.motor_control.wheelbase = wheelbase;
    nav_config.motor_control.control_frequency = control_frequency;
    
    // ����������޸�
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
