/*********************************************************************************************************************
* �ļ�����          config_navigation.h
* ����˵��          ����ϵͳ���ò����ļ�
* ����              LittleMaster
* �汾��Ϣ          v1.0
* �޸ļ�¼
* ����              ����                �汾              ��ע
* 2025-09-18        LittleMaster        v1.0              ��������ϵͳ�����ļ�
*
* �ļ�����˵����
* ���ļ����й�����ϵͳ�����п����ò���������·�������ꡢ���Ʋ�����ϵͳ���õ�
* ͨ��ͳһ�����ù�������ģ����ѭ����������ߴ���Ŀ�ά����
*
* ���÷��ࣺ
* 1. ·�����������ã�X1-X6, Y1-Y6 ·���ؼ�������
* 2. ���Ʋ������ã�R1, R2 ���ư뾶����
* 3. PID�������ã�Kp, Ki, Kd ����������
* 4. ϵͳ�������ã��ٶȡ���̡��Ƕȵ�ϵͳ����
* 5. Flash�洢���ã��洢ҳ�桢���ݷ�Χ������
* 6. UI��ʾ���ã���ʾģʽ��������Ӧ������
*
* ʹ�÷�ʽ��
* 1. ����Ҫʹ�����õ��ļ��а�����ͷ�ļ�
* 2. ͨ���궨���ȫ�ֱ����������ò���
* 3. ����ʱ��ͨ��UI�����޸Ŀɱ����
* 4. ϵͳ�������Զ�����Ĭ������
********************************************************************************************************************/

#ifndef _CONFIG_NAVIGATION_H_
#define _CONFIG_NAVIGATION_H_

#include "zf_common_typedef.h"

//=================================================·������������================================================
// Ĭ��·�������꣨��ͨ��UI���������
#define NAV_DEFAULT_X1              (0.0f)      // ·����1 X����
#define NAV_DEFAULT_Y1              (10.0f)     // ·����1 Y����
#define NAV_DEFAULT_X2              (0.0f)      // ·����2 X����
#define NAV_DEFAULT_Y2              (10.0f)     // ·����2 Y����
#define NAV_DEFAULT_X3              (0.0f)      // ·����3 X����
#define NAV_DEFAULT_Y3              (20.0f)     // ·����3 Y����
#define NAV_DEFAULT_X4              (0.0f)      // ·����4 X����
#define NAV_DEFAULT_Y4              (20.0f)     // ·����4 Y����
#define NAV_DEFAULT_X5              (0.0f)      // ·����5 X����
#define NAV_DEFAULT_Y5              (20.0f)     // ·����5 Y����
#define NAV_DEFAULT_X6              (0.0f)      // ·����6 X����
#define NAV_DEFAULT_Y6              (20.0f)     // ·����6 Y����

//=================================================���Ʋ�������================================================
#define NAV_DEFAULT_R1              (30)        // ���ư뾶1
#define NAV_DEFAULT_R2              (30)        // ���ư뾶2

//=================================================PID��������================================================
// ��Ҫ·������PID���� (PIDG - ��������)
#define NAV_DEFAULT_PID_KP          (24.3f)     // PID����ϵ�� (��Ҫ·������)
#define NAV_DEFAULT_PID_KI          (0.0f)      // PID����ϵ��
#define NAV_DEFAULT_PID_KD          (29.5f)     // PID΢��ϵ��
#define NAV_DEFAULT_PID_SPEED       (100)       // PIDĬ���ٶ�

// ����ٶȻ�PID���� (PIDS - �ٶȿ���)
#define MOTOR_SPEED_PID_KP          (30.0f)     // �ٶȻ�����ϵ��
#define MOTOR_SPEED_PID_KI          (2.4f)      // �ٶȻ�����ϵ��
#define MOTOR_SPEED_PID_KD          (0.0f)      // �ٶȻ�΢��ϵ��

// ת�����PID���� (PIDD - �������)
#define DIRECTION_PID_KP            (12.9f)     // ������Ʊ���ϵ��
#define DIRECTION_PID_KI            (0.0f)      // ������ƻ���ϵ��
#define DIRECTION_PID_KD            (13.6f)     // �������΢��ϵ��

// ֱ����ʻPID���� (PID1 - ֱ�߷���)
#define STRAIGHT_LINE_PID_KP        (10.0f)     // ֱ�߷������ϵ��
#define STRAIGHT_LINE_PID_KI        (0.0f)      // ֱ�߷������ϵ��
#define STRAIGHT_LINE_PID_KD        (10.0f)     // ֱ�߷���΢��ϵ��

// �����ٶ�PID���� (PIDSL - �����ٶȣ��̳�PIDS)
#define LEFT_WHEEL_SPEED_PID_KP     (30.0f)     // �����ٶȱ���ϵ�� (�̳�PIDS)
#define LEFT_WHEEL_SPEED_PID_KI     (2.4f)      // �����ٶȻ���ϵ�� (�̳�PIDS)
#define LEFT_WHEEL_SPEED_PID_KD     (0.0f)      // �����ٶ�΢��ϵ�� (�̳�PIDS)

//=================================================ϵͳ��������================================================
#define NAV_DEFAULT_TARGET_SPEED    (0)         // Ĭ��Ŀ���ٶ�
#define NAV_FUYA_DEFAULT            (0)         // ��������Ĭ��״̬

//=================================================Flash�洢����================================================
#define NAV_FLASH_MAX_SIZE          (500)       // Flash�洢�����ҳ��
#define NAV_COORD_RECORD            (25000)     // �����¼����
#define NAV_FLASH_END_PAGE          (1)         // Flash��ֹҳ��
#define NAV_FLASH_START_PAGE        (40)        // Flash��ʼҳ��
#define NAV_SET_MILEAGE             (400)       // ������嵱����400����/����

//=================================================��ѧ��������================================================
#ifndef M_PI
#define M_PI                        (3.1415926535897932384626433832795f)
#endif
#define NAV_DEG_TO_RAD(angle)       ((angle) * M_PI / 180.0f)
#define NAV_RAD_TO_DEG(angle)       ((angle) * 180.0f / M_PI)

//=================================================UI��������================================================
#define NAV_UI_REFRESH_RATE         (50)        // UIˢ���� (ms)
#define NAV_KEY_DEBOUNCE_TIME       (20)        // ��������ʱ�� (ms)

//=================================================���ݽṹ����================================================
// ����ϵͳ���ýṹ��
typedef struct
{
    // ·��������
    float path_points_x[6];                     // X�������� [X1-X6]
    float path_points_y[6];                     // Y�������� [Y1-Y6]
    
    // ���Ʋ���
    int control_radius[2];                      // ���ư뾶 [R1, R2]
    
    // ����PID����
    float nav_pid_kp;                           // ����PID����ϵ��
    float nav_pid_ki;                           // ����PID����ϵ��
    float nav_pid_kd;                           // ����PID΢��ϵ��
    int nav_pid_speed;                          // ����PID�ٶ�
    
    // �������PID����
    struct {
        // �����ٶȻ�PID
        float left_speed_kp;                    // �����ٶȻ�����ϵ��
        float left_speed_ki;                    // �����ٶȻ�����ϵ��
        float left_speed_kd;                    // �����ٶȻ�΢��ϵ��
        
        // ����λ�û�PID
        float left_position_kp;                 // ����λ�û�����ϵ��
        float left_position_ki;                 // ����λ�û�����ϵ��
        float left_position_kd;                 // ����λ�û�΢��ϵ��
        
        // �ҵ���ٶȻ�PID
        float right_speed_kp;                   // �ҵ���ٶȻ�����ϵ��
        float right_speed_ki;                   // �ҵ���ٶȻ�����ϵ��
        float right_speed_kd;                   // �ҵ���ٶȻ�΢��ϵ��
        
        // �ҵ��λ�û�PID
        float right_position_kp;                // �ҵ��λ�û�����ϵ��
        float right_position_ki;                // �ҵ��λ�û�����ϵ��
        float right_position_kd;                // �ҵ��λ�û�΢��ϵ��
        
        // ������Ʋ���
        float max_speed;                        // ����ٶ����� (m/s)
        float max_acceleration;                 // �����ٶ����� (m/s?)
        float wheelbase;                        // �־� (mm)
        int control_frequency;                  // ����Ƶ�� (Hz)
        
    } motor_control;
    
    // ϵͳ���в���
    int target_speed;                           // Ŀ���ٶ�
    uint8 fuya_enable;                          // ��������ʹ��
    uint8 go_flag;                              // ������־
    uint8 go_state;                             // ����״̬
    
    // ϵͳ״̬
    uint8 config_loaded;                        // �����Ѽ��ر�־
    uint8 config_modified;                      // �������޸ı�־
    
} nav_config_struct;

// ����ϵͳ���ݽṹ��
typedef struct
{
    // ����������
    int encoder_sum;                            // �������ܺ�
    int encoder_left;                           // �������
    
    // ·������
    int max_error_point_count;                  // �����������
    int actual_error_point_count;               // ʵ�ʴ��������
    int current_error_point_index;              // ��ǰ���������
    
    // �������
    int total_mileage;                          // �����
    int last_mileage;                           // �ϴ����
    
    // �ǶȺͷ�������
    float last_yaw_angle;                       // �ϴ�ƫ����
    float current_yaw_angle;                    // ��ǰƫ����
    float error_direction;                      // ������
    int error_angle_direction;                  // ����Ƕȷ���
    
    // ��������
    float curvature;                            // ��ǰ����
    float curvature_straight;                   // ֱ������
    
    // ״̬��־
    uint8 error_make_flag;                      // �������ɱ�־
    uint8 curvature_threshold_flag;             // ������ֵ��־
    uint8 straight_angle_flag;                  // ֱ�Ǳ�־
    uint8 last_straight_flag;                   // �ϴ�ֱ�Ǳ�־
    
    // ״̬������
    int positive_reset_state;                   // ����������״̬��
    int negative_reset_state;                   // ����������״̬��
    int continuous_straight_flag;               // ����ֱ�Ǳ�־
    
    // ��ֵ״̬
    bool was_high_threshold;                    // �Ƿ�����������ֵ
    bool was_low_threshold;                     // �Ƿ�����������ֵ
    bool high_to_mid_reset;                     // �ߵ��е����ñ�־
    bool mid_to_low_cancel;                     // �е��͵�ȡ����־
    bool low_to_mid_reset;                      // �͵��е����ñ�־
    bool mid_to_high_cancel;                    // �е��ߵ�ȡ����־
    
    // ʱ�����
    uint8 time_counter_1;                       // ʱ�������1
    uint8 time_counter_2;                       // ʱ�������2
    
    // ���㻺��
    int calculation_buffer;                     // ���㻺��
    
} nav_data_struct;

//=================================================ȫ�ֱ�������================================================
extern nav_config_struct nav_config;           // ����ϵͳ����
extern nav_data_struct nav_data;               // ����ϵͳ����

// ������������
extern float nav_errors_coords[NAV_COORD_RECORD];       // ������������
extern int nav_mileage_list[NAV_COORD_RECORD];          // ����б�����
extern int32 nav_yaw_buffer[4000];                     // ƫ���ǻ�������

//=================================================��������================================================
//-------------------------------------------------------------------------------------------------------------------
// �������     ��������ϵͳ��ʼ��
// ����˵��     void
// ���ز���     uint8                          ��ʼ����� (0=�ɹ�, 1=ʧ��)
// ʹ��ʾ��     nav_config_init();
// ��ע��Ϣ     ����Ĭ�����ò�������ʼ������ϵͳ
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_config_init(void);

//-------------------------------------------------------------------------------------------------------------------
// �������     ����Ĭ������
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     nav_config_load_default();
// ��ע��Ϣ     �ָ����в�����Ĭ��ֵ
//-------------------------------------------------------------------------------------------------------------------
void nav_config_load_default(void);

//-------------------------------------------------------------------------------------------------------------------
// �������     ���浱ǰ���õ�Flash
// ����˵��     void
// ���ز���     uint8                          ������ (0=�ɹ�, 1=ʧ��)
// ʹ��ʾ��     nav_config_save();
// ��ע��Ϣ     ����ǰ���ò������浽Flash�洢
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_config_save(void);

//-------------------------------------------------------------------------------------------------------------------
// �������     ��Flash��������
// ����˵��     void
// ���ز���     uint8                          ���ؽ�� (0=�ɹ�, 1=ʧ��)
// ʹ��ʾ��     nav_config_load();
// ��ע��Ϣ     ��Flash�洢�������ò���
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_config_load(void);

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ·��������
// ����˵��     point_index                    ·�������� (0-5 ��Ӧ X1-X6)
// ����˵��     x                              X����ָ��
// ����˵��     y                              Y����ָ��
// ���ز���     uint8                          ��ȡ��� (0=�ɹ�, 1=ʧ��)
// ʹ��ʾ��     nav_config_get_path_point(0, &x1, &y1);
// ��ע��Ϣ     ��ȡָ��·���������ֵ
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_config_get_path_point(uint8 point_index, float *x, float *y);

//-------------------------------------------------------------------------------------------------------------------
// �������     ����·��������
// ����˵��     point_index                    ·�������� (0-5 ��Ӧ X1-X6)
// ����˵��     x                              X����ֵ
// ����˵��     y                              Y����ֵ
// ���ز���     uint8                          ���ý�� (0=�ɹ�, 1=ʧ��)
// ʹ��ʾ��     nav_config_set_path_point(0, 10.5f, 20.3f);
// ��ע��Ϣ     ����ָ��·���������ֵ
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_config_set_path_point(uint8 point_index, float x, float y);

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ���ư뾶����
// ����˵��     radius_index                   �뾶���� (0=R1, 1=R2)
// ���ز���     int                            �뾶ֵ
// ʹ��ʾ��     r1 = nav_config_get_control_radius(0);
// ��ע��Ϣ     ��ȡָ�����ư뾶����
//-------------------------------------------------------------------------------------------------------------------
int nav_config_get_control_radius(uint8 radius_index);

//-------------------------------------------------------------------------------------------------------------------
// �������     ���ÿ��ư뾶����
// ����˵��     radius_index                   �뾶���� (0=R1, 1=R2)
// ����˵��     radius                         �뾶ֵ
// ���ز���     uint8                          ���ý�� (0=�ɹ�, 1=ʧ��)
// ʹ��ʾ��     nav_config_set_control_radius(0, 35);
// ��ע��Ϣ     ����ָ�����ư뾶����
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_config_set_control_radius(uint8 radius_index, int radius);

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡPID����
// ����˵��     kp                             ����ϵ��ָ��
// ����˵��     ki                             ����ϵ��ָ��
// ����˵��     kd                             ΢��ϵ��ָ��
// ����˵��     speed                          �ٶ�ָ��
// ���ز���     void
// ʹ��ʾ��     nav_config_get_pid(&kp, &ki, &kd, &speed);
// ��ע��Ϣ     ��ȡPID����������
//-------------------------------------------------------------------------------------------------------------------
void nav_config_get_pid(float *kp, float *ki, float *kd, int *speed);

//-------------------------------------------------------------------------------------------------------------------
// �������     ����PID����
// ����˵��     kp                             ����ϵ��
// ����˵��     ki                             ����ϵ��
// ����˵��     kd                             ΢��ϵ��
// ����˵��     speed                          �ٶ�
// ���ز���     void
// ʹ��ʾ��     nav_config_set_pid(1.2f, 0.1f, 0.05f, 120);
// ��ע��Ϣ     ����PID����������
//-------------------------------------------------------------------------------------------------------------------
void nav_config_set_pid(float kp, float ki, float kd, int speed);

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ�������PID����
// ����˵��     motor_id                       ���ID (0=����, 1=�ҵ��)
// ����˵��     pid_type                       PID���� (0=�ٶȻ�, 1=λ�û�)
// ����˵��     kp                             ����ϵ��ָ��
// ����˵��     ki                             ����ϵ��ָ��
// ����˵��     kd                             ΢��ϵ��ָ��
// ���ز���     uint8                          ��ȡ��� (0=�ɹ�, 1=ʧ��)
// ʹ��ʾ��     nav_config_get_motor_pid(0, 0, &kp, &ki, &kd);
// ��ע��Ϣ     ��ȡָ�������PID����������
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_config_get_motor_pid(uint8 motor_id, uint8 pid_type, float *kp, float *ki, float *kd);

//-------------------------------------------------------------------------------------------------------------------
// �������     ���õ������PID����
// ����˵��     motor_id                       ���ID (0=����, 1=�ҵ��)
// ����˵��     pid_type                       PID���� (0=�ٶȻ�, 1=λ�û�)
// ����˵��     kp                             ����ϵ��
// ����˵��     ki                             ����ϵ��
// ����˵��     kd                             ΢��ϵ��
// ���ز���     uint8                          ���ý�� (0=�ɹ�, 1=ʧ��)
// ʹ��ʾ��     nav_config_set_motor_pid(0, 0, 8.0f, 0.5f, 0.1f);
// ��ע��Ϣ     ����ָ�������PID����������
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_config_set_motor_pid(uint8 motor_id, uint8 pid_type, float kp, float ki, float kd);

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ������Ʋ���
// ����˵��     max_speed                      ����ٶ�ָ�� (m/s)
// ����˵��     max_acceleration               �����ٶ�ָ�� (m/s?)
// ����˵��     wheelbase                      �־�ָ�� (mm)
// ����˵��     control_frequency              ����Ƶ��ָ�� (Hz)
// ���ز���     void
// ʹ��ʾ��     nav_config_get_motor_params(&max_speed, &max_acc, &wheelbase, &freq);
// ��ע��Ϣ     ��ȡ�������ϵͳ����
//-------------------------------------------------------------------------------------------------------------------
void nav_config_get_motor_params(float *max_speed, float *max_acceleration, float *wheelbase, int *control_frequency);

//-------------------------------------------------------------------------------------------------------------------
// �������     ���õ�����Ʋ���
// ����˵��     max_speed                      ����ٶ� (m/s)
// ����˵��     max_acceleration               �����ٶ� (m/s?)
// ����˵��     wheelbase                      �־� (mm)
// ����˵��     control_frequency              ����Ƶ�� (Hz)
// ���ز���     void
// ʹ��ʾ��     nav_config_set_motor_params(2.0f, 5.0f, 150.0f, 1000);
// ��ע��Ϣ     ���õ������ϵͳ����
//-------------------------------------------------------------------------------------------------------------------
void nav_config_set_motor_params(float max_speed, float max_acceleration, float wheelbase, int control_frequency);

//-------------------------------------------------------------------------------------------------------------------
// �������     ��������ϵͳ��ʼ��
// ����˵��     void
// ���ز���     uint8                          ��ʼ����� (0=�ɹ�, 1=ʧ��)
// ʹ��ʾ��     nav_data_init();
// ��ע��Ϣ     ��ʼ���������ݽṹ�������
//-------------------------------------------------------------------------------------------------------------------
uint8 nav_data_init(void);

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ��������ָ��
// ����˵��     void
// ���ز���     nav_data_struct*               �������ݽṹ��ָ��
// ʹ��ʾ��     nav_data_struct *data = nav_data_get_pointer();
// ��ע��Ϣ     ��ȡ�������ݽṹ���ָ�룬����ֱ�ӷ���
//-------------------------------------------------------------------------------------------------------------------
nav_data_struct* nav_data_get_pointer(void);

//-------------------------------------------------------------------------------------------------------------------
// �������     ���õ�������
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     nav_data_reset();
// ��ע��Ϣ     �����е�����������Ϊ��ʼֵ
//-------------------------------------------------------------------------------------------------------------------
void nav_data_reset(void);

// ��ݷ��ʺ궨�壨�����ݣ�

// ���ò������ʺ�
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

// ����PID���Ʋ����������Ժ꣩
#define PIDG                            (nav_config.nav_pid_kp)     // PID����ϵ���������Ժ꣩
#define pid_kp                          (nav_config.nav_pid_kp)
#define pid_ki                          (nav_config.nav_pid_ki)
#define pid_kd                          (nav_config.nav_pid_kd)
#define pid_speed                       (nav_config.nav_pid_speed)

// �������PID�������ʺ�
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

// ���ݱ������ʺ꣨�����ָ�ȡ����һ����
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

// ������ʺ�
#define errors_coords                   nav_errors_coords
#define Mileage_All_sum_list            nav_mileage_list
#define data_yaw_buffer                 nav_yaw_buffer

// ���淶������ӳ�䣨�������
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

//================================================= ȫ�ַ��ʽӿ� =================================================
nav_config_struct* nav_config_get(void);
nav_data_struct* nav_data_get(void);


#endif // _CONFIG_NAVIGATION_H_
