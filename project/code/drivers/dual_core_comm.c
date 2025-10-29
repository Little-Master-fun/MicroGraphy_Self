/*********************************************************************************************************************
* �ļ�����          dual_core_comm.c
* ����˵��          ˫��ͨ��ʵ��
* ����              AI Assistant
* �汾��Ϣ          v1.0
********************************************************************************************************************/

#include "dual_core_comm.h"
#include "zf_driver_ipc.h"
#include "imu_ahrs_complementary.h"
#include "driver_encoder.h"
#include "motor_control.h"
#include "nav_control_ahrs.h"
#include <string.h>

//=================================================�����ڴ����================================================

// ʹ��volatileȷ�������������Ż�������Щ�����ķ���
SHARED_DATA_SECTION volatile core0_to_core1_data_t shared_core0_data = {0};
SHARED_DATA_SECTION volatile core1_to_core0_data_t shared_core1_data = {0};
SHARED_DATA_SECTION volatile uint8 core0_data_ready = 0;
SHARED_DATA_SECTION volatile uint8 core1_data_ready = 0;

//=================================================���ر���================================================

static uint32 core0_timestamp = 0;
static uint32 core1_timestamp = 0;

//=================================================CM7_0���ĺ���================================================

/**
 * @brief  CM7_0���ĳ�ʼ��
 */
void dual_core_comm_init_core0(void)
{
    // ��չ�������
    memset((void*)&shared_core0_data, 0, sizeof(core0_to_core1_data_t));
    memset((void*)&shared_core1_data, 0, sizeof(core1_to_core0_data_t));
    
    core0_data_ready = 0;
    core1_data_ready = 0;
    core0_timestamp = 0;
    
    // ��ʼ��IPCͨ�ţ�CM7_0ʹ��IPC_PORT_1��
    ipc_communicate_init(IPC_PORT_1, dual_core_ipc_callback_core0);
    
    printf("[CORE0] ˫��ͨ�ų�ʼ�����\n");
}

/**
 * @brief  ���´��������ݵ������ڴ棨CM7_0���ã�
 */
void dual_core_update_sensor_data(void)
{
    core0_timestamp++;
    
    // ��ȡAHRS��̬����
    ahrs_euler_angles_t euler;
    ahrs_get_euler_angles(&euler);
    
    shared_core0_data.yaw = euler.yaw;
    shared_core0_data.pitch = euler.pitch;
    shared_core0_data.roll = euler.roll;
    
    // ��ȡ���������ݣ����ٶȣ�
    // ע�⣺gyro_z��Ҫ��IMUԭʼ�����л�ȡ��������ʱ��Ϊ0��ʵ��ʹ��ʱ��Ҫ����
    shared_core0_data.gyro_z = 0.0f;  // TODO: ��IMU��ȡZ����ٶ�
    
    // ��ȡ����������
    shared_core0_data.encoder_left = encoder_get_pulse(ENCODER_ID_LEFT);
    shared_core0_data.encoder_right = encoder_get_pulse(ENCODER_ID_RIGHT);
    
    // ��ȡ����λ�����ݣ�����У�
    if (nav_ahrs.initialized) {
        shared_core0_data.distance = nav_ahrs.current_distance;
        // λ��������Ҫͨ����̼Ƽ��㣬����ʹ���ۻ�����
        shared_core0_data.position_x = 0.0f;  // TODO: ���λ�ü���
        shared_core0_data.position_y = 0.0f;
        shared_core0_data.velocity = (nav_ahrs.left_speed + nav_ahrs.right_speed) / 2.0f;
    }
    
    shared_core0_data.data_valid = 1;
    shared_core0_data.timestamp = core0_timestamp;
    
    // ������ݾ���
    core0_data_ready = 1;
    
    // ͨ��IPC֪ͨCM7_1
    ipc_send_data(IPC_MSG_SENSOR_DATA_READY);
}

/**
 * @brief  �ӹ����ڴ��ȡ�������ݣ�CM7_0���ã�
 */
void dual_core_read_control_data(void)
{
    if (core1_data_ready && shared_core1_data.data_valid) {
        // ��ȡĿ���ٶ�
        float left_speed = shared_core1_data.target_speed_left;
        float right_speed = shared_core1_data.target_speed_right;
        
        // Ӧ�õ��������
        motor_set_target_speed(left_speed, right_speed);
        
        // ���������־
        core1_data_ready = 0;
    }
}

/**
 * @brief  CM7_0 IPC�ص�����
 */
void dual_core_ipc_callback_core0(uint32 ipc_data)
{
    switch(ipc_data) {
        case IPC_MSG_CONTROL_DATA_READY:
            // CM7_1�������µĿ�������
            core1_data_ready = 1;
            break;
            
        case IPC_MSG_CORE1_INIT_DONE:
            printf("[CORE0] �յ�CM7_1��ʼ�������Ϣ\n");
            break;
            
        default:
            break;
    }
}

//=================================================CM7_1���ĺ���================================================

/**
 * @brief  CM7_1���ĳ�ʼ��
 */
void dual_core_comm_init_core1(void)
{
    core1_timestamp = 0;
    
    // ��ʼ��IPCͨ�ţ�CM7_1ʹ��IPC_PORT_2��
    ipc_communicate_init(IPC_PORT_2, dual_core_ipc_callback_core1);
    
    printf("[CORE1] ˫��ͨ�ų�ʼ�����\n");
    
    // ֪ͨCM7_0��ʼ�����
    ipc_send_data(IPC_MSG_CORE1_INIT_DONE);
}

/**
 * @brief  �ӹ����ڴ��ȡ���������ݣ�CM7_1���ã�
 */
void dual_core_read_sensor_data(void)
{
    if (core0_data_ready && shared_core0_data.data_valid) {
        // �����Ѿ��ڹ����ڴ��У�����ֱ�ӷ���
        // ���ﲻ��Ҫ��������������㷨��ֱ�Ӷ�ȡshared_core0_data
        
        // ���������־
        core0_data_ready = 0;
    }
}

/**
 * @brief  ���¿������ݵ������ڴ棨CM7_1���ã�
 */
void dual_core_update_control_data(void)
{
    core1_timestamp++;
    
    // �ӵ���ϵͳ��ȡĿ���ٶȣ�ֱ�Ӷ�ȡnav_ahrs�ṹ�壩
    shared_core1_data.target_speed_left = nav_ahrs.left_speed;
    shared_core1_data.target_speed_right = nav_ahrs.right_speed;
    
    // ����״̬��Ϣ
    shared_core1_data.control_mode = 2;  // �Զ�����ģʽ
    shared_core1_data.nav_running = (nav_ahrs.mode == NAV_AHRS_MODE_REPLAY);
    shared_core1_data.current_path_index = nav_ahrs.path.current_index;
    
    shared_core1_data.data_valid = 1;
    shared_core1_data.timestamp = core1_timestamp;
    
    // ������ݾ���
    core1_data_ready = 1;
    
    // ͨ��IPC֪ͨCM7_0
    ipc_send_data(IPC_MSG_CONTROL_DATA_READY);
}

/**
 * @brief  CM7_1 IPC�ص�����
 */
void dual_core_ipc_callback_core1(uint32 ipc_data)
{
    switch(ipc_data) {
        case IPC_MSG_SENSOR_DATA_READY:
            // CM7_0�������µĴ���������
            core0_data_ready = 1;
            break;
            
        case IPC_MSG_NAV_START:
            printf("[CORE1] �յ���ʼ������Ϣ\n");
            break;
            
        case IPC_MSG_NAV_STOP:
            printf("[CORE1] �յ�ֹͣ������Ϣ\n");
            break;
            
        default:
            break;
    }
}

//=================================================��������================================================

/**
 * @brief  ��ȡ���������ݣ���CM7_1�����㷨ʹ�ã�
 */
void dual_core_get_sensor_data_for_nav(float* yaw, float* gyro_z, float* velocity)
{
    if (shared_core0_data.data_valid) {
        if (yaw) *yaw = shared_core0_data.yaw;
        if (gyro_z) *gyro_z = shared_core0_data.gyro_z;
        if (velocity) *velocity = shared_core0_data.velocity;
    }
}


