/*********************************************************************************************************************
* �ļ�����          dual_core_comm.h
* ����˵��          ˫��ͨ�����ݽṹ����
* ����              AI Assistant
* �汾��Ϣ          v1.0
* �޸ļ�¼
* ����              ����                �汾
* 2025-01-XX        AI Assistant       1.0v
* 
* �ļ�����˵����
* ���ļ�����CM7_0��CM7_1����֮��Ĺ������ݽṹ
* CM7_0�������ݲɼ��͵ײ����
* CM7_1���𵼺��㷨�;���
********************************************************************************************************************/

#ifndef _DUAL_CORE_COMM_H_
#define _DUAL_CORE_COMM_H_

#include "zf_common_typedef.h"

//=================================================�������ݽṹ================================================

// ��CM7_0���͵�CM7_1�����ݣ����������ݣ�
typedef struct
{
    // ��̬���ݣ�����AHRS��
    float yaw;              // ƫ���� (��)
    float pitch;            // ������ (��)
    float roll;             // ����� (��)
    
    // ���ٶȣ����������ǣ�
    float gyro_z;           // Z����ٶ� (��/��)
    
    // λ�����ݣ����Ա�������
    float position_x;       // Xλ�� (mm)
    float position_y;       // Yλ�� (mm)
    float distance;         // �ۼƾ��� (mm)
    
    // �ٶ�����
    float velocity;         // ��ǰ�ٶ� (m/s)
    
    // ����������
    int32 encoder_left;     // �����������
    int32 encoder_right;    // �ұ���������
    
    // ������Ч��־
    uint8 data_valid;       // ������Ч�Ա�־
    uint32 timestamp;       // ʱ���
    
} core0_to_core1_data_t;

// ��CM7_1���͵�CM7_0�����ݣ�����ָ�
typedef struct
{
    // Ŀ���ٶȣ����Ե����㷨��
    float target_speed_left;    // ����Ŀ���ٶ� (m/s)
    float target_speed_right;   // ����Ŀ���ٶ� (m/s)
    
    // ����ģʽ
    uint8 control_mode;         // 0=ֹͣ, 1=�ֶ�, 2=�Զ�����
    
    // ����״̬
    uint8 nav_running;          // �����Ƿ�������
    uint16 current_path_index;  // ��ǰ·��������
    
    // ������Ч��־
    uint8 data_valid;           // ������Ч�Ա�־
    uint32 timestamp;           // ʱ���
    
} core1_to_core0_data_t;

//=================================================�����ڴ���================================================

// ʹ�����Խ����ݷ����ڹ����ڴ�����
// CYT4BB������CM7���Ŀ��Է�����ͬ��RAM����

#if defined(__ICCARM__)
    // IAR������
    #define SHARED_DATA_SECTION __attribute__((section(".shared_data")))
#elif defined(__GNUC__)
    // GCC������
    #define SHARED_DATA_SECTION __attribute__((section(".shared_data")))
#else
    #define SHARED_DATA_SECTION
#endif

// �����ڴ�������������Ķ����Է��ʣ�
extern volatile core0_to_core1_data_t shared_core0_data;  // CM7_0д�룬CM7_1��ȡ
extern volatile core1_to_core0_data_t shared_core1_data;  // CM7_1д�룬CM7_0��ȡ

// ����ͬ����־
extern volatile uint8 core0_data_ready;  // CM7_0���ݾ�����־
extern volatile uint8 core1_data_ready;  // CM7_1���ݾ�����־

//=================================================�ӿں���================================================

// CM7_0����ʹ�õĺ���
void dual_core_comm_init_core0(void);                           // CM7_0��ʼ��
void dual_core_update_sensor_data(void);                        // ���´��������ݵ������ڴ�
void dual_core_read_control_data(void);                         // �ӹ����ڴ��ȡ��������
void dual_core_ipc_callback_core0(uint32 ipc_data);            // CM7_0 IPC�ص�����

// CM7_1����ʹ�õĺ���
void dual_core_comm_init_core1(void);                           // CM7_1��ʼ��
void dual_core_read_sensor_data(void);                          // �ӹ����ڴ��ȡ����������
void dual_core_update_control_data(void);                       // ���¿������ݵ������ڴ�
void dual_core_ipc_callback_core1(uint32 ipc_data);            // CM7_1 IPC�ص�����

//=================================================IPC��Ϣ����================================================

// IPC��Ϣ����
#define IPC_MSG_SENSOR_DATA_READY       0x0001  // ���������ݾ���
#define IPC_MSG_CONTROL_DATA_READY      0x0002  // �������ݾ���
#define IPC_MSG_CORE1_INIT_DONE         0x0010  // CM7_1��ʼ�����
#define IPC_MSG_NAV_START               0x0020  // ��ʼ����
#define IPC_MSG_NAV_STOP                0x0030  // ֹͣ����

#endif // _DUAL_CORE_COMM_H_


