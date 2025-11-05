/*********************************************************************************************************************
* 文件名称          driver_key.h
* 功能说明          按键驱动程序头文件
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本
* 2025-11-05        LittleMaster       1.0v
* 
* 文件作用说明：
* 本文件是按键输入驱动程序头文件
* 
* 硬件接线（根据原理图）：
*     按键组1（左侧）：
*     KEY0     P20.0    通过470Ω电阻
*     KEY1     P1.1     
*     KEY2     P1.0     
*     KEY3     P20.1    通过470Ω电阻
*     
*     按键组2（右侧）：
*     KEY4     P20.2    通过470Ω电阻
*     KEY5     P0.3     
*     KEY6     P1.1     
*     KEY7     P20.3    通过470Ω电阻
********************************************************************************************************************/

#ifndef _DRIVER_KEY_H_
#define _DRIVER_KEY_H_

#include "zf_common_typedef.h"

//=================================================硬件引脚定义================================================
// 按键组1（左侧）
#define KEY0_PIN            (P20_0)                     // 按键0引脚
#define KEY1_PIN            (P01_1)                     // 按键1引脚
#define KEY2_PIN            (P01_0)                     // 按键2引脚
#define KEY3_PIN            (P20_1)                     // 按键3引脚

// 按键组2（右侧）
#define KEY4_PIN            (P20_2)                     // 按键4引脚
#define KEY5_PIN            (P00_3)                     // 按键5引脚
#define KEY6_PIN            (P01_1)                     // 按键6引脚（注意：与KEY1共用）
#define KEY7_PIN            (P20_3)                     // 按键7引脚

//=================================================配置参数================================================
#define KEY_NUM             (8)                         // 按键数量
#define KEY_DEBOUNCE_TIME   (20)                        // 消抖时间（ms）
#define KEY_LONG_PRESS_TIME (1000)                      // 长按时间（ms）

//=================================================按键枚举定义================================================
// 按键ID枚举
typedef enum
{
    KEY_ID_0 = 0,
    KEY_ID_1 = 1,
    KEY_ID_2 = 2,
    KEY_ID_3 = 3,
    KEY_ID_4 = 4,
    KEY_ID_5 = 5,
    KEY_ID_6 = 6,
    KEY_ID_7 = 7,
} key_id_enum;

// 按键状态枚举
typedef enum
{
    KEY_STATE_IDLE = 0,                                 // 空闲状态
    KEY_STATE_PRESS,                                    // 按下
    KEY_STATE_RELEASE,                                  // 释放
    KEY_STATE_LONG_PRESS,                               // 长按
} key_state_enum;

// 按键事件枚举
typedef enum
{
    KEY_EVENT_NONE = 0,                                 // 无事件
    KEY_EVENT_PRESS,                                    // 按下事件
    KEY_EVENT_RELEASE,                                  // 释放事件
    KEY_EVENT_LONG_PRESS,                               // 长按事件
    KEY_EVENT_CLICK,                                    // 单击事件
} key_event_enum;

//=================================================数据结构定义================================================
// 单个按键数据结构
typedef struct
{
    uint8 current_state;                                // 当前电平状态
    uint8 last_state;                                   // 上次电平状态
    key_state_enum state;                               // 按键状态
    key_event_enum event;                               // 按键事件
    uint32 press_time;                                  // 按下时刻
    uint32 release_time;                                // 释放时刻
    uint8 is_long_press;                                // 是否触发了长按
} key_data_struct;

// 按键系统结构
typedef struct
{
    key_data_struct keys[KEY_NUM];                      // 所有按键数据
    uint32 scan_count;                                  // 扫描计数
} key_system_struct;

//=================================================函数声明================================================
// 基础控制函数
void            key_init                    (void);
void            key_scan                    (void);
uint8           key_get_state               (key_id_enum key_id);
key_event_enum  key_get_event               (key_id_enum key_id);
void            key_clear_event             (key_id_enum key_id);

// 判断函数
uint8           key_is_pressed              (key_id_enum key_id);
uint8           key_is_released             (key_id_enum key_id);
uint8           key_is_long_press           (key_id_enum key_id);

// 回调函数类型定义
typedef void (*key_callback_func)(key_id_enum key_id, key_event_enum event);

// 注册回调函数
void            key_register_callback       (key_callback_func callback);

// 调试函数
void            key_print_status            (void);

// 全局变量声明
extern key_system_struct key_system;

#endif // _DRIVER_KEY_H_

