/*********************************************************************************************************************
* 文件名称          driver_photoelectric.h
* 功能说明          光电管阵列驱动程序头文件
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本
* 2025-11-05        LittleMaster       1.0v
* 
* 文件作用说明：
* 本文件是18路光电管阵列的驱动程序头文件
* 
* 硬件接线（根据光电管接口原理图）：
*     通道     MCU引脚      功能
*     CH0      P13.0        光电管0
*     CH1      P13.1        光电管1
*     CH2      P13.2        光电管2
*     CH3      P13.3        光电管3
*     CH4      P13.4        光电管4
*     CH5      P13.5        光电管5
*     CH6      P13.6        光电管6
*     CH7      P13.7        光电管7
*     CH8      P14.0        光电管8
*     CH9      P14.1        光电管9
*     CH10     P14.2        光电管10
*     CH11     P14.3        光电管11
*     CH12     P14.4        光电管12
*     CH13     P14.5        光电管13
*     CH14     P15.0        光电管14
*     CH15     P15.1        光电管15
*     CH16     P15.2        光电管16
*     CH17     P15.3        光电管17
********************************************************************************************************************/

#ifndef _DRIVER_PHOTOELECTRIC_H_
#define _DRIVER_PHOTOELECTRIC_H_

#include "zf_common_typedef.h"

//=================================================硬件引脚定义================================================
// 根据光电管接口原理图（18路，通道0-17）
#define PHOTO_CH0_PIN           (P13_0)                 // 光电管通道0
#define PHOTO_CH1_PIN           (P13_1)                 // 光电管通道1
#define PHOTO_CH2_PIN           (P13_2)                 // 光电管通道2
#define PHOTO_CH3_PIN           (P13_3)                 // 光电管通道3
#define PHOTO_CH4_PIN           (P13_4)                 // 光电管通道4
#define PHOTO_CH5_PIN           (P13_5)                 // 光电管通道5
#define PHOTO_CH6_PIN           (P13_6)                 // 光电管通道6
#define PHOTO_CH7_PIN           (P13_7)                 // 光电管通道7
#define PHOTO_CH8_PIN           (P14_0)                 // 光电管通道8
#define PHOTO_CH9_PIN           (P14_1)                 // 光电管通道9
#define PHOTO_CH10_PIN          (P14_2)                 // 光电管通道10
#define PHOTO_CH11_PIN          (P14_3)                 // 光电管通道11
#define PHOTO_CH12_PIN          (P14_4)                 // 光电管通道12
#define PHOTO_CH13_PIN          (P14_5)                 // 光电管通道13
#define PHOTO_CH14_PIN          (P15_0)                 // 光电管通道14
#define PHOTO_CH15_PIN          (P15_1)                 // 光电管通道15
#define PHOTO_CH16_PIN          (P15_2)                 // 光电管通道16
#define PHOTO_CH17_PIN          (P15_3)                 // 光电管通道17

//=================================================配置参数================================================
#define PHOTO_CHANNEL_NUM       (18)                    // 光电管通道数量
#define PHOTO_BLACK_THRESHOLD   (1)                     // 黑线阈值（1=检测到黑线）
#define PHOTO_WHITE_THRESHOLD   (0)                     // 白线阈值（0=检测到白线）

//=================================================数据结构定义================================================
// 光电管数据结构
typedef struct
{
    uint8 raw_data[PHOTO_CHANNEL_NUM];                  // 原始数据（0或1）
    uint32 raw_value;                                   // 原始值（位图形式，32位）
    float line_position;                                // 线位置（-1.0 ~ 1.0，0为中心）
    uint8 line_width;                                   // 线宽（检测到的连续黑线数量）
    uint8 valid;                                        // 数据有效标志
} photoelectric_data_struct;

//=================================================函数声明================================================
// 基础控制函数
void    photoelectric_init                  (void);
void    photoelectric_read                  (void);
uint8   photoelectric_get_channel           (uint8 channel);
uint32  photoelectric_get_raw_value         (void);
void    photoelectric_get_all_channels      (uint8 *data);

// 数据处理函数
float   photoelectric_get_line_position     (void);
uint8   photoelectric_get_line_width        (void);
uint8   photoelectric_is_valid              (void);
void    photoelectric_calculate_position    (void);

// 调试函数
void    photoelectric_print_status          (void);

// 全局变量声明
extern photoelectric_data_struct photoelectric_data;

#endif // _DRIVER_PHOTOELECTRIC_H_

