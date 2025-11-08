/*********************************************************************************************************************
* 文件名称          driver_screen.h
* 功能说明          2.54寸TFT屏幕驱动程序头文件 (PZ254V-11-08P)
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本
* 2025-11-05        LittleMaster       1.0v
* 
* 文件作用说明：
* 本文件是2.54寸SPI TFT屏幕的驱动程序头文件
* 
* 硬件接线（根据原理图 H9）：
*     屏幕引脚          MCU引脚           功能
*     引脚1            VCC3V3            电源3.3V
*     引脚2            P12.1             SPI_MOSI (数据)
*     引脚3            P12.4             SPI_SCK (时钟)
*     引脚4            P22.3             DC (数据/命令选择)
*     引脚5            P12.3             CS (片选)
*     引脚6            (未连接)          -
*     引脚7            P11.0             RST (复位) 或 BL (背光)
*     引脚8            GND               地
********************************************************************************************************************/

#ifndef _DRIVER_SCREEN_H_
#define _DRIVER_SCREEN_H_

#include "zf_common_typedef.h"

//=================================================屏幕基本配置================================================
// 注意：原理图SCK为P12.4，但硬件SPI1的CLK只能是P12.2，因此使用软件SPI
#define SCREEN_USE_SOFT_SPI             (1)                                     // 使用软件 SPI 方式驱动

#if SCREEN_USE_SOFT_SPI
//====================================================软件 SPI 驱动==================================================
#define SCREEN_SOFT_SPI_DELAY           (0)                                     // 软件 SPI 的时钟延时周期
#define SCREEN_SCK_PIN                  (P12_4)                                 // 软件 SPI SCK 引脚 (原理图引脚3)
#define SCREEN_MOSI_PIN                 (P12_1)                                 // 软件 SPI MOSI 引脚 (原理图引脚2)
//====================================================软件 SPI 驱动==================================================
#else
//====================================================硬件 SPI 驱动==================================================
// 警告：硬件SPI1的CLK只能是P12.2，无法使用原理图的P12.4！
#define SCREEN_SPI_SPEED                (30*1000*1000)                          // 硬件 SPI 速率 30MHz
#define SCREEN_SPI                      (SPI_1)                                 // 硬件 SPI 号
#define SCREEN_SCK_PIN                  (SPI1_CLK_P12_2)                        // 硬件 SPI SCK 引脚 P12.2 (非原理图)
#define SCREEN_MOSI_PIN                 (SPI1_MOSI_P12_1)                       // 硬件 SPI MOSI 引脚 P12.1
#define SCREEN_MISO_PIN                 (SPI_MISO_NULL)                         // 屏幕没有MISO引脚
//====================================================硬件 SPI 驱动==================================================
#endif

#define SCREEN_DC_PIN                   (P22_3)                                 // 数据/命令选择引脚
#define SCREEN_CS_PIN                   (P12_3)                                 // 片选引脚
#define SCREEN_RST_PIN                  (P11_0)                                 // 复位引脚
#define SCREEN_BL_PIN                   (P11_0)                                 // 背光引脚（与RST共用，根据实际情况调整）

//=================================================屏幕参数配置================================================
#define SCREEN_WIDTH                    (240)                                   // 屏幕宽度（需要根据实际屏幕确认）
#define SCREEN_HEIGHT                   (320)                                   // 屏幕高度（需要根据实际屏幕确认）

//=================================================显示方向枚举================================================
typedef enum
{
    SCREEN_DIR_0,                                                               // 正向显示
    SCREEN_DIR_90,                                                              // 逆时针旋转90度
    SCREEN_DIR_180,                                                             // 逆时针旋转180度
    SCREEN_DIR_270,                                                             // 逆时针旋转270度
} screen_dir_enum;

//=================================================字体大小枚举================================================
typedef enum
{
    SCREEN_FONT_6X8,                                                            // 6x8 字体
    SCREEN_FONT_8X16,                                                           // 8x16 字体
    SCREEN_FONT_16X16,                                                          // 16x16 中文字体 (需要字库支持)
} screen_font_size_enum;

//=================================================颜色定义 (RGB565格式)================================================
#define SCREEN_COLOR_WHITE              (0xFFFF)
#define SCREEN_COLOR_BLACK              (0x0000)
#define SCREEN_COLOR_RED                (0xF800)
#define SCREEN_COLOR_GREEN              (0x07E0)
#define SCREEN_COLOR_BLUE               (0x001F)
#define SCREEN_COLOR_YELLOW             (0xFFE0)
#define SCREEN_COLOR_CYAN               (0x07FF)
#define SCREEN_COLOR_MAGENTA            (0xF81F)
#define SCREEN_COLOR_GRAY               (0x8410)

//=================================================默认配置================================================
#define SCREEN_DEFAULT_DIR              (SCREEN_DIR_0)                          // 默认显示方向
#define SCREEN_DEFAULT_PENCOLOR         (SCREEN_COLOR_WHITE)                    // 默认画笔颜色
#define SCREEN_DEFAULT_BGCOLOR          (SCREEN_COLOR_BLACK)                    // 默认背景颜色
#define SCREEN_DEFAULT_FONT             (SCREEN_FONT_8X16)                      // 默认字体

//=================================================函数声明================================================
// 基础控制函数
void    screen_init                     (void);
void    screen_clear                    (uint16 color);
void    screen_set_dir                  (screen_dir_enum dir);
void    screen_set_color                (uint16 pen_color, uint16 bg_color);
void    screen_set_font                 (screen_font_size_enum font);
void    screen_backlight                (uint8 enable);

// 绘图函数
void    screen_draw_point               (uint16 x, uint16 y, uint16 color);
void    screen_draw_line                (uint16 x1, uint16 y1, uint16 x2, uint16 y2, uint16 color);
void    screen_draw_rectangle           (uint16 x1, uint16 y1, uint16 x2, uint16 y2, uint16 color);
void    screen_fill_rectangle           (uint16 x1, uint16 y1, uint16 x2, uint16 y2, uint16 color);
void    screen_draw_circle              (uint16 x, uint16 y, uint16 radius, uint16 color);
void    screen_fill_circle              (uint16 x, uint16 y, uint16 radius, uint16 color);

// 文字显示函数
void    screen_show_char                (uint16 x, uint16 y, const char dat);
void    screen_show_string              (uint16 x, uint16 y, const char *str);
void    screen_show_int                 (uint16 x, uint16 y, int32 dat, uint8 num);
void    screen_show_uint                (uint16 x, uint16 y, uint32 dat, uint8 num);
void    screen_show_float               (uint16 x, uint16 y, float dat, uint8 num, uint8 pointnum);

// 图像显示函数
void    screen_show_image               (uint16 x, uint16 y, uint16 w, uint16 h, const uint16 *image);

#endif // _DRIVER_SCREEN_H_

