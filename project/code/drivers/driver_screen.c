/*********************************************************************************************************************
* 文件名称          driver_screen.c
* 功能说明          2.54寸TFT屏幕驱动程序实现文件 (PZ254V-11-08P)
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本
* 2025-11-05        LittleMaster       1.0v
********************************************************************************************************************/

#include "driver_screen.h"
#include "zf_common_headfile.h"
#include "zf_common_font.h"
#include <math.h>

//=================================================全局变量定义================================================
static uint16               screen_width_max    = SCREEN_WIDTH;
static uint16               screen_height_max   = SCREEN_HEIGHT;
static uint16               screen_pencolor     = SCREEN_DEFAULT_PENCOLOR;
static uint16               screen_bgcolor      = SCREEN_DEFAULT_BGCOLOR;
static screen_dir_enum      screen_dir          = SCREEN_DEFAULT_DIR;
static screen_font_size_enum screen_font        = SCREEN_DEFAULT_FONT;

#if SCREEN_USE_SOFT_SPI
static soft_spi_info_struct screen_spi;
#endif

//=================================================内部函数声明================================================
static void screen_write_command(uint8 cmd);
static void screen_write_data(uint8 data);
static void screen_write_data_16bit(uint16 data);
static void screen_set_region(uint16 x1, uint16 y1, uint16 x2, uint16 y2);
static void screen_hardware_reset(void);

//=================================================宏定义辅助函数================================================
#define SCREEN_DC(x)        ((x) ? (gpio_set_level(SCREEN_DC_PIN, GPIO_HIGH)) : (gpio_set_level(SCREEN_DC_PIN, GPIO_LOW)))
#define SCREEN_RST(x)       ((x) ? (gpio_set_level(SCREEN_RST_PIN, GPIO_HIGH)) : (gpio_set_level(SCREEN_RST_PIN, GPIO_LOW)))
#define SCREEN_CS(x)        ((x) ? (gpio_set_level(SCREEN_CS_PIN, GPIO_HIGH)) : (gpio_set_level(SCREEN_CS_PIN, GPIO_LOW)))
#define SCREEN_BL(x)        ((x) ? (gpio_set_level(SCREEN_BL_PIN, GPIO_HIGH)) : (gpio_set_level(SCREEN_BL_PIN, GPIO_LOW)))

//=================================================底层通信函数================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     写8位数据到SPI
//-------------------------------------------------------------------------------------------------------------------
static inline void screen_write_8bit(uint8 data)
{
#if SCREEN_USE_SOFT_SPI
    soft_spi_write_8bit(&screen_spi, data);
#else
    spi_write_8bit(SCREEN_SPI, data);
#endif
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     写16位数据到SPI
//-------------------------------------------------------------------------------------------------------------------
static inline void screen_write_16bit(uint16 data)
{
#if SCREEN_USE_SOFT_SPI
    soft_spi_write_16bit(&screen_spi, data);
#else
    spi_write_16bit(SCREEN_SPI, data);
#endif
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     写命令
//-------------------------------------------------------------------------------------------------------------------
static void screen_write_command(uint8 cmd)
{
    SCREEN_CS(0);
    SCREEN_DC(0);
    screen_write_8bit(cmd);
    SCREEN_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     写8位数据
//-------------------------------------------------------------------------------------------------------------------
static void screen_write_data(uint8 data)
{
    SCREEN_CS(0);
    SCREEN_DC(1);
    screen_write_8bit(data);
    SCREEN_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     写16位数据
//-------------------------------------------------------------------------------------------------------------------
static void screen_write_data_16bit(uint16 data)
{
    SCREEN_CS(0);
    SCREEN_DC(1);
    screen_write_16bit(data);
    SCREEN_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     硬件复位
//-------------------------------------------------------------------------------------------------------------------
static void screen_hardware_reset(void)
{
    SCREEN_RST(1);
    system_delay_ms(10);
    SCREEN_RST(0);
    system_delay_ms(10);
    SCREEN_RST(1);
    system_delay_ms(120);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置显示区域
//-------------------------------------------------------------------------------------------------------------------
static void screen_set_region(uint16 x1, uint16 y1, uint16 x2, uint16 y2)
{
    // 根据不同的屏幕控制器，这些命令可能需要调整
    // 这里使用通用的 ST7789/ILI9341 命令集
    
    // 列地址设置
    screen_write_command(0x2A);
    screen_write_data(x1 >> 8);
    screen_write_data(x1 & 0xFF);
    screen_write_data(x2 >> 8);
    screen_write_data(x2 & 0xFF);
    
    // 行地址设置
    screen_write_command(0x2B);
    screen_write_data(y1 >> 8);
    screen_write_data(y1 & 0xFF);
    screen_write_data(y2 >> 8);
    screen_write_data(y2 & 0xFF);
    
    // 写入内存
    screen_write_command(0x2C);
}

//=================================================外部接口实现================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     屏幕初始化
//-------------------------------------------------------------------------------------------------------------------
void screen_init(void)
{
    // 初始化GPIO引脚
    gpio_init(SCREEN_DC_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(SCREEN_CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(SCREEN_RST_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    
    // 初始化SPI
#if SCREEN_USE_SOFT_SPI
    soft_spi_init(&screen_spi, SCREEN_SCK_PIN, SCREEN_MOSI_PIN, SPI_MISO_NULL, 
                  SPI_CS_NULL, SCREEN_SOFT_SPI_DELAY, 0);
#else
    spi_init(SCREEN_SPI, SPI_MODE0, SCREEN_SPI_SPEED, SCREEN_SCK_PIN, 
             SCREEN_MOSI_PIN, SCREEN_MISO_PIN, SPI_CS_NULL);
#endif
    
    // 硬件复位
    screen_hardware_reset();
    
    // 初始化序列（通用 ST7789 初始化序列，可能需要根据实际屏幕调整）
    screen_write_command(0x11); // Sleep out
    system_delay_ms(120);
    
    screen_write_command(0x36); // Memory Data Access Control
    screen_write_data(0x00);
    
    screen_write_command(0x3A); // Interface Pixel Format
    screen_write_data(0x05);    // 16bit/pixel
    
    screen_write_command(0xB2); // Porch Setting
    screen_write_data(0x0C);
    screen_write_data(0x0C);
    screen_write_data(0x00);
    screen_write_data(0x33);
    screen_write_data(0x33);
    
    screen_write_command(0xB7); // Gate Control
    screen_write_data(0x35);
    
    screen_write_command(0xBB); // VCOM Setting
    screen_write_data(0x19);
    
    screen_write_command(0xC0); // LCM Control
    screen_write_data(0x2C);
    
    screen_write_command(0xC2); // VDV and VRH Command Enable
    screen_write_data(0x01);
    
    screen_write_command(0xC3); // VRH Set
    screen_write_data(0x12);
    
    screen_write_command(0xC4); // VDV Set
    screen_write_data(0x20);
    
    screen_write_command(0xC6); // Frame Rate Control
    screen_write_data(0x0F);
    
    screen_write_command(0xD0); // Power Control 1
    screen_write_data(0xA4);
    screen_write_data(0xA1);
    
    screen_write_command(0xE0); // Positive Voltage Gamma Control
    screen_write_data(0xD0);
    screen_write_data(0x04);
    screen_write_data(0x0D);
    screen_write_data(0x11);
    screen_write_data(0x13);
    screen_write_data(0x2B);
    screen_write_data(0x3F);
    screen_write_data(0x54);
    screen_write_data(0x4C);
    screen_write_data(0x18);
    screen_write_data(0x0D);
    screen_write_data(0x0B);
    screen_write_data(0x1F);
    screen_write_data(0x23);
    
    screen_write_command(0xE1); // Negative Voltage Gamma Control
    screen_write_data(0xD0);
    screen_write_data(0x04);
    screen_write_data(0x0C);
    screen_write_data(0x11);
    screen_write_data(0x13);
    screen_write_data(0x2C);
    screen_write_data(0x3F);
    screen_write_data(0x44);
    screen_write_data(0x51);
    screen_write_data(0x2F);
    screen_write_data(0x1F);
    screen_write_data(0x1F);
    screen_write_data(0x20);
    screen_write_data(0x23);
    
    screen_write_command(0x21); // Display Inversion On
    
    screen_write_command(0x29); // Display on
    system_delay_ms(120);
    
    // 清屏
    screen_clear(screen_bgcolor);
    
    // 打开背光
    screen_backlight(1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清屏
//-------------------------------------------------------------------------------------------------------------------
void screen_clear(uint16 color)
{
    uint32 i;
    uint32 total_pixels = screen_width_max * screen_height_max;
    
    screen_set_region(0, 0, screen_width_max - 1, screen_height_max - 1);
    
    SCREEN_CS(0);
    SCREEN_DC(1);
    
    for(i = 0; i < total_pixels; i++)
    {
        screen_write_16bit(color);
    }
    
    SCREEN_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置显示方向
//-------------------------------------------------------------------------------------------------------------------
void screen_set_dir(screen_dir_enum dir)
{
    screen_dir = dir;
    
    screen_write_command(0x36);
    
    switch(dir)
    {
        case SCREEN_DIR_0:
            screen_write_data(0x00);
            screen_width_max = SCREEN_WIDTH;
            screen_height_max = SCREEN_HEIGHT;
            break;
        case SCREEN_DIR_90:
            screen_write_data(0x60);
            screen_width_max = SCREEN_HEIGHT;
            screen_height_max = SCREEN_WIDTH;
            break;
        case SCREEN_DIR_180:
            screen_write_data(0xC0);
            screen_width_max = SCREEN_WIDTH;
            screen_height_max = SCREEN_HEIGHT;
            break;
        case SCREEN_DIR_270:
            screen_write_data(0xA0);
            screen_width_max = SCREEN_HEIGHT;
            screen_height_max = SCREEN_WIDTH;
            break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置颜色
//-------------------------------------------------------------------------------------------------------------------
void screen_set_color(uint16 pen_color, uint16 bg_color)
{
    screen_pencolor = pen_color;
    screen_bgcolor = bg_color;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置字体
//-------------------------------------------------------------------------------------------------------------------
void screen_set_font(screen_font_size_enum font)
{
    screen_font = font;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     背光控制
//-------------------------------------------------------------------------------------------------------------------
void screen_backlight(uint8 enable)
{
    SCREEN_BL(enable);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     画点
//-------------------------------------------------------------------------------------------------------------------
void screen_draw_point(uint16 x, uint16 y, uint16 color)
{
    if(x >= screen_width_max || y >= screen_height_max)
        return;
    
    screen_set_region(x, y, x, y);
    screen_write_data_16bit(color);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     画线
//-------------------------------------------------------------------------------------------------------------------
void screen_draw_line(uint16 x1, uint16 y1, uint16 x2, uint16 y2, uint16 color)
{
    int16 dx = x2 - x1;
    int16 dy = y2 - y1;
    int16 ux = ((dx > 0) << 1) - 1;
    int16 uy = ((dy > 0) << 1) - 1;
    int16 x = x1, y = y1, eps = 0;
    
    dx = abs(dx);
    dy = abs(dy);
    
    if(dx > dy)
    {
        for(x = x1; x != x2; x += ux)
        {
            screen_draw_point(x, y, color);
            eps += dy;
            if((eps << 1) >= dx)
            {
                y += uy;
                eps -= dx;
            }
        }
    }
    else
    {
        for(y = y1; y != y2; y += uy)
        {
            screen_draw_point(x, y, color);
            eps += dx;
            if((eps << 1) >= dy)
            {
                x += ux;
                eps -= dy;
            }
        }
    }
    screen_draw_point(x2, y2, color);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     画矩形
//-------------------------------------------------------------------------------------------------------------------
void screen_draw_rectangle(uint16 x1, uint16 y1, uint16 x2, uint16 y2, uint16 color)
{
    screen_draw_line(x1, y1, x2, y1, color);
    screen_draw_line(x2, y1, x2, y2, color);
    screen_draw_line(x2, y2, x1, y2, color);
    screen_draw_line(x1, y2, x1, y1, color);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     填充矩形
//-------------------------------------------------------------------------------------------------------------------
void screen_fill_rectangle(uint16 x1, uint16 y1, uint16 x2, uint16 y2, uint16 color)
{
    uint16 i, j;
    
    if(x1 > x2) { uint16 temp = x1; x1 = x2; x2 = temp; }
    if(y1 > y2) { uint16 temp = y1; y1 = y2; y2 = temp; }
    
    screen_set_region(x1, y1, x2, y2);
    
    SCREEN_CS(0);
    SCREEN_DC(1);
    
    for(i = y1; i <= y2; i++)
    {
        for(j = x1; j <= x2; j++)
        {
            screen_write_16bit(color);
        }
    }
    
    SCREEN_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     画圆
//-------------------------------------------------------------------------------------------------------------------
void screen_draw_circle(uint16 x, uint16 y, uint16 radius, uint16 color)
{
    int16 a = 0, b = radius;
    int16 d = 3 - (radius << 1);
    
    while(a <= b)
    {
        screen_draw_point(x - b, y - a, color);
        screen_draw_point(x + b, y - a, color);
        screen_draw_point(x - a, y + b, color);
        screen_draw_point(x - b, y - a, color);
        screen_draw_point(x - a, y - b, color);
        screen_draw_point(x + b, y + a, color);
        screen_draw_point(x + a, y - b, color);
        screen_draw_point(x + a, y + b, color);
        screen_draw_point(x - b, y + a, color);
        a++;
        
        if(d < 0)
        {
            d += (4 * a) + 6;
        }
        else
        {
            d += 10 + (4 * (a - b));
            b--;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     填充圆
//-------------------------------------------------------------------------------------------------------------------
void screen_fill_circle(uint16 x, uint16 y, uint16 radius, uint16 color)
{
    int16 a = 0, b = radius;
    int16 d = 3 - (radius << 1);
    
    while(a <= b)
    {
        screen_draw_line(x - b, y - a, x + b, y - a, color);
        screen_draw_line(x - a, y - b, x + a, y - b, color);
        screen_draw_line(x - b, y + a, x + b, y + a, color);
        screen_draw_line(x - a, y + b, x + a, y + b, color);
        a++;
        
        if(d < 0)
        {
            d += (4 * a) + 6;
        }
        else
        {
            d += 10 + (4 * (a - b));
            b--;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示字符
//-------------------------------------------------------------------------------------------------------------------
void screen_show_char(uint16 x, uint16 y, const char dat)
{
    uint8 i, j;
    uint8 temp;
    
    for(i = 0; i < 8; i++)
    {
        temp = ascii_font_8x16[dat - 32][i];
        for(j = 0; j < 8; j++)
        {
            if(temp & 0x01)
                screen_draw_point(x + j, y + i, screen_pencolor);
            else
                screen_draw_point(x + j, y + i, screen_bgcolor);
            temp >>= 1;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示字符串
//-------------------------------------------------------------------------------------------------------------------
void screen_show_string(uint16 x, uint16 y, const char *str)
{
    uint16 x_temp = x;
    
    while(*str != '\0')
    {
        if(*str == '\n')
        {
            x_temp = x;
            y += 16;
        }
        else
        {
            if(x_temp > (screen_width_max - 8))
            {
                x_temp = x;
                y += 16;
            }
            screen_show_char(x_temp, y, *str);
            x_temp += 8;
        }
        str++;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示整数
//-------------------------------------------------------------------------------------------------------------------
void screen_show_int(uint16 x, uint16 y, int32 dat, uint8 num)
{
    char str[12];
    sprintf(str, "%d", dat);
    screen_show_string(x, y, str);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示无符号整数
//-------------------------------------------------------------------------------------------------------------------
void screen_show_uint(uint16 x, uint16 y, uint32 dat, uint8 num)
{
    char str[12];
    sprintf(str, "%u", dat);
    screen_show_string(x, y, str);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示浮点数
//-------------------------------------------------------------------------------------------------------------------
void screen_show_float(uint16 x, uint16 y, float dat, uint8 num, uint8 pointnum)
{
    char str[16];
    sprintf(str, "%.*f", pointnum, dat);
    screen_show_string(x, y, str);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示图像
//-------------------------------------------------------------------------------------------------------------------
void screen_show_image(uint16 x, uint16 y, uint16 w, uint16 h, const uint16 *image)
{
    uint32 i;
    uint32 total_pixels = w * h;
    
    screen_set_region(x, y, x + w - 1, y + h - 1);
    
    SCREEN_CS(0);
    SCREEN_DC(1);
    
    for(i = 0; i < total_pixels; i++)
    {
        screen_write_16bit(image[i]);
    }
    
    SCREEN_CS(1);
}

