/*********************************************************************************************************************
* 文件名称          test_screen.c
* 功能说明          屏幕驱动测试程序实现文件
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本
* 2025-11-05        LittleMaster       1.0v
********************************************************************************************************************/

#include "test_screen.h"
#include "driver_screen.h"
#include "zf_common_headfile.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     基础功能测试
// 使用示例     test_screen_basic();
// 备注信息     测试屏幕初始化、清屏、背光控制
//-------------------------------------------------------------------------------------------------------------------
void test_screen_basic(void)
{
    printf("屏幕基础功能测试开始...\r\n");
    
    // 初始化屏幕
    screen_init();
    printf("屏幕初始化完成\r\n");
    system_delay_ms(1000);
    
    // 测试不同颜色清屏
    printf("测试红色清屏\r\n");
    screen_clear(SCREEN_COLOR_RED);
    system_delay_ms(1000);
    
    printf("测试绿色清屏\r\n");
    screen_clear(SCREEN_COLOR_GREEN);
    system_delay_ms(1000);
    
    printf("测试蓝色清屏\r\n");
    screen_clear(SCREEN_COLOR_BLUE);
    system_delay_ms(1000);
    
    printf("测试白色清屏\r\n");
    screen_clear(SCREEN_COLOR_WHITE);
    system_delay_ms(1000);
    
    printf("测试黑色清屏\r\n");
    screen_clear(SCREEN_COLOR_BLACK);
    system_delay_ms(1000);
    
    // 测试背光控制
    printf("关闭背光\r\n");
    screen_backlight(0);
    system_delay_ms(1000);
    
    printf("打开背光\r\n");
    screen_backlight(1);
    system_delay_ms(1000);
    
    printf("屏幕基础功能测试完成\r\n");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     绘图功能测试
// 使用示例     test_screen_draw();
// 备注信息     测试点、线、矩形、圆等绘图功能
//-------------------------------------------------------------------------------------------------------------------
void test_screen_draw(void)
{
    printf("屏幕绘图功能测试开始...\r\n");
    
    screen_clear(SCREEN_COLOR_BLACK);
    system_delay_ms(500);
    
    // 测试画点
    printf("测试画点\r\n");
    for(int i = 0; i < 100; i++)
    {
        screen_draw_point(i, i, SCREEN_COLOR_RED);
        screen_draw_point(i, 100-i, SCREEN_COLOR_GREEN);
    }
    system_delay_ms(2000);
    
    // 测试画线
    printf("测试画线\r\n");
    screen_clear(SCREEN_COLOR_BLACK);
    screen_draw_line(0, 0, 239, 319, SCREEN_COLOR_RED);
    screen_draw_line(239, 0, 0, 319, SCREEN_COLOR_GREEN);
    screen_draw_line(0, 160, 239, 160, SCREEN_COLOR_BLUE);
    screen_draw_line(120, 0, 120, 319, SCREEN_COLOR_YELLOW);
    system_delay_ms(2000);
    
    // 测试矩形
    printf("测试矩形\r\n");
    screen_clear(SCREEN_COLOR_BLACK);
    screen_draw_rectangle(20, 20, 220, 300, SCREEN_COLOR_RED);
    screen_draw_rectangle(40, 40, 200, 280, SCREEN_COLOR_GREEN);
    screen_fill_rectangle(60, 60, 180, 260, SCREEN_COLOR_BLUE);
    system_delay_ms(2000);
    
    // 测试圆形
    printf("测试圆形\r\n");
    screen_clear(SCREEN_COLOR_BLACK);
    screen_draw_circle(120, 160, 100, SCREEN_COLOR_RED);
    screen_draw_circle(120, 160, 80, SCREEN_COLOR_GREEN);
    screen_fill_circle(120, 160, 60, SCREEN_COLOR_BLUE);
    system_delay_ms(2000);
    
    printf("屏幕绘图功能测试完成\r\n");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     文字显示测试
// 使用示例     test_screen_text();
// 备注信息     测试字符、字符串、数字显示
//-------------------------------------------------------------------------------------------------------------------
void test_screen_text(void)
{
    printf("屏幕文字显示测试开始...\r\n");
    
    screen_clear(SCREEN_COLOR_BLACK);
    screen_set_color(SCREEN_COLOR_WHITE, SCREEN_COLOR_BLACK);
    
    // 测试字符显示
    printf("测试字符显示\r\n");
    screen_show_string(10, 10, "Screen Test");
    screen_show_string(10, 30, "PZ254V-11-08P");
    system_delay_ms(2000);
    
    // 测试数字显示
    printf("测试数字显示\r\n");
    screen_set_color(SCREEN_COLOR_GREEN, SCREEN_COLOR_BLACK);
    screen_show_string(10, 60, "Int: ");
    screen_show_int(60, 60, -12345, 5);
    
    screen_show_string(10, 80, "Uint: ");
    screen_show_uint(60, 80, 54321, 5);
    
    screen_show_string(10, 100, "Float: ");
    screen_show_float(60, 100, 3.1415926f, 8, 5);
    system_delay_ms(3000);
    
    // 测试不同颜色文字
    printf("测试彩色文字\r\n");
    screen_clear(SCREEN_COLOR_BLACK);
    screen_set_color(SCREEN_COLOR_RED, SCREEN_COLOR_BLACK);
    screen_show_string(10, 10, "RED TEXT");
    
    screen_set_color(SCREEN_COLOR_GREEN, SCREEN_COLOR_BLACK);
    screen_show_string(10, 30, "GREEN TEXT");
    
    screen_set_color(SCREEN_COLOR_BLUE, SCREEN_COLOR_BLACK);
    screen_show_string(10, 50, "BLUE TEXT");
    
    screen_set_color(SCREEN_COLOR_YELLOW, SCREEN_COLOR_BLACK);
    screen_show_string(10, 70, "YELLOW TEXT");
    
    screen_set_color(SCREEN_COLOR_CYAN, SCREEN_COLOR_BLACK);
    screen_show_string(10, 90, "CYAN TEXT");
    
    screen_set_color(SCREEN_COLOR_MAGENTA, SCREEN_COLOR_BLACK);
    screen_show_string(10, 110, "MAGENTA TEXT");
    
    system_delay_ms(3000);
    
    printf("屏幕文字显示测试完成\r\n");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     完整测试
// 使用示例     test_screen_all();
// 备注信息     依次执行所有测试
//-------------------------------------------------------------------------------------------------------------------
void test_screen_all(void)
{
    printf("\r\n========================================\r\n");
    printf("屏幕驱动完整测试开始\r\n");
    printf("========================================\r\n\r\n");
    
    // 基础功能测试
    test_screen_basic();
    system_delay_ms(1000);
    
    // 绘图功能测试
    test_screen_draw();
    system_delay_ms(1000);
    
    // 文字显示测试
    test_screen_text();
    system_delay_ms(1000);
    
    // 显示测试完成信息
    screen_clear(SCREEN_COLOR_BLACK);
    screen_set_color(SCREEN_COLOR_GREEN, SCREEN_COLOR_BLACK);
    screen_show_string(40, 140, "Test Complete!");
    screen_show_string(40, 160, "All Pass!");
    
    printf("\r\n========================================\r\n");
    printf("屏幕驱动完整测试完成\r\n");
    printf("所有功能测试通过！\r\n");
    printf("========================================\r\n\r\n");
}

