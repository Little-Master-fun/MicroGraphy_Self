/*********************************************************************************************************************
* 文件名称          test_nav_example.c
* 功能说明          导航测试系统使用示例
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本
* 2025-09-19        LittleMaster       1.0v
* 
* 使用说明：
* 本文件展示如何在主函数中调用导航测试系统
* 可以直接将 test_nav_main_example() 函数的内容复制到实际的main函数中使用
********************************************************************************************************************/

#include "test_nav.h"
#include "zf_common_headfile.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导航测试主函数示例
// 参数说明     void
// 返回参数     void
// 使用示例     在main.c中调用：test_nav_main_example();
//-------------------------------------------------------------------------------------------------------------------
void test_nav_main_example(void)
{
    // 1. 初始化系统时钟（如果需要）
    // system_clock_init();
    
    // 2. 初始化调试串口（如果需要）
    // debug_init();
    
    // 3. 初始化显示屏
    ips114_init();
    ips114_clear();
    
    // 4. 显示启动信息
    ips114_show_string(0, 0, "MicroGraphy 2.0");
    ips114_show_string(0, 16, "Navigation Test");
    ips114_show_string(0, 32, "Initializing...");
    system_delay_ms(2000);
    
    // 5. 调用完整导航测试
    test_nav_complete();
    
    // 6. 测试结束后的处理
    ips114_clear();
    ips114_show_string(0, 0, "Test Complete");
    ips114_show_string(0, 16, "System Halt");
    
    // 7. 进入无限循环
    while(1)
    {
        system_delay_ms(1000);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     简化的单独测试示例
// 参数说明     void  
// 返回参数     void
// 使用示例     只测试路径生成和验证，不进行实际导航
//-------------------------------------------------------------------------------------------------------------------
void test_nav_path_only_example(void)
{
    // 初始化显示屏
    ips114_init();
    ips114_clear();
    
    // 显示测试信息
    ips114_show_string(0, 0, "Path Generation Test");
    
    // 测试路径生成
    if (test_nav_create_square_path())
    {
        ips114_show_string(0, 16, "Path Created: OK");
        
        // 验证路径
        if (test_nav_verify_path())
        {
            ips114_show_string(0, 32, "Path Verified: OK");
        }
        else
        {
            ips114_show_string(0, 32, "Path Verified: FAIL");
        }
        
        // 显示路径信息
        nav_path_t *path = &nav_system.current_path;
        char info[50];
        sprintf(info, "Points: %d", path->waypoint_count);
        ips114_show_string(0, 48, info);
        sprintf(info, "Length: %.1fm", path->total_length);
        ips114_show_string(0, 64, info);
    }
    else
    {
        ips114_show_string(0, 16, "Path Created: FAIL");
    }
    
    // 显示完成信息
    ips114_show_string(0, 96, "Test Complete");
    
    while(1)
    {
        system_delay_ms(1000);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     主函数示例（完整版）
//-------------------------------------------------------------------------------------------------------------------
int main(void)
{
    // 基础系统初始化
    clock_init(SYSTEM_CLOCK_600M);      // 初始化系统时钟
    debug_init();                        // 初始化调试接口
    
    // 启动导航测试
    test_nav_main_example();
    
    return 0;
}
