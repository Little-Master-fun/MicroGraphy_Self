/*********************************************************************************************************************
* 文件名称          test_key.c
* 功能说明          按键驱动测试程序实现文件
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本
* 2025-11-05        LittleMaster       1.0v
********************************************************************************************************************/

#include "test_key.h"
#include "driver_key.h"
#include "zf_common_headfile.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按键事件回调函数示例
//-------------------------------------------------------------------------------------------------------------------
static void my_key_callback(key_id_enum key_id, key_event_enum event)
{
    const char *event_name[] = {"NONE", "按下", "释放", "长按", "单击"};
    
    printf("【回调】KEY%d - %s\r\n", key_id, event_name[event]);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     基础功能测试
// 使用示例     test_key_basic();
// 备注信息     测试按键初始化和基本状态读取
//-------------------------------------------------------------------------------------------------------------------
void test_key_basic(void)
{
    uint8 i;
    uint32 test_count = 0;
    
    printf("\r\n========================================\r\n");
    printf("按键基础功能测试\r\n");
    printf("========================================\r\n\r\n");
    
    // 初始化按键
    key_init();
    system_delay_ms(500);
    
    printf("请按下任意按键进行测试（测试20秒）...\r\n\r\n");
    
    while(test_count < 10000)  // 20秒（2ms * 10000）
    {
        // 扫描按键
        key_scan();
        
        // 检查所有按键状态
        for(i = 0; i < KEY_NUM; i++)
        {
            uint8 state = key_get_state((key_id_enum)i);
            
            // 如果按键被按下，打印信息
            if(state == 0)
            {
                printf("KEY%d 被按下\r\n", i);
            }
        }
        
        system_delay_ms(2);
        test_count++;
    }
    
    printf("\r\n基础功能测试完成\r\n");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     事件检测测试
// 使用示例     test_key_event();
// 备注信息     测试按键事件检测（单击、长按等）
//-------------------------------------------------------------------------------------------------------------------
void test_key_event(void)
{
    uint32 test_count = 0;
    const char *event_name[] = {"无", "按下", "释放", "长按", "单击"};
    
    printf("\r\n========================================\r\n");
    printf("按键事件检测测试\r\n");
    printf("========================================\r\n\r\n");
    
    printf("请测试以下功能：\r\n");
    printf("  1. 快速按下并释放 - 产生'单击'事件\r\n");
    printf("  2. 长按不放（>1秒）- 产生'长按'事件\r\n");
    printf("测试时间：20秒\r\n\r\n");
    
    while(test_count < 10000)  // 20秒
    {
        uint8 i;
        
        // 扫描按键
        key_scan();
        
        // 检查所有按键事件
        for(i = 0; i < KEY_NUM; i++)
        {
            key_event_enum event = key_get_event((key_id_enum)i);
            
            if(event != KEY_EVENT_NONE)
            {
                printf("KEY%d 事件: %s\r\n", i, event_name[event]);
                
                // 清除事件
                key_clear_event((key_id_enum)i);
            }
        }
        
        system_delay_ms(2);
        test_count++;
    }
    
    printf("\r\n事件检测测试完成\r\n");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     回调函数测试
// 使用示例     test_key_callback();
// 备注信息     测试按键回调函数机制
//-------------------------------------------------------------------------------------------------------------------
void test_key_callback(void)
{
    uint32 test_count = 0;
    
    printf("\r\n========================================\r\n");
    printf("按键回调函数测试\r\n");
    printf("========================================\r\n\r\n");
    
    // 注册回调函数
    key_register_callback(my_key_callback);
    
    printf("已注册回调函数\r\n");
    printf("请按下任意按键，将通过回调函数输出信息\r\n");
    printf("测试时间：20秒\r\n\r\n");
    
    while(test_count < 10000)  // 20秒
    {
        // 扫描按键（回调会在扫描中自动触发）
        key_scan();
        
        system_delay_ms(2);
        test_count++;
    }
    
    printf("\r\n回调函数测试完成\r\n");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     完整测试
// 使用示例     test_key_all();
// 备注信息     依次执行所有测试
//-------------------------------------------------------------------------------------------------------------------
void test_key_all(void)
{
    printf("\r\n");
    printf("╔════════════════════════════════════════╗\r\n");
    printf("║       按键驱动完整测试开始             ║\r\n");
    printf("╚════════════════════════════════════════╝\r\n");
    
    // 基础功能测试
    test_key_basic();
    system_delay_ms(1000);
    
    // 事件检测测试
    test_key_event();
    system_delay_ms(1000);
    
    // 回调函数测试
    test_key_callback();
    
    printf("\r\n");
    printf("╔════════════════════════════════════════╗\r\n");
    printf("║       按键驱动完整测试完成             ║\r\n");
    printf("║         所有功能测试通过！             ║\r\n");
    printf("╚════════════════════════════════════════╝\r\n");
    printf("\r\n");
}

