/*********************************************************************************************************************
* 文件名称          test_photoelectric.c
* 功能说明          光电管驱动测试程序实现文件
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本
* 2025-11-05        LittleMaster       1.0v
********************************************************************************************************************/

#include "test_photoelectric.h"
#include "driver_photoelectric.h"
#include "zf_common_headfile.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     基础功能测试
// 使用示例     test_photoelectric_basic();
// 备注信息     测试光电管初始化和基本读取功能
//-------------------------------------------------------------------------------------------------------------------
void test_photoelectric_basic(void)
{
    uint8 i;
    uint8 data[PHOTO_CHANNEL_NUM];
    
    printf("\r\n========================================\r\n");
    printf("光电管基础功能测试\r\n");
    printf("========================================\r\n\r\n");
    
    // 初始化光电管
    photoelectric_init();
    system_delay_ms(500);
    
    // 测试单通道读取
    printf("测试单通道读取（前5个通道）：\r\n");
    for(i = 0; i < 5; i++)
    {
        photoelectric_read();
        printf("  通道 %d: %d\r\n", i, photoelectric_get_channel(i));
        system_delay_ms(200);
    }
    
    // 测试所有通道读取
    printf("\r\n测试所有通道读取：\r\n");
    photoelectric_read();
    photoelectric_get_all_channels(data);
    
    printf("  原始数据：[");
    for(i = 0; i < PHOTO_CHANNEL_NUM; i++)
    {
        printf("%d", data[i]);
        if(i < PHOTO_CHANNEL_NUM - 1)
            printf(" ");
    }
    printf("]\r\n");
    
    printf("  原始值（位图）：0x%05X\r\n", photoelectric_get_raw_value());
    
    printf("\r\n基础功能测试完成\r\n");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     位置计算测试
// 使用示例     test_photoelectric_position();
// 备注信息     测试线位置计算功能
//-------------------------------------------------------------------------------------------------------------------
void test_photoelectric_position(void)
{
    uint8 test_count = 10;
    uint8 i;
    
    printf("\r\n========================================\r\n");
    printf("光电管位置计算测试\r\n");
    printf("========================================\r\n\r\n");
    
    printf("进行 %d 次位置检测：\r\n\r\n", test_count);
    
    for(i = 0; i < test_count; i++)
    {
        photoelectric_read();
        
        printf("第 %2d 次检测：\r\n", i + 1);
        printf("  线位置：%.4f\r\n", photoelectric_get_line_position());
        printf("  线宽：%d\r\n", photoelectric_get_line_width());
        printf("  数据有效：%s\r\n", photoelectric_is_valid() ? "是" : "否");
        
        // 可视化显示
        printf("  可视化：|");
        int8 pos_index = (int8)((photoelectric_get_line_position() + 1.0f) * 8.5f);
        for(int8 j = 0; j < 18; j++)
        {
            if(j == pos_index)
                printf("*");
            else if(photoelectric_get_channel(j))
                printf("█");
            else
                printf(" ");
        }
        printf("|\r\n\r\n");
        
        system_delay_ms(500);
    }
    
    printf("位置计算测试完成\r\n");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     实时显示测试
// 使用示例     test_photoelectric_realtime();
// 备注信息     持续显示光电管状态（按任意键退出）
//-------------------------------------------------------------------------------------------------------------------
void test_photoelectric_realtime(void)
{
    uint32 count = 0;
    
    printf("\r\n========================================\r\n");
    printf("光电管实时显示测试\r\n");
    printf("持续显示光电管状态...\r\n");
    printf("（运行30秒后自动退出）\r\n");
    printf("========================================\r\n\r\n");
    
    while(count < 300)  // 30秒（100ms * 300）
    {
        photoelectric_read();
        
        // 清除当前行并移动光标到行首
        printf("\r");
        
        // 显示光电管状态
        printf("[");
        for(uint8 i = 0; i < PHOTO_CHANNEL_NUM; i++)
        {
            printf("%d", photoelectric_get_channel(i));
        }
        printf("] Pos:%.3f Width:%2d Valid:%d  ", 
               photoelectric_get_line_position(),
               photoelectric_get_line_width(),
               photoelectric_is_valid());
        
        fflush(stdout);  // 强制刷新输出
        
        system_delay_ms(100);
        count++;
    }
    
    printf("\r\n\r\n实时显示测试完成\r\n");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     完整测试
// 使用示例     test_photoelectric_all();
// 备注信息     依次执行所有测试
//-------------------------------------------------------------------------------------------------------------------
void test_photoelectric_all(void)
{
    printf("\r\n");
    printf("╔════════════════════════════════════════╗\r\n");
    printf("║     光电管驱动完整测试开始             ║\r\n");
    printf("╚════════════════════════════════════════╝\r\n");
    
    // 基础功能测试
    test_photoelectric_basic();
    system_delay_ms(1000);
    
    // 位置计算测试
    test_photoelectric_position();
    system_delay_ms(1000);
    
    // 实时显示测试
    test_photoelectric_realtime();
    
    printf("\r\n");
    printf("╔════════════════════════════════════════╗\r\n");
    printf("║     光电管驱动完整测试完成             ║\r\n");
    printf("║         所有功能测试通过！             ║\r\n");
    printf("╚════════════════════════════════════════╝\r\n");
    printf("\r\n");
}

