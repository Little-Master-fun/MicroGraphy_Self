/*********************************************************************************************************************
* 文件名称          driver_photoelectric.c
* 功能说明          光电管阵列驱动程序实现文件
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本
* 2025-11-05        LittleMaster       1.0v
********************************************************************************************************************/

#include "driver_photoelectric.h"
#include "zf_common_headfile.h"

//=================================================全局变量定义================================================
photoelectric_data_struct photoelectric_data = {0};

// 光电管引脚数组
static const gpio_pin_enum photo_pins[PHOTO_CHANNEL_NUM] = {
    PHOTO_CH0_PIN,  PHOTO_CH1_PIN,  PHOTO_CH2_PIN,  PHOTO_CH3_PIN,
    PHOTO_CH4_PIN,  PHOTO_CH5_PIN,  PHOTO_CH6_PIN,  PHOTO_CH7_PIN,
    PHOTO_CH8_PIN,  PHOTO_CH9_PIN,  PHOTO_CH10_PIN, PHOTO_CH11_PIN,
    PHOTO_CH12_PIN, PHOTO_CH13_PIN, PHOTO_CH14_PIN, PHOTO_CH15_PIN,
    PHOTO_CH16_PIN, PHOTO_CH17_PIN
};

//=================================================外部接口实现================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     光电管阵列初始化
// 使用示例     photoelectric_init();
// 备注信息     初始化所有光电管引脚为输入模式
//-------------------------------------------------------------------------------------------------------------------
void photoelectric_init(void)
{
    uint8 i;
    
    // 初始化所有光电管引脚为输入，上拉模式
    for(i = 0; i < PHOTO_CHANNEL_NUM; i++)
    {
        gpio_init(photo_pins[i], GPI, GPIO_HIGH, GPI_PULL_UP);
    }
    
    // 初始化数据结构
    memset(&photoelectric_data, 0, sizeof(photoelectric_data_struct));
    
    printf("光电管阵列初始化完成，通道数：%d\r\n", PHOTO_CHANNEL_NUM);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取光电管数据
// 使用示例     photoelectric_read();
// 备注信息     读取所有通道并更新数据结构
//-------------------------------------------------------------------------------------------------------------------
void photoelectric_read(void)
{
    uint8 i;
    uint32 raw_value = 0;
    
    // 读取所有通道
    for(i = 0; i < PHOTO_CHANNEL_NUM; i++)
    {
        // 读取GPIO电平（0=黑线，1=白线 或相反，取决于传感器类型）
        photoelectric_data.raw_data[i] = !gpio_get_level(photo_pins[i]);  // 取反，使1表示检测到线
        
        // 构建位图
        if(photoelectric_data.raw_data[i])
        {
            raw_value |= (1 << i);
        }
    }
    
    photoelectric_data.raw_value = raw_value;
    
    // 计算线位置
    photoelectric_calculate_position();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取单个通道状态
// 参数说明     channel         通道号 (0-17)
// 返回参数     uint8           通道状态 (0或1)
// 使用示例     status = photoelectric_get_channel(5);
//-------------------------------------------------------------------------------------------------------------------
uint8 photoelectric_get_channel(uint8 channel)
{
    if(channel >= PHOTO_CHANNEL_NUM)
        return 0;
    
    return photoelectric_data.raw_data[channel];
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取原始值（位图形式）
// 返回参数     uint32          原始值
// 使用示例     value = photoelectric_get_raw_value();
//-------------------------------------------------------------------------------------------------------------------
uint32 photoelectric_get_raw_value(void)
{
    return photoelectric_data.raw_value;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取所有通道数据
// 参数说明     data            数据数组指针（至少18字节）
// 使用示例     uint8 data[18]; photoelectric_get_all_channels(data);
//-------------------------------------------------------------------------------------------------------------------
void photoelectric_get_all_channels(uint8 *data)
{
    if(data == NULL)
        return;
    
    memcpy(data, photoelectric_data.raw_data, PHOTO_CHANNEL_NUM);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算线位置
// 使用示例     photoelectric_calculate_position();
// 备注信息     使用加权平均算法计算线的位置，范围 -1.0 ~ 1.0
//-------------------------------------------------------------------------------------------------------------------
void photoelectric_calculate_position(void)
{
    int32 sum_position = 0;
    int32 sum_weight = 0;
    uint8 i;
    uint8 line_width = 0;
    
    // 使用加权平均法计算线位置
    for(i = 0; i < PHOTO_CHANNEL_NUM; i++)
    {
        if(photoelectric_data.raw_data[i])
        {
            // 权重为 (i - 8.5)，使中心位置为0
            // 范围：-8.5 到 8.5
            int32 weight = (i * 2 - 17);  // 相当于 (i - 8.5) * 2
            sum_position += weight * photoelectric_data.raw_data[i];
            sum_weight += photoelectric_data.raw_data[i];
            line_width++;
        }
    }
    
    if(sum_weight > 0)
    {
        // 计算位置，范围 -1.0 ~ 1.0
        photoelectric_data.line_position = (float)sum_position / (float)(sum_weight * 17);
        photoelectric_data.line_width = line_width;
        photoelectric_data.valid = 1;
    }
    else
    {
        // 没有检测到线
        photoelectric_data.line_position = 0.0f;
        photoelectric_data.line_width = 0;
        photoelectric_data.valid = 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取线位置
// 返回参数     float           线位置 (-1.0 ~ 1.0)
// 使用示例     position = photoelectric_get_line_position();
//-------------------------------------------------------------------------------------------------------------------
float photoelectric_get_line_position(void)
{
    return photoelectric_data.line_position;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取线宽
// 返回参数     uint8           线宽（检测到的传感器数量）
// 使用示例     width = photoelectric_get_line_width();
//-------------------------------------------------------------------------------------------------------------------
uint8 photoelectric_get_line_width(void)
{
    return photoelectric_data.line_width;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检查数据是否有效
// 返回参数     uint8           1=有效，0=无效
// 使用示例     if(photoelectric_is_valid()) { ... }
//-------------------------------------------------------------------------------------------------------------------
uint8 photoelectric_is_valid(void)
{
    return photoelectric_data.valid;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     打印光电管状态（调试用）
// 使用示例     photoelectric_print_status();
//-------------------------------------------------------------------------------------------------------------------
void photoelectric_print_status(void)
{
    uint8 i;
    
    printf("光电管状态：[");
    for(i = 0; i < PHOTO_CHANNEL_NUM; i++)
    {
        printf("%d", photoelectric_data.raw_data[i]);
        if(i < PHOTO_CHANNEL_NUM - 1)
            printf(" ");
    }
    printf("]");
    
    printf(" | 位置: %.3f", photoelectric_data.line_position);
    printf(" | 线宽: %d", photoelectric_data.line_width);
    printf(" | 有效: %s", photoelectric_data.valid ? "是" : "否");
    printf(" | 原始值: 0x%05X\r\n", photoelectric_data.raw_value);
}

