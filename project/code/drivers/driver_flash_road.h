/*********************************************************************************************************************
* 文件名称          flash_road.h
* 功能说明          Flash存储路径数据驱动程序头文件
* 作者              LittleMaster
* 版本信息          v2.0
* 修改记录
* 日期              作者                备注
* 2025-09-18        LittleMaster        代码结构优化，添加详细注释
*
* 文件作用说明：
* 本文件为Flash存储系统的头文件，用于存储和读取机器人路径数据
* 支持整数和浮点数数据的分区存储，具备数据完整性验证功能
*
********************************************************************************************************************/

#ifndef _FLASH_ROAD_H_
#define _FLASH_ROAD_H_

#include "zf_common_typedef.h"

//=================================================配置参数定义================================================
#define FLASH_ROAD_SECTION_INDEX        (0)         // 存储数据用的扇区
#define FLASH_ROAD_MAX_DATA_POINTS      (24576)     // 最大数据点数
#define FLASH_ROAD_MAX_PAGES_PER_TYPE   (48)        // 每种数据类型最大页数
#define FLASH_ROAD_SIZE_INFO_INDEX      (0)         // 存储大小信息的页索引
#define FLASH_ROAD_IN_OUT_PARAMS        (6)         // 进出段参数数量

//=================================================状态枚举定义================================================
// Flash存储状态枚举
typedef enum
{
    FLASH_ROAD_STATUS_IDLE          = 0,        // 空闲状态
    FLASH_ROAD_STATUS_STORING       = 1,        // 正在存储
    FLASH_ROAD_STATUS_READING       = 2,        // 正在读取
    FLASH_ROAD_STATUS_STORE_OK      = 3,        // 存储完成
    FLASH_ROAD_STATUS_READ_OK       = 4,        // 读取完成
    FLASH_ROAD_STATUS_ERROR         = 5,        // 操作错误
    FLASH_ROAD_STATUS_NO_DATA       = 6,        // 无数据
    FLASH_ROAD_STATUS_DATA_CORRUPT  = 7         // 数据损坏
} flash_road_status_enum;

// Flash操作结果枚举
typedef enum
{
    FLASH_ROAD_RESULT_OK            = 0,        // 操作成功
    FLASH_ROAD_RESULT_ERROR         = 1,        // 操作失败
    FLASH_ROAD_RESULT_INVALID_PARAM = 2,        // 参数无效
    FLASH_ROAD_RESULT_NO_SPACE      = 3,        // 存储空间不足
    FLASH_ROAD_RESULT_DATA_CORRUPT  = 4         // 数据损坏
} flash_road_result_enum;

//=================================================数据结构定义================================================
// Flash存储信息结构
typedef struct
{
    uint32 data_points;                         // 实际数据点数
    uint32 total_pages;                         // 总页数
    flash_road_status_enum status;              // 当前状态
    uint8 store_flag;                           // 存储标志
} flash_road_info_struct;

//=================================================全局变量声明================================================
extern int in_duan[20];                         // 进段参数数组
extern int out_duan[20];                        // 出段参数数组
extern flash_road_info_struct flash_road_info;  // Flash存储信息

// 路径数据存储变量
extern uint32 max_error_point_mem;              // 最大数据点数量
extern int32 Mileage_All_sum_list[FLASH_ROAD_MAX_DATA_POINTS];  // 里程数据数组
extern float errors_coords[FLASH_ROAD_MAX_DATA_POINTS];         // 坐标数据数组

//=================================================函数声明================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     Flash路径数据存储系统初始化
// 参数说明     void
// 返回参数     flash_road_result_enum     初始化结果
// 使用示例     flash_road_init();
// 备注信息     初始化Flash存储系统，检查存储状态
//-------------------------------------------------------------------------------------------------------------------
flash_road_result_enum flash_road_init(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     存储路径数据到Flash
// 参数说明     void
// 返回参数     flash_road_result_enum     存储结果
// 使用示例     flash_road_store();
// 备注信息     将路径数据存储到Flash，包括里程数据和坐标数据
//-------------------------------------------------------------------------------------------------------------------
flash_road_result_enum flash_road_store(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     从Flash读取路径数据
// 参数说明     void
// 返回参数     flash_road_result_enum     读取结果
// 使用示例     flash_road_load();
// 备注信息     从Flash读取路径数据到内存数组
//-------------------------------------------------------------------------------------------------------------------
flash_road_result_enum flash_road_load(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清除Flash中的路径数据
// 参数说明     void
// 返回参数     flash_road_result_enum     清除结果
// 使用示例     flash_road_clear();
// 备注信息     擦除Flash中存储的所有路径数据
//-------------------------------------------------------------------------------------------------------------------
flash_road_result_enum flash_road_clear(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     验证Flash数据完整性
// 参数说明     void
// 返回参数     flash_road_result_enum     验证结果
// 使用示例     flash_road_verify();
// 备注信息     检查Flash中数据的完整性和有效性
//-------------------------------------------------------------------------------------------------------------------
flash_road_result_enum flash_road_verify(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取Flash存储状态
// 参数说明     void
// 返回参数     flash_road_status_enum     当前状态
// 使用示例     status = flash_road_get_status();
// 备注信息     获取当前Flash存储系统的状态
//-------------------------------------------------------------------------------------------------------------------
flash_road_status_enum flash_road_get_status(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取存储的数据点数量
// 参数说明     void
// 返回参数     uint32                     数据点数量
// 使用示例     count = flash_road_get_data_count();
// 备注信息     获取Flash中存储的有效数据点数量
//-------------------------------------------------------------------------------------------------------------------
uint32 flash_road_get_data_count(void);

// 兼容性接口（保持向后兼容）
void flash_road_memery_store(void);         // 旧接口：存储数据
void flash_road_memery_get(void);           // 旧接口：读取数据

#endif