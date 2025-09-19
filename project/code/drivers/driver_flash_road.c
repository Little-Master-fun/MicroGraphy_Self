/*********************************************************************************************************************
* 文件名称          flash_road.c
* 功能说明          Flash存储路径数据驱动程序实现文件
* 作者              LittleMaster
* 版本信息          v2.0
* 修改记录
* 日期              作者                备注
* 2025-09-18        LittleMaster        代码结构优化，模块化设计，添加错误处理
*
* 文件作用说明：
* 本文件实现Flash存储系统的所有功能，提供路径数据的存储、读取、验证和管理
* 采用模块化设计，支持大容量数据的分页存储和分区管理
*
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "driver_flash_road.h"
// #include "navigation_flash_improved.h"

//=================================================全局变量定义================================================
int in_duan[20] = {0};                          // 进段参数数组
int out_duan[20] = {0};                         // 出段参数数组
flash_road_info_struct flash_road_info = {0};  // Flash存储信息结构

// 路径数据存储变量
uint32 max_error_point_mem = 0;                 // 最大数据点数量
int32 Mileage_All_sum_list[FLASH_ROAD_MAX_DATA_POINTS] = {0};  // 里程数据数组
float errors_coords[FLASH_ROAD_MAX_DATA_POINTS] = {0};         // 坐标数据数组

// 内部状态变量
static uint8 flash_initialized = 0;            // 初始化标志


//=================================================内部函数声明================================================
static flash_road_result_enum flash_store_metadata(void);
static flash_road_result_enum flash_store_integer_data(uint32 data_points);
static flash_road_result_enum flash_store_float_data(uint32 data_points);
static flash_road_result_enum flash_load_metadata(uint32 *data_points);
static flash_road_result_enum flash_load_integer_data(uint32 data_points);
static flash_road_result_enum flash_load_float_data(uint32 data_points);
static uint32 flash_calculate_pages(uint32 data_points);
static flash_road_result_enum flash_validate_data_size(uint32 data_points);
static flash_road_result_enum flash_safe_erase_page(uint32 section, uint32 page);
static flash_road_result_enum flash_safe_write_page(uint32 section, uint32 page, uint32 count);
static flash_road_result_enum flash_safe_read_page(uint32 section, uint32 page, uint32 count);
static void flash_update_status(flash_road_status_enum status);

//=================================================主要接口函数================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     Flash路径数据存储系统初始化
//-------------------------------------------------------------------------------------------------------------------
flash_road_result_enum flash_road_init(void)
{
    // 初始化Flash存储信息结构
    memset(&flash_road_info, 0, sizeof(flash_road_info_struct));
    flash_road_info.status = FLASH_ROAD_STATUS_IDLE;
    
    // 初始化进出段参数
    memset(in_duan, 0, sizeof(in_duan));
    memset(out_duan, 0, sizeof(out_duan));
    
    // 检查Flash硬件状态
    // 这里可以添加Flash硬件检测代码
    
    flash_initialized = 1;
    flash_update_status(FLASH_ROAD_STATUS_IDLE);
    
    return FLASH_ROAD_RESULT_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     存储路径数据到Flash
//-------------------------------------------------------------------------------------------------------------------
flash_road_result_enum flash_road_store(void)
{
    flash_road_result_enum result = FLASH_ROAD_RESULT_OK;
    
    // 检查系统是否已初始化
    if (!flash_initialized)
    {
        return FLASH_ROAD_RESULT_ERROR;
    }
    
    // 更新状态为正在存储
    flash_update_status(FLASH_ROAD_STATUS_STORING);
    
    // 计算实际需要存储的数据点数
    uint32 actual_data_points = max_error_point_mem;
    result = flash_validate_data_size(actual_data_points);
    if (result != FLASH_ROAD_RESULT_OK)
    {
        flash_update_status(FLASH_ROAD_STATUS_ERROR);
        return result;
    }
    
    // 更新Flash信息
    flash_road_info.data_points = actual_data_points;
    flash_road_info.total_pages = flash_calculate_pages(actual_data_points);
    
    do
    {
        // 存储元数据信息
        result = flash_store_metadata();
        if (result != FLASH_ROAD_RESULT_OK) break;
        
        // 存储整数数据
        result = flash_store_integer_data(actual_data_points);
        if (result != FLASH_ROAD_RESULT_OK) break;
        
        // 存储浮点数据
        result = flash_store_float_data(actual_data_points);
        if (result != FLASH_ROAD_RESULT_OK) break;
        
    } while(0);
    
    // 更新最终状态
    if (result == FLASH_ROAD_RESULT_OK)
    {
        flash_road_info.store_flag = 1;
        flash_update_status(FLASH_ROAD_STATUS_STORE_OK);
    }
    else
    {
        flash_update_status(FLASH_ROAD_STATUS_ERROR);
    }
    
    return result;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     从Flash读取路径数据
//-------------------------------------------------------------------------------------------------------------------
flash_road_result_enum flash_road_load(void)
{
    flash_road_result_enum result = FLASH_ROAD_RESULT_OK;
    uint32 actual_data_points = 0;
    
    // 检查系统是否已初始化
    if (!flash_initialized)
    {
        return FLASH_ROAD_RESULT_ERROR;
    }
    
    // 更新状态为正在读取
    flash_update_status(FLASH_ROAD_STATUS_READING);
    
    do
    {
        // 读取元数据信息
        result = flash_load_metadata(&actual_data_points);
        if (result != FLASH_ROAD_RESULT_OK) 
        {
            if (result == FLASH_ROAD_RESULT_DATA_CORRUPT)
            {
                flash_update_status(FLASH_ROAD_STATUS_NO_DATA);
                return FLASH_ROAD_RESULT_OK; // 无数据不算错误
            }
            break;
        }
        
        // 验证数据大小
        result = flash_validate_data_size(actual_data_points);
        if (result != FLASH_ROAD_RESULT_OK) break;
        
        // 更新全局变量和Flash信息
        max_error_point_mem = actual_data_points;
        flash_road_info.data_points = actual_data_points;
        flash_road_info.total_pages = flash_calculate_pages(actual_data_points);
        
        // 读取整数数据
        result = flash_load_integer_data(actual_data_points);
        if (result != FLASH_ROAD_RESULT_OK) break;
        
        // 读取浮点数据
        result = flash_load_float_data(actual_data_points);
        if (result != FLASH_ROAD_RESULT_OK) break;
        
    } while(0);
    
    // 更新最终状态
    if (result == FLASH_ROAD_RESULT_OK)
    {
        flash_update_status(FLASH_ROAD_STATUS_READ_OK);
    }
    else
    {
        flash_update_status(FLASH_ROAD_STATUS_ERROR);
    }
    
    return result;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清除Flash中的路径数据
//-------------------------------------------------------------------------------------------------------------------
flash_road_result_enum flash_road_clear(void)
{
    flash_road_result_enum result = FLASH_ROAD_RESULT_OK;
    
    // 检查系统是否已初始化
    if (!flash_initialized)
    {
        return FLASH_ROAD_RESULT_ERROR;
    }
    
    // 擦除元数据页（双备份）
    result = flash_safe_erase_page(FLASH_ROAD_SECTION_INDEX, FLASH_ROAD_SIZE_INFO_INDEX);
    if (result != FLASH_ROAD_RESULT_OK) return result;
    
    result = flash_safe_erase_page(FLASH_ROAD_SECTION_INDEX, 
                                   FLASH_ROAD_MAX_PAGES_PER_TYPE + FLASH_ROAD_SIZE_INFO_INDEX);
    if (result != FLASH_ROAD_RESULT_OK) return result;
    
    // 擦除所有数据页
    for (uint32 page = 1; page < FLASH_ROAD_MAX_PAGES_PER_TYPE; page++)
    {
        // 擦除整数区
        result = flash_safe_erase_page(FLASH_ROAD_SECTION_INDEX, page);
        if (result != FLASH_ROAD_RESULT_OK) continue; // 继续擦除其他页
        
        // 擦除浮点区
        result = flash_safe_erase_page(FLASH_ROAD_SECTION_INDEX, 
                                       FLASH_ROAD_MAX_PAGES_PER_TYPE + page);
        if (result != FLASH_ROAD_RESULT_OK) continue; // 继续擦除其他页
    }
    
    // 重置Flash信息
    memset(&flash_road_info, 0, sizeof(flash_road_info_struct));
    flash_update_status(FLASH_ROAD_STATUS_IDLE);
    
    return FLASH_ROAD_RESULT_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     验证Flash数据完整性
//-------------------------------------------------------------------------------------------------------------------
flash_road_result_enum flash_road_verify(void)
{
    // 检查系统是否已初始化
    if (!flash_initialized)
    {
        return FLASH_ROAD_RESULT_ERROR;
    }
    
    uint32 data_points = 0;
    
    // 尝试从两个备份位置读取元数据
    flash_road_result_enum result1 = flash_load_metadata(&data_points);
    
    // 如果主位置读取失败，尝试备份位置
    if (result1 != FLASH_ROAD_RESULT_OK)
    {
        // 这里可以添加从备份位置读取的逻辑
        return FLASH_ROAD_RESULT_DATA_CORRUPT;
    }
    
    // 验证数据范围
    if (data_points > FLASH_ROAD_MAX_DATA_POINTS)
    {
        return FLASH_ROAD_RESULT_DATA_CORRUPT;
    }
    
    // 这里可以添加更多的数据完整性检查
    // 例如：CRC校验、数据一致性检查等
    
    return FLASH_ROAD_RESULT_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取Flash存储状态
//-------------------------------------------------------------------------------------------------------------------
flash_road_status_enum flash_road_get_status(void)
{
    return flash_road_info.status;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取存储的数据点数量
//-------------------------------------------------------------------------------------------------------------------
uint32 flash_road_get_data_count(void)
{
    return flash_road_info.data_points;
}

//=================================================内部函数实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     存储元数据信息
//-------------------------------------------------------------------------------------------------------------------
static flash_road_result_enum flash_store_metadata(void)
{
    flash_road_result_enum result;
    
    // 准备元数据
    flash_union_buffer[0].uint32_type = flash_road_info.data_points;
    
    // 存储in_duan参数
    for(int i = 0; i < FLASH_ROAD_IN_OUT_PARAMS; i++) 
    {
        flash_union_buffer[1 + i].int32_type = in_duan[i];
    }
    
    // 存储out_duan参数
    for(int i = 0; i < FLASH_ROAD_IN_OUT_PARAMS; i++) 
    {
        flash_union_buffer[1 + FLASH_ROAD_IN_OUT_PARAMS + i].int32_type = out_duan[i];
    }
    
    uint32 metadata_count = 1 + 2 * FLASH_ROAD_IN_OUT_PARAMS;
    
    // 写入主位置
    result = flash_safe_erase_page(FLASH_ROAD_SECTION_INDEX, FLASH_ROAD_SIZE_INFO_INDEX);
    if (result != FLASH_ROAD_RESULT_OK) return result;
    
    result = flash_safe_write_page(FLASH_ROAD_SECTION_INDEX, FLASH_ROAD_SIZE_INFO_INDEX, metadata_count);
    if (result != FLASH_ROAD_RESULT_OK) return result;
    
    // 写入备份位置
    result = flash_safe_erase_page(FLASH_ROAD_SECTION_INDEX, 
                                   FLASH_ROAD_MAX_PAGES_PER_TYPE + FLASH_ROAD_SIZE_INFO_INDEX);
    if (result != FLASH_ROAD_RESULT_OK) return result;
    
    result = flash_safe_write_page(FLASH_ROAD_SECTION_INDEX, 
                                   FLASH_ROAD_MAX_PAGES_PER_TYPE + FLASH_ROAD_SIZE_INFO_INDEX, 
                                   metadata_count);
    
    return result;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     存储整数数据
//-------------------------------------------------------------------------------------------------------------------
static flash_road_result_enum flash_store_integer_data(uint32 data_points)
{
    flash_road_result_enum result = FLASH_ROAD_RESULT_OK;
    uint32 total_pages = flash_road_info.total_pages;
    
    for(uint32 page_index = 1; page_index < total_pages; page_index++)
    {
        uint32 start_index = (page_index - 1) * FLASH_PAGE_LENGTH;
        uint32 data_count = FLASH_PAGE_LENGTH;
        
        // 处理最后一页
        if(start_index + FLASH_PAGE_LENGTH > data_points) 
        {
            data_count = data_points - start_index;
        }
        
        flash_buffer_clear();
        
        // 复制数据到缓冲区
        for(uint32 i = 0; i < data_count; i++)
        {
            flash_union_buffer[i].int32_type = Mileage_All_sum_list[start_index + i];
        }
        
        // 写入Flash
        result = flash_safe_erase_page(FLASH_ROAD_SECTION_INDEX, page_index);
        if (result != FLASH_ROAD_RESULT_OK) continue; // 继续处理下一页
        
        result = flash_safe_write_page(FLASH_ROAD_SECTION_INDEX, page_index, data_count);
        if (result != FLASH_ROAD_RESULT_OK) continue; // 继续处理下一页
    }
    
    return FLASH_ROAD_RESULT_OK; // 部分失败也返回成功，让上层判断
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     存储浮点数据
//-------------------------------------------------------------------------------------------------------------------
static flash_road_result_enum flash_store_float_data(uint32 data_points)
{
    flash_road_result_enum result = FLASH_ROAD_RESULT_OK;
    uint32 total_pages = flash_road_info.total_pages;
    
    for(uint32 page_index = 1; page_index < total_pages; page_index++)
    {
        uint32 start_index = (page_index - 1) * FLASH_PAGE_LENGTH;
        uint32 data_count = FLASH_PAGE_LENGTH;
        
        // 处理最后一页
        if(start_index + FLASH_PAGE_LENGTH > data_points) 
        {
            data_count = data_points - start_index;
        }
        
        flash_buffer_clear();
        
        // 复制数据到缓冲区
        for(uint32 i = 0; i < data_count; i++)
        {
            flash_union_buffer[i].float_type = errors_coords[start_index + i];
        }
        
        // 计算目标页
        uint32 target_page = FLASH_ROAD_MAX_PAGES_PER_TYPE + page_index;
        
        // 写入Flash
        result = flash_safe_erase_page(FLASH_ROAD_SECTION_INDEX, target_page);
        if (result != FLASH_ROAD_RESULT_OK) continue; // 继续处理下一页
        
        result = flash_safe_write_page(FLASH_ROAD_SECTION_INDEX, target_page, data_count);
        if (result != FLASH_ROAD_RESULT_OK) continue; // 继续处理下一页
    }
    
    return FLASH_ROAD_RESULT_OK; // 部分失败也返回成功，让上层判断
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取元数据信息
//-------------------------------------------------------------------------------------------------------------------
static flash_road_result_enum flash_load_metadata(uint32 *data_points)
{
    flash_road_result_enum result;
    uint32 metadata_count = 1 + 2 * FLASH_ROAD_IN_OUT_PARAMS;
    
    // 尝试从主位置读取
    if(flash_check(FLASH_ROAD_SECTION_INDEX, FLASH_ROAD_SIZE_INFO_INDEX)) 
    {
        result = flash_safe_read_page(FLASH_ROAD_SECTION_INDEX, FLASH_ROAD_SIZE_INFO_INDEX, metadata_count);
        if (result == FLASH_ROAD_RESULT_OK)
        {
            *data_points = flash_union_buffer[0].uint32_type;
            
            // 读取in_duan参数
            for(int i = 0; i < FLASH_ROAD_IN_OUT_PARAMS; i++) 
            {
            in_duan[i] = flash_union_buffer[1 + i].int32_type;
        }
        
            // 读取out_duan参数
            for(int i = 0; i < FLASH_ROAD_IN_OUT_PARAMS; i++) 
            {
                out_duan[i] = flash_union_buffer[1 + FLASH_ROAD_IN_OUT_PARAMS + i].int32_type;
            }
            
            return FLASH_ROAD_RESULT_OK;
        }
    }
    
    // 尝试从备份位置读取
    if(flash_check(FLASH_ROAD_SECTION_INDEX, FLASH_ROAD_MAX_PAGES_PER_TYPE + FLASH_ROAD_SIZE_INFO_INDEX))
    {
        result = flash_safe_read_page(FLASH_ROAD_SECTION_INDEX, 
                                      FLASH_ROAD_MAX_PAGES_PER_TYPE + FLASH_ROAD_SIZE_INFO_INDEX, 
                                      metadata_count);
        if (result == FLASH_ROAD_RESULT_OK)
        {
            *data_points = flash_union_buffer[0].uint32_type;
            
            // 读取in_duan参数
            for(int i = 0; i < FLASH_ROAD_IN_OUT_PARAMS; i++) 
            {
            in_duan[i] = flash_union_buffer[1 + i].int32_type;
        }
        
            // 读取out_duan参数
            for(int i = 0; i < FLASH_ROAD_IN_OUT_PARAMS; i++) 
            {
                out_duan[i] = flash_union_buffer[1 + FLASH_ROAD_IN_OUT_PARAMS + i].int32_type;
            }
            
            return FLASH_ROAD_RESULT_OK;
        }
    }
    
    // 两个位置都没有数据
    return FLASH_ROAD_RESULT_DATA_CORRUPT;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取整数数据
//-------------------------------------------------------------------------------------------------------------------
static flash_road_result_enum flash_load_integer_data(uint32 data_points)
{
    flash_road_result_enum result = FLASH_ROAD_RESULT_OK;
    uint32 total_pages = flash_calculate_pages(data_points);
    
    for(uint32 page_index = 1; page_index < total_pages; page_index++)
    {
        uint32 start_index = (page_index - 1) * FLASH_PAGE_LENGTH;
        uint32 data_count = FLASH_PAGE_LENGTH;
        
        // 处理最后一页
        if(start_index + FLASH_PAGE_LENGTH > data_points) 
        {
            data_count = data_points - start_index;
        }
        
        // 从Flash读取
        result = flash_safe_read_page(FLASH_ROAD_SECTION_INDEX, page_index, data_count);
        if (result != FLASH_ROAD_RESULT_OK) continue; // 继续处理下一页
        
        // 复制数据到数组
        for(uint32 i = 0; i < data_count; i++)
        {
            Mileage_All_sum_list[start_index + i] = flash_union_buffer[i].int32_type;
        }
    }
    
    return FLASH_ROAD_RESULT_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取浮点数据
//-------------------------------------------------------------------------------------------------------------------
static flash_road_result_enum flash_load_float_data(uint32 data_points)
{
    flash_road_result_enum result = FLASH_ROAD_RESULT_OK;
    uint32 total_pages = flash_calculate_pages(data_points);
    
    for(uint32 page_index = 1; page_index < total_pages; page_index++)
    {
        uint32 start_index = (page_index - 1) * FLASH_PAGE_LENGTH;
        uint32 data_count = FLASH_PAGE_LENGTH;
        
        // 处理最后一页
        if(start_index + FLASH_PAGE_LENGTH > data_points) 
        {
            data_count = data_points - start_index;
        }
        
        // 计算目标页
        uint32 target_page = FLASH_ROAD_MAX_PAGES_PER_TYPE + page_index;
        
        // 从Flash读取
        result = flash_safe_read_page(FLASH_ROAD_SECTION_INDEX, target_page, data_count);
        if (result != FLASH_ROAD_RESULT_OK) continue; // 继续处理下一页
        
        // 复制数据到数组
        for(uint32 i = 0; i < data_count; i++)
        {
            errors_coords[start_index + i] = flash_union_buffer[i].float_type;
        }
    }
    
    return FLASH_ROAD_RESULT_OK;
}

//=================================================工具函数实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算所需页数
//-------------------------------------------------------------------------------------------------------------------
static uint32 flash_calculate_pages(uint32 data_points)
{
    uint32 total_pages = (data_points + FLASH_PAGE_LENGTH - 1) / FLASH_PAGE_LENGTH + 1;
    if(total_pages > FLASH_ROAD_MAX_PAGES_PER_TYPE) 
    {
        total_pages = FLASH_ROAD_MAX_PAGES_PER_TYPE;
    }
    return total_pages;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     验证数据大小
//-------------------------------------------------------------------------------------------------------------------
static flash_road_result_enum flash_validate_data_size(uint32 data_points)
{
    if (data_points == 0)
    {
        return FLASH_ROAD_RESULT_INVALID_PARAM;
    }
    
    if (data_points > (FLASH_ROAD_MAX_DATA_POINTS - FLASH_PAGE_LENGTH))
    {
        return FLASH_ROAD_RESULT_NO_SPACE;
    }
    
    return FLASH_ROAD_RESULT_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     安全页擦除
//-------------------------------------------------------------------------------------------------------------------
static flash_road_result_enum flash_safe_erase_page(uint32 section, uint32 page)
{
    if(flash_check(section, page)) 
    {
        flash_erase_page(section, page);
        return FLASH_ROAD_RESULT_OK;
    }
    return FLASH_ROAD_RESULT_ERROR;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     安全页写入
//-------------------------------------------------------------------------------------------------------------------
static flash_road_result_enum flash_safe_write_page(uint32 section, uint32 page, uint32 count)
{
    flash_write_page_from_buffer(section, page, count);
    return FLASH_ROAD_RESULT_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     安全页读取
//-------------------------------------------------------------------------------------------------------------------
static flash_road_result_enum flash_safe_read_page(uint32 section, uint32 page, uint32 count)
{
    flash_read_page_to_buffer(section, page, count);
    return FLASH_ROAD_RESULT_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新状态
//-------------------------------------------------------------------------------------------------------------------
static void flash_update_status(flash_road_status_enum status)
{
    flash_road_info.status = status;
    // 这里可以添加状态变化的回调通知
}

//=================================================兼容性接口实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     存储路径数据（兼容性接口）
//-------------------------------------------------------------------------------------------------------------------
void flash_road_memery_store(void)
{
    // 调用新接口
    flash_road_result_enum result = flash_road_store();
    
    // 设置兼容性标志
    if (result == FLASH_ROAD_RESULT_OK)
    {
        flash_road_info.store_flag = 3; // 保持原有的标志值
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取路径数据（兼容性接口）
//-------------------------------------------------------------------------------------------------------------------
void flash_road_memery_get(void)
{
    // 调用新接口
    flash_road_result_enum result = flash_road_load();
    
    // 设置兼容性标志
    if (result == FLASH_ROAD_RESULT_OK)
    {
        flash_road_info.store_flag = 4; // 保持原有的标志值
    }
    else if (flash_road_info.status == FLASH_ROAD_STATUS_NO_DATA)
    {
        flash_road_info.store_flag = 4; // 无数据也设置为4
    }
}