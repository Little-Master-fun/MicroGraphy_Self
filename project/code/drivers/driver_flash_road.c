/*********************************************************************************************************************
* �ļ�����          flash_road.c
* ����˵��          Flash�洢·��������������ʵ���ļ�
* ����              LittleMaster
* �汾��Ϣ          v2.0
* �޸ļ�¼
* ����              ����                ��ע
* 2025-09-18        LittleMaster        ����ṹ�Ż���ģ�黯��ƣ���Ӵ�����
*
* �ļ�����˵����
* ���ļ�ʵ��Flash�洢ϵͳ�����й��ܣ��ṩ·�����ݵĴ洢����ȡ����֤�͹���
* ����ģ�黯��ƣ�֧�ִ��������ݵķ�ҳ�洢�ͷ�������
*
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "driver_flash_road.h"
// #include "navigation_flash_improved.h"

//=================================================ȫ�ֱ�������================================================
int in_duan[20] = {0};                          // ���β�������
int out_duan[20] = {0};                         // ���β�������
flash_road_info_struct flash_road_info = {0};  // Flash�洢��Ϣ�ṹ

// ·�����ݴ洢����
uint32 max_error_point_mem = 0;                 // ������ݵ�����
int32 Mileage_All_sum_list[FLASH_ROAD_MAX_DATA_POINTS] = {0};  // �����������
float errors_coords[FLASH_ROAD_MAX_DATA_POINTS] = {0};         // ������������

// �ڲ�״̬����
static uint8 flash_initialized = 0;            // ��ʼ����־


//=================================================�ڲ���������================================================
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

//=================================================��Ҫ�ӿں���================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     Flash·�����ݴ洢ϵͳ��ʼ��
//-------------------------------------------------------------------------------------------------------------------
flash_road_result_enum flash_road_init(void)
{
    // ��ʼ��Flash�洢��Ϣ�ṹ
    memset(&flash_road_info, 0, sizeof(flash_road_info_struct));
    flash_road_info.status = FLASH_ROAD_STATUS_IDLE;
    
    // ��ʼ�������β���
    memset(in_duan, 0, sizeof(in_duan));
    memset(out_duan, 0, sizeof(out_duan));
    
    // ���FlashӲ��״̬
    // ����������FlashӲ��������
    
    flash_initialized = 1;
    flash_update_status(FLASH_ROAD_STATUS_IDLE);
    
    return FLASH_ROAD_RESULT_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     �洢·�����ݵ�Flash
//-------------------------------------------------------------------------------------------------------------------
flash_road_result_enum flash_road_store(void)
{
    flash_road_result_enum result = FLASH_ROAD_RESULT_OK;
    
    // ���ϵͳ�Ƿ��ѳ�ʼ��
    if (!flash_initialized)
    {
        return FLASH_ROAD_RESULT_ERROR;
    }
    
    // ����״̬Ϊ���ڴ洢
    flash_update_status(FLASH_ROAD_STATUS_STORING);
    
    // ����ʵ����Ҫ�洢�����ݵ���
    uint32 actual_data_points = max_error_point_mem;
    result = flash_validate_data_size(actual_data_points);
    if (result != FLASH_ROAD_RESULT_OK)
    {
        flash_update_status(FLASH_ROAD_STATUS_ERROR);
        return result;
    }
    
    // ����Flash��Ϣ
    flash_road_info.data_points = actual_data_points;
    flash_road_info.total_pages = flash_calculate_pages(actual_data_points);
    
    do
    {
        // �洢Ԫ������Ϣ
        result = flash_store_metadata();
        if (result != FLASH_ROAD_RESULT_OK) break;
        
        // �洢��������
        result = flash_store_integer_data(actual_data_points);
        if (result != FLASH_ROAD_RESULT_OK) break;
        
        // �洢��������
        result = flash_store_float_data(actual_data_points);
        if (result != FLASH_ROAD_RESULT_OK) break;
        
    } while(0);
    
    // ��������״̬
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
// �������     ��Flash��ȡ·������
//-------------------------------------------------------------------------------------------------------------------
flash_road_result_enum flash_road_load(void)
{
    flash_road_result_enum result = FLASH_ROAD_RESULT_OK;
    uint32 actual_data_points = 0;
    
    // ���ϵͳ�Ƿ��ѳ�ʼ��
    if (!flash_initialized)
    {
        return FLASH_ROAD_RESULT_ERROR;
    }
    
    // ����״̬Ϊ���ڶ�ȡ
    flash_update_status(FLASH_ROAD_STATUS_READING);
    
    do
    {
        // ��ȡԪ������Ϣ
        result = flash_load_metadata(&actual_data_points);
        if (result != FLASH_ROAD_RESULT_OK) 
        {
            if (result == FLASH_ROAD_RESULT_DATA_CORRUPT)
            {
                flash_update_status(FLASH_ROAD_STATUS_NO_DATA);
                return FLASH_ROAD_RESULT_OK; // �����ݲ������
            }
            break;
        }
        
        // ��֤���ݴ�С
        result = flash_validate_data_size(actual_data_points);
        if (result != FLASH_ROAD_RESULT_OK) break;
        
        // ����ȫ�ֱ�����Flash��Ϣ
        max_error_point_mem = actual_data_points;
        flash_road_info.data_points = actual_data_points;
        flash_road_info.total_pages = flash_calculate_pages(actual_data_points);
        
        // ��ȡ��������
        result = flash_load_integer_data(actual_data_points);
        if (result != FLASH_ROAD_RESULT_OK) break;
        
        // ��ȡ��������
        result = flash_load_float_data(actual_data_points);
        if (result != FLASH_ROAD_RESULT_OK) break;
        
    } while(0);
    
    // ��������״̬
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
// �������     ���Flash�е�·������
//-------------------------------------------------------------------------------------------------------------------
flash_road_result_enum flash_road_clear(void)
{
    flash_road_result_enum result = FLASH_ROAD_RESULT_OK;
    
    // ���ϵͳ�Ƿ��ѳ�ʼ��
    if (!flash_initialized)
    {
        return FLASH_ROAD_RESULT_ERROR;
    }
    
    // ����Ԫ����ҳ��˫���ݣ�
    result = flash_safe_erase_page(FLASH_ROAD_SECTION_INDEX, FLASH_ROAD_SIZE_INFO_INDEX);
    if (result != FLASH_ROAD_RESULT_OK) return result;
    
    result = flash_safe_erase_page(FLASH_ROAD_SECTION_INDEX, 
                                   FLASH_ROAD_MAX_PAGES_PER_TYPE + FLASH_ROAD_SIZE_INFO_INDEX);
    if (result != FLASH_ROAD_RESULT_OK) return result;
    
    // ������������ҳ
    for (uint32 page = 1; page < FLASH_ROAD_MAX_PAGES_PER_TYPE; page++)
    {
        // ����������
        result = flash_safe_erase_page(FLASH_ROAD_SECTION_INDEX, page);
        if (result != FLASH_ROAD_RESULT_OK) continue; // ������������ҳ
        
        // ����������
        result = flash_safe_erase_page(FLASH_ROAD_SECTION_INDEX, 
                                       FLASH_ROAD_MAX_PAGES_PER_TYPE + page);
        if (result != FLASH_ROAD_RESULT_OK) continue; // ������������ҳ
    }
    
    // ����Flash��Ϣ
    memset(&flash_road_info, 0, sizeof(flash_road_info_struct));
    flash_update_status(FLASH_ROAD_STATUS_IDLE);
    
    return FLASH_ROAD_RESULT_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��֤Flash����������
//-------------------------------------------------------------------------------------------------------------------
flash_road_result_enum flash_road_verify(void)
{
    // ���ϵͳ�Ƿ��ѳ�ʼ��
    if (!flash_initialized)
    {
        return FLASH_ROAD_RESULT_ERROR;
    }
    
    uint32 data_points = 0;
    
    // ���Դ���������λ�ö�ȡԪ����
    flash_road_result_enum result1 = flash_load_metadata(&data_points);
    
    // �����λ�ö�ȡʧ�ܣ����Ա���λ��
    if (result1 != FLASH_ROAD_RESULT_OK)
    {
        // ���������Ӵӱ���λ�ö�ȡ���߼�
        return FLASH_ROAD_RESULT_DATA_CORRUPT;
    }
    
    // ��֤���ݷ�Χ
    if (data_points > FLASH_ROAD_MAX_DATA_POINTS)
    {
        return FLASH_ROAD_RESULT_DATA_CORRUPT;
    }
    
    // ���������Ӹ�������������Լ��
    // ���磺CRCУ�顢����һ���Լ���
    
    return FLASH_ROAD_RESULT_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡFlash�洢״̬
//-------------------------------------------------------------------------------------------------------------------
flash_road_status_enum flash_road_get_status(void)
{
    return flash_road_info.status;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ�洢�����ݵ�����
//-------------------------------------------------------------------------------------------------------------------
uint32 flash_road_get_data_count(void)
{
    return flash_road_info.data_points;
}

//=================================================�ڲ�����ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     �洢Ԫ������Ϣ
//-------------------------------------------------------------------------------------------------------------------
static flash_road_result_enum flash_store_metadata(void)
{
    flash_road_result_enum result;
    
    // ׼��Ԫ����
    flash_union_buffer[0].uint32_type = flash_road_info.data_points;
    
    // �洢in_duan����
    for(int i = 0; i < FLASH_ROAD_IN_OUT_PARAMS; i++) 
    {
        flash_union_buffer[1 + i].int32_type = in_duan[i];
    }
    
    // �洢out_duan����
    for(int i = 0; i < FLASH_ROAD_IN_OUT_PARAMS; i++) 
    {
        flash_union_buffer[1 + FLASH_ROAD_IN_OUT_PARAMS + i].int32_type = out_duan[i];
    }
    
    uint32 metadata_count = 1 + 2 * FLASH_ROAD_IN_OUT_PARAMS;
    
    // д����λ��
    result = flash_safe_erase_page(FLASH_ROAD_SECTION_INDEX, FLASH_ROAD_SIZE_INFO_INDEX);
    if (result != FLASH_ROAD_RESULT_OK) return result;
    
    result = flash_safe_write_page(FLASH_ROAD_SECTION_INDEX, FLASH_ROAD_SIZE_INFO_INDEX, metadata_count);
    if (result != FLASH_ROAD_RESULT_OK) return result;
    
    // д�뱸��λ��
    result = flash_safe_erase_page(FLASH_ROAD_SECTION_INDEX, 
                                   FLASH_ROAD_MAX_PAGES_PER_TYPE + FLASH_ROAD_SIZE_INFO_INDEX);
    if (result != FLASH_ROAD_RESULT_OK) return result;
    
    result = flash_safe_write_page(FLASH_ROAD_SECTION_INDEX, 
                                   FLASH_ROAD_MAX_PAGES_PER_TYPE + FLASH_ROAD_SIZE_INFO_INDEX, 
                                   metadata_count);
    
    return result;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     �洢��������
//-------------------------------------------------------------------------------------------------------------------
static flash_road_result_enum flash_store_integer_data(uint32 data_points)
{
    flash_road_result_enum result = FLASH_ROAD_RESULT_OK;
    uint32 total_pages = flash_road_info.total_pages;
    
    for(uint32 page_index = 1; page_index < total_pages; page_index++)
    {
        uint32 start_index = (page_index - 1) * FLASH_PAGE_LENGTH;
        uint32 data_count = FLASH_PAGE_LENGTH;
        
        // �������һҳ
        if(start_index + FLASH_PAGE_LENGTH > data_points) 
        {
            data_count = data_points - start_index;
        }
        
        flash_buffer_clear();
        
        // �������ݵ�������
        for(uint32 i = 0; i < data_count; i++)
        {
            flash_union_buffer[i].int32_type = Mileage_All_sum_list[start_index + i];
        }
        
        // д��Flash
        result = flash_safe_erase_page(FLASH_ROAD_SECTION_INDEX, page_index);
        if (result != FLASH_ROAD_RESULT_OK) continue; // ����������һҳ
        
        result = flash_safe_write_page(FLASH_ROAD_SECTION_INDEX, page_index, data_count);
        if (result != FLASH_ROAD_RESULT_OK) continue; // ����������һҳ
    }
    
    return FLASH_ROAD_RESULT_OK; // ����ʧ��Ҳ���سɹ������ϲ��ж�
}

//-------------------------------------------------------------------------------------------------------------------
// �������     �洢��������
//-------------------------------------------------------------------------------------------------------------------
static flash_road_result_enum flash_store_float_data(uint32 data_points)
{
    flash_road_result_enum result = FLASH_ROAD_RESULT_OK;
    uint32 total_pages = flash_road_info.total_pages;
    
    for(uint32 page_index = 1; page_index < total_pages; page_index++)
    {
        uint32 start_index = (page_index - 1) * FLASH_PAGE_LENGTH;
        uint32 data_count = FLASH_PAGE_LENGTH;
        
        // �������һҳ
        if(start_index + FLASH_PAGE_LENGTH > data_points) 
        {
            data_count = data_points - start_index;
        }
        
        flash_buffer_clear();
        
        // �������ݵ�������
        for(uint32 i = 0; i < data_count; i++)
        {
            flash_union_buffer[i].float_type = errors_coords[start_index + i];
        }
        
        // ����Ŀ��ҳ
        uint32 target_page = FLASH_ROAD_MAX_PAGES_PER_TYPE + page_index;
        
        // д��Flash
        result = flash_safe_erase_page(FLASH_ROAD_SECTION_INDEX, target_page);
        if (result != FLASH_ROAD_RESULT_OK) continue; // ����������һҳ
        
        result = flash_safe_write_page(FLASH_ROAD_SECTION_INDEX, target_page, data_count);
        if (result != FLASH_ROAD_RESULT_OK) continue; // ����������һҳ
    }
    
    return FLASH_ROAD_RESULT_OK; // ����ʧ��Ҳ���سɹ������ϲ��ж�
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡԪ������Ϣ
//-------------------------------------------------------------------------------------------------------------------
static flash_road_result_enum flash_load_metadata(uint32 *data_points)
{
    flash_road_result_enum result;
    uint32 metadata_count = 1 + 2 * FLASH_ROAD_IN_OUT_PARAMS;
    
    // ���Դ���λ�ö�ȡ
    if(flash_check(FLASH_ROAD_SECTION_INDEX, FLASH_ROAD_SIZE_INFO_INDEX)) 
    {
        result = flash_safe_read_page(FLASH_ROAD_SECTION_INDEX, FLASH_ROAD_SIZE_INFO_INDEX, metadata_count);
        if (result == FLASH_ROAD_RESULT_OK)
        {
            *data_points = flash_union_buffer[0].uint32_type;
            
            // ��ȡin_duan����
            for(int i = 0; i < FLASH_ROAD_IN_OUT_PARAMS; i++) 
            {
            in_duan[i] = flash_union_buffer[1 + i].int32_type;
        }
        
            // ��ȡout_duan����
            for(int i = 0; i < FLASH_ROAD_IN_OUT_PARAMS; i++) 
            {
                out_duan[i] = flash_union_buffer[1 + FLASH_ROAD_IN_OUT_PARAMS + i].int32_type;
            }
            
            return FLASH_ROAD_RESULT_OK;
        }
    }
    
    // ���Դӱ���λ�ö�ȡ
    if(flash_check(FLASH_ROAD_SECTION_INDEX, FLASH_ROAD_MAX_PAGES_PER_TYPE + FLASH_ROAD_SIZE_INFO_INDEX))
    {
        result = flash_safe_read_page(FLASH_ROAD_SECTION_INDEX, 
                                      FLASH_ROAD_MAX_PAGES_PER_TYPE + FLASH_ROAD_SIZE_INFO_INDEX, 
                                      metadata_count);
        if (result == FLASH_ROAD_RESULT_OK)
        {
            *data_points = flash_union_buffer[0].uint32_type;
            
            // ��ȡin_duan����
            for(int i = 0; i < FLASH_ROAD_IN_OUT_PARAMS; i++) 
            {
            in_duan[i] = flash_union_buffer[1 + i].int32_type;
        }
        
            // ��ȡout_duan����
            for(int i = 0; i < FLASH_ROAD_IN_OUT_PARAMS; i++) 
            {
                out_duan[i] = flash_union_buffer[1 + FLASH_ROAD_IN_OUT_PARAMS + i].int32_type;
            }
            
            return FLASH_ROAD_RESULT_OK;
        }
    }
    
    // ����λ�ö�û������
    return FLASH_ROAD_RESULT_DATA_CORRUPT;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ��������
//-------------------------------------------------------------------------------------------------------------------
static flash_road_result_enum flash_load_integer_data(uint32 data_points)
{
    flash_road_result_enum result = FLASH_ROAD_RESULT_OK;
    uint32 total_pages = flash_calculate_pages(data_points);
    
    for(uint32 page_index = 1; page_index < total_pages; page_index++)
    {
        uint32 start_index = (page_index - 1) * FLASH_PAGE_LENGTH;
        uint32 data_count = FLASH_PAGE_LENGTH;
        
        // �������һҳ
        if(start_index + FLASH_PAGE_LENGTH > data_points) 
        {
            data_count = data_points - start_index;
        }
        
        // ��Flash��ȡ
        result = flash_safe_read_page(FLASH_ROAD_SECTION_INDEX, page_index, data_count);
        if (result != FLASH_ROAD_RESULT_OK) continue; // ����������һҳ
        
        // �������ݵ�����
        for(uint32 i = 0; i < data_count; i++)
        {
            Mileage_All_sum_list[start_index + i] = flash_union_buffer[i].int32_type;
        }
    }
    
    return FLASH_ROAD_RESULT_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ��������
//-------------------------------------------------------------------------------------------------------------------
static flash_road_result_enum flash_load_float_data(uint32 data_points)
{
    flash_road_result_enum result = FLASH_ROAD_RESULT_OK;
    uint32 total_pages = flash_calculate_pages(data_points);
    
    for(uint32 page_index = 1; page_index < total_pages; page_index++)
    {
        uint32 start_index = (page_index - 1) * FLASH_PAGE_LENGTH;
        uint32 data_count = FLASH_PAGE_LENGTH;
        
        // �������һҳ
        if(start_index + FLASH_PAGE_LENGTH > data_points) 
        {
            data_count = data_points - start_index;
        }
        
        // ����Ŀ��ҳ
        uint32 target_page = FLASH_ROAD_MAX_PAGES_PER_TYPE + page_index;
        
        // ��Flash��ȡ
        result = flash_safe_read_page(FLASH_ROAD_SECTION_INDEX, target_page, data_count);
        if (result != FLASH_ROAD_RESULT_OK) continue; // ����������һҳ
        
        // �������ݵ�����
        for(uint32 i = 0; i < data_count; i++)
        {
            errors_coords[start_index + i] = flash_union_buffer[i].float_type;
        }
    }
    
    return FLASH_ROAD_RESULT_OK;
}

//=================================================���ߺ���ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     ��������ҳ��
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
// �������     ��֤���ݴ�С
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
// �������     ��ȫҳ����
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
// �������     ��ȫҳд��
//-------------------------------------------------------------------------------------------------------------------
static flash_road_result_enum flash_safe_write_page(uint32 section, uint32 page, uint32 count)
{
    flash_write_page_from_buffer(section, page, count);
    return FLASH_ROAD_RESULT_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȫҳ��ȡ
//-------------------------------------------------------------------------------------------------------------------
static flash_road_result_enum flash_safe_read_page(uint32 section, uint32 page, uint32 count)
{
    flash_read_page_to_buffer(section, page, count);
    return FLASH_ROAD_RESULT_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����״̬
//-------------------------------------------------------------------------------------------------------------------
static void flash_update_status(flash_road_status_enum status)
{
    flash_road_info.status = status;
    // ����������״̬�仯�Ļص�֪ͨ
}

//=================================================�����Խӿ�ʵ��================================================

//-------------------------------------------------------------------------------------------------------------------
// �������     �洢·�����ݣ������Խӿڣ�
//-------------------------------------------------------------------------------------------------------------------
void flash_road_memery_store(void)
{
    // �����½ӿ�
    flash_road_result_enum result = flash_road_store();
    
    // ���ü����Ա�־
    if (result == FLASH_ROAD_RESULT_OK)
    {
        flash_road_info.store_flag = 3; // ����ԭ�еı�־ֵ
    }
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ·�����ݣ������Խӿڣ�
//-------------------------------------------------------------------------------------------------------------------
void flash_road_memery_get(void)
{
    // �����½ӿ�
    flash_road_result_enum result = flash_road_load();
    
    // ���ü����Ա�־
    if (result == FLASH_ROAD_RESULT_OK)
    {
        flash_road_info.store_flag = 4; // ����ԭ�еı�־ֵ
    }
    else if (flash_road_info.status == FLASH_ROAD_STATUS_NO_DATA)
    {
        flash_road_info.store_flag = 4; // ������Ҳ����Ϊ4
    }
}