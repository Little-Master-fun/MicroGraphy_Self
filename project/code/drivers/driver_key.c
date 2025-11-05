/*********************************************************************************************************************
* 文件名称          driver_key.c
* 功能说明          按键驱动程序实现文件
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本
* 2025-11-05        LittleMaster       1.0v
********************************************************************************************************************/

#include "driver_key.h"
#include "zf_common_headfile.h"

//=================================================全局变量定义================================================
key_system_struct key_system = {0};

// 按键引脚数组
static const gpio_pin_enum key_pins[KEY_NUM] = {
    KEY0_PIN, KEY1_PIN, KEY2_PIN, KEY3_PIN,
    KEY4_PIN, KEY5_PIN, KEY6_PIN, KEY7_PIN
};

// 按键回调函数指针
static key_callback_func key_callback = NULL;

//=================================================内部函数声明================================================
static uint32 key_get_time_ms(void);

//=================================================外部接口实现================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按键系统初始化
// 使用示例     key_init();
// 备注信息     初始化所有按键引脚为输入模式，上拉
//-------------------------------------------------------------------------------------------------------------------
void key_init(void)
{
    uint8 i;
    
    // 初始化所有按键引脚为输入，上拉模式（按键按下为低电平）
    for(i = 0; i < KEY_NUM; i++)
    {
        gpio_init(key_pins[i], GPI, GPIO_HIGH, GPI_PULL_UP);
    }
    
    // 初始化数据结构
    memset(&key_system, 0, sizeof(key_system_struct));
    
    // 初始化所有按键状态
    for(i = 0; i < KEY_NUM; i++)
    {
        key_system.keys[i].state = KEY_STATE_IDLE;
        key_system.keys[i].event = KEY_EVENT_NONE;
        key_system.keys[i].last_state = 1;  // 初始为高电平（未按下）
    }
    
    printf("按键系统初始化完成，按键数量：%d\r\n", KEY_NUM);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取系统时间（毫秒）
// 备注信息     内部函数，用于计时
//-------------------------------------------------------------------------------------------------------------------
static uint32 key_get_time_ms(void)
{
    // 这里需要根据实际的时钟系统实现
    // 简单实现：使用扫描计数器（假设每2ms扫描一次）
    return key_system.scan_count * 2;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按键扫描
// 使用示例     key_scan();
// 备注信息     定时调用此函数（建议2-10ms），进行按键扫描和事件检测
//-------------------------------------------------------------------------------------------------------------------
void key_scan(void)
{
    uint8 i;
    uint32 current_time = key_get_time_ms();
    
    key_system.scan_count++;
    
    for(i = 0; i < KEY_NUM; i++)
    {
        key_data_struct *key = &key_system.keys[i];
        
        // 读取当前电平（0=按下，1=释放）
        key->current_state = gpio_get_level(key_pins[i]);
        
        // 状态机处理
        switch(key->state)
        {
            case KEY_STATE_IDLE:
                if(key->current_state == 0 && key->last_state == 1)
                {
                    // 检测到按下
                    key->press_time = current_time;
                    key->is_long_press = 0;
                    key->state = KEY_STATE_PRESS;
                    key->event = KEY_EVENT_PRESS;
                    
                    // 触发回调
                    if(key_callback != NULL)
                    {
                        key_callback((key_id_enum)i, KEY_EVENT_PRESS);
                    }
                }
                break;
                
            case KEY_STATE_PRESS:
                if(key->current_state == 0)
                {
                    // 持续按下，检查是否长按
                    if(!key->is_long_press && (current_time - key->press_time) >= KEY_LONG_PRESS_TIME)
                    {
                        key->is_long_press = 1;
                        key->state = KEY_STATE_LONG_PRESS;
                        key->event = KEY_EVENT_LONG_PRESS;
                        
                        // 触发回调
                        if(key_callback != NULL)
                        {
                            key_callback((key_id_enum)i, KEY_EVENT_LONG_PRESS);
                        }
                    }
                }
                else if(key->current_state == 1 && key->last_state == 0)
                {
                    // 检测到释放
                    key->release_time = current_time;
                    key->state = KEY_STATE_RELEASE;
                    
                    // 如果按下时间短，则为单击
                    if((key->release_time - key->press_time) < KEY_LONG_PRESS_TIME)
                    {
                        key->event = KEY_EVENT_CLICK;
                        
                        // 触发回调
                        if(key_callback != NULL)
                        {
                            key_callback((key_id_enum)i, KEY_EVENT_CLICK);
                        }
                    }
                }
                break;
                
            case KEY_STATE_LONG_PRESS:
                if(key->current_state == 1 && key->last_state == 0)
                {
                    // 长按后释放
                    key->release_time = current_time;
                    key->state = KEY_STATE_RELEASE;
                    key->event = KEY_EVENT_RELEASE;
                    
                    // 触发回调
                    if(key_callback != NULL)
                    {
                        key_callback((key_id_enum)i, KEY_EVENT_RELEASE);
                    }
                }
                break;
                
            case KEY_STATE_RELEASE:
                // 消抖后回到空闲状态
                if((current_time - key->release_time) >= KEY_DEBOUNCE_TIME)
                {
                    key->state = KEY_STATE_IDLE;
                    key->event = KEY_EVENT_NONE;
                }
                break;
        }
        
        key->last_state = key->current_state;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取按键状态
// 参数说明     key_id          按键ID (0-7)
// 返回参数     uint8           按键电平状态 (0=按下, 1=释放)
// 使用示例     state = key_get_state(KEY_ID_0);
//-------------------------------------------------------------------------------------------------------------------
uint8 key_get_state(key_id_enum key_id)
{
    if(key_id >= KEY_NUM)
        return 1;
    
    return key_system.keys[key_id].current_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取按键事件
// 参数说明     key_id          按键ID (0-7)
// 返回参数     key_event_enum  按键事件
// 使用示例     event = key_get_event(KEY_ID_0);
//-------------------------------------------------------------------------------------------------------------------
key_event_enum key_get_event(key_id_enum key_id)
{
    if(key_id >= KEY_NUM)
        return KEY_EVENT_NONE;
    
    return key_system.keys[key_id].event;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清除按键事件
// 参数说明     key_id          按键ID (0-7)
// 使用示例     key_clear_event(KEY_ID_0);
//-------------------------------------------------------------------------------------------------------------------
void key_clear_event(key_id_enum key_id)
{
    if(key_id >= KEY_NUM)
        return;
    
    key_system.keys[key_id].event = KEY_EVENT_NONE;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     判断按键是否被按下
// 参数说明     key_id          按键ID (0-7)
// 返回参数     uint8           1=按下, 0=未按下
// 使用示例     if(key_is_pressed(KEY_ID_0)) { ... }
//-------------------------------------------------------------------------------------------------------------------
uint8 key_is_pressed(key_id_enum key_id)
{
    if(key_id >= KEY_NUM)
        return 0;
    
    return (key_system.keys[key_id].state == KEY_STATE_PRESS || 
            key_system.keys[key_id].state == KEY_STATE_LONG_PRESS);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     判断按键是否被释放
// 参数说明     key_id          按键ID (0-7)
// 返回参数     uint8           1=释放, 0=未释放
// 使用示例     if(key_is_released(KEY_ID_0)) { ... }
//-------------------------------------------------------------------------------------------------------------------
uint8 key_is_released(key_id_enum key_id)
{
    if(key_id >= KEY_NUM)
        return 1;
    
    return (key_system.keys[key_id].state == KEY_STATE_IDLE);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     判断按键是否长按
// 参数说明     key_id          按键ID (0-7)
// 返回参数     uint8           1=长按, 0=非长按
// 使用示例     if(key_is_long_press(KEY_ID_0)) { ... }
//-------------------------------------------------------------------------------------------------------------------
uint8 key_is_long_press(key_id_enum key_id)
{
    if(key_id >= KEY_NUM)
        return 0;
    
    return (key_system.keys[key_id].state == KEY_STATE_LONG_PRESS);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     注册按键回调函数
// 参数说明     callback        回调函数指针
// 使用示例     key_register_callback(my_key_handler);
//-------------------------------------------------------------------------------------------------------------------
void key_register_callback(key_callback_func callback)
{
    key_callback = callback;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     打印按键状态（调试用）
// 使用示例     key_print_status();
//-------------------------------------------------------------------------------------------------------------------
void key_print_status(void)
{
    uint8 i;
    const char *state_str[] = {"IDLE", "PRESS", "RELEASE", "LONG"};
    const char *event_str[] = {"NONE", "PRESS", "RELEASE", "LONG", "CLICK"};
    
    printf("按键状态：\r\n");
    for(i = 0; i < KEY_NUM; i++)
    {
        if(key_system.keys[i].state != KEY_STATE_IDLE || 
           key_system.keys[i].event != KEY_EVENT_NONE)
        {
            printf("  KEY%d: State=%s, Event=%s, Level=%d\r\n",
                   i,
                   state_str[key_system.keys[i].state],
                   event_str[key_system.keys[i].event],
                   key_system.keys[i].current_state);
        }
    }
}

