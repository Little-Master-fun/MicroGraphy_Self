/*********************************************************************************************************************
* 文件名称          test_nav.c
* 功能说明          导航系统测试程序实现文件
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本
* 2025-09-19        LittleMaster       1.0v
* 
* 文件作用说明：
* 本文件实现导航系统的完整测试功能，包括正方形路径生成、导航控制、状态显示等
* 
* 测试流程：
* 1. 初始化导航系统和按键系统
* 2. 生成1×1m正方形测试路径
* 3. 按键控制导航启动/停止
* 4. 实时显示导航状态和统计信息
* 5. 循环导航直到手动停止
* 
* 按键定义：
* KEY_1: 生成并保存正方形路径
* KEY_2: 开始/暂停导航
* KEY_3: 停止导航并返回
* KEY_4: 紧急停车
********************************************************************************************************************/

#include "test_nav.h"
#include "driver_flash_road.h"
#include "zf_driver_timer.h"

//=================================================全局变量定义================================================
static test_nav_status_enum test_status = TEST_NAV_IDLE;    // 测试状态
static test_nav_stats_t test_stats = {0};                   // 测试统计信息
static uint32 last_display_update_ms = 0;                   // 上次显示更新时间
static uint8 key_initialized = 0;                          // 按键初始化标志
static uint8 timer_initialized = 0;                        // 定时器初始化标志

//=================================================内部函数声明================================================
static void test_nav_init_system(void);
static void test_nav_clear_display(void);
static float test_nav_calculate_distance(const nav_position_t *p1, const nav_position_t *p2);
static void test_nav_update_statistics(void);
static const char* test_nav_status_to_string(nav_status_enum status);
static uint32 test_nav_get_system_time_ms(void);

//=================================================主测试函数实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     完整导航系统测试
//-------------------------------------------------------------------------------------------------------------------
void test_nav_complete(void)
{
    // 初始化系统
    test_nav_init_system();
    
    // 显示欢迎界面
    test_nav_clear_display();
    ips114_show_string(0, 0, "Navigation Test v1.0");
    ips114_show_string(0, 16, "KEY1: Create Path");
    ips114_show_string(0, 32, "KEY2: Start/Pause");
    ips114_show_string(0, 48, "KEY3: Stop & Return");
    ips114_show_string(0, 64, "KEY4: Emergency Stop");
    ips114_show_string(0, 96, "Press KEY1 to begin...");
    
    test_status = TEST_NAV_IDLE;
    uint32 last_update_ms = 0;
    
    // 主测试循环
    while(1)
    {
        uint32 current_time = test_nav_get_system_time_ms();
        
        // 定期扫描按键（每10ms）
        if (current_time - last_update_ms >= 10)
        {
            key_scanner();
            last_update_ms = current_time;
        }
        
        // 处理按键输入
        if (test_nav_handle_keys())
        {
            // 有按键动作时更新显示
            test_nav_display_status();
        }
        
        // 定期更新显示和状态
        if (current_time - last_display_update_ms >= TEST_DISPLAY_UPDATE_MS)
        {
            test_nav_display_status();
            last_display_update_ms = current_time;
        }
        
        // 导航状态检查和更新
        if (test_status == TEST_NAV_NAVIGATING)
        {
            // 更新导航系统
            nav_update();
            
            // 检查导航状态
            nav_status_enum nav_status = nav_get_status();
            if (nav_status == NAV_STATUS_COMPLETED)
            {
                test_status = TEST_NAV_COMPLETED;
                test_stats.loops_completed++;
                
                // 自动重新开始导航（循环测试）
                system_delay_ms(1000);
                if (nav_start(NAV_MODE_PURE_PURSUIT))
                {
                    test_status = TEST_NAV_NAVIGATING;
                }
            }
            else if (nav_status == NAV_STATUS_ERROR || nav_status == NAV_STATUS_EMERGENCY_STOP)
            {
                test_status = TEST_NAV_ERROR;
            }
            
            // 更新统计信息
            test_nav_update_statistics();
        }
        
        // 退出条件检查
        if (test_status == TEST_NAV_IDLE && 
            key_get_state(KEY_3) == KEY_LONG_PRESS)
        {
            key_clear_state(KEY_3);
            break;  // 长按KEY3退出测试
        }
        
        system_delay_ms(5);  // 短暂延时，降低CPU占用
    }
    
    // 测试结束处理
    nav_stop(0);
    test_nav_clear_display();
    ips114_show_string(0, 0, "Navigation Test End");
    ips114_show_string(0, 16, "Thank you!");
    
    // 显示最终统计
    char stats_info[50];
    sprintf(stats_info, "Loops: %d", test_stats.loops_completed);
    ips114_show_string(0, 32, stats_info);
    sprintf(stats_info, "Max Error: %.2fm", test_stats.max_cross_track_error);
    ips114_show_string(0, 48, stats_info);
    
    system_delay_ms(3000);  // 显示3秒统计信息
}

//=================================================路径生成函数实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     生成1×1m正方形测试路径
//-------------------------------------------------------------------------------------------------------------------
uint8 test_nav_create_square_path(void)
{
    static nav_path_t square_path = {0};
    
    // 清空路径数据
    memset(&square_path, 0, sizeof(nav_path_t));
    
    // 设置路径基本信息
    strcpy(square_path.path_name, "正方形测试路径");
    square_path.waypoint_count = 5;  // 4个角点 + 1个回到起点
    square_path.current_waypoint = 0;
    
    // 定义正方形的4个角点（以当前位置为起点）
    
    // 路径点0: 起点 (0, 0)
    square_path.waypoints[0].position.x = 0.0f;
    square_path.waypoints[0].position.y = 0.0f;
    square_path.waypoints[0].target_speed = TEST_SQUARE_SPEED;
    square_path.waypoints[0].tolerance = TEST_WAYPOINT_TOLERANCE;
    square_path.waypoints[0].point_type = 0;  // 普通点
    
    // 路径点1: 右前角 (1, 0)
    square_path.waypoints[1].position.x = TEST_SQUARE_SIZE;
    square_path.waypoints[1].position.y = 0.0f;
    square_path.waypoints[1].target_speed = TEST_CORNER_SPEED;
    square_path.waypoints[1].tolerance = TEST_WAYPOINT_TOLERANCE;
    square_path.waypoints[1].point_type = 1;  // 转弯点
    
    // 路径点2: 右后角 (1, 1)  
    square_path.waypoints[2].position.x = TEST_SQUARE_SIZE;
    square_path.waypoints[2].position.y = TEST_SQUARE_SIZE;
    square_path.waypoints[2].target_speed = TEST_CORNER_SPEED;
    square_path.waypoints[2].tolerance = TEST_WAYPOINT_TOLERANCE;
    square_path.waypoints[2].point_type = 1;  // 转弯点
    
    // 路径点3: 左后角 (0, 1)
    square_path.waypoints[3].position.x = 0.0f;
    square_path.waypoints[3].position.y = TEST_SQUARE_SIZE;
    square_path.waypoints[3].target_speed = TEST_CORNER_SPEED;
    square_path.waypoints[3].tolerance = TEST_WAYPOINT_TOLERANCE;
    square_path.waypoints[3].point_type = 1;  // 转弯点
    
    // 路径点4: 回到起点 (0, 0)
    square_path.waypoints[4].position.x = 0.0f;
    square_path.waypoints[4].position.y = 0.0f;
    square_path.waypoints[4].target_speed = TEST_SQUARE_SPEED;
    square_path.waypoints[4].tolerance = TEST_WAYPOINT_TOLERANCE;
    square_path.waypoints[4].point_type = 2;  // 终点
    
    // 计算路径总长度
    square_path.total_length = 4.0f * TEST_SQUARE_SIZE;  // 正方形周长
    
    // 加载路径到导航系统
    if (!nav_load_path("正方形测试路径"))
    {
        // 直接设置路径数据（绕过Flash存储）
        memcpy(&nav_system.current_path, &square_path, sizeof(nav_path_t));
        
        test_status = TEST_NAV_PATH_READY;
        return 1;
    }
    
    test_status = TEST_NAV_PATH_READY;
    return 1;
}

//=================================================显示函数实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导航状态实时显示
//-------------------------------------------------------------------------------------------------------------------
void test_nav_display_status(void)
{
    char display_info[50];
    
    // 清屏
    test_nav_clear_display();
    
    // 显示标题
    ips114_show_string(0, 0, "Nav Test Status");
    
    // 显示测试状态
    const char* status_text = "";
    switch(test_status)
    {
        case TEST_NAV_IDLE:           status_text = "IDLE"; break;
        case TEST_NAV_PATH_READY:     status_text = "PATH_READY"; break;
        case TEST_NAV_NAVIGATING:     status_text = "NAVIGATING"; break;
        case TEST_NAV_PAUSED:         status_text = "PAUSED"; break;
        case TEST_NAV_COMPLETED:      status_text = "COMPLETED"; break;
        case TEST_NAV_ERROR:          status_text = "ERROR"; break;
        default:                      status_text = "UNKNOWN"; break;
    }
    
    sprintf(display_info, "Status: %s", status_text);
    ips114_show_string(0, 16, display_info);
    
    // 显示导航系统状态
    nav_status_enum nav_status = nav_get_status();
    const char* nav_status_text = test_nav_status_to_string(nav_status);
    sprintf(display_info, "Nav: %s", nav_status_text);
    ips114_show_string(0, 32, display_info);
    
    // 显示当前位置
    nav_position_t current_pos = {0};
    if (nav_get_current_position(&current_pos))
    {
        sprintf(display_info, "Pos: %.2f,%.2f", current_pos.x, current_pos.y);
        ips114_show_string(0, 48, display_info);
    }
    
    // 显示横向偏差
    float cross_error = nav_get_cross_track_error();
    sprintf(display_info, "Error: %.3fm", cross_error);
    ips114_show_string(0, 64, display_info);
    
    // 显示统计信息
    sprintf(display_info, "Loops: %d", test_stats.loops_completed);
    ips114_show_string(0, 80, display_info);
    
    // 显示当前速度（如果导航中）
    if (test_status == TEST_NAV_NAVIGATING)
    {
        sprintf(display_info, "Speed: %.2fm/s", nav_system.sensor_fusion.fused_pose.velocity.linear);
        ips114_show_string(0, 96, display_info);
    }
    
    // 显示按键提示（根据状态）
    if (test_status == TEST_NAV_IDLE)
    {
        ips114_show_string(0, 112, "KEY1: Create Path");
    }
    else if (test_status == TEST_NAV_PATH_READY)
    {
        ips114_show_string(0, 112, "KEY2: Start Nav");
    }
    else if (test_status == TEST_NAV_NAVIGATING)
    {
        ips114_show_string(0, 112, "KEY2:Pause KEY4:Stop");
    }
}

//=================================================按键处理函数实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按键控制处理
//-------------------------------------------------------------------------------------------------------------------
uint8 test_nav_handle_keys(void)
{
    uint8 key_action = 0;
    
    // KEY_1: 生成并保存正方形路径
    if (key_get_state(KEY_1) == KEY_SHORT_PRESS)
    {
        key_clear_state(KEY_1);
        key_action = 1;
        
        test_nav_clear_display();
        ips114_show_string(0, 0, "Creating Path...");
        
        if (test_nav_create_square_path())
        {
            ips114_show_string(0, 16, "Path Created!");
            ips114_show_string(0, 32, "Size: 1x1m Square");
            ips114_show_string(0, 48, "Points: 5");
            ips114_show_string(0, 64, "Press KEY2 to start");
        }
        else
        {
            ips114_show_string(0, 16, "Path Create Failed!");
            test_status = TEST_NAV_ERROR;
        }
        
        system_delay_ms(1500);  // 显示结果1.5秒
    }
    
    // KEY_2: 开始/暂停导航
    else if (key_get_state(KEY_2) == KEY_SHORT_PRESS)
    {
        key_clear_state(KEY_2);
        key_action = 1;
        
        if (test_status == TEST_NAV_PATH_READY)
        {
            // 开始导航
            test_nav_clear_display();
            ips114_show_string(0, 0, "Starting Navigation...");
            
            if (nav_start(NAV_MODE_PURE_PURSUIT))
            {
                test_status = TEST_NAV_NAVIGATING;
                test_stats.test_start_time_ms = test_nav_get_system_time_ms();
                ips114_show_string(0, 16, "Navigation Started!");
                system_delay_ms(1000);
            }
            else
            {
                ips114_show_string(0, 16, "Start Failed!");
                test_status = TEST_NAV_ERROR;
                system_delay_ms(1500);
            }
        }
        else if (test_status == TEST_NAV_NAVIGATING)
        {
            // 暂停导航
            nav_pause(1);
            test_status = TEST_NAV_PAUSED;
            ips114_show_string(0, 112, "Navigation Paused");
            system_delay_ms(500);
        }
        else if (test_status == TEST_NAV_PAUSED)
        {
            // 恢复导航
            nav_pause(0);
            test_status = TEST_NAV_NAVIGATING;
            ips114_show_string(0, 112, "Navigation Resumed");
            system_delay_ms(500);
        }
    }
    
    // KEY_3: 停止导航并返回
    else if (key_get_state(KEY_3) == KEY_SHORT_PRESS)
    {
        key_clear_state(KEY_3);
        key_action = 1;
        
        if (test_status == TEST_NAV_NAVIGATING || test_status == TEST_NAV_PAUSED)
        {
            nav_stop(0);  // 正常停车
            test_status = TEST_NAV_IDLE;
            
            test_nav_clear_display();
            ips114_show_string(0, 0, "Navigation Stopped");
            ips114_show_string(0, 16, "Returning to idle...");
            system_delay_ms(1500);
        }
        else
        {
            test_status = TEST_NAV_IDLE;
        }
    }
    
    // KEY_4: 紧急停车
    else if (key_get_state(KEY_4) == KEY_SHORT_PRESS)
    {
        key_clear_state(KEY_4);
        key_action = 1;
        
        test_nav_emergency_stop();
    }
    
    return key_action;
}

//=================================================辅助函数实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导航路径验证测试
//-------------------------------------------------------------------------------------------------------------------
uint8 test_nav_verify_path(void)
{
    nav_path_t *path = &nav_system.current_path;
    
    // 基本验证
    if (path->waypoint_count != 5)
    {
        return 0;
    }
    
    // 验证路径点坐标
    if (fabsf(path->waypoints[0].position.x - 0.0f) > 0.01f ||
        fabsf(path->waypoints[0].position.y - 0.0f) > 0.01f)
    {
        return 0;
    }
    
    // 验证路径长度
    float calculated_length = 0.0f;
    for (uint16 i = 1; i < path->waypoint_count; i++)
    {
        calculated_length += test_nav_calculate_distance(
            &path->waypoints[i-1].position, 
            &path->waypoints[i].position
        );
    }
    
    if (fabsf(calculated_length - path->total_length) > 0.1f)
    {
        return 0;
    }
    
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导航性能统计
//-------------------------------------------------------------------------------------------------------------------
uint8 test_nav_get_statistics(test_nav_stats_t *stats)
{
    if (stats == NULL) return 0;
    
    *stats = test_stats;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     紧急停车测试
//-------------------------------------------------------------------------------------------------------------------
void test_nav_emergency_stop(void)
{
    nav_stop(1);  // 紧急停车
    test_status = TEST_NAV_ERROR;
    
    test_nav_clear_display();
    ips114_show_string(0, 0, "EMERGENCY STOP!");
    ips114_show_string(0, 16, "System Halted");
    ips114_show_string(0, 32, "Check Safety");
    ips114_show_string(0, 48, "Press KEY3 to reset");
    
    // 等待用户确认
    while(1)
    {
        key_scanner();
        if (key_get_state(KEY_3) == KEY_SHORT_PRESS)
        {
            key_clear_state(KEY_3);
            break;
        }
        system_delay_ms(100);
    }
    
    test_nav_reset();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导航系统复位
//-------------------------------------------------------------------------------------------------------------------
void test_nav_reset(void)
{
    test_status = TEST_NAV_IDLE;
    memset(&test_stats, 0, sizeof(test_nav_stats_t));
    
    test_nav_clear_display();
    ips114_show_string(0, 0, "System Reset");
    ips114_show_string(0, 16, "Ready for new test");
    system_delay_ms(1000);
}

//=================================================内部函数实现================================================

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化测试系统
//-------------------------------------------------------------------------------------------------------------------
static void test_nav_init_system(void)
{
    // 初始化导航系统
    if (!nav_init())
    {
        ips114_clear();
        ips114_show_string(0, 0, "Nav Init Failed!");
        while(1); // 初始化失败则停止
    }
    
    // 初始化按键系统
    if (!key_initialized)
    {
        key_init(10);  // 10ms扫描周期
        key_initialized = 1;
    }
    
    // 清空按键状态
    key_clear_all_state();
    
    // 重置测试状态
    test_status = TEST_NAV_IDLE;
    memset(&test_stats, 0, sizeof(test_nav_stats_t));
    last_display_update_ms = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清空显示
//-------------------------------------------------------------------------------------------------------------------
static void test_nav_clear_display(void)
{
    ips114_clear();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算两点间距离
//-------------------------------------------------------------------------------------------------------------------
static float test_nav_calculate_distance(const nav_position_t *p1, const nav_position_t *p2)
{
    if (p1 == NULL || p2 == NULL) return 0.0f;
    
    float dx = p2->x - p1->x;
    float dy = p2->y - p1->y;
    return sqrtf(dx * dx + dy * dy);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新统计信息
//-------------------------------------------------------------------------------------------------------------------
static void test_nav_update_statistics(void)
{
    if (test_stats.test_start_time_ms == 0) return;
    
    // 更新测试时间
    test_stats.total_test_time_ms = test_nav_get_system_time_ms() - test_stats.test_start_time_ms;
    
    // 更新最大横向误差
    float current_error = fabsf(nav_get_cross_track_error());
    if (current_error > test_stats.max_cross_track_error)
    {
        test_stats.max_cross_track_error = current_error;
    }
    
    // 计算平均横向误差（简化版本）
    static float error_sum = 0.0f;
    static uint32 error_count = 0;
    
    error_sum += current_error;
    error_count++;
    test_stats.avg_cross_track_error = error_sum / error_count;
    
    // 更新行驶距离
    nav_statistics_t nav_stats;
    if (nav_get_statistics(&nav_stats))
    {
        test_stats.total_distance = nav_stats.total_distance_traveled;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导航状态转换为字符串
//-------------------------------------------------------------------------------------------------------------------
static const char* test_nav_status_to_string(nav_status_enum status)
{
    switch(status)
    {
        case NAV_STATUS_IDLE:             return "IDLE";
        case NAV_STATUS_INITIALIZING:     return "INIT";
        case NAV_STATUS_PATH_LOADING:     return "LOADING";
        case NAV_STATUS_NAVIGATING:       return "NAVIGATING";
        case NAV_STATUS_PAUSED:           return "PAUSED";
        case NAV_STATUS_COMPLETED:        return "COMPLETED";
        case NAV_STATUS_ERROR:            return "ERROR";
        case NAV_STATUS_EMERGENCY_STOP:   return "EMERGENCY";
        default:                          return "UNKNOWN";
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取系统时间（毫秒）
//-------------------------------------------------------------------------------------------------------------------
static uint32 test_nav_get_system_time_ms(void)
{
    // 使用系统定时器获取毫秒时间
    if (!timer_initialized) {
        timer_init(TC_TIME2_CH1, TIMER_MS);  // 初始化定时器为毫秒模式
        timer_start(TC_TIME2_CH1);           // 启动定时器
        timer_initialized = 1;
    }
    return timer_get(TC_TIME2_CH1);
}
