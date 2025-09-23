/*********************************************************************************************************************
* 导航系统测试程序 - IPS114安全显示版本
* 
* IPS114显示安全区域 (240×135像素，8×16字体)：
* - 标题区域：y=0-16    - "Nav Test"
* - 菜单区域：y=32-64   - 路径和模式信息  
* - 状态区域：y=80-112  - 实时导航状态显示
* - 最大安全y坐标：119 (135-16=119，避免断言错误)
********************************************************************************************************************/

#include "test_nav.h"
#include "nav_control.h"
#include "zf_device_ips114.h"
#include "zf_common_headfile.h"
#include <stdio.h>

// Display initial screen info
static void display_init_screen(void)
{
    ips114_clear();
    ips114_set_color(RGB565_WHITE, RGB565_BLACK);
    
    // Title - keep within 240x135 screen
    ips114_show_string(5, 0, "Nav Test");
    ips114_show_string(5, 16, "========");
    
    // System info - use 8x16 font spacing (16 pixels per line)
    ips114_show_string(5, 32, "Path: 1x1m Sq");
    ips114_show_string(5, 48, "Speed: 1.0m/s");
    ips114_show_string(5, 64, "Mode: Auto");
    
    // Status area starts at y=80
    ips114_show_string(5, 80, "Status: Init...");
}

// Navigation system test main function
void test_nav_system(void)
{
    nav_status_enum status;
    uint32 loop_count = 0;
    
    printf("=== Nav System Test Start ===\n");
    
    // Initialize display
    ips114_init();
    display_init_screen();
    
    // 1. Initialize navigation system
    printf("Initializing nav system...\n");
    status = nav_init();
    if (status != NAV_STATUS_OK) {
        printf("Nav system init failed!\n");
        ips114_set_color(RGB565_RED, RGB565_BLACK);
        ips114_show_string(5, 96, "ERROR: Init Failed!");
        return;
    }
    
    // 2. Set desired cruise speed
    printf("Setting cruise speed: 1.0 m/s\n");
    nav_set_desired_speed(1.0f);  // 1.0 m/s
    
    // 3. Load test path
    printf("Loading 1x1m square path\n");
    nav_load_preset_path(0);      // 0 = 1×1m square path
    
    // 4. Start auto tracking
    printf("Starting auto tracking\n");
    nav_set_mode(NAV_MODE_AUTO);
    
    printf("Nav system started! Path tracking...\n");
    
    // 5. Main loop - continuous nav control and status display
    while(1)
    {
        // Update navigation system
        status = nav_update();
        
        // Update screen display every 20 loops (1 second)
        if (loop_count % 20 == 0) {
            nav_system_t nav_info;
            char status_str[16];
            char pos_str[16];
            char speed_str[16];
            char dist_str[16];
            
            // Get current navigation status
            nav_get_system_status(&nav_info);
            
            // Clear status display area within screen bounds (240x135)
            ips114_set_color(RGB565_WHITE, RGB565_BLACK);
            ips114_show_string(5, 80, "               ");
            ips114_show_string(5, 96, "               ");
            ips114_show_string(5, 112, "               ");  // y=112 is safe, 112+16=128<135
            
            // Format status info with short English
            switch (nav_info.status) {
                case NAV_STATUS_OK:
                    sprintf(status_str, "St: Run");
                    ips114_set_color(RGB565_WHITE, RGB565_BLACK);
                    break;
                case NAV_STATUS_ERROR:
                    sprintf(status_str, "St: ERR");
                    ips114_set_color(RGB565_RED, RGB565_BLACK);
                    break;
                case NAV_STATUS_PATH_COMPLETE:
                    sprintf(status_str, "St: Done");
                    ips114_set_color(RGB565_GREEN, RGB565_BLACK);
                    break;
                case NAV_STATUS_NO_PATH:
                    sprintf(status_str, "St: NoPath");
                    ips114_set_color(RGB565_YELLOW, RGB565_BLACK);
                    break;
                case NAV_STATUS_OFF_TRACK:
                    sprintf(status_str, "St: OffTrk");
                    ips114_set_color(RGB565_YELLOW, RGB565_BLACK);
                    break;
                default:
                    sprintf(status_str, "St: ?%d", nav_info.status);
                    ips114_set_color(RGB565_WHITE, RGB565_BLACK);
                    break;
            }
            
            sprintf(pos_str, "X%.2f Y%.2f", 
                    nav_info.current_pose.x, nav_info.current_pose.y);
            sprintf(speed_str, "V %.2fm/s", 
                    nav_info.current_pose.linear_velocity);
            sprintf(dist_str, "D %.2fm", 
                    nav_info.output.distance_to_target);
            
            // Display status info within safe coordinates (avoid y>119)
            ips114_show_string(5, 80, status_str);
            ips114_set_color(RGB565_WHITE, RGB565_BLACK);
            ips114_show_string(5, 96, pos_str);
            ips114_show_string(5, 112, speed_str);
            // Note: Remove distance display to avoid assertion error at y=128
        }
        
        // Print detailed status to serial every 200 loops (10 seconds)
        if (loop_count % 200 == 0) {
            nav_system_t nav_info;
            nav_get_system_status(&nav_info);
            printf("St:%d Pos:(%.2f,%.2f) Ang:%.1f Spd:%.2f\n",
                   nav_info.status,
                   nav_info.current_pose.x,
                   nav_info.current_pose.y,
                   nav_info.current_pose.theta * 180.0f / M_PI,
                   nav_info.current_pose.linear_velocity);
        }
        
        // Handle special status
        switch (status) {
            case NAV_STATUS_PATH_COMPLETE:
                printf("Path complete! Looping...\n");
                break;
            case NAV_STATUS_OFF_TRACK:
                printf("Warning: Off track!\n");
                break;
            case NAV_STATUS_ERROR:
                printf("Error: Nav system fault\n");
                break;
            default:
                break;
        }
        
        loop_count++;
        system_delay_ms(50);  // 50ms control cycle
    }
}