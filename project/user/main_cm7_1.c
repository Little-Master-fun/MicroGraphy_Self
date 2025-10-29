/*********************************************************************************************************************
* CYT4BB Opensourec Library 即（ CYT4BB 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT4BB 开源库的一部分
*
* CYT4BB 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main_cm7_1
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-1-4       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "nav_control_ahrs.h"
#include "dual_core_comm.h"
#include "test_nav_ahrs.h"

// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 双核架构说明：
// CM7_1核心：导航算法和决策
//   - 从CM7_0接收传感器数据
//   - 执行导航算法计算 (5ms)
//   - 路径规划和决策
//   - 发送控制指令给CM7_0

// **************************** 代码区域 ****************************

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_info_init();                  // 调试串口信息初始化
    
    // 1. 初始化双核通信
    dual_core_comm_init_core1();
    
    // 2. 等待CM7_0初始化完成
    system_delay_ms(2000);
    
    // 3. 初始化导航系统
    nav_ahrs_init();
    
    // 4. 生成测试路径（可选择正方形或直线）
    // 选项1: 生成正方形路径 (1m × 1m)
    // test_nav_ahrs_generate_square_path(1.0f);
    
    // 选项2: 生成直线路径 (2m, 0度方向)
    test_nav_ahrs_generate_straight_path(2.0f, 0.0f);
    
    // 5. 延时等待系统稳定
    system_delay_ms(1000);
    
    // 6. 启动导航算法定时器
    pit_ms_init(PIT_CH10, 5);  // 导航算法 (5ms) - CM7_1使用CH10避免与CM7_0冲突
    
    // 7. 启动导航
    nav_ahrs_reset();
    nav_ahrs_set_speed(2.5f);  // 设置基础速度 2.5 m/s
    nav_ahrs_set_mode(NAV_AHRS_MODE_REPLAY);
    
    // 主循环
    while(true)
    {
        system_delay_ms(100);
    }
}

// **************************** 代码区域 ****************************
