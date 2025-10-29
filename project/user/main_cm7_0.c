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
* 文件名称          main_cm7_0
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

#include "driver_encoder.h"
#include "driver_motor.h"
#include "motor_control.h"
#include "imu_ahrs_complementary.h"
#include "dual_core_comm.h"

// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 双核架构说明：
// CM7_0核心：数据采集和底层控制
//   - IMU数据采集与AHRS姿态解算 (1ms)
//   - 编码器数据读取
//   - 电机PWM控制 (2ms)
//   - 传感器数据共享给CM7_1 (5ms)
// CM7_1核心：导航算法和决策
//   - 导航算法计算 (5ms)
//   - 路径规划
//   - 控制指令发送给CM7_0

// **************************** 代码区域 ****************************


int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_init();                       // 调试串口信息初始化
    
    // 1. 初始化双核通信
    dual_core_comm_init_core0();
    
    // 2. 初始化电机控制
    motor_pid_init();
    motor_init();
    
    // 3. 初始化编码器
    encoder_init();
    
    // 4. 初始化AHRS姿态解算
    ahrs_complementary_init();
    
    // 5. 延时等待IMU稳定
    system_delay_ms(1000);
    
    // 6. 启动定时器中断
    pit_ms_init(PIT_CH2, 1);  // IMU数据采集 (1ms)
    pit_ms_init(PIT_CH1, 2);  // 电机控制 (2ms)
    pit_ms_init(PIT_CH0, 5);  // 数据共享 (5ms)
    
    // 主循环
    while(true)
    {
        // CM7_0主循环可以执行一些低优先级任务
        // 例如：显示屏更新、日志输出等
        
        system_delay_ms(100);
    }
}

// **************************** 代码区域 ****************************
