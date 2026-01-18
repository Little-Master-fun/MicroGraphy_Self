/*********************************************************************************************************************
* 文件名称          test_line_tracking.h
* 功能说明          光电管巡线控制测试头文件
* 作者              LittleMaster
* 版本信息          v1.0
* 修改记录
* 日期              作者                版本
* 2025-11-05        LittleMaster       1.0v
********************************************************************************************************************/

#ifndef __TEST_LINE_TRACKING_H__
#define __TEST_LINE_TRACKING_H__

#include "zf_common_headfile.h"

/**
 * @brief   测试光电管巡线功能
 * @return  void
 */
void test_line_tracking(void);

/**
 * @brief   测试光电管传感器读取（不进行控制）
 * @return  void
 */
void test_line_tracking_sensor_only(void);

/**
 * @brief   PID参数调试模式
 * @return  void
 */
void test_line_tracking_pid_tuning(void);

#endif // __TEST_LINE_TRACKING_H__

