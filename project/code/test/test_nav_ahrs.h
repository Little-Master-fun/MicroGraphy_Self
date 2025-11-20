/*********************************************************************************************************************
* 文件名称          test_nav_ahrs.h
* 功能说明          AHRS导航系统测试头文件
* 作者              AI Assistant
* 版本信息          v1.2
* 修改记录
* 日期              作者                版本
* 2025-01-XX        AI Assistant       1.0v
* 2025-01-XX        AI Assistant       1.2v - 添加路径生成函数
* 
* 文件作用说明：
* 本文件提供AHRS导航系统的测试路径生成和启动函数声明
********************************************************************************************************************/

#ifndef _TEST_NAV_AHRS_H_
#define _TEST_NAV_AHRS_H_

#include "nav_control_ahrs.h"
#include "zf_common_typedef.h"

//=================================================路径生成函数声明================================================

/**
 * @brief  生成正方形测试路径
 * @param  size     正方形边长 (m)
 * @retval 状态
 * @note   生成一个正方形路径，每隔NAV_AHRS_DISTANCE_PER_POINT记录一个航向角
 */
nav_ahrs_status_enum test_nav_ahrs_generate_square_path(float size);

/**
 * @brief  生成直线测试路径
 * @param  length       直线长度 (m)
 * @param  direction    前进方向角度 (度), 0=向右, 90=向上, 180=向左, 270=向下
 * @retval 状态
 * @note   生成一条指定长度和方向的直线路径
 */
nav_ahrs_status_enum test_nav_ahrs_generate_straight_path(float length, float direction);

/**
 * @brief  生成长方形测试路径
 * @param  length       长方形长边 (m)
 * @param  width        长方形短边 (m)
 * @retval 状态
 * @note   生成一个长方形路径，顺序为：右(长边)->上(短边)->左(长边)->下(短边)
 */
nav_ahrs_status_enum test_nav_ahrs_generate_rectangle_path(float length, float width);

/**
 * @brief  生成多圈长方形测试路径
 * @param  length       长方形长边 (m)
 * @param  width        长方形短边 (m)
 * @param  laps         生成的圈数
 * @retval 状态
 * @note   一次性生成指定圈数的长方形路径点，适用于需要预先生成多圈路径的场景
 */
nav_ahrs_status_enum test_nav_ahrs_generate_multi_laps_path(float length, float width, uint16 laps);

//=================================================启动函数声明================================================

/**
 * @brief  一键启动AHRS导航系统（正方形路径）
 * @retval 是否成功
 * @note   调用此函数完成所有初始化并开始导航
 */
uint8 test_nav_ahrs_quick_start(void);

/**
 * @brief  一键启动AHRS导航系统（直线路径）
 * @param  length       直线长度 (m)
 * @param  direction    方向角度 (度)
 * @retval 是否成功
 * @note   调用此函数完成所有初始化并开始直线导航
 */
uint8 test_nav_ahrs_quick_start_straight(float length, float direction);

/**
 * @brief  导航系统主循环（在定时器中调用）
 * @param  dt: 更新周期（秒），例如0.01表示10ms
 * @note   建议5-10ms周期调用
 */
void test_nav_ahrs_loop(float dt);

#endif // _TEST_NAV_AHRS_H_
