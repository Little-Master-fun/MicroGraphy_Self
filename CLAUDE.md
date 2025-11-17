# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述
这是一个基于Infineon CYT4BB微控制器的双核自动驾驶小车项目（MicroGraphy2.0）。项目使用IAR开发环境，采用双核架构实现导航控制和电机控制。

## 构建��运行

### 编译项目
1. 使用IAR打开工程文件：`project/iar/cyt4bb7.eww`
2. 双核工程配置：
   - CM7_0核心：`project/iar/project_config/cyt4bb7_cm_7_0.ewp`
   - CM7_1核心：`project/iar/project_config/cyt4bb7_cm_7_1.ewp`

### 清理临时文件
```bash
# Windows系统
project/iar/删除临时文件IAR.bat
```

### 调试配置
- 双核同时调试配置：`project/iar/project_config/cm7_0_cm7_1_debug.xml`

## 架构设计

### 双核架构分工
- **CM7_0核心**（底层数据采集和控制）：
  - IMU数据采集和AHRS姿态解算（1ms周期）
  - 编码器数据读取
  - 电机PWM控制（2ms周期）
  - 向CM7_1共享数据（5ms周期）

- **CM7_1核心**（高级算法和决策）：
  - 导航算法运算（5ms周期）
  - 路径规划
  - 控制指令传输给CM7_0

### 核心模块
- **导航控制系统** (`nav_control_ahrs.h/c`)：基于航向角和里程的导航系统，支持路径点记录和回放
- **IMU AHRS互补滤波** (`imu_ahrs_complementary.h/c`)：姿态解算
- **电机控制** (`motor_control.h/c`)：双PID控制（直行/转弯）
- **双核通信** (`dual_core_comm.h/c`)：核间数据共享机制

## 关键配置参数
位于 `project/code/config/config_infineon.h`：
- 导航参数：轮距0.134m，轮径0.065m
- 最大路径点数：2000
- 每点距离：4mm
- 控制周期：CM7_0(1-2ms)，CM7_1(5ms)

## 代码组织结构
```
project/
├── code/          # 用户代码目录
│   ├── config/    # 配置文件
│   ├── control/   # 控制算法（导航、电机、IMU）
│   ├── drivers/   # 硬件驱动（编码器、电机、���感器）
│   ├── test/      # 测试模块
│   └── ui/        # 用户界面
├── user/          # 主程序入口
│   ├── main_cm7_0.c  # CM7_0核心主程序
│   └── main_cm7_1.c  # CM7_1核心主程序
└── iar/           # IAR工程文件

libraries/         # 逐飞科技提供的底层库
├── zf_driver/     # 硬件驱动抽象层
├── zf_device/     # 外设驱动库
└── sdk/           # Infineon SDK
```

## 导航系统工作流程
1. 生成测试路径（正方形/长方形/直线）
2. CM7_0采集IMU和编码器数据，计算姿态和里程
3. CM7_1执行导航算法，计算目标航向角
4. 通过PID控制计算左右轮速度差
5. CM7_0执行电机PWM输出

## 注意事项
- 文件编码：代码文件使用GBK编码（中文注释）
- GPL3.0开源协议
- 基于逐飞科技CYT4BB开源库