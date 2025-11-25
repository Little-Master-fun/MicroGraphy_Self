# IMU 数据记录器 - 使用模式说明

## 功能概述

数据记录器现在支持**独立控制**Flash 存储和无线串口发送，提供三种工作模式：

| 模式                 | Flash 存储 | 无线发送 | 适用场景                   |
| -------------------- | ---------- | -------- | -------------------------- |
| **模式 0：仅 Flash** | ?         | ?       | 离线数据采集，后期批量导出 |
| **模式 1：仅无线**   | ?         | ?       | **实时监控**，无需存储     |
| **模式 2：双模式**   | ?         | ?       | 实时监控+数据备份          |

---

## API 接口

### 1. 模式设置（推荐）

```c
void imu_logger_set_mode(uint8 mode);
```

**参数说明：**

- `mode = 0` - 仅 Flash 存储
- `mode = 1` - **仅无线串口实时发送**
- `mode = 2` - 同时使用 Flash 和无线

**示例：**

```c
// 初始化
imu_logger_init();

// 设置为"仅无线实时发送"模式
imu_logger_set_mode(1);  // 自动发送CSV表头

// 启动记录
imu_logger_start();

// 在主循环中更新
while(1)
{
    imu_logger_update(left_speed, right_speed);  // 自动通过串口发送数据
    system_delay_ms(20);  // 50Hz采样
}
```

---

### 2. 独立控制接口

#### 2.1 Flash 存储控制

```c
// 使能/禁用Flash存储
void imu_logger_flash_enable(uint8 enable);

// 查询Flash存储状态
uint8 imu_logger_flash_is_enabled(void);
```

**示例：**

```c
// 关闭Flash存储（节省存储空间）
imu_logger_flash_enable(0);

// 检查状态
if(imu_logger_flash_is_enabled())
{
    printf("Flash存储已启用\n");
}
```

#### 2.2 无线发送控制

```c
// 使能/禁用无线实时发送
void imu_logger_wireless_enable(uint8 enable);

// 查询无线发送状态
uint8 imu_logger_wireless_is_enabled(void);

// 获取已发送记录数
uint32 imu_logger_wireless_get_count(void);
```

**示例：**

```c
// 开启无线实时发送
imu_logger_wireless_enable(1);

// 检查状态
if(imu_logger_wireless_is_enabled())
{
    printf("无线发送已启用，已发送 %d 条记录\n",
           imu_logger_wireless_get_count());
}
```

---

## 典型使用场景

### 场景 1：仅无线实时监控（推荐用于调试）

**优点：**

- ? 实时查看数据（CSV 格式）
- ? 不占用 Flash 存储
- ? 可直接用 Python/串口工具接收

**代码示例：**

```c
void wireless_realtime_mode(void)
{
    // 初始化
    imu_logger_init();

    // 设置为"仅无线"模式
    imu_logger_set_mode(1);

    // 启动记录
    imu_logger_start();

    printf("实时数据通过无线串口发送中...\n");

    // 主循环
    while(1)
    {
        // 采集并通过无线串口发送
        imu_logger_update(motor_get_left_speed(), motor_get_right_speed());

        system_delay_ms(20);  // 50Hz采样率
    }
}
```

**接收数据（Python 脚本）：**

```python
import serial
import csv

ser = serial.Serial('COM3', 115200, timeout=1)
output_file = open('imu_data.csv', 'w', newline='')
writer = csv.writer(output_file)

print("接收实时数据...")

while True:
    line = ser.readline().decode('utf-8').strip()
    if line:
        print(line)  # 打印到控制台
        writer.writerow(line.split(','))  # 写入CSV
        output_file.flush()  # 立即保存
```

---

### 场景 2：仅 Flash 离线采集

**优点：**

- ? 长时间数据采集（无线传输有时延）
- ? 稳定可靠，不受无线干扰
- ? 后期批量导出分析

**代码示例：**

```c
void flash_offline_mode(void)
{
    // 初始化
    imu_logger_init();

    // 设置为"仅Flash"模式
    imu_logger_set_mode(0);

    // 启动记录
    imu_logger_start();

    printf("数据记录到Flash中...\n");

    // 采集5分钟数据
    for(uint32 i = 0; i < 15000; i++)  // 50Hz * 300s
    {
        imu_logger_update(motor_get_left_speed(), motor_get_right_speed());
        system_delay_ms(20);
    }

    // 停止记录
    imu_logger_stop();

    // 保存到Flash
    imu_logger_save_to_flash();

    // 后续通过无线导出
    printf("开始导出Flash数据...\n");
    imu_logger_wireless_enable(1);
    imu_logger_wireless_send_flash(0, 1000);  // 导出1000条记录
}
```

---

### 场景 3：双模式（实时监控+备份）

**优点：**

- ? 实时查看数据走势
- ? Flash 备份防止数据丢失
- ? 适合重要实验

**代码示例：**

```c
void dual_mode(void)
{
    // 初始化
    imu_logger_init();

    // 设置为"双模式"
    imu_logger_set_mode(2);

    // 启动记录
    imu_logger_start();

    printf("数据同时存储到Flash和无线发送\n");

    // 主循环
    while(1)
    {
        // 数据会同时：
        // 1. 存入Flash缓冲区
        // 2. 通过无线串口发送
        imu_logger_update(motor_get_left_speed(), motor_get_right_speed());

        system_delay_ms(20);
    }
}
```

---

## 性能对比

| 特性         | 仅 Flash | 仅无线                 | 双模式          |
| ------------ | -------- | ---------------------- | --------------- |
| **采样率**   | 50Hz     | **50Hz (延迟 5ms)**    | 50Hz (延迟 5ms) |
| **数据容量** | 1000 条  | ∞（流式传输）          | 1000 条         |
| **实时性**   | ?       | ?                     | ?              |
| **可靠性**   | ?       | ??（受无线稳定性影响） | ?              |
| **功耗**     | 低       | 中                     | 高              |

---

## 数据格式

### CSV 格式（无线发送）

```csv
timestamp_ms,power_on_time_s,temperature_deg,left_wheel_speed,right_wheel_speed,...
1234,10.5,25.3,120.5,118.2,...
1254,10.5,25.3,121.0,119.0,...
```

### Flash 存储格式

- **头部**：魔术字(0x12345678) + 记录数量
- **数据**：二进制格式，每条 60 字节

---

## 常见问题

### Q1：为什么无线发送有延迟？

**A:** 每条记录通过串口发送需要约 5ms，建议采样率 ≤50Hz。如需更高采样率，请使用"仅 Flash"模式。

### Q2：仅无线模式下，数据会丢失吗？

**A:** 如果无线连接中断，数据会丢失。对于重要实验，建议使用"双模式"。

### Q3：如何切换模式？

**A:** 调用 `imu_logger_set_mode(mode)` 即可，可在运行时动态切换。

### Q4：Flash 存满了怎么办？

**A:** 调用 `imu_logger_erase_flash()` 清空 Flash，或使用"仅无线"模式避免占用 Flash。

---

## 快速参考

```c
// ============ 初始化 ============
imu_logger_init();

// ============ 模式选择 ============
imu_logger_set_mode(1);  // 0-仅Flash, 1-仅无线, 2-双模式

// ============ 启动记录 ============
imu_logger_start();

// ============ 主循环 ============
while(1)
{
    imu_logger_update(left_speed, right_speed);
    system_delay_ms(20);
}

// ============ 停止记录 ============
imu_logger_stop();

// ============ Flash操作 ============
imu_logger_save_to_flash();       // 保存缓冲区到Flash
imu_logger_wireless_send_flash(0, 100);  // 导出Flash数据

// ============ 查询状态 ============
printf("Flash状态: %s\n", imu_logger_flash_is_enabled() ? "启用" : "禁用");
printf("无线状态: %s\n", imu_logger_wireless_is_enabled() ? "启用" : "禁用");
printf("已发送: %d 条\n", imu_logger_wireless_get_count());
```

---

## 推荐配置

**日常调试：** `imu_logger_set_mode(1)` - 仅无线实时监控  
**数据采集：** `imu_logger_set_mode(0)` - 仅 Flash 存储  
**重要实验：** `imu_logger_set_mode(2)` - 双模式备份

**注意：** 首次使用时，建议先用"仅无线"模式测试数据是否正确，再进行大规模采集。
