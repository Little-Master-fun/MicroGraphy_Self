# IMU 数据记录器 - 无线串口传输使用指南

## 功能概述

新增了通过无线串口模块实时传输 IMU 训练数据的功能，支持：

- ? 实时传输采集的数据（CSV 格式）
- ? 批量发送缓冲区数据
- ? 批量发送 Flash 存储的数据
- ? 可控制的传输使能/禁用

---

## 使用方法

### 1. 初始化

```c
#include "imu_data_logger.h"
#include "driver_wireless_uart.h"

void main(void)
{
    // 初始化无线串口模块
    wireless_uart_init();

    // 初始化数据记录器
    imu_logger_init();

    // ... 其他初始化 ...
}
```

### 2. 实时传输模式

#### 开启实时传输

```c
// 使能无线串口实时传输
imu_logger_wireless_enable(1);

// 开始记录数据（每条数据会自动通过无线串口发送）
imu_logger_start();
```

**效果**：

- 自动发送 CSV 表头
- 每记录一条数据，立即通过无线串口发送
- 上位机可实时接收并保存为 CSV 文件

#### 关闭实时传输

```c
// 禁用无线串口实时传输
imu_logger_wireless_enable(0);
```

### 3. 批量传输缓冲区数据

```c
// 先停止记录
imu_logger_stop();

// 发送缓冲区中的所有数据
uint32 sent = imu_logger_wireless_send_buffer(0, 0);
printf("已发送 %u 条记录\r\n", sent);

// 或者发送指定范围的数据
// 例如：发送第100-200条记录
uint32 sent = imu_logger_wireless_send_buffer(100, 100);
```

### 4. 批量传输 Flash 数据

```c
// 发送Flash中存储的所有数据
uint32 sent = imu_logger_wireless_send_flash(0, 0);
printf("已发送 %u 条记录\r\n", sent);

// 或者发送指定范围的数据
// 例如：发送第0-500条记录
uint32 sent = imu_logger_wireless_send_flash(0, 500);
```

---

## 串口命令控制

### 扩展命令处理函数

```c
void imu_logger_command_handler(uint8 *cmd, uint16 len)
{
    // ... 原有命令 ...

    // 新增无线传输命令
    if(strncmp((char*)cmd, "WIRELESS_ON", 11) == 0)
    {
        imu_logger_wireless_enable(1);
        printf("无线实时传输已开启\r\n");
    }
    else if(strncmp((char*)cmd, "WIRELESS_OFF", 12) == 0)
    {
        imu_logger_wireless_enable(0);
        printf("无线实时传输已关闭\r\n");
    }
    else if(strncmp((char*)cmd, "SEND_BUFFER", 11) == 0)
    {
        uint32 sent = imu_logger_wireless_send_buffer(0, 0);
        printf("已发送缓冲区数据 %u 条\r\n", sent);
    }
    else if(strncmp((char*)cmd, "SEND_FLASH", 10) == 0)
    {
        uint32 sent = imu_logger_wireless_send_flash(0, 0);
        printf("已发送Flash数据 %u 条\r\n", sent);
    }
    else if(strncmp((char*)cmd, "WIRELESS_STATUS", 15) == 0)
    {
        uint8 enabled = imu_logger_wireless_is_enabled();
        uint32 count = imu_logger_wireless_get_count();
        printf("无线传输状态: %s\r\n", enabled ? "开启" : "关闭");
        printf("已传输记录数: %u\r\n", count);
    }
}
```

### 串口命令列表

| 命令              | 功能            | 说明                |
| ----------------- | --------------- | ------------------- |
| `WIRELESS_ON`     | 开启实时传输    | 每条新数据自动发送  |
| `WIRELESS_OFF`    | 关闭实时传输    | 仅记录不发送        |
| `SEND_BUFFER`     | 发送缓冲区数据  | 批量发送当前缓冲区  |
| `SEND_FLASH`      | 发送 Flash 数据 | 批量发送 Flash 存储 |
| `WIRELESS_STATUS` | 查看传输状态    | 显示状态和计数      |

---

## 数据接收（上位机）

### 使用串口助手接收

1. **打开串口助手**

   - 波特率：115200
   - 数据位：8
   - 停止位：1
   - 校验位：无

2. **开启接收**

   - 选择"文本模式"
   - 勾选"保存数据"
   - 文件名：`imu_training_data.csv`

3. **发送命令**

   ```
   WIRELESS_ON
   LOG_START
   ```

4. **等待采集完成**

   - 实时查看数据流
   - CSV 数据会自动保存

5. **停止采集**
   ```
   LOG_STOP
   WIRELESS_OFF
   ```

### 使用 Python 脚本接收

```python
import serial
import time

# 打开串口
ser = serial.Serial('COM3', 115200, timeout=1)

# 打开CSV文件
with open('imu_training_data.csv', 'w') as f:
    print("开始接收数据...")

    # 发送开启命令
    ser.write(b'WIRELESS_ON\r\n')
    time.sleep(0.5)
    ser.write(b'LOG_START\r\n')

    # 接收数据
    try:
        while True:
            line = ser.readline().decode('utf-8', errors='ignore')
            if line:
                print(line.strip())
                f.write(line)
                f.flush()
    except KeyboardInterrupt:
        print("\n停止接收")
        ser.write(b'LOG_STOP\r\n')
        ser.write(b'WIRELESS_OFF\r\n')

ser.close()
print("数据接收完成")
```

---

## 性能说明

### 传输速率

- **波特率**: 115200 bps
- **单条记录**: 约 150 字节
- **理论速率**: 约 77 条/秒
- **实际速率**: 约 50 条/秒（含延迟保护）

### 延迟设置

代码中每条数据发送后有 5ms 延迟：

```c
system_delay_ms(5);  // 避免缓冲区溢出
```

**调整建议**：

- 实时传输：保持 5ms 延迟
- 批量传输：可减少到 2-3ms
- 高速传输：可去掉延迟，但需确保接收端能跟上

---

## 典型使用场景

### 场景 1：实时监控采集

```c
// 初始化
imu_logger_init();
wireless_uart_init();

// 开启实时传输
imu_logger_wireless_enable(1);
imu_logger_start();

// 采集1分钟（约3000条数据）
system_delay_ms(60000);

// 停止
imu_logger_stop();
imu_logger_wireless_enable(0);

printf("采集完成，数据已通过无线串口发送\r\n");
```

### 场景 2：先记录后传输

```c
// 关闭无线传输，仅记录
imu_logger_wireless_enable(0);
imu_logger_start();

// 采集2分钟
system_delay_ms(120000);

// 停止记录
imu_logger_stop();

// 保存到Flash
imu_logger_save_to_flash();

// 稍后通过无线发送
printf("准备发送数据，请连接上位机...\r\n");
system_delay_ms(5000);
imu_logger_wireless_send_flash(0, 0);
```

### 场景 3：分段传输大数据

```c
// Flash中有1000条记录，分5次发送
for(uint32 i = 0; i < 5; i++)
{
    printf("正在发送第 %u 段...\r\n", i+1);
    imu_logger_wireless_send_flash(i * 200, 200);
    system_delay_ms(1000);  // 间隔1秒
}
printf("全部发送完成\r\n");
```

---

## 注意事项

1. **无线模块初始化**

   - 必须先调用 `wireless_uart_init()`
   - 确认无线模块连接正常

2. **缓冲区管理**

   - 实时传输时缓冲区仍会保存数据
   - 可同时进行实时传输和 Flash 存储

3. **传输可靠性**

   - 无线传输可能丢包
   - 建议关键数据保存到 Flash 作为备份

4. **上位机接收**

   - 确保接收端缓冲区足够大
   - 建议使用支持大文件的串口助手

5. **数据格式**
   - CSV 格式，兼容 Excel/Python
   - 包含完整的表头说明

---

## API 函数总览

| 函数                                | 功能              |
| ----------------------------------- | ----------------- |
| `imu_logger_wireless_enable()`      | 使能/禁用实时传输 |
| `imu_logger_wireless_send_header()` | 发送 CSV 表头     |
| `imu_logger_wireless_send_record()` | 发送单条记录      |
| `imu_logger_wireless_send_buffer()` | 批量发送缓冲区    |
| `imu_logger_wireless_send_flash()`  | 批量发送 Flash    |
| `imu_logger_wireless_is_enabled()`  | 查询传输状态      |
| `imu_logger_wireless_get_count()`   | 获取传输计数      |

---

## 故障排除

### 问题 1：无数据输出

- 检查无线模块是否初始化
- 检查是否调用 `imu_logger_wireless_enable(1)`
- 检查是否开始记录 `imu_logger_start()`

### 问题 2：数据乱码

- 检查波特率是否匹配（115200）
- 检查无线模块连接是否稳定
- 尝试重启无线模块

### 问题 3：传输中断

- 检查缓冲区是否满
- 检查无线模块供电
- 减少传输延迟时间

### 问题 4：上位机接收慢

- 增大串口助手接收缓冲区
- 使用更高效的接收程序
- 增加代码中的延迟时间

---

## 版本历史

- **v1.0** (2025-11-25)
  - 初始版本
  - 支持实时传输和批量传输
  - 支持缓冲区和 Flash 数据发送
