# AHRS 姿态解算调试说明

## 调试变量说明

已添加全局调试变量 `g_ahrs_debug`，可以通过在线调试器查看。

## 调试步骤

### 1. 查看传感器配置信息

在程序初始化完成后，查看以下变量：

```
g_ahrs_debug.actual_sens_rate1        // 实际配置的灵敏度Rate1（应该是6400）
g_ahrs_debug.actual_sens_rate2        // 实际配置的灵敏度Rate2（应该是6400）
g_ahrs_debug.config_sens_rate1        // 代码中配置的灵敏度（6400.0）
g_ahrs_debug.expected_sensitivity     // 期望的转换因子（12800.0 = 6400 * 2）
```

**关键检查点：**

- `actual_sens_rate1` 应该等于 6400
- 如果不是 6400，说明传感器配置错误！

### 2. 保持设备静止，查看静态数据

设备静止时查看：

```
g_ahrs_debug.raw_gyro_x/y/z          // 原始LSB值（应该接近0，小幅波动）
g_ahrs_debug.gyro_x/y/z_dps          // 转换后角速度(度/秒)（应该接近0）
g_ahrs_debug.update_count            // 更新计数（应该持续增加）
```

**预期值：**

- 原始 LSB 值：几十到几百的范围
- dps 值：接近 0（±0.5 以内）

### 3. 慢速旋转设备 90 度（约 2 秒完成）

在旋转过程中观察：

```
g_ahrs_debug.raw_gyro_z              // Z轴原始LSB值
g_ahrs_debug.gyro_z_dps              // Z轴角速度(度/秒)
g_ahrs_debug.actual_conversion_factor // 实际转换因子
```

**预期值（以 90 度/2 秒 = 45 度/秒为例）：**

- `gyro_z_dps` 应该显示约 45 dps
- `raw_gyro_z` 应该约为 45 \* 12800 = 576,000 LSB（累加 2 次）
- `actual_conversion_factor` 应该约等于 12800

### 4. 旋转完成后查看姿态角

```
g_ahrs_debug.yaw_deg                 // 当前偏航角（-180~180度）
g_ahrs_debug.yaw_accumulated_deg     // 累积偏航角（应该接近90度）
```

## 问题诊断

### 场景 1：actual_sens_rate1 != 6400

**问题：** 传感器配置的灵敏度不对
**解决：** 检查传感器初始化代码，确认 `SCH1_init` 传入的灵敏度参数

### 场景 2：gyro_z_dps 很小，但 raw_gyro_z 也很小

**问题：** 传感器数据本身就小，可能是：

1. 传感器硬件问题
2. 传感器工作在错误的量程
3. 数据读取有问题

### 场景 3：gyro_z_dps 很小，但 raw_gyro_z 正常

**问题：** 转换公式错误
**检查：** `actual_conversion_factor` 的值

- 如果约等于 192000（12800 \* 15），说明多除了 15 倍！
- 需要检查 `SCH1_convert_data` 的转换公式

### 场景 4：gyro_z_dps 正常，但 yaw_accumulated_deg 很小

**问题：** 姿态解算算法有问题
**检查：**

- DEG_TO_RAD 转换是否正确（0.017453）
- DELTA_T 是否正确（0.002252）
- 四元数更新算法

## 15 倍偏差的可能原因

90 度 / 6 度 = 15 倍

可能的原因：

1. **灵敏度配置错误**：实际是 96000 而不是 6400（96000/6400=15）
2. **数据位数错误**：读取了错误的位数导致数据缩小
3. **重复除法**：某处重复除以了 AVG_FACTOR 或其他系数
4. **单位错误**：传感器输出的不是 20 位数据，而是其他位宽

## 调试顺序

1. 先查看 `actual_sens_rate1` 确认配置
2. 再查看 `raw_gyro_z` 和 `gyro_z_dps` 的关系
3. 计算 `actual_conversion_factor`，看是否等于 12800
4. 如果 `actual_conversion_factor ≈ 192000`，说明问题在转换公式
5. 如果 `actual_conversion_factor ≈ 12800`，说明问题在传感器配置或数据读取

## 快速测试

在调试器中设置断点，手动旋转设备，观察：

- 旋转时 `raw_gyro_z` 的变化幅度
- 对应的 `gyro_z_dps` 值
- 计算比值验证转换因子
