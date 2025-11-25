import serial
import time
from datetime import datetime

# ===================== 配置区：改这里 =====================
SERIAL_PORT = "COM5"      # 串口号
BAUDRATE    = 115200      # 波特率，要和无线串口模块一致
# ========================================================

# MCU 那边 CSV 的表头
HEADER_LINE = (
    "timestamp_ms,power_on_time_s,temperature_deg,temp_rate_degps,"
    "left_speed,right_speed,linear_vel,is_stationary,"
    "imu963ra_acc_x,imu963ra_acc_y,imu963ra_acc_z,"
    "imu963ra_gyro_z_raw,imu963ra_gyro_z_dps,"
    "sch16tk10_gyro_z_raw,sch16tk10_gyro_z_dps,"
    "gyro_diff_dps"
)

def main():
    # 生成带时间戳的文件名
    filename = datetime.now().strftime("imu_wireless_log_%Y%m%d_%H%M%S.csv")
    print(f"[INFO] 保存数据到: {filename}")

    # 打开串口
    print(f"[INFO] 打开串口 {SERIAL_PORT} @ {BAUDRATE} ...")
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)

    # 打开文件
    f = open(filename, "w", encoding="utf-8", newline="")

    header_written = False      # 记录是否已经写过表头
    line_count = 0

    print("[INFO] 开始接收数据，按 Ctrl + C 停止。\n")

    try:
        while True:
            # 读取一行（直到 \n）
            raw = ser.readline()
            if not raw:
                continue

            # 解码成字符串，并去掉 \r\n
            try:
                s = raw.decode("utf-8", errors="ignore").strip()
            except UnicodeDecodeError:
                continue

            if not s:
                continue

            # 如果 MCU 发来了表头
            if s.startswith("timestamp_ms"):
                if not header_written:
                    f.write(s + "\n")
                    f.flush()
                    header_written = True
                    print("[INFO] 收到 MCU 表头，已写入文件。")
                else:
                    # 重复表头（比如你重新 enable 无线模式），可以选择跳过
                    print("[WARN] 收到重复表头，已跳过。")
                continue

            # 如果还没写过表头，说明脚本启动时错过了那一行，自动补一行固定表头
            if not header_written:
                f.write(HEADER_LINE + "\n")
                f.flush()
                header_written = True
                print("[WARN] 没收到表头，使用默认表头写入。")

            # 正常数据行，直接写入
            f.write(s + "\n")
            line_count += 1

            # 偶尔 flush 一下，防止断电丢数据
            if line_count % 50 == 0:
                f.flush()
                print(f"[INFO] 已接收 {line_count} 行数据...")

            # print(s)

    except KeyboardInterrupt:
        print("\n[INFO] 用户中断，停止接收。")
    finally:
        try:
            ser.close()
        except Exception:
            pass
        try:
            f.close()
        except Exception:
            pass
        print(f"[INFO] 串口已关闭，文件已保存，共 {line_count} 行数据。")

if __name__ == "__main__":
    main()
