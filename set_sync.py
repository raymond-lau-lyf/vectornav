import serial
import time

# 目标串口和波特率
PORT = "/dev/ttyACM1"
BAUD = 921600

# 定义命令（每条都是以 0xAB 开头的 binary 命令）
cmds = [
    b'\xAB\x06\x00\x00\x00\x01',       # Set AsyncDataOutputType = 1 (IMU data only)
    b'\xAB\x07\x00\x00\x03\x20',       # Set AsyncDataOutputFreq = 800 (0x0320)
    b'\xAB\x20\x00\x00\x00\x03',       # Set SyncInMode = 3 (COUNT) TODO
    # b'\xAB\x20\x00\x00\x00\x05',       # Set SyncInMode = 5 (ASYNC) TODO
    b'\xAB\x21\x00\x00\x00\x00',       # Set SyncInEdge = 0 (RISING)
    b'\xAB\x1E\x00\x00\x00\x01',       # Set SerialAsyncOutputFields = 1 (SyncInCount only)
    b'\xAB\x05\x00\x00\x00\x01',       # Set Communication Protocol to Binary
    b'\xAB\xFF'                        # Save to Flash (Write Settings to NVM)
]

# 打开串口
with serial.Serial(PORT, BAUD, timeout=1) as ser:
    for cmd in cmds:
        ser.write(cmd)
        print(f"Sent: {cmd.hex()}")
        time.sleep(0.1)  # 稍微等一下防止命令过快
