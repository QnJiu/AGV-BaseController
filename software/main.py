import serial
import struct
import time
import threading

# 1. 打开串口
try:
    ser = serial.Serial('COM8', 115200, timeout=0.05)
    print("✅ 串口打开成功！")
except Exception as e:
    print(f"❌ 串口打开失败: {e}")
    exit()


# 2. 定义【接收数据】的线程任务
def receive_data():
    last_print_time = 0  # 【新增】：用来记录上一次在屏幕上打印的时间

    while True:
        try:
            if ser.in_waiting > 0:
                byte_1 = ser.read(1)
                if byte_1 == b'\xAA':
                    byte_2 = ser.read(1)
                    if byte_2 == b'\x55':
                        data = ser.read(6)
                        if len(data) == 6:
                            payload = data[0:4]
                            checksum_recv = data[4]
                            frame_tail = data[5]

                            checksum_calc = sum(payload) & 0xFF

                            if checksum_calc == checksum_recv and frame_tail == 0x0A:
                                left_speed = struct.unpack('>h', data[0:2])[0]
                                right_speed = struct.unpack('>h', data[2:4])[0]

                                # 【核心修复】：限制打印频率！只有距离上次打印超过 0.5 秒，才允许再次上屏
                                current_time = time.time()
                                if current_time - last_print_time > 0.5:
                                    if left_speed != 0 or right_speed != 0:
                                        print(f" [实时状态] 左轮: {left_speed} | 右轮: {right_speed}")
                                    last_print_time = current_time  # 更新最后打印时间
        except:
            pass
        time.sleep(0.01)


# 3. 启动接收线程
t = threading.Thread(target=receive_data, daemon=True)
t.start()
# 4. 主线程变成【指令发送台】
print("\n🎮 遥控终端已启动！")
print("请输入指令并按回车: [G]前进  [B]后退  [S]停止")
try:
    while True:
        cmd = input()  # 等待你敲击键盘
        if cmd.upper() in ['G', 'B', 'S']:
            ser.write(cmd.upper().encode())
            print(f"🚀 成功发送: {cmd.upper()}")
        else:
            print("⚠️ 未知指令，请输入 G, B 或 S")
except KeyboardInterrupt:
    ser.close()
    print("\n🛑 程序退出，串口已释放。")