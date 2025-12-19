import socket
import serial
import json
import time

# Cấu hình Serial nối với STM32
# Lưu ý: check cổng com (ví dụ /dev/ttyUSB0 hoặc /dev/ttyACM0)
stm32 = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

# Cấu hình UDP Server
UDP_IP = "0.0.0.0" # Lắng nghe tất cả
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print("NUC Bridge Started...")

while True:
    try:
        data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
        command = json.loads(data.decode())
        
        v = int(command.get('v', 0)) # Tốc độ tiến lùi
        w = int(command.get('w', 0)) # Tốc độ quay
        
        # Đóng gói gửi xuống STM32
        # Format ví dụ: <V,W>\n  (Ví dụ: <50,-20>)
        msg_to_stm = f"<{v},{w}>\n"
        stm32.write(msg_to_stm.encode())
        
        # print(f"Sent to STM32: {msg_to_stm}")
        
    except Exception as e:
        print(e)
