#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import json
import math
import socket       
import threading    
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray 

# --- CẤU HÌNH UDP ---
UDP_IP = "0.0.0.0"  
UDP_PORT = 5005     

class STM32Bridge(Node):
    def __init__(self):
        super().__init__('stm32_bridge')

        # Khai báo tham số với giá trị mặc định
        self.declare_parameter('wheel_radius', 0.034)
        self.declare_parameter('Lx', 0.12)
        self.declare_parameter('Ly', 0.105)

        # Đọc giá trị tham số khi node khởi chạy
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.Lx = self.get_parameter('Lx').get_parameter_value().double_value
        self.Ly = self.get_parameter('Ly').get_parameter_value().double_value

        self.get_logger().info(f"Đã tải tham số: R={self.wheel_radius}, Lx={self.Lx}, Ly={self.Ly}")
        
        self.enc_pub = self.create_publisher(Int32MultiArray, 'encoder_deltas', 10)
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)

        # --- ⭐ 2. THÊM SUBSCRIBER MỚI ---
        self.pid_sub = self.create_subscription(
            Float32MultiArray, 
            'pid_params', 
            self.pid_callback, 
            10)
        self.get_logger().info("Đã đăng ký subscriber /pid_params")
        # --- KẾT THÚC CODE MỚI ---
        

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        
        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value

        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"Kết nối cổng serial {port} thành công.")
        except Exception as e:
            self.get_logger().error(f"Không thể mở cổng serial {port}: {e}")
            rclpy.shutdown()
            return
        
        self.serial_buffer = b""
        self.create_timer(0.02, self.read_serial)
        
        
        #  UDP SERVER 
        self.get_logger().info(f"Đang khởi tạo UDP Server tại port {UDP_PORT}...")
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.bind((UDP_IP, UDP_PORT))
        self.udp_thread = threading.Thread(target=self.udp_listener, daemon=True)
        self.udp_thread.start()

    def udp_listener(self):
        while rclpy.ok():
            try:
                # Nhận dữ liệu từ Laptop
                data, addr = self.udp_sock.recvfrom(1024)
                message = json.loads(data.decode())
                
                # Lấy dữ liệu x (tiến/lùi) và z (quay)
                linear_x = float(message.get("x", 0.0))
                angular_z = float(message.get("z", 0.0))
                
                # TẠO MỘT MESSAGE ROS GIẢ (FAKE TWIST)
                # Để tái sử dụng lại hàm tính toán Mecanum bên dưới
                fake_msg = Twist()
                fake_msg.linear.x = linear_x
                fake_msg.angular.z = angular_z
                
                # Gọi trực tiếp hàm callback cũ để tính toán và gửi xuống STM32
                self.cmd_callback(fake_msg)
                
            except Exception as e:
                self.get_logger().warn(f"Lỗi nhận UDP: {e}")

    def read_serial(self):
        try:
            # 1. Đọc TẤT CẢ dữ liệu đang chờ trong bộ đệm OS (KHÔNG BLOCKING)
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting)
                self.serial_buffer += data

            # 2. Xử lý TẤT CẢ các dòng hoàn chỉnh có trong buffer nội bộ
            while b"\n" in self.serial_buffer:
                # Tách dòng đầu tiên ra khỏi buffer
                line_bytes, self.serial_buffer = self.serial_buffer.split(b"\n", 1)
                line = line_bytes.decode('utf-8').strip()

                # Xử lý dòng đã tách (giống code cũ của bạn)
                if line and line.startswith('d,'):
                    parts = line.split(',')
                    if len(parts) == 5:
                        msg = Int32MultiArray()
                        try:
                            delta_fl = int(parts[1])
                            delta_fr = int(parts[2])
                            delta_bl = int(parts[3])
                            delta_br = int(parts[4])
                            msg.data = [delta_fl, delta_fr, delta_bl, delta_br]
                            self.enc_pub.publish(msg)
                        except ValueError:
                            self.get_logger().warn(f"Dữ liệu odom không hợp lệ (không phải số): {line}")
                    # else: Bỏ qua dòng nếu không đủ 5 phần tử
                # else: Bỏ qua dòng nếu không bắt đầu bằng 'd,'

        except Exception as e:
            self.get_logger().warn(f"Lỗi khi đọc serial: {e}")




    # --- ⭐ 3. THÊM HÀM CALLBACK MỚI ---
    def pid_callback(self, msg: Float32MultiArray):
        if len(msg.data) == 3:
            kp, ki, kd = msg.data[0], msg.data[1], msg.data[2]
            
            # Gửi lệnh 'p' xuống STM32
            command = f"p,{kp:.4f},{ki:.4f},{kd:.4f}\n"
            
            try:
                self.ser.write(command.encode('utf-8'))
                self.get_logger().info(f"Đã gửi lệnh PID: {command.strip()}")
            except Exception as e:
                self.get_logger().warn(f"Lỗi gửi lệnh PID qua UART: {e}")
        else:
            self.get_logger().warn(f"Lệnh PID không hợp lệ: {msg.data}")
    # --- KẾT THÚC CODE MỚI ---
    
    # Sửa hàm cmd_callback
    def cmd_callback(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        vth = msg.angular.z

        L_sum = self.Lx + self.Ly
        # CÔNG THỨC KINEMATICS (Phiên bản gốc, đúng 100%)
        w_fl = (1/self.wheel_radius) * (vx - vy - L_sum * vth)
        w_fr = (1/self.wheel_radius) * (vx + vy + L_sum * vth)
        w_bl = (1/self.wheel_radius) * (vx + vy - L_sum * vth)
        w_br = (1/self.wheel_radius) * (vx - vy + L_sum * vth)
        
        self.get_logger().info(f"Commanded (rad/s): [FL:{w_fl:.2f}, FR:{w_fr:.2f}, BL:{w_bl:.2f}, BR:{w_br:.2f}]")
        
        # Định dạng: "s,w_fl,w_fr,w_bl,w_br\n" (Phiên bản gốc, đúng 100%)
        command = f"s,{w_fl:.4f},{w_fr:.4f},{w_bl:.4f},{w_br:.4f}\n"

       # self.get_logger().info(f"Đang gửi qua Serial: {command.strip()}")

        try:
            self.ser.write(command.encode('utf-8'))
        except Exception as e:
            self.get_logger().warn(f"Lỗi gửi dữ liệu qua UART: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = STM32Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
