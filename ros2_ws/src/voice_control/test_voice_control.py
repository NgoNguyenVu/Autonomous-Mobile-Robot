import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import speech_recognition as sr
import time

# --- CẤU HÌNH ---
# Nếu vẫn lỗi mic, hãy thử xóa dòng này và dùng self.mic = sr.Microphone()
MIC_INDEX = None  
TOPIC_NAME = '/cmd_vel' 
# ----------------

class MecanumVoiceControl(Node):
    def __init__(self):
        super().__init__('mecanum_voice_control')
        self.publisher_ = self.create_publisher(Twist, TOPIC_NAME, 10)
        
        self.recognizer = sr.Recognizer()
        
        # Cấu hình độ nhạy
        self.recognizer.energy_threshold = 300 
        self.recognizer.dynamic_energy_threshold = True
        self.recognizer.pause_threshold = 0.5 
        
        try:
            # Nếu MIC_INDEX là None thì lấy mic mặc định, nếu có số thì lấy theo số
            if MIC_INDEX is None:
                self.mic = sr.Microphone()
            else:
                self.mic = sr.Microphone(device_index=MIC_INDEX)
        except Exception as e:
            self.get_logger().error(f"Lỗi Mic: {e}")
            return

        self.get_logger().info("--- MECANUM VOICE READY ---")
        
        self.listen_and_move()

    def listen_and_move(self):
        # --- SỬA LỖI SEGMENTATION FAULT TẠI ĐÂY ---
        # Mở Mic MỘT LẦN DUY NHẤT bên ngoài vòng lặp
        with self.mic as source:
            print("Calibrating noise... (Silence please)")
            self.recognizer.adjust_for_ambient_noise(source, duration=1.0)
            print("Done! Listening...")

            while rclpy.ok():
                try:
                    print("\nListening...", end='', flush=True)
                    # Mic đã mở sẵn, chỉ việc nghe
                    audio = self.recognizer.listen(source, timeout=None, phrase_time_limit=3)
                    
                    try:
                        command = self.recognizer.recognize_google(audio).lower()
                        print(f" -> Heard: '{command}'")
                        self.execute_command(command)
                        
                    except sr.UnknownValueError:
                        print(".", end='', flush=True)
                    except sr.RequestError:
                        print("\n[!] Internet Error.")
                    
                except Exception as e:
                    # Bắt lỗi nhẹ để không crash chương trình
                    print(f"\nError: {e}")
                    pass

    def execute_command(self, text):
        msg = Twist()
        
        # --- 1. ĐI THẲNG / LÙI ---
        if "go" in text or "forward" in text or "move" in text:
            msg.linear.x = 0.15
            self.get_logger().info(" => GO FORWARD")
            self.publisher_.publish(msg)
            
        elif "back" in text or "backward" in text or "reverse" in text:
            msg.linear.x = -0.15
            self.get_logger().info(" => GO BACKWARD")
            self.publisher_.publish(msg)

        # --- 2. TRƯỢT NGANG ---
        elif "slide left" in text or "step left" in text:
            msg.linear.y = 0.2
            self.get_logger().info(" => SLIDE LEFT")
            self.publisher_.publish(msg)

        elif "slide right" in text or "step right" in text:
            msg.linear.y = -0.2
            self.get_logger().info(" => SLIDE RIGHT")
            self.publisher_.publish(msg)

        # --- 3. XOAY TẠI CHỖ ---
        elif "turn left" in text or "rotate left" in text:
            msg.angular.z = 0.5
            self.get_logger().info(" => TURN LEFT")
            self.publisher_.publish(msg)

        elif "turn right" in text or "rotate right" in text:
            msg.angular.z = -0.5
            self.get_logger().info(" => TURN RIGHT")
            self.publisher_.publish(msg)

        # --- 4. DỪNG XE ---
        elif "stop" in text or "halt" in text or "wait" in text:
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.angular.z = 0.0
            self.get_logger().info(" => STOP !!!")
            for _ in range(5):
                self.publisher_.publish(msg)
                time.sleep(0.05)

def main(args=None):
    rclpy.init(args=args)
    node = MecanumVoiceControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
