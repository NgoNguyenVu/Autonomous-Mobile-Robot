# 🤖 Autonomous Mobile Robot - Graduation Project

![Robot Demo](https://www.youtube.com/watch?v=VJ9UU_NtgG0&t=1s)

Đồ án tốt nghiệp: Hệ thống Robot tự hành giám sát thông minh tích hợp AI và Web Control.
Dự án bao gồm: Điều khiển xe Mecanum, tự động khám phá map ,định vị SLAM, tự động tránh vật cản, nhận diện vật thể (AI) và điều khiển bằng giọng nói.


## ✨ Tính năng chính (Features)

* **Web Dashboard:** Giao diện giám sát camera, bản đồ và điều khiển robot từ xa (ReactJS).
* **Explore Map:** Tự động khám phá và mở bản đồ.
* **AI Object Detection:** Nhận diện vật thể thời gian thực sử dụng YOLOv8.
* **Navigation:** Tự động lập bản đồ (SLAM) và tìm đường đi ngắn nhất.
* **Voice Control:** Điều khiển robot bằng giọng nói tiếng Việt/Anh.
* **Hardware Control:** Giao tiếp với vi điều khiển STM32 qua Serial.

## 🛠️ Cấu trúc dự án (Project Structure)

* `ros2_ws/`: Chứa source code ROS 2 (Điều khiển, SLAM, Navigation).
    * `explorer_map`: Thuật toán khám phá bản đồ.
    * `mecanum_control`: Kinematics cho bánh xe Mecanum.
    * `voice_control`: Xử lý lệnh giọng nói.
    * `stm32_bridge`: Cầu nối giao tiếp Hardware.
* `web_interface/`: Source code Web App (ReactJS + Vite).
* `ai_server.py`: Server xử lý AI (YOLOv8) độc lập.
