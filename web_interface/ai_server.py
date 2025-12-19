import cv2
import threading
import time
import socket
import json
import math
from ultralytics import YOLO
from flask import Flask, Response, request, jsonify # Thêm request, jsonify
from flask_cors import CORS

# --- CẤU HÌNH HỆ THỐNG ---
IP_NUC = "192.168.1.21"
PORT_CONTROL = 5005
TOPIC_NAME = "/image_raw"
STREAM_URL = f"http://{IP_NUC}:8080/stream?topic={TOPIC_NAME}&type=mjpeg"

# --- CẤU HÌNH AI & CAMERA ---
AI_SIZE = 320
CONF_THRESHOLD = 0.5

# --- CẤU HÌNH FOLLOW ---
TARGET_HEIGHT = 180
MAX_SPEED_X = 0.4
MAX_ROT_Z = 1.0
KP_LINEAR = 1.5
KP_ANGULAR = 1.0
KD_ANGULAR = 0.5

udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

app = Flask(__name__)
CORS(app) # Cho phép React gọi API

class SharedState:
    def __init__(self):
        self.raw_frame = None
        self.processed_frame = None
        self.lock = threading.Lock()
        self.running = True
        
        # --- THÊM BIẾN NÀY ---
        self.follow_mode = False # Mặc định là OFF (chỉ detect, không chạy)

state = SharedState()

# --- API ĐỂ REACT GỌI ---
@app.route('/set_follow_mode', methods=['POST'])
def set_follow_mode():
    data = request.json
    mode = data.get('active', False)
    with state.lock:
        state.follow_mode = mode
    
    # Nếu tắt follow, gửi lệnh dừng xe ngay lập tức cho an toàn
    if not mode:
        udp_socket.sendto(json.dumps({"x": 0.0, "z": 0.0}).encode(), (IP_NUC, PORT_CONTROL))
        
    return jsonify({"status": "success", "mode": state.follow_mode})

# --- LUỒNG 1: CAMERA (Giữ nguyên) ---
def capture_worker():
    # ... (Code giữ nguyên như cũ) ...
    print(f"Connecting to Camera stream: {STREAM_URL}")
    cap = cv2.VideoCapture(STREAM_URL)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    
    while state.running:
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                with state.lock:
                    state.raw_frame = frame
            else:
                cap.release()
                time.sleep(1)
                cap = cv2.VideoCapture(STREAM_URL)
        else:
            time.sleep(1)
            cap = cv2.VideoCapture(STREAM_URL)
        time.sleep(0.001)
    cap.release()

# --- LUỒNG 2: AI & ĐIỀU KHIỂN (Đã sửa logic) ---
# --- LUỒNG 2: AI & ĐIỀU KHIỂN (Đã sửa chế độ Detect linh hoạt) ---
def ai_worker():
    print("Loading YOLOv8 Model...")
    model = YOLO("yolov8n.pt")
    
    # Biến PD Controller
    prev_err_x = 0.0
    last_time = time.time()
    
    # --- BIẾN MỚI: BỘ NHỚ HƯỚNG & TRẠNG THÁI ---
    last_err_x = 0.0          
    last_seen_time = time.time()
    
    # Cấu hình tìm kiếm
    SEARCH_TIMEOUT = 4.0      
    SEARCH_SPEED_Z = 0.5      
    
    # Cờ kiểm tra: Đã nhìn thấy mục tiêu lần nào chưa?
    has_locked_target = False 
    
    # Biến để detect việc chuyển chế độ
    last_mode_status = False

    while state.running:
        img_input = None
        with state.lock:
            current_mode = state.follow_mode
            if state.raw_frame is not None:
                img_input = state.raw_frame.copy()
        
        # --- LOGIC RESET KHI BẬT/TẮT FOLLOW ---
        if current_mode and not last_mode_status:
            has_locked_target = False
            prev_err_x = 0.0
        
        last_mode_status = current_mode

        if img_input is None:
            time.sleep(0.01)
            continue

        try:
            current_time = time.time()
            dt = current_time - last_time
            if dt == 0: dt = 0.001 

            # --- SỬA Ở ĐÂY: QUYẾT ĐỊNH CLASS CẦN DETECT ---
            # Nếu đang Follow -> Chỉ detect người (class 0)
            # Nếu đang Tắt Follow -> Detect tất cả (None)
            target_classes = [0] if current_mode else None

            # Truyền biến target_classes vào model
            results = model(img_input, imgsz=AI_SIZE, verbose=False, conf=CONF_THRESHOLD, classes=target_classes, iou=0.5)
            
            frame_h, frame_w = img_input.shape[:2]
            center_x_frame = frame_w // 2
            
            cmd_linear_x = 0.0
            cmd_angular_z = 0.0
            detected = False

            # --- 1. CÓ NHÌN THẤY VẬT THỂ (NGƯỜI HOẶC VẬT KHÁC) ---
            if len(results[0].boxes) > 0:
                detected = True
                
                # Sắp xếp lấy vật to nhất
                boxes = sorted(results[0].boxes, key=lambda x: (x.xyxy[0][2]-x.xyxy[0][0]) * (x.xyxy[0][3]-x.xyxy[0][1]), reverse=True)
                box = boxes[0]
                
                # --- CHỈ TÍNH TOÁN DI CHUYỂN KHI ĐANG Ở FOLLOW MODE ---
                if current_mode: 
                    has_locked_target = True # Đánh dấu đã thấy người
                    last_seen_time = current_time 
                    
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    cx = (x1 + x2) / 2
                    box_h = y2 - y1
                    
                    # P-D Controller
                    err_x = (center_x_frame - cx) / (frame_w / 2) 
                    last_err_x = err_x 
                    
                    d_err_x = (err_x - prev_err_x) / dt 
                    cmd_angular_z = (err_x * KP_ANGULAR) + (d_err_x * KD_ANGULAR)
                    cmd_angular_z = cmd_angular_z * MAX_ROT_Z

                    # P Controller
                    err_h = (TARGET_HEIGHT - box_h) / TARGET_HEIGHT
                    cmd_linear_x = -1 * err_h * KP_LINEAR * MAX_SPEED_X

                    prev_err_x = err_x

            # --- 2. KHÔNG THẤY GÌ (MẤT DẤU) ---
            else:
                if current_mode:
                    if has_locked_target:
                        time_lost = current_time - last_seen_time
                        if time_lost < SEARCH_TIMEOUT:
                            cmd_linear_x = 0.0
                            if last_err_x > 0.05:
                                cmd_angular_z = SEARCH_SPEED_Z 
                            elif last_err_x < -0.05:
                                cmd_angular_z = -SEARCH_SPEED_Z
                            else:
                                cmd_angular_z = 0.0
                        else:
                            cmd_linear_x = 0.0
                            cmd_angular_z = 0.0
                            has_locked_target = False 
                    else:
                        cmd_linear_x = 0.0
                        cmd_angular_z = 0.0

            # --- DEADBAND & CLAMP ---
            if detected and current_mode and abs(cmd_angular_z) < 0.1: cmd_angular_z = 0 
            
            cmd_linear_x = max(-MAX_SPEED_X, min(MAX_SPEED_X, cmd_linear_x))
            cmd_angular_z = max(-MAX_ROT_Z, min(MAX_ROT_Z, cmd_angular_z))

            # --- GỬI LỆNH ---
            if current_mode:
                msg_payload = {"x": round(cmd_linear_x, 2), "z": round(cmd_angular_z, 2)}
                udp_socket.sendto(json.dumps(msg_payload).encode(), (IP_NUC, PORT_CONTROL))
            
            last_time = current_time

            # --- VẼ DEBUG ---
            annotated_frame = results[0].plot()
            with state.lock:
                state.processed_frame = annotated_frame
                
        except Exception as e:
            print(f"Error: {e}")
# --- LUỒNG 3: FLASK DISPLAY (Giữ nguyên) ---
def generate_frames():
    while True:
        frame_out = None
        with state.lock:
            if state.processed_frame is not None:
                frame_out = state.processed_frame
            elif state.raw_frame is not None:
                frame_out = state.raw_frame
        
        if frame_out is None:
            time.sleep(0.1)
            continue

        h, w = frame_out.shape[:2]
        if w > 640:
            frame_out = cv2.resize(frame_out, (640, int(h * 640/w)))

        ret, buffer = cv2.imencode('.jpg', frame_out, [cv2.IMWRITE_JPEG_QUALITY, 60])
        if ret:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        time.sleep(0.04)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    t1 = threading.Thread(target=capture_worker, daemon=True)
    t2 = threading.Thread(target=ai_worker, daemon=True)
    t1.start()
    t2.start()
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)