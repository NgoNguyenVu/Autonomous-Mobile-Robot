import React, { useState, useEffect } from 'react';
import { RosProvider, useRos } from './contexts/RosContext'; 
import ConnectionStatus from './components/ConnectionStatus';
import TeleopControls from './components/TeleopControls';
import MapViewer from './components/MapViewer';
import WaypointManager from './components/WaypointManager';
import AICamera from './components/AICamera';

function MainLayout() {
  const { ros } = useRos();
  
  // --- STATE ---
  const [interactionMode, setInteractionMode] = useState('NONE');
  const [showLaser, setShowLaser] = useState(false);
  const [showTeleopPopup, setShowTeleopPopup] = useState(false);
  const [waypoints, setWaypoints] = useState(() => {
    try {
      // Thử lấy dữ liệu từ localStorage
      const saved = localStorage.getItem('robot_waypoints');
      // Nếu có thì parse JSON, nếu không thì trả về mảng rỗng []
      return saved ? JSON.parse(saved) : [];
    } catch (e) {
      console.error("Lỗi đọc localStorage:", e);
      return [];
    }
  });
  
  // State cho tính năng chạy tuần tra (Patrol)
  const [robotPose, setRobotPose] = useState(null);
  const [sequenceState, setSequenceState] = useState({
      active: false,      // Đang chạy hay không
      currentIndex: -1,   // Đang chạy đến điểm index mấy
      isWaiting: false    // Đang đếm ngược 1s hay không
  });

  useEffect(() => {
    // Mỗi khi biến waypoints thay đổi (thêm, xóa), lưu ngay vào máy
    localStorage.setItem('robot_waypoints', JSON.stringify(waypoints));
  }, [waypoints]);

  // =========================================================
  // LOGIC 1: LẮNG NGHE VỊ TRÍ ROBOT ĐỂ TÍNH KHOẢNG CÁCH
  // =========================================================
  useEffect(() => {
    if (!ros) return;
    // Subscribe topic chứa vị trí robot (thường là /amcl_pose hoặc /web/robot_pose do bạn tự publish)
    const poseTopic = new window.ROSLIB.Topic({ 
        ros: ros, 
        name: '/web/robot_pose', // Đảm bảo topic này publish đúng geometry_msgs/PoseStamped hoặc tương tự
        messageType: 'geometry_msgs/msg/TransformStamped' // Hoặc loại msg mà bạn đang dùng
    });
    
    const handlePose = (msg) => {
        // Lưu ý: msg cấu trúc phải khớp với topic. 
        // Nếu dùng TransformStamped: msg.transform.translation
        // Nếu dùng PoseStamped: msg.pose.position
        const t = msg.transform.translation; 
        setRobotPose({ x: t.x, y: t.y }); 
    };
    poseTopic.subscribe(handlePose);

    return () => poseTopic.unsubscribe();
  }, [ros]);

  // =========================================================
  // LOGIC 2: BỘ NÃO ĐIỀU KHIỂN TUẦN TRA (QUAN TRỌNG NHẤT)
  // =========================================================
  useEffect(() => {
    // Chỉ chạy khi mode active, có vị trí robot và không đang chờ nghỉ
    if (!sequenceState.active || !robotPose || sequenceState.isWaiting) return;

    // Lấy điểm đích hiện tại
    const targetWp = waypoints[sequenceState.currentIndex];
    
    // Nếu mất điểm đích (do xóa user xóa đột ngột) -> Dừng
    if (!targetWp) {
        finishSequence(); 
        return;
    }

    // Tính khoảng cách (Euclidean)
    const dx = robotPose.x - targetWp.position.x;
    const dy = robotPose.y - targetWp.position.y;
    const distance = Math.sqrt(dx*dx + dy*dy);

    // KIỂM TRA: Nếu gần đến nơi (dưới 20cm)
    if (distance < 0.2) {
        console.log(`✅ Đã đến: ${targetWp.name}. Nghỉ 1s...`);
        
        // 1. Chuyển sang trạng thái chờ
        setSequenceState(prev => ({ ...prev, isWaiting: true }));

        // 2. Đặt lịch 1 giây sau chạy tiếp
        setTimeout(() => {
            const nextIndex = sequenceState.currentIndex + 1;
            
            // Nếu còn điểm tiếp theo
            if (nextIndex < waypoints.length) {
                console.log(`🚀 Đi tiếp tới điểm #${nextIndex + 1}`);
                
                // Cập nhật trạng thái sang điểm mới
                setSequenceState({
                    active: true,
                    currentIndex: nextIndex,
                    isWaiting: false
                });
                
                // Gửi lệnh robot đi
                gotoWaypoint(waypoints[nextIndex]);
            } else {
                // Hết danh sách -> Kết thúc
                alert("🎉 Đã hoàn thành lộ trình!");
                finishSequence();
            }
        }, 1000); // 1000ms = 1 giây
    }
  }, [robotPose, sequenceState, waypoints]);


  // =========================================================
  // CÁC HÀM XỬ LÝ (ACTIONS)
  // =========================================================

  // Bắt đầu chạy tất cả
  const startSequence = () => {
      if (waypoints.length < 2) {
          alert("Cần ít nhất 2 điểm để chạy tuần tra!");
          return;
      }
      console.log("▶ Bắt đầu chạy tuần tra...");
      setSequenceState({ active: true, currentIndex: 0, isWaiting: false });
      gotoWaypoint(waypoints[0]); // Đi điểm đầu tiên
  };

  // Dừng chạy
  const finishSequence = () => {
      console.log("⏹ Dừng tuần tra.");
      setSequenceState({ active: false, currentIndex: -1, isWaiting: false });
      // Gửi lệnh cancel goal nếu cần (cần setup topic /action/cancel riêng)
  };

  // Gửi lệnh robot đi đến 1 điểm
  const gotoWaypoint = (wp) => {
      if (!ros) return;
      const currentTime = new Date();
      const secs = Math.floor(currentTime.getTime() / 1000);
      const nsecs = Math.round((currentTime.getTime() % 1000) * 1000000);

      const goalMsg = new window.ROSLIB.Message({
          header: { frame_id: 'map', stamp: { sec: secs, nanosec: nsecs } },
          pose: { position: wp.position, orientation: wp.orientation }
      });
      
      const topic = new window.ROSLIB.Topic({ 
          ros: ros, 
          name: '/goal_pose', 
          messageType: 'geometry_msgs/msg/PoseStamped' 
      });
      topic.publish(goalMsg);
  };

  // Xử lý khi thêm waypoint từ bản đồ
  const handleMapAction = (data) => {
    if (interactionMode === 'WAYPOINT' && data) {
        const name = prompt("Tên Waypoint:", `Point ${waypoints.length + 1}`);
        if (name) {
            setWaypoints(prev => [...prev, { 
                id: Date.now(), 
                name, 
                position: data.position, 
                orientation: data.orientation 
            }]);
        }
    }
    setInteractionMode('NONE');
  };

  // Style cho nút bấm chọn chế độ
  const btnStyle = (mode) => ({
    padding: '12px', width: '100%', marginBottom: '10px',
    border: 'none', borderRadius: '6px', cursor: 'pointer', 
    fontWeight: 'bold', fontSize: '0.9rem',
    backgroundColor: interactionMode === mode ? '#ffc107' : '#444',
    color: interactionMode === mode ? '#000' : '#fff',
    transition: 'all 0.2s', display: 'flex', alignItems: 'center', justifyContent: 'center', gap: '8px'
  });

  return (
      <div className="app-container">
        
        {/* --- CỘT TRÁI: MAP --- */}
        <div className="map-container">
          <MapViewer 
            interactionMode={interactionMode} 
            onActionComplete={handleMapAction} 
            waypoints={waypoints} 
            showLaser={showLaser}
          />

          <div className="floating-teleop-wrapper">
            
            {/* 1. Khung điều khiển: Chỉ hiện khi showTeleopPopup = true */}
            {showTeleopPopup && (
                <div className="teleop-widget-box">
                    <TeleopControls />
                </div>
            )}

            {/* 2. Nút Icon Toggle: Bấm để bật/tắt */}
            <button 
                className={`map-floating-btn ${showTeleopPopup ? 'active' : ''}`}
                onClick={() => setShowTeleopPopup(!showTeleopPopup)}
                title="Bật/Tắt điều khiển"
            >
                {/* Nếu đang mở thì hiện mũi tên xuống, đang đóng thì hiện gamepad */}
                {showTeleopPopup ? '🔻' : '🎮'}
            </button>
          </div>
          {/* === KẾT THÚC CỤM ĐIỀU KHIỂN NỔI === */}


          {interactionMode !== 'NONE' && (
              <div style={{
                  position: 'absolute', top: 20, left: '50%', transform: 'translateX(-50%)', 
                  background: 'rgba(0,0,0,0.8)', color: '#ffc107', padding: '8px 20px', 
                  borderRadius: '30px', zIndex: 10, border: '1px solid #ffc107', fontWeight: 'bold'
              }}>
                  {interactionMode === 'POSE' && '🎯 Click & Kéo để đặt vị trí xe'}
                  {interactionMode === 'GOAL' && '🏁 Click & Kéo để đặt đích đến'}
                  {interactionMode === 'WAYPOINT' && '📍 Click & Kéo để thêm điểm nhớ'}
              </div>
          )}
        </div>

        {/* --- CỘT PHẢI: SIDEBAR --- */}
        <div className="sidebar-container">
          <ConnectionStatus />
          <AICamera />

          {/* MỚI: MAP TOOLS - GOM HÀNG NGANG */}
          <div>
            <p style={{fontSize: '0.8rem', color: '#888', marginBottom: '8px', textTransform: 'uppercase', fontWeight:'bold'}}>Map Tools</p>
            <div className="tools-grid">
                <button className={`tool-btn ${interactionMode === 'POSE' ? 'active' : ''}`} onClick={() => setInteractionMode(interactionMode === 'POSE' ? 'NONE' : 'POSE')}>
                    <span className="tool-icon">🎯</span>
                    <span>Init Pose</span>
                </button>
                <button className={`tool-btn ${interactionMode === 'GOAL' ? 'active' : ''}`} onClick={() => setInteractionMode(interactionMode === 'GOAL' ? 'NONE' : 'GOAL')}>
                    <span className="tool-icon">🏁</span>
                    <span>Nav Goal</span>
                </button>
                <button className={`tool-btn ${interactionMode === 'WAYPOINT' ? 'active' : ''}`} onClick={() => setInteractionMode(interactionMode === 'WAYPOINT' ? 'NONE' : 'WAYPOINT')}>
                    <span className="tool-icon">📍</span>
                    <span>Add Point</span>
                </button>

                <button 
                    className={`tool-btn ${showLaser ? 'active' : ''}`} 
                    onClick={() => setShowLaser(!showLaser)}
                    style={{ border: showLaser ? '1px solid #ff4444' : '1px solid #444', color: showLaser ? '#ff4444' : '#aaa' }}
                >
                    <span className="tool-icon">🔦</span>
                    <span>Laser: {showLaser ? 'ON' : 'OFF'}</span>
                </button>
            </div>
          </div>

          {/* WAYPOINT MANAGER - CÓ THANH CUỘN */}
          <div style={{flex: 1, overflow: 'hidden', display: 'flex', flexDirection: 'column'}}>
              <WaypointManager 
                waypoints={waypoints} 
                setWaypoints={setWaypoints} 
                onGoto={gotoWaypoint} 
                onRunSequence={startSequence}     
                onStopSequence={finishSequence}   
                isRunning={sequenceState.active}  
                currentTargetId={sequenceState.active ? waypoints[sequenceState.currentIndex]?.id : null}
              />
          </div>
        </div>
      </div>
  );
}

function App() {
  return <RosProvider><MainLayout /></RosProvider>;
}

export default App;