// File: src/contexts/RosContext.jsx

import React, {
  createContext,
  useContext,
  useEffect,
  useRef,
  useState,
} from 'react';

// --- CẤU HÌNH ---
// !! THAY ĐỔI IP NÀY !!
const ROSBRIDGE_URL = 'ws://192.168.1.21:9090'; // Thay bằng IP của NUC

// Tạo Context
const RosContext = createContext();

// Tạo Provider Component (một component "bọc" ngoài)
export function RosProvider({ children }) {
  const [isConnected, setIsConnected] = useState(false);
  // Dùng useRef để đối tượng ros không bị tạo lại mỗi lần re-render
  const ros = useRef(new ROSLIB.Ros());

  useEffect(() => {
    // Chỉ chạy 1 lần khi mount
    const rosInstance = ros.current;

    rosInstance.on('connection', () => {
      console.log('Đã kết nối tới ROSBridge!');
      setIsConnected(true);
    });

    rosInstance.on('error', (error) => {
      console.error('Lỗi kết nối ROSBridge: ', error);
      setIsConnected(false);
    });

    rosInstance.on('close', () => {
      console.log('Đã ngắt kết nối khỏi ROSBridge.');
      setIsConnected(false);
      // Tự động kết nối lại sau 3 giây
      setTimeout(() => {
        try {
          rosInstance.connect(ROSBRIDGE_URL);
        } catch (e) {
          console.error('Lỗi khi kết nối lại: ', e);
        }
      }, 3000);
    });

    // Thử kết nối
    try {
      rosInstance.connect(ROSBRIDGE_URL);
    } catch (e) {
      console.error('Lỗi kết nối ban đầu: ', e);
    }

    // Hàm dọn dẹp: ngắt kết nối khi component bị unmount
    return () => {
      if (rosInstance && rosInstance.isConnected) {
        rosInstance.close();
      }
    };
  }, []); // [] đảm bảo useEffect chỉ chạy 1 lần

  // Cung cấp "value" cho các component con
  const value = {
    ros: ros.current, // Đối tượng ros
    isConnected: isConnected, // Trạng thái
  };

  return <RosContext.Provider value={value}>{children}</RosContext.Provider>;
}

// Tạo một "hook" tùy chỉnh để dễ dàng truy cập context
export function useRos() {
  return useContext(RosContext);
}