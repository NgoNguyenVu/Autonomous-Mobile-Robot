// File: src/components/ConnectionStatus.jsx

import React from 'react';
import { useRos } from '../contexts/RosContext'; // Import hook của chúng ta

function ConnectionStatus() {
  const { isConnected } = useRos(); // Lấy trạng thái từ context

  return (
    <div className="status-container">
      <div
        className={`status-light ${
          isConnected ? 'status-connected' : 'status-disconnected'
        }`}
      ></div>
      <span>{isConnected ? 'Đã kết nối ROS' : 'Đang chờ...'}</span>
    </div>
  );
}

export default ConnectionStatus;