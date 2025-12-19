import React, { useState, useEffect, useRef } from 'react';
import { useRos } from '../contexts/RosContext';

const ROSLIB = typeof window !== 'undefined' ? window.ROSLIB : null;

const MAX_LINEAR_SPEED = 0.5;
const MAX_ANGULAR_SPEED = 1.0; 
const MAX_STRAFE_SPEED = 0.3;

function TeleopControls() {
  const { ros, isConnected } = useRos();
  
  const [linearSpeed, setLinearSpeed] = useState({ x: 0, y: 0 });
  const [angularSpeed, setAngularSpeed] = useState(0);
  const [activeBtn, setActiveBtn] = useState(null); 

  const cmdVelTopic = useRef(null);
  const publishInterval = useRef(null);

  useEffect(() => {
    if (isConnected && ros) {
      cmdVelTopic.current = new ROSLIB.Topic({
        ros: ros,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/Twist',
      });
    }
  }, [isConnected, ros]);

  useEffect(() => {
    if (!isConnected || !cmdVelTopic.current) return;
    if (publishInterval.current) {
        clearInterval(publishInterval.current);
        publishInterval.current = null;
    }

    const twist = new ROSLIB.Message({
      linear: { x: linearSpeed.x, y: linearSpeed.y, z: 0.0 },
      angular: { x: 0.0, y: 0.0, z: angularSpeed },
    });

    if (linearSpeed.x !== 0 || linearSpeed.y !== 0 || angularSpeed !== 0) {
      publishInterval.current = setInterval(() => {
        cmdVelTopic.current.publish(twist);
      }, 100); 
    } else {
      cmdVelTopic.current.publish(twist);
    }
    
    return () => {
        if (publishInterval.current) {
            clearInterval(publishInterval.current);
            publishInterval.current = null;
        }
    };
  }, [isConnected, linearSpeed, angularSpeed]);

  const handleButtonMove = (linearX, linearY, angularZ, btnId) => {
    setLinearSpeed({
      x: linearX * MAX_LINEAR_SPEED,
      y: linearY * MAX_STRAFE_SPEED,
    });
    setAngularSpeed(angularZ * MAX_ANGULAR_SPEED);
    setActiveBtn(btnId);
  };

  const handleButtonStop = () => {
    setLinearSpeed({ x: 0, y: 0 });
    setAngularSpeed(0);
    setActiveBtn(null);
  };

  const getButtonProps = (x, y, z, id) => ({
    onMouseDown: () => handleButtonMove(x, y, z, id),
    onMouseUp: handleButtonStop,
    onMouseLeave: handleButtonStop,
    onTouchStart: (e) => { e.preventDefault(); handleButtonMove(x, y, z, id); },
    onTouchEnd: handleButtonStop,
    className: `control-btn ${id === 'stop' ? 'stop-btn' : ''} ${activeBtn === id ? 'active' : ''}`
  });

  if (!isConnected) return <div style={{color:'#aaa'}}>Connecting...</div>;

  return (
    <div className="teleop-wrapper">
      <div className="teleop-grid">
        {/* HÀNG 1 */}
        <button {...getButtonProps(1, 1, 0, 'fl')}>↖</button>
        <button {...getButtonProps(1, 0, 0, 'fwd')}>↑</button>
        <button {...getButtonProps(1, -1, 0, 'fr')}>↗</button>

        {/* HÀNG 2 */}
        <button {...getButtonProps(0, 0, 1, 'rot_l')}>↺</button>
        <button {...getButtonProps(0, 0, 0, 'stop')} onClick={handleButtonStop}>STOP</button>
        <button {...getButtonProps(0, 0, -1, 'rot_r')}>↻</button>

        {/* HÀNG 3 */}
        <button {...getButtonProps(0, 1, 0, 'left')}>←</button>
        <button {...getButtonProps(-1, 0, 0, 'back')}>↓</button>
        <button {...getButtonProps(0, -1, 0, 'right')}>→</button>
      </div>
    </div>
  );
}

export default TeleopControls;