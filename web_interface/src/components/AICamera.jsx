// src/components/AICamera.jsx
import React, { useState } from 'react';

const AICamera = () => {
    // --- CẤU HÌNH ---
    const SERVER_HOST = "localhost"; 
    const CAM_URL = `http://${SERVER_HOST}:5000/video_feed`; 
    const API_URL = `http://${SERVER_HOST}:5000/set_follow_mode`;

    const [imgError, setImgError] = useState(false);
    const [isFollowing, setIsFollowing] = useState(false);
    const [isLoading, setIsLoading] = useState(false);

    // Hàm gọi API bật/tắt Follow
    const toggleFollow = async () => {
        if (isLoading) return; // Tránh spam click
        setIsLoading(true);
        
        const newMode = !isFollowing;
        try {
            const response = await fetch(API_URL, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ active: newMode }),
            });

            const data = await response.json();
            if (response.ok) {
                setIsFollowing(data.mode);
            }
        } catch (error) {
            console.error("Lỗi:", error);
            // alert("Mất kết nối tới Server Laptop!");
        } finally {
            setIsLoading(false);
        }
    };

    return (
        <div className="ai-camera-panel" style={{ 
            background: '#1a1a1a', 
            padding: '10px', 
            borderRadius: '8px',
            border: '1px solid #333',
            boxShadow: '0 4px 12px rgba(0,0,0,0.5)'
        }}>
            {/* --- STYLE CHO NÚT GẠT (TOGGLE) --- */}
            <style>{`
                .toggle-switch {
                    position: relative;
                    width: 140px;
                    height: 32px;
                    background-color: #333;
                    border-radius: 20px;
                    cursor: pointer;
                    display: flex;
                    align-items: center;
                    padding: 2px;
                    transition: all 0.3s ease;
                    border: 1px solid #444;
                }
                .toggle-switch.active {
                    background-color: #2e7d32; /* Màu xanh khi Follow */
                    border-color: #4caf50;
                }
                .toggle-knob {
                    position: absolute;
                    left: 2px;
                    width: 68px;
                    height: 26px;
                    background-color: #fff;
                    border-radius: 16px;
                    transition: all 0.3s cubic-bezier(0.68, -0.55, 0.27, 1.55);
                    box-shadow: 0 2px 5px rgba(0,0,0,0.3);
                    display: flex;
                    align-items: center;
                    justify-content: center;
                    font-size: 11px;
                    font-weight: bold;
                    color: #000;
                    text-transform: uppercase;
                }
                .toggle-switch.active .toggle-knob {
                    left: calc(100% - 70px); /* Đẩy nút sang phải */
                    background-color: #fff;
                    color: #2e7d32;
                }
                .toggle-labels {
                    width: 100%;
                    display: flex;
                    justify-content: space-between;
                    padding: 0 15px;
                    font-size: 10px;
                    font-weight: bold;
                    color: #aaa;
                    pointer-events: none;
                    text-transform: uppercase;
                }
            `}</style>

            {/* --- HEADER GỌN GÀNG --- */}
            <div className="ai-camera-header" style={{ 
                display: 'flex', 
                justifyContent: 'space-between', 
                alignItems: 'center', 
                marginBottom: '10px',
                height: '40px'
            }}>
                {/* 1. Title */}
                <div style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
                    <span style={{ fontSize: '18px' }}>👁️</span>
                    <span style={{ fontWeight: 'bold', color: '#00f2ff', fontFamily: 'monospace', fontSize: '16px' }}>
                        VISION VIEW
                    </span>
                </div>

                {/* 2. Nút gạt DETECT <-> FOLLOW (Chỉ hiện khi Camera online) */}
                {!imgError && (
                    <div 
                        className={`toggle-switch ${isFollowing ? 'active' : ''}`} 
                        onClick={toggleFollow}
                    >
                        <div className="toggle-labels">
                            <span>Detect</span>
                            <span>Follow</span>
                        </div>
                        <div className="toggle-knob">
                            {isLoading ? "..." : (isFollowing ? "ON" : "OFF")}
                        </div>
                    </div>
                )}

                {/* 3. Trạng thái Live */}
                <div style={{ 
                    fontSize: '11px', 
                    fontWeight: 'bold',
                    color: imgError ? '#ff4d4f' : '#52c41a',
                    border: `1px solid ${imgError ? '#ff4d4f' : '#52c41a'}`,
                    padding: '2px 8px',
                    borderRadius: '4px',
                    minWidth: '60px',
                    textAlign: 'center'
                }}>
                    {imgError ? 'OFFLINE' : '● LIVE'}
                </div>
            </div>
            
            {/* --- VIDEO CONTENT --- */}
            <div className="ai-camera-content" style={{ 
                position: 'relative', 
                minHeight: '240px', 
                backgroundColor: '#000',
                borderRadius: '6px',
                overflow: 'hidden'
            }}>
                {!imgError ? (
                    <img 
                        src={CAM_URL} 
                        alt="AI Stream" 
                        onError={() => setImgError(true)}
                        style={{ width: '100%', height: '100%', objectFit: 'cover', display: 'block' }}
                    />
                ) : (
                    <div style={{ 
                        display: 'flex', flexDirection: 'column', alignItems: 'center', justifyContent: 'center', 
                        height: '240px', color: '#666', gap: '10px'
                    }}>
                        <p>⚠️ Mất tín hiệu Camera</p>
                        <button 
                            onClick={() => setImgError(false)} 
                            style={{ 
                                cursor: 'pointer', padding: '6px 16px', borderRadius: '4px',
                                background: '#333', color: 'white', border: '1px solid #555' 
                            }}
                        >
                            Thử lại
                        </button>
                    </div>
                )}
            </div>
        </div>
    );
};

export default AICamera;