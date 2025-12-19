import React, { useState, useEffect } from 'react'; // <--- 1. Thêm useState, useEffect

const WaypointManager = ({ waypoints, setWaypoints, onGoto, onRunSequence, onStopSequence, isRunning, currentTargetId }) => {

    // <--- 2. State để nhớ điểm nào đang chạy thủ công
    const [manualActiveId, setManualActiveId] = useState(null);

    // Nếu bắt đầu chạy tuần tra (Run All), thì reset cái chạy thủ công đi cho đỡ rối
    useEffect(() => {
        if (isRunning) {
            setManualActiveId(null);
        }
    }, [isRunning]);

    const handleManualGoto = (wp) => {
        setManualActiveId(wp.id); // Đánh dấu điểm này đang active
        onGoto(wp); // Gọi hàm gửi lệnh gốc
    };

    return (
        <div style={{marginTop: '10px', background: '#2c313a', padding: '10px', borderRadius: '8px', display:'flex', flexDirection:'column', flex:1, overflow:'hidden'}}>
            
            {/* Header */}
            <div style={{display: 'flex', justifyContent: 'space-between', alignItems: 'center', borderBottom: '1px solid #444', paddingBottom: '10px', marginBottom: '10px'}}>
                <h3 style={{fontSize: '0.9rem', margin: 0, color: '#eee'}}>📍 Waypoints ({waypoints.length})</h3>
                <div>
                    {!isRunning ? (
                        <button onClick={onRunSequence} disabled={waypoints.length < 2}
                            style={{
                                background: waypoints.length < 2 ? '#444' : '#007bff', 
                                border: 'none', color: '#fff', padding: '5px 10px', borderRadius: '4px', 
                                cursor: waypoints.length < 2 ? 'not-allowed' : 'pointer', fontSize: '0.75rem', fontWeight: 'bold'
                            }}>
                            ▶ Run All
                        </button>
                    ) : (
                        <button onClick={onStopSequence}
                            style={{
                                background: '#dc3545', border: 'none', color: '#fff', 
                                padding: '5px 10px', borderRadius: '4px', cursor: 'pointer', fontSize: '0.75rem', fontWeight: 'bold'
                            }}>
                            <span className="blink">●</span> STOP
                        </button>
                    )}
                </div>
            </div>

            {/* List */}
            <div style={{flex: 1, overflow: 'hidden', position: 'relative'}}>
                 {waypoints.length === 0 ? (
                    <p style={{fontSize: '0.8rem', color: '#888', textAlign: 'center', marginTop: '20px'}}>Chưa có điểm nào.</p>
                 ) : (
                    <ul className="wp-list-container" style={{listStyle: 'none', padding: 0, margin: 0, height: '100%'}}>
                        {waypoints.map((wp, index) => {
                            // Logic kiểm tra Active
                            const isSequenceActive = isRunning && wp.id === currentTargetId; // Đang chạy tuần tra
                            const isManualActive = manualActiveId === wp.id; // Đang chạy thủ công (vừa bấm nút)
                            
                            // Nếu 1 trong 2 cái trên true thì dòng đó sáng lên
                            const isRowActive = isSequenceActive || isManualActive;

                            return (
                                <li key={wp.id} style={{
                                    display: 'flex', justifyContent: 'space-between', marginBottom: '6px', 
                                    background: isRowActive ? 'rgba(230, 126, 34, 0.2)' : '#222', // Đổi màu nền nếu active
                                    border: isRowActive ? '1px solid #e67e22' : '1px solid #333', // Đổi viền nếu active
                                    padding: '8px', borderRadius: '4px', alignItems: 'center',
                                    transition: 'all 0.2s'
                                }}>
                                    <div style={{display: 'flex', alignItems: 'center', flex: 1, overflow:'hidden'}}>
                                        <span style={{fontSize: '0.7rem', marginRight: '8px', color: '#666'}}>#{index + 1}</span>
                                        <div style={{display: 'flex', flexDirection: 'column', overflow:'hidden'}}>
                                            <span style={{
                                                fontSize: '0.85rem', 
                                                fontWeight: isRowActive ? 'bold' : 'normal', 
                                                color: isRowActive ? '#e67e22' : '#ddd', 
                                                whiteSpace:'nowrap', overflow:'hidden', textOverflow:'ellipsis'
                                            }}>
                                                {wp.name}
                                            </span>
                                            {isRowActive && <span style={{fontSize: '0.65rem', color: '#e67e22'}} className="blink">Moving...</span>}
                                        </div>
                                    </div>

                                    {!isRunning && (
                                        <div style={{display: 'flex', gap: '4px'}}>
                                            {/* 3. NÚT PLAY ĐÃ ĐƯỢC NÂNG CẤP 
                                                - Nếu đang active: Màu cam, icon xoay (Loading)
                                                - Nếu bình thường: Màu xanh, icon Play
                                            */}
                                            <button 
                                                onClick={() => handleManualGoto(wp)} 
                                                style={{
                                                    background: isManualActive ? '#e67e22' : '#28a745', // Cam hoặc Xanh
                                                    border: 'none', color: '#fff', 
                                                    width: '24px', height: '24px', borderRadius: '4px', cursor: 'pointer',
                                                    display: 'flex', alignItems: 'center', justifyContent: 'center',
                                                    transition: 'background 0.2s'
                                                }}
                                                title={isManualActive ? "Đang di chuyển..." : "Đi tới điểm này"}
                                            >
                                                {isManualActive ? <span className="spin">↻</span> : '▶'}
                                            </button>

                                            <button onClick={() => setWaypoints(prev => prev.filter(p => p.id !== wp.id))} style={{background: '#dc3545', border: 'none', color: '#fff', width: '24px', height: '24px', borderRadius: '4px', cursor: 'pointer'}}>✕</button>
                                        </div>
                                    )}
                                </li>
                            );
                        })}
                    </ul>
                 )}
            </div>
             {/* Thêm animation xoay tròn cho icon loading */}
             <style>{`
                .blink { animation: blinker 1s linear infinite; } 
                @keyframes blinker { 50% { opacity: 0.5; } }
                .spin { display: inline-block; animation: spinner 1s linear infinite; font-weight: bold; }
                @keyframes spinner { 100% { transform: rotate(360deg); } }
            `}</style>
        </div>
    );
};

export default WaypointManager;