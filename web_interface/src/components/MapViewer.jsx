import React, { useEffect, useRef } from 'react';
import { useRos } from '../contexts/RosContext.jsx';

function MapViewer({ interactionMode, onActionComplete, waypoints = [], showLaser = false }) {
  const { ros, isConnected } = useRos();
  
  // Refs Map
  const viewerRef = useRef(null);
  const mapClientRef = useRef(null);
  const mapContainerRef = useRef(null);
  const waypointLayerRef = useRef(null);
  const pathLayerRef = useRef(null);      
  const robotLayerRef = useRef(null);      
  const interactionLayerRef = useRef(null);

  // Refs Robot 
  const robotMarkerRef = useRef(null); 
  const poseTopicRef = useRef(null);

  // Refs Drag Drop
  const dragStartRef = useRef(null);
  const isDraggingRef = useRef(false);
  const dragLineRef = useRef(null); 

  // Refs path
  const pathShapeRef = useRef(null);
  const pathTopicRef = useRef(null);

  // Refs laser 
  const laserShapeRef = useRef(null); 
  const scanTopicRef = useRef(null); 

  // ==================================================================================
  // EFFECT 1: SETUP MAP, ROBOT 
  // ==================================================================================
  useEffect(() => {
    if (!isConnected || !ros || !mapContainerRef.current) return;

    const container = mapContainerRef.current;

    // --- 1. INIT VIEWER ---
    if (!viewerRef.current) {
      viewerRef.current = new window.ROS2D.Viewer({
        divID: 'map-viewer',
        width: container.clientWidth,
        height: container.clientHeight,
        background: '#7f7f7f',
      });

        if (!pathLayerRef.current) {
            pathLayerRef.current = new window.createjs.Container();
            viewerRef.current.scene.addChild(pathLayerRef.current);
        }
        if (!robotLayerRef.current) {
            robotLayerRef.current = new window.createjs.Container();
            viewerRef.current.scene.addChild(robotLayerRef.current);
        }
        if (!interactionLayerRef.current) {
            interactionLayerRef.current = new window.createjs.Container();
            viewerRef.current.scene.addChild(interactionLayerRef.current);
        }
    }

    // --- 2. INIT MAP CLIENT ---
    if (!mapClientRef.current) {
      mapClientRef.current = new window.ROS2D.OccupancyGridClient({
        ros: ros,
        rootObject: viewerRef.current.scene,
        topic: '/map',
        continuous: true,
      });
      mapClientRef.current.on('change', () => {
        try {
           viewerRef.current.scaleToDimensions(mapClientRef.current.currentGrid.width, mapClientRef.current.currentGrid.height);
           viewerRef.current.shift(mapClientRef.current.currentGrid.pose.position.x, mapClientRef.current.currentGrid.pose.position.y);
        } catch (e) {}
        
        if (viewerRef.current) {
        const scene = viewerRef.current.scene;
        if (pathLayerRef.current) scene.setChildIndex(pathLayerRef.current, scene.numChildren - 1);
        if (robotLayerRef.current) scene.setChildIndex(robotLayerRef.current, scene.numChildren - 1);
        if (interactionLayerRef.current) scene.setChildIndex(interactionLayerRef.current, scene.numChildren - 1);
        
        scene.update(); 
        }
      });
    }

    // --- 3. INIT ROBOT MARKER  ---
    if (!robotMarkerRef.current) {
        const robotContainer = new window.createjs.Container(); 
        
        const halo = new window.createjs.Shape();
        halo.graphics
            .beginFill("rgba(0, 123, 255, 0.3)") 
            .drawCircle(0, 0, 0.5); 

        const arrow = new window.createjs.Shape();
        arrow.graphics
            .setStrokeStyle(0.03)
            .beginStroke("white")
            .beginFill("#007bff")
            .moveTo(0.35, 0)
            .lineTo(-0.2, 0.25)
            .lineTo(-0.1, 0)
            .lineTo(-0.2, -0.25)
            .closePath();
        robotContainer.addChild(halo, arrow);
        robotContainer.visible = false;

        robotContainer.scaleX = 1 / 3; 
        robotContainer.scaleY = 1 / 3;
        
        robotMarkerRef.current = robotContainer;
        if (robotLayerRef.current) {
            robotLayerRef.current.addChild(robotMarkerRef.current); 
        }
    }
    // --- 4. SUBSCRIBE ROBOT POSE  ---
    if (!poseTopicRef.current) {
        poseTopicRef.current = new window.ROSLIB.Topic({ ros: ros, name: '/web/robot_pose' });
        poseTopicRef.current.subscribe((msg) => {
            const t = msg.transform.translation;
            const r = msg.transform.rotation;
            
            const siny = 2 * (r.w * r.z + r.x * r.y);
            const cosy = 1 - 2 * (r.y * r.y + r.z * r.z);
            const theta = Math.atan2(siny, cosy) * 180.0 / Math.PI;

            if (robotMarkerRef.current) {
                robotMarkerRef.current.x = t.x;
                robotMarkerRef.current.y = -t.y;
                robotMarkerRef.current.rotation = -theta;
                if (!robotMarkerRef.current.visible) robotMarkerRef.current.visible = true;
            }

            if (laserShapeRef.current) {
                laserShapeRef.current.x = t.x;
                laserShapeRef.current.y = -t.y;
                laserShapeRef.current.rotation = -theta; 
            }

            viewerRef.current.scene.update();
        });
    }

    // --- 5. HANDLE RESIZE  ---
    const handleResize = () => {
      if (!viewerRef.current || !container) return;
      
      const canvas = viewerRef.current.scene.canvas;
      const dpr = window.devicePixelRatio || 1;
      const cssW = container.clientWidth;
      const cssH = container.clientHeight;

      canvas.style.width = cssW + 'px'; 
      canvas.style.height = cssH + 'px';
      
      canvas.width = Math.round(cssW * dpr); 
      canvas.height = Math.round(cssH * dpr);

      try {
          const ctx = canvas.getContext('2d');
          if (ctx) { 
            ctx.setTransform(dpr, 0, 0, dpr, 0, 0); 
            ctx.imageSmoothingEnabled = false; 
          }
          canvas.style.imageRendering = 'pixelated';
      } catch (err) {}

      if (mapClientRef.current?.currentGrid) {
          try { viewerRef.current.scaleToDimensions(mapClientRef.current.currentGrid.width, mapClientRef.current.currentGrid.height); } catch (e) {}
      }
    };

    const resizeObserver = new ResizeObserver(() => {
        handleResize();
    });
    if (container) resizeObserver.observe(container);
    handleResize(); 

    // --------------------------------------------------------
    // ZOOM 
    // --------------------------------------------------------
    const handleWheel = (event) => {
        if (!viewerRef.current) return;
        
        event.preventDefault(); 
        
        const stage = viewerRef.current.scene;
        const zoomFactor = 1.1; 

        if (event.deltaY < 0) {
            stage.scaleX *= zoomFactor;
            stage.scaleY *= zoomFactor;
        } else {
            stage.scaleX /= zoomFactor;
            stage.scaleY /= zoomFactor;
        }
        
        stage.update();
    };

    container.addEventListener('wheel', handleWheel, { passive: false });

    return () => {
      resizeObserver.disconnect();

      container.removeEventListener('wheel', handleWheel);

      
      if (mapClientRef.current) { try { mapClientRef.current.unsubscribe(); } catch (e) {} mapClientRef.current = null; }
      if (poseTopicRef.current) { poseTopicRef.current.unsubscribe(); poseTopicRef.current = null; }
    };

  }, [isConnected, ros]);


  // ==================================================================================
  // EFFECT: LASER SCAN
  // ==================================================================================
  useEffect(() => {
    if (!ros || !isConnected || !viewerRef.current) return;

    if (!laserShapeRef.current && robotLayerRef.current) {
        const shape = new window.createjs.Shape();
        shape.visible = showLaser;
        robotLayerRef.current.addChildAt(shape, 0); 
        laserShapeRef.current = shape;
    }

    if (!scanTopicRef.current) {
        scanTopicRef.current = new window.ROSLIB.Topic({
            ros: ros,
            name: '/scan', 
            messageType: 'sensor_msgs/msg/LaserScan'
        });

        scanTopicRef.current.subscribe((message) => {
            if (!laserShapeRef.current) return;

            const g = laserShapeRef.current.graphics;
            g.clear(); 

            g.beginFill("#FF0000"); 

            const angle_min = message.angle_min;
            const angle_increment = message.angle_increment;
            const ranges = message.ranges;
            
            for (let i = 0; i < ranges.length; i++) {
                const r = ranges[i];
                if (r < message.range_min || r > message.range_max || !isFinite(r)) continue;

                const angle = angle_min + (i * angle_increment);

                const x = r * Math.cos(angle);
                const y = r * Math.sin(angle);

                g.drawRect(x, -y, 0.03, 0.03); 
            }
            
            g.endFill();
            viewerRef.current.scene.update();
        });
    }

    return () => {
        if (scanTopicRef.current) {
            scanTopicRef.current.unsubscribe();
            scanTopicRef.current = null;
        }
    };
  }, [ros, isConnected]);

  // ==================================================================================
  // EFFECT 2: PATH 
  // ==================================================================================
  useEffect(() => {
    if (!ros || !isConnected || !viewerRef.current) return;

    if (!pathShapeRef.current && pathLayerRef.current) {
        pathShapeRef.current = new window.createjs.Shape();
        pathLayerRef.current.addChild(pathShapeRef.current); 
    }

    if (!pathTopicRef.current) {
        pathTopicRef.current = new window.ROSLIB.Topic({
            ros: ros,
            name: '/plan', 
            messageType: 'nav_msgs/msg/Path'
        });

        pathTopicRef.current.subscribe((message) => {
            if (!pathShapeRef.current) return;
            
            const g = pathShapeRef.current.graphics;
            g.clear(); 

            if (message.poses && message.poses.length > 0) {
                g.setStrokeStyle(0.05, "round", "round").beginStroke("#00FF00");
                g.moveTo(message.poses[0].pose.position.x, -message.poses[0].pose.position.y);

                for (let i = 1; i < message.poses.length; i++) {
                    g.lineTo(message.poses[i].pose.position.x, -message.poses[i].pose.position.y);
                }
                
                g.endStroke();

                viewerRef.current.scene.update();
            }
        });
    }

    return () => {
        if (pathTopicRef.current) {
            pathTopicRef.current.unsubscribe();
            pathTopicRef.current = null;
        }
    };
  }, [ros, isConnected]);


  // ==================================================================================
  // EFFECT 3: WAYPOINTS 
  // ==================================================================================
  useEffect(() => {
    if (!viewerRef.current) return;

    if (!waypointLayerRef.current) {
        waypointLayerRef.current = new window.createjs.Container();
        viewerRef.current.scene.addChild(waypointLayerRef.current);
    }
    const layer = waypointLayerRef.current;
    
    layer.removeAllChildren();

    waypoints.forEach((wp) => {
        const marker = new window.createjs.Shape();
        marker.graphics
            .beginStroke("#FFF").setStrokeStyle(0.02) 
            .beginFill("#FFA500") 
            .drawCircle(0, 0, 0.1); 
        
        marker.x = wp.position.x;
        marker.y = -wp.position.y; 

        const label = new window.createjs.Text(wp.name, "bold 0.2px Arial", "#FFFFFF");
        label.textAlign = "center";
        label.textBaseline = "bottom";
        label.x = wp.position.x;
        label.y = -wp.position.y - 0.15; 
        label.shadow = new window.createjs.Shadow("#000000", 1, 1, 0);

        layer.addChild(marker, label);
    });

    viewerRef.current.scene.update();

  }, [waypoints, viewerRef.current]); 



  // ==================================================================================
  // EFFECT: ON/OFF LASER
  // ==================================================================================
  useEffect(() => {
    if (laserShapeRef.current && viewerRef.current) {
        laserShapeRef.current.visible = showLaser;
        
        viewerRef.current.scene.update();
    }
  }, [showLaser]);


  // ==================================================================================
  // EFFECT 4: LOGIC DRAG DROP
  // ==================================================================================
  useEffect(() => {
    if (!mapContainerRef.current) return;
    const container = mapContainerRef.current;

    if (!dragLineRef.current && interactionLayerRef.current) {
        dragLineRef.current = new window.createjs.Shape();
        interactionLayerRef.current.addChild(dragLineRef.current); 
    }

    const handleMouseDown = (event) => {
        if (interactionMode === 'NONE' || !viewerRef.current) return;
        event.preventDefault();

        const rect = container.getBoundingClientRect();
        const coords = viewerRef.current.scene.globalToLocal(event.clientX - rect.left, event.clientY - rect.top);
        
        dragStartRef.current = coords;
        isDraggingRef.current = true;
    };

    const handleMouseMove = (event) => {
        if (!isDraggingRef.current || !dragStartRef.current || !viewerRef.current) return;

        const rect = container.getBoundingClientRect();
        const currentCoords = viewerRef.current.scene.globalToLocal(event.clientX - rect.left, event.clientY - rect.top);
        
        const startX = dragStartRef.current.x;
        const startY = dragStartRef.current.y;
        const endX = currentCoords.x;
        const endY = currentCoords.y;

        const g = dragLineRef.current.graphics;
        g.clear();
        
        const scale = viewerRef.current.scene.scaleX; 
        const strokeSize = 6 / scale; 
        const arrowSize = 10 / scale;

        const color = interactionMode === 'POSE' ? '#00FF00' : (interactionMode === 'GOAL' ? '#FF00FF' : '#FFA500');

        g.setStrokeStyle(strokeSize, "round", "round").beginStroke(color);
        g.moveTo(startX, startY);
        g.lineTo(endX, endY);
        
        const angle = Math.atan2(endY - startY, endX - startX);
        g.lineTo(endX - arrowSize * Math.cos(angle - Math.PI/6), endY - arrowSize * Math.sin(angle - Math.PI/6));
        g.moveTo(endX, endY);
        g.lineTo(endX - arrowSize * Math.cos(angle + Math.PI/6), endY - arrowSize * Math.sin(angle + Math.PI/6));
        g.endStroke();
        
        g.beginFill(color).drawCircle(startX, startY, strokeSize * 2);

        viewerRef.current.scene.update();
    };

    const handleMouseUp = (event) => {
        if (!isDraggingRef.current || !dragStartRef.current) return;

        const rect = container.getBoundingClientRect();
        const endCoords = viewerRef.current.scene.globalToLocal(event.clientX - rect.left, event.clientY - rect.top);

        const dx = endCoords.x - dragStartRef.current.x;
        const dy = -(endCoords.y - dragStartRef.current.y); 
        let angleRad = Math.atan2(dy, dx);
        
        if (Math.sqrt(dx*dx + dy*dy) < 0.1) angleRad = 0.0;

        const qz = Math.sin(angleRad / 2);
        const qw = Math.cos(angleRad / 2);

    const position = { x: dragStartRef.current.x, y: -dragStartRef.current.y, z: 0.0 };
        const orientation = { x: 0, y: 0, z: qz, w: qw };

        if (interactionMode === 'POSE') {
            // ---  Initial Pose  ---
            const poseMsg = new window.ROSLIB.Message({
                header: { frame_id: 'map' },
                pose: { 
                    pose: { position, orientation },
                    covariance: [0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.06]
                }
            });
            const topic = new window.ROSLIB.Topic({ ros: ros, name: '/initialpose', messageType: 'geometry_msgs/msg/PoseWithCovarianceStamped' });
            topic.publish(poseMsg);
            console.log("Published Initial Pose");
        } 
        else if (interactionMode === 'GOAL') {
            // ---  Nav Goal  ---
            const goalMsg = new window.ROSLIB.Message({
                header: { frame_id: 'map' },
                pose: { position, orientation }
            });
            const topic = new window.ROSLIB.Topic({ ros: ros, name: '/goal_pose', messageType: 'geometry_msgs/msg/PoseStamped' });
            topic.publish(goalMsg);
            console.log("Published Nav Goal");
        }
        else if (interactionMode === 'WAYPOINT') {
            // ---  Waypoint  ---
            if (onActionComplete) {
                onActionComplete({ position, orientation });
            }
        }

        isDraggingRef.current = false;
        dragStartRef.current = null;
        if (dragLineRef.current) dragLineRef.current.graphics.clear();
        
        if (interactionMode !== 'WAYPOINT' && onActionComplete) onActionComplete(null);

        viewerRef.current.scene.update();
    };

    container.addEventListener('mousedown', handleMouseDown);
    window.addEventListener('mousemove', handleMouseMove);
    window.addEventListener('mouseup', handleMouseUp);

    return () => {
        container.removeEventListener('mousedown', handleMouseDown);
        window.removeEventListener('mousemove', handleMouseMove);
        window.removeEventListener('mouseup', handleMouseUp);
        if (dragLineRef.current) dragLineRef.current.graphics.clear();
    };

  }, [isConnected, ros, interactionMode, onActionComplete]);

  return (
    <div style={{ position: 'relative', width: '100%', height: '100%', overflow: 'hidden', backgroundColor: '#7f7f7f' }}>
      <div id="map-viewer" ref={mapContainerRef} style={{ width: '100%', height: '100%', cursor: interactionMode !== 'NONE' ? 'crosshair' : 'default'  }} />
    </div>
  );
}

export default MapViewer;