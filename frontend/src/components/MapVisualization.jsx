import React, { useEffect, useRef, useState } from 'react';
import ROSLIB from 'roslib';

const MapVisualization = ({ ros }) => {
  const canvasRef = useRef(null);
  const [mapData, setMapData] = useState(null);
  const [waypoints, setWaypoints] = useState([]);
  const [robotPath, setRobotPath] = useState([]);

  // Canvas settings
  const CANVAS_WIDTH = 800;
  const CANVAS_HEIGHT = 600;
  const SCALE = 50; // pixels per meter
  const OFFSET_X = CANVAS_WIDTH / 2;
  const OFFSET_Y = CANVAS_HEIGHT / 2;

  useEffect(() => {
    if (!ros) return;

    // Subscribe to map data from SLAM bridge
    const mapTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/frontend/map_data',
      messageType: 'std_msgs/String'
    });

    const waypointTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/frontend/waypoints',
      messageType: 'std_msgs/String'
    });

    const mapCallback = (message) => {
      try {
        const data = JSON.parse(message.data);
        setMapData(data);
        if (data.path) {
          setRobotPath(data.path);
        }
      } catch (error) {
        console.error('Error parsing map data:', error);
      }
    };

    const waypointCallback = (message) => {
      try {
        const data = JSON.parse(message.data);
        setWaypoints(data.waypoints || []);
      } catch (error) {
        console.error('Error parsing waypoints:', error);
      }
    };

    mapTopic.subscribe(mapCallback);
    waypointTopic.subscribe(waypointCallback);

    // Cleanup
    return () => {
      mapTopic.unsubscribe(mapCallback);
      waypointTopic.unsubscribe(waypointCallback);
    };
  }, [ros]);

  useEffect(() => {
    drawMap();
  }, [mapData, waypoints, robotPath]);

  const worldToCanvas = (worldX, worldY) => {
    return {
      x: OFFSET_X + worldX * SCALE,
      y: OFFSET_Y - worldY * SCALE // Flip Y for standard math coordinates
    };
  };

  const drawMap = () => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, CANVAS_WIDTH, CANVAS_HEIGHT);

    // Draw grid
    drawGrid(ctx);

    // Draw robot path
    drawRobotPath(ctx);

    // Draw waypoints
    drawWaypoints(ctx);

    // Draw robot
    drawRobot(ctx);

    // Draw info panel
    drawInfoPanel(ctx);
  };

  const drawGrid = (ctx) => {
    ctx.strokeStyle = '#e0e0e0';
    ctx.lineWidth = 1;

    // Vertical lines
    for (let x = 0; x < CANVAS_WIDTH; x += SCALE) {
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, CANVAS_HEIGHT);
      ctx.stroke();
    }

    // Horizontal lines
    for (let y = 0; y < CANVAS_HEIGHT; y += SCALE) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(CANVAS_WIDTH, y);
      ctx.stroke();
    }

    // Coordinate labels
    ctx.fillStyle = '#666';
    ctx.font = '12px Arial';
    ctx.textAlign = 'center';

    for (let i = -5; i <= 5; i++) {
      const worldX = i;
      const canvasPos = worldToCanvas(worldX, 0);
      if (canvasPos.x >= 0 && canvasPos.x <= CANVAS_WIDTH) {
        ctx.fillText(worldX.toString(), canvasPos.x, OFFSET_Y + 15);
      }
    }

    for (let i = -5; i <= 5; i++) {
      const worldY = i;
      const canvasPos = worldToCanvas(0, worldY);
      if (canvasPos.y >= 0 && canvasPos.y <= CANVAS_HEIGHT) {
        ctx.fillText(worldY.toString(), OFFSET_X - 15, canvasPos.y + 4);
      }
    }
  };

  const drawRobotPath = (ctx) => {
    if (robotPath.length < 2) return;

    ctx.strokeStyle = '#4CAF50';
    ctx.lineWidth = 3;
    ctx.beginPath();

    robotPath.forEach((point, index) => {
      const canvasPos = worldToCanvas(point.x, point.y);
      if (index === 0) {
        ctx.moveTo(canvasPos.x, canvasPos.y);
      } else {
        ctx.lineTo(canvasPos.x, canvasPos.y);
      }
    });

    ctx.stroke();

    // Draw path points
    ctx.fillStyle = '#4CAF50';
    robotPath.forEach((point) => {
      const canvasPos = worldToCanvas(point.x, point.y);
      ctx.beginPath();
      ctx.arc(canvasPos.x, canvasPos.y, 2, 0, 2 * Math.PI);
      ctx.fill();
    });
  };

  const drawWaypoints = (ctx) => {
    waypoints.forEach((waypoint, index) => {
      const canvasPos = worldToCanvas(waypoint.x, waypoint.y);

      // Draw waypoint circle
      ctx.fillStyle = waypoint.reached ? '#4CAF50' : '#FF9800';
      ctx.strokeStyle = '#fff';
      ctx.lineWidth = 2;

      ctx.beginPath();
      ctx.arc(canvasPos.x, canvasPos.y, 8, 0, 2 * Math.PI);
      ctx.fill();
      ctx.stroke();

      // Draw waypoint number
      ctx.fillStyle = '#fff';
      ctx.font = '12px Arial';
      ctx.textAlign = 'center';
      ctx.fillText((index + 1).toString(), canvasPos.x, canvasPos.y + 4);
    });
  };

  const drawRobot = (ctx) => {
    if (!mapData || !mapData.robot) return;

    const robot = mapData.robot;
    const canvasPos = worldToCanvas(robot.x, robot.y);

    // Robot body (circle)
    ctx.fillStyle = '#2196F3';
    ctx.strokeStyle = '#1976D2';
    ctx.lineWidth = 3;

    ctx.beginPath();
    ctx.arc(canvasPos.x, canvasPos.y, 12, 0, 2 * Math.PI);
    ctx.fill();
    ctx.stroke();

    // Robot heading indicator (triangle)
    const headingRad = (robot.heading * Math.PI) / 180;
    const triangleLength = 15;

    ctx.fillStyle = '#1976D2';
    ctx.beginPath();
    ctx.moveTo(canvasPos.x, canvasPos.y);
    ctx.lineTo(
      canvasPos.x + triangleLength * Math.cos(headingRad - Math.PI/6),
      canvasPos.y - triangleLength * Math.sin(headingRad - Math.PI/6)
    );
    ctx.lineTo(
      canvasPos.x + triangleLength * Math.cos(headingRad + Math.PI/6),
      canvasPos.y - triangleLength * Math.sin(headingRad + Math.PI/6)
    );
    ctx.closePath();
    ctx.fill();
  };

  const drawInfoPanel = (ctx) => {
    if (!mapData) return;

    const robot = mapData.robot;
    const sensors = mapData.sensors || {};

    // Background
    ctx.fillStyle = 'rgba(255, 255, 255, 0.9)';
    ctx.fillRect(10, 10, 250, 120);

    // Border
    ctx.strokeStyle = '#ccc';
    ctx.lineWidth = 1;
    ctx.strokeRect(10, 10, 250, 120);

    // Text
    ctx.fillStyle = '#333';
    ctx.font = '12px Arial';
    ctx.textAlign = 'left';

    let y = 25;
    ctx.fillText('Robot Position:', 15, y);
    y += 15;
    if (robot) {
      ctx.fillText(`X: ${robot.x.toFixed(2)}m, Y: ${robot.y.toFixed(2)}m`, 20, y);
      y += 15;
      ctx.fillText(`Heading: ${robot.heading.toFixed(1)}°`, 20, y);
    }

    y += 20;
    ctx.fillText('Sensors:', 15, y);
    y += 15;

    if (sensors.imu && sensors.imu.accel_x !== undefined) {
      ctx.fillText(`IMU: ${sensors.imu.accel_x.toFixed(2)}, ${sensors.imu.accel_y.toFixed(2)}, ${sensors.imu.accel_z.toFixed(2)}`, 20, y);
      y += 15;
    }

    if (sensors.gps && sensors.gps.lat !== undefined) {
      ctx.fillText(`GPS: ${sensors.gps.lat.toFixed(6)}, ${sensors.gps.lon.toFixed(6)}`, 20, y);
    }
  };

  return (
    <div className="map-visualization">
      <h3>SLAM Map Visualization</h3>
      <div className="map-container">
        <canvas
          ref={canvasRef}
          width={CANVAS_WIDTH}
          height={CANVAS_HEIGHT}
          style={{
            border: '1px solid #ccc',
            backgroundColor: '#f9f9f9'
          }}
        />
      </div>
      <div className="map-legend">
        <div className="legend-item">
          <div className="legend-color" style={{ backgroundColor: '#2196F3' }}></div>
          <span>Robot Position</span>
        </div>
        <div className="legend-item">
          <div className="legend-color" style={{ backgroundColor: '#4CAF50' }}></div>
          <span>Robot Path</span>
        </div>
        <div className="legend-item">
          <div className="legend-color" style={{ backgroundColor: '#FF9800' }}></div>
          <span>Pending Waypoints</span>
        </div>
        <div className="legend-item">
          <div className="legend-color" style={{ backgroundColor: '#4CAF50' }}></div>
          <span>Completed Waypoints</span>
        </div>
      </div>

      <style jsx>{`
        .map-visualization {
          padding: 20px;
        }

        .map-container {
          margin: 20px 0;
        }

        .map-legend {
          display: flex;
          gap: 20px;
          flex-wrap: wrap;
        }

        .legend-item {
          display: flex;
          align-items: center;
          gap: 8px;
        }

        .legend-color {
          width: 16px;
          height: 16px;
          border-radius: 50%;
        }
      `}</style>
    </div>
  );
};

export default MapVisualization;
