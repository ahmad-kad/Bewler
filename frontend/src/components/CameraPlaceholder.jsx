import React, { useState, useEffect, useRef } from 'react';

/**
 * Camera Placeholder Component
 *
 * Provides a mock camera feed for testing computer vision components
 * without requiring actual camera hardware.
 *
 * Features:
 * - Test pattern generation (checkerboard, gradients, color bars)
 * - Simulated frame rate and resolution controls
 * - ArUco marker overlay simulation
 * - Object detection placeholder overlays
 * - Clear "MOCK CAMERA" labeling
 * - Performance metrics display
 */
export const CameraPlaceholder = ({
  onStartFeed,
  onStopFeed,
  isConnected,
  width = 640,
  height = 480
}) => {
  const canvasRef = useRef(null);
  const [isStreaming, setIsStreaming] = useState(false);
  const [frameRate, setFrameRate] = useState(10);
  const [testPattern, setTestPattern] = useState('checkerboard');
  const [showArucoMarkers, setShowArucoMarkers] = useState(false);
  const [showObjectDetection, setShowObjectDetection] = useState(false);
  const [performanceMetrics, setPerformanceMetrics] = useState({
    fps: 0,
    frameCount: 0,
    resolution: `${width}x${height}`,
    lastFrameTime: 0
  });

  const streamRef = useRef(null);
  const frameCountRef = useRef(0);
  const lastFrameTimeRef = useRef(0);

  // Available test patterns
  const testPatterns = {
    checkerboard: { name: 'Checkerboard', description: 'Classic test pattern for camera calibration' },
    gradient: { name: 'Gradient', description: 'Smooth color gradients for exposure testing' },
    colorbars: { name: 'Color Bars', description: 'SMPTE color bars for color accuracy' },
    noise: { name: 'Noise', description: 'Random noise for compression testing' },
    grid: { name: 'Grid', description: 'Alignment grid for distortion testing' }
  };

  // Start/stop camera feed
  const toggleFeed = () => {
    if (isStreaming) {
      stopFeed();
    } else {
      startFeed();
    }
  };

  const startFeed = () => {
    if (!isConnected) {
      alert('ROS2 connection required for camera testing');
      return;
    }

    setIsStreaming(true);
    frameCountRef.current = 0;
    lastFrameTimeRef.current = Date.now();

    // Start rendering loop
    streamRef.current = setInterval(() => {
      renderFrame();
      frameCountRef.current++;

      // Update performance metrics
      const now = Date.now();
      const timeDiff = now - lastFrameTimeRef.current;
      if (timeDiff >= 1000) { // Update FPS every second
        const fps = Math.round((frameCountRef.current * 1000) / timeDiff);
        setPerformanceMetrics(prev => ({
          ...prev,
          fps: fps,
          frameCount: frameCountRef.current,
          lastFrameTime: now
        }));
        frameCountRef.current = 0;
        lastFrameTimeRef.current = now;
      }
    }, 1000 / frameRate);

    // Notify parent component
    if (onStartFeed) {
      onStartFeed();
    }
  };

  const stopFeed = () => {
    if (streamRef.current) {
      clearInterval(streamRef.current);
      streamRef.current = null;
    }

    setIsStreaming(false);
    setPerformanceMetrics(prev => ({
      ...prev,
      fps: 0,
      frameCount: 0
    }));

    // Clear canvas
    const canvas = canvasRef.current;
    if (canvas) {
      const ctx = canvas.getContext('2d');
      ctx.fillStyle = '#000000';
      ctx.fillRect(0, 0, width, height);

      ctx.fillStyle = '#ffffff';
      ctx.font = '24px Arial';
      ctx.textAlign = 'center';
      ctx.fillText('CAMERA FEED STOPPED', width / 2, height / 2);
    }

    // Notify parent component
    if (onStopFeed) {
      onStopFeed();
    }
  };

  // Render a single frame
  const renderFrame = () => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext('2d');

    // Clear canvas
    ctx.clearRect(0, 0, width, height);

    // Render test pattern
    switch (testPattern) {
      case 'checkerboard':
        renderCheckerboard(ctx);
        break;
      case 'gradient':
        renderGradient(ctx);
        break;
      case 'colorbars':
        renderColorBars(ctx);
        break;
      case 'noise':
        renderNoise(ctx);
        break;
      case 'grid':
        renderGrid(ctx);
        break;
      default:
        renderCheckerboard(ctx);
    }

    // Add ArUco markers if enabled
    if (showArucoMarkers) {
      renderArucoMarkers(ctx);
    }

    // Add object detection overlays if enabled
    if (showObjectDetection) {
      renderObjectDetection(ctx);
    }

    // Add mock camera overlay
    renderMockOverlay(ctx);
  };

  const renderCheckerboard = (ctx) => {
    const squareSize = 40;
    for (let y = 0; y < height; y += squareSize) {
      for (let x = 0; x < width; x += squareSize) {
        const isBlack = ((x / squareSize) + (y / squareSize)) % 2 === 0;
        ctx.fillStyle = isBlack ? '#000000' : '#ffffff';
        ctx.fillRect(x, y, squareSize, squareSize);
      }
    }
  };

  const renderGradient = (ctx) => {
    // Horizontal red gradient
    const gradientRed = ctx.createLinearGradient(0, 0, width, 0);
    gradientRed.addColorStop(0, '#ff0000');
    gradientRed.addColorStop(1, '#000000');
    ctx.fillStyle = gradientRed;
    ctx.fillRect(0, 0, width, height / 3);

    // Horizontal green gradient
    const gradientGreen = ctx.createLinearGradient(0, height / 3, width, height / 3);
    gradientGreen.addColorStop(0, '#00ff00');
    gradientGreen.addColorStop(1, '#000000');
    ctx.fillStyle = gradientGreen;
    ctx.fillRect(0, height / 3, width, height / 3);

    // Horizontal blue gradient
    const gradientBlue = ctx.createLinearGradient(0, 2 * height / 3, width, 2 * height / 3);
    gradientBlue.addColorStop(0, '#0000ff');
    gradientBlue.addColorStop(1, '#000000');
    ctx.fillStyle = gradientBlue;
    ctx.fillRect(0, 2 * height / 3, width, height / 3);
  };

  const renderColorBars = (ctx) => {
    const barWidth = width / 8;
    const colors = ['#ffffff', '#ffff00', '#00ff00', '#00ffff', '#0000ff', '#ff00ff', '#ff0000', '#000000'];

    colors.forEach((color, index) => {
      ctx.fillStyle = color;
      ctx.fillRect(index * barWidth, 0, barWidth, height);
    });
  };

  const renderNoise = (ctx) => {
    const imageData = ctx.createImageData(width, height);
    const data = imageData.data;

    for (let i = 0; i < data.length; i += 4) {
      const gray = Math.floor(Math.random() * 256);
      data[i] = gray;     // Red
      data[i + 1] = gray; // Green
      data[i + 2] = gray; // Blue
      data[i + 3] = 255;  // Alpha
    }

    ctx.putImageData(imageData, 0, 0);
  };

  const renderGrid = (ctx) => {
    ctx.strokeStyle = '#00ff00';
    ctx.lineWidth = 1;

    // Vertical lines
    for (let x = 0; x < width; x += 50) {
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, height);
      ctx.stroke();
    }

    // Horizontal lines
    for (let y = 0; y < height; y += 50) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(width, y);
      ctx.stroke();
    }

    // Center cross
    ctx.strokeStyle = '#ff0000';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(width / 2, 0);
    ctx.lineTo(width / 2, height);
    ctx.moveTo(0, height / 2);
    ctx.lineTo(width, height / 2);
    ctx.stroke();
  };

  const renderArucoMarkers = (ctx) => {
    // Simulate ArUco markers with simple squares and IDs
    const markers = [
      { x: 100, y: 100, id: '42' },
      { x: 400, y: 200, id: '17' },
      { x: 200, y: 350, id: '99' }
    ];

    markers.forEach(marker => {
      // Draw marker border
      ctx.strokeStyle = '#00ff00';
      ctx.lineWidth = 3;
      ctx.strokeRect(marker.x - 25, marker.y - 25, 50, 50);

      // Draw marker ID
      ctx.fillStyle = '#00ff00';
      ctx.font = '16px Arial';
      ctx.textAlign = 'center';
      ctx.fillText(`ID: ${marker.id}`, marker.x, marker.y + 35);

      // Draw corner markers (simplified ArUco pattern)
      ctx.fillStyle = '#000000';
      ctx.fillRect(marker.x - 20, marker.y - 20, 10, 10);
      ctx.fillRect(marker.x + 10, marker.y - 20, 10, 10);
      ctx.fillRect(marker.x - 20, marker.y + 10, 10, 10);
    });
  };

  const renderObjectDetection = (ctx) => {
    // Simulate detected objects
    const objects = [
      { x: 150, y: 120, w: 80, h: 60, label: 'Rock', confidence: 0.85 },
      { x: 320, y: 280, w: 100, h: 70, label: 'Sample', confidence: 0.92 },
      { x: 500, y: 180, w: 60, h: 40, label: 'Obstacle', confidence: 0.78 }
    ];

    objects.forEach(obj => {
      // Draw bounding box
      ctx.strokeStyle = '#ff0000';
      ctx.lineWidth = 2;
      ctx.strokeRect(obj.x, obj.y, obj.w, obj.h);

      // Draw label background
      ctx.fillStyle = 'rgba(255, 0, 0, 0.8)';
      ctx.fillRect(obj.x, obj.y - 25, obj.w, 20);

      // Draw label text
      ctx.fillStyle = '#ffffff';
      ctx.font = '12px Arial';
      ctx.textAlign = 'left';
      ctx.fillText(`${obj.label} ${(obj.confidence * 100).toFixed(0)}%`, obj.x + 5, obj.y - 10);
    });
  };

  const renderMockOverlay = (ctx) => {
    // Add "MOCK CAMERA" warning overlay
    ctx.fillStyle = 'rgba(255, 255, 0, 0.8)';
    ctx.fillRect(10, 10, 200, 30);

    ctx.fillStyle = '#000000';
    ctx.font = 'bold 16px Arial';
    ctx.textAlign = 'left';
    ctx.fillText('⚠️ MOCK CAMERA FEED', 15, 30);

    // Add timestamp
    ctx.fillStyle = 'rgba(0, 0, 0, 0.7)';
    ctx.fillRect(width - 150, height - 25, 140, 20);

    ctx.fillStyle = '#ffffff';
    ctx.font = '12px Arial';
    ctx.textAlign = 'right';
    ctx.fillText(new Date().toLocaleTimeString(), width - 10, height - 10);
  };

  // Update frame rate
  useEffect(() => {
    if (isStreaming && streamRef.current) {
      stopFeed();
      startFeed(); // Restart with new frame rate
    }
  }, [frameRate]);

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      if (streamRef.current) {
        clearInterval(streamRef.current);
      }
    };
  }, []);

  return (
    <div className="bg-white rounded-lg shadow p-6">
      <div className="flex items-center justify-between mb-4">
        <div>
          <h2 className="text-xl font-semibold text-gray-900">📹 Camera Feed Test</h2>
          <p className="text-sm text-gray-600 mt-1">
            Mock camera feed for computer vision component testing
          </p>
        </div>
        <div className="flex items-center space-x-2">
          <span className={`px-3 py-1 rounded-full text-sm font-medium ${
            isConnected ? 'bg-green-100 text-green-800' : 'bg-red-100 text-red-800'
          }`}>
            {isConnected ? '🟢 ROS2 Connected' : '🔴 ROS2 Disconnected'}
          </span>
        </div>
      </div>

      {/* Mock Camera Warning */}
      <div className="bg-yellow-50 border border-yellow-200 rounded-lg p-4 mb-6">
        <div className="flex">
          <div className="flex-shrink-0">
            <span className="text-yellow-600 text-xl">⚠️</span>
          </div>
          <div className="ml-3">
            <h3 className="text-sm font-medium text-yellow-800">
              Mock Camera Feed
            </h3>
            <div className="mt-2 text-sm text-yellow-700">
              <p>
                This camera feed is <strong>GENERATED/SIMULATED</strong> for testing purposes.
                No real camera hardware is required. All image processing and computer vision
                algorithms can be tested and validated using these test patterns.
              </p>
            </div>
          </div>
        </div>
      </div>

      {/* Controls */}
      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4 mb-6">
        {/* Feed Control */}
        <div>
          <label className="block text-sm font-medium text-gray-700 mb-1">
            Camera Feed
          </label>
          <button
            onClick={toggleFeed}
            disabled={!isConnected}
            className={`w-full px-4 py-2 rounded-md text-sm font-medium ${
              isStreaming
                ? 'bg-red-600 text-white hover:bg-red-700'
                : 'bg-green-600 text-white hover:bg-green-700'
            } disabled:opacity-50 disabled:cursor-not-allowed`}
          >
            {isStreaming ? '⏹️ Stop Feed' : '▶️ Start Feed'}
          </button>
        </div>

        {/* Frame Rate */}
        <div>
          <label className="block text-sm font-medium text-gray-700 mb-1">
            Frame Rate (FPS)
          </label>
          <select
            value={frameRate}
            onChange={(e) => setFrameRate(parseInt(e.target.value))}
            className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
          >
            <option value={5}>5 FPS</option>
            <option value={10}>10 FPS</option>
            <option value={15}>15 FPS</option>
            <option value={30}>30 FPS</option>
          </select>
        </div>

        {/* Test Pattern */}
        <div>
          <label className="block text-sm font-medium text-gray-700 mb-1">
            Test Pattern
          </label>
          <select
            value={testPattern}
            onChange={(e) => setTestPattern(e.target.value)}
            className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
          >
            {Object.entries(testPatterns).map(([key, pattern]) => (
              <option key={key} value={key}>{pattern.name}</option>
            ))}
          </select>
        </div>

        {/* Pattern Description */}
        <div>
          <label className="block text-sm font-medium text-gray-700 mb-1">
            Pattern Info
          </label>
          <div className="px-3 py-2 bg-gray-50 rounded-md text-sm text-gray-600 min-h-[38px] flex items-center">
            {testPatterns[testPattern]?.description || 'Unknown pattern'}
          </div>
        </div>
      </div>

      {/* Vision Features */}
      <div className="mb-6">
        <h3 className="text-sm font-medium text-gray-700 mb-3">Computer Vision Features</h3>
        <div className="flex flex-wrap gap-4">
          <label className="flex items-center">
            <input
              type="checkbox"
              checked={showArucoMarkers}
              onChange={(e) => setShowArucoMarkers(e.target.checked)}
              className="mr-2"
            />
            <span className="text-sm text-gray-700">ArUco Markers</span>
          </label>
          <label className="flex items-center">
            <input
              type="checkbox"
              checked={showObjectDetection}
              onChange={(e) => setShowObjectDetection(e.target.checked)}
              className="mr-2"
            />
            <span className="text-sm text-gray-700">Object Detection</span>
          </label>
        </div>
      </div>

      {/* Camera Feed Display */}
      <div className="mb-6">
        <div className="relative bg-black rounded-lg overflow-hidden" style={{ width: '100%', maxWidth: '640px' }}>
          <canvas
            ref={canvasRef}
            width={width}
            height={height}
            className="w-full h-auto"
            style={{ display: 'block' }}
          />

          {!isStreaming && (
            <div className="absolute inset-0 flex items-center justify-center bg-black bg-opacity-75">
              <div className="text-center text-white">
                <div className="text-4xl mb-2">📹</div>
                <div className="text-lg font-medium">Camera Feed Stopped</div>
                <div className="text-sm opacity-75">Click "Start Feed" to begin testing</div>
              </div>
            </div>
          )}
        </div>
      </div>

      {/* Performance Metrics */}
      <div className="bg-gray-50 rounded-lg p-4">
        <h3 className="text-sm font-medium text-gray-700 mb-3">Performance Metrics</h3>
        <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-blue-600">{performanceMetrics.fps}</div>
            <div className="text-sm text-gray-600">FPS</div>
          </div>
          <div className="text-center">
            <div className="text-2xl font-bold text-green-600">{performanceMetrics.frameCount}</div>
            <div className="text-sm text-gray-600">Frames</div>
          </div>
          <div className="text-center">
            <div className="text-2xl font-bold text-purple-600">{performanceMetrics.resolution}</div>
            <div className="text-sm text-gray-600">Resolution</div>
          </div>
          <div className="text-center">
            <div className="text-2xl font-bold text-orange-600">
              {isStreaming ? 'Active' : 'Stopped'}
            </div>
            <div className="text-sm text-gray-600">Status</div>
          </div>
        </div>
      </div>

      {/* Test Actions */}
      <div className="mt-6 pt-6 border-t border-gray-200">
        <div className="flex flex-wrap gap-3">
          <button
            onClick={() => onStartFeed && onStartFeed()}
            disabled={!isConnected || isStreaming}
            className="px-4 py-2 bg-blue-600 text-white rounded-md hover:bg-blue-700 disabled:opacity-50 disabled:cursor-not-allowed"
          >
            🧪 Run Vision Tests
          </button>
          <button
            onClick={() => {
              setTestPattern('checkerboard');
              setShowArucoMarkers(true);
              setShowObjectDetection(false);
            }}
            className="px-4 py-2 bg-gray-600 text-white rounded-md hover:bg-gray-700"
          >
            🎯 Calibration Mode
          </button>
          <button
            onClick={() => {
              setTestPattern('noise');
              setShowArucoMarkers(false);
              setShowObjectDetection(true);
            }}
            className="px-4 py-2 bg-gray-600 text-white rounded-md hover:bg-gray-700"
          >
            🔍 Detection Mode
          </button>
        </div>
      </div>
    </div>
  );
};

export default CameraPlaceholder;
