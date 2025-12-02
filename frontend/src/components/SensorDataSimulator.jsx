import React, { useState, useEffect } from 'react';

/**
 * Sensor Data Simulator Component
 *
 * Provides controls for simulating CAN bus sensor data without real hardware.
 * Allows testing of autonomy systems with realistic, controllable sensor inputs.
 *
 * Features:
 * - IMU sensor controls (accelerometer, gyroscope, temperature)
 * - GPS position and heading controls
 * - Battery monitoring simulation
 * - Motor encoder feedback
 * - Environmental sensors (temperature, pressure, humidity)
 * - Real-time data publishing to WebSocket bridge
 * - Clear mock data labeling
 */
export const SensorDataSimulator = ({
  onDataChange,
  isConnected,
  websocketUrl = 'ws://localhost:8765'
}) => {
  const [sensorData, setSensorData] = useState({
    // IMU Data
    imu: {
      accel_x: 0.0,
      accel_y: 0.0,
      accel_z: 9.81,
      gyro_x: 0.0,
      gyro_y: 0.0,
      gyro_z: 0.0,
      temperature: 25.0
    },

    // GPS Data
    gps: {
      latitude: 38.406,
      longitude: -110.792,
      altitude: 1500.0,
      heading: 0.0,
      speed: 0.0,
      satellites: 12,
      hdop: 0.8
    },

    // Battery Data
    battery: {
      voltage: 24.0,
      current: 5.0,
      charge_level: 85.0,
      temperature: 30.0
    },

    // Motor Data
    motor_left: {
      position: 0.0,
      velocity: 0.0,
      current: 0.0,
      temperature: 25.0
    },

    motor_right: {
      position: 0.0,
      velocity: 0.0,
      current: 0.0,
      temperature: 25.0
    },

    // Environmental Data
    environment: {
      cpu_temp: 45.0,
      ambient_temp: 22.0,
      humidity: 45.0,
      pressure: 1013.25
    }
  });

  const [autoUpdate, setAutoUpdate] = useState(false);
  const [updateInterval, setUpdateInterval] = useState(1000); // 1 second
  const [lastUpdate, setLastUpdate] = useState(null);
  const [connectionStatus, setConnectionStatus] = useState('disconnected');

  // WebSocket connection for sending mock data
  const [wsConnection, setWsConnection] = useState(null);

  // Initialize WebSocket connection
  useEffect(() => {
    if (isConnected) {
      try {
        const ws = new WebSocket(websocketUrl);
        ws.onopen = () => {
          setConnectionStatus('connected');
          setWsConnection(ws);
        };
        ws.onclose = () => {
          setConnectionStatus('disconnected');
          setWsConnection(null);
        };
        ws.onerror = () => {
          setConnectionStatus('error');
        };
      } catch (error) {
        setConnectionStatus('error');
      }
    }

    return () => {
      if (wsConnection) {
        wsConnection.close();
      }
    };
  }, [isConnected, websocketUrl]);

  // Auto-update simulation
  useEffect(() => {
    if (!autoUpdate) return;

    const interval = setInterval(() => {
      setSensorData(prevData => {
        const newData = { ...prevData };

        // Simulate realistic sensor variations
        // IMU noise
        newData.imu.accel_x += (Math.random() - 0.5) * 0.02;
        newData.imu.accel_y += (Math.random() - 0.5) * 0.02;
        newData.imu.accel_z = 9.81 + (Math.random() - 0.5) * 0.1;
        newData.imu.gyro_x += (Math.random() - 0.5) * 0.002;
        newData.imu.gyro_y += (Math.random() - 0.5) * 0.002;
        newData.imu.gyro_z += (Math.random() - 0.5) * 0.002;

        // GPS movement (very slow)
        newData.gps.latitude += (Math.random() - 0.5) * 0.00001;
        newData.gps.longitude += (Math.random() - 0.5) * 0.00001;
        newData.gps.heading = (newData.gps.heading + (Math.random() - 0.5) * 2) % 360;

        // Battery discharge
        if (newData.battery.current > 0) {
          newData.battery.charge_level = Math.max(0, newData.battery.charge_level - 0.01);
          newData.battery.voltage = 25.2 - (100 - newData.battery.charge_level) * 0.01;
        }

        // Temperature variations
        Object.keys(newData).forEach(sensor => {
          if (newData[sensor].temperature !== undefined) {
            newData[sensor].temperature += (Math.random() - 0.5) * 0.1;
          }
        });

        return newData;
      });
    }, updateInterval);

    return () => clearInterval(interval);
  }, [autoUpdate, updateInterval]);

  // Notify parent of data changes
  useEffect(() => {
    if (onDataChange) {
      onDataChange(sensorData);
    }
  }, [sensorData, onDataChange]);

  // Send data to WebSocket
  const sendSensorData = () => {
    if (!wsConnection || wsConnection.readyState !== WebSocket.OPEN) {
      alert('WebSocket connection not available');
      return;
    }

    const message = {
      type: 'can_mock_data',
      sensors: sensorData,
      timestamp: Date.now(),
      mock: true,
      source: 'sensor_data_simulator',
      warning: 'NOT REAL CAN BUS DATA - SIMULATED FOR TESTING'
    };

    wsConnection.send(JSON.stringify(message));
    setLastUpdate(new Date());
  };

  // Update individual sensor values
  const updateSensorValue = (sensor, field, value) => {
    setSensorData(prev => ({
      ...prev,
      [sensor]: {
        ...prev[sensor],
        [field]: parseFloat(value) || 0
      }
    }));
  };

  // Preset configurations
  const loadPreset = (preset) => {
    const presets = {
      idle: {
        imu: { accel_x: 0, accel_y: 0, accel_z: 9.81, gyro_x: 0, gyro_y: 0, gyro_z: 0, temperature: 25 },
        gps: { latitude: 38.406, longitude: -110.792, altitude: 1500, heading: 0, speed: 0 },
        battery: { voltage: 24.0, current: 0.0, charge_level: 100, temperature: 25 },
        motor_left: { position: 0, velocity: 0, current: 0, temperature: 25 },
        motor_right: { position: 0, velocity: 0, current: 0, temperature: 25 }
      },
      moving: {
        imu: { accel_x: 0.1, accel_y: 0, accel_z: 9.8, gyro_x: 0, gyro_y: 0, gyro_z: 0.05, temperature: 28 },
        gps: { latitude: 38.406, longitude: -110.792, altitude: 1500, heading: 45, speed: 1.5 },
        battery: { voltage: 23.8, current: 8.0, charge_level: 85, temperature: 32 },
        motor_left: { position: 10.5, velocity: 1.5, current: 2.1, temperature: 28 },
        motor_right: { position: 10.5, velocity: 1.5, current: 2.1, temperature: 28 }
      },
      calibration: {
        imu: { accel_x: 0, accel_y: 0, accel_z: 9.81, gyro_x: 0, gyro_y: 0, gyro_z: 0, temperature: 25 },
        gps: { latitude: 38.406, longitude: -110.792, altitude: 1500, heading: 0, speed: 0 },
        battery: { voltage: 24.0, current: 0.0, charge_level: 95, temperature: 26 },
        motor_left: { position: 0, velocity: 0, current: 0, temperature: 25 },
        motor_right: { position: 0, velocity: 0, current: 0, temperature: 25 }
      }
    };

    if (presets[preset]) {
      setSensorData(presets[preset]);
    }
  };

  return (
    <div className="space-y-6">
      {/* Header */}
      <div className="bg-white rounded-lg shadow p-6">
        <div className="flex items-center justify-between mb-4">
          <div>
            <h2 className="text-xl font-semibold text-gray-900">🔌 CAN Bus Sensor Simulator</h2>
            <p className="text-sm text-gray-600 mt-1">
              Control mock sensor data for testing autonomy systems
            </p>
          </div>
          <div className="flex items-center space-x-4">
            <span className={`px-3 py-1 rounded-full text-sm font-medium ${
              connectionStatus === 'connected' ? 'bg-green-100 text-green-800' :
              connectionStatus === 'error' ? 'bg-red-100 text-red-800' :
              'bg-yellow-100 text-yellow-800'
            }`}>
              WebSocket: {connectionStatus.toUpperCase()}
            </span>
            <button
              onClick={sendSensorData}
              disabled={!isConnected || connectionStatus !== 'connected'}
              className="px-4 py-2 bg-blue-600 text-white rounded-md hover:bg-blue-700 disabled:opacity-50 disabled:cursor-not-allowed"
            >
              📤 Send Data
            </button>
          </div>
        </div>

        {/* Mock Data Warning */}
        <div className="bg-orange-50 border border-orange-200 rounded-lg p-4">
          <div className="flex">
            <div className="flex-shrink-0">
              <span className="text-orange-600 text-xl">⚠️</span>
            </div>
            <div className="ml-3">
              <h3 className="text-sm font-medium text-orange-800">
                Mock Sensor Data
              </h3>
              <div className="mt-2 text-sm text-orange-700">
                <p>
                  All sensor readings below are <strong>SIMULATED</strong> for testing purposes.
                  No real CAN bus hardware is required. These values will be sent to the
                  WebSocket bridge and can be consumed by ROS2 nodes for system validation.
                </p>
              </div>
            </div>
          </div>
        </div>
      </div>

      {/* Controls */}
      <div className="bg-white rounded-lg shadow p-6">
        <h3 className="text-lg font-semibold text-gray-900 mb-4">Simulation Controls</h3>
        <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
          <div>
            <label className="flex items-center">
              <input
                type="checkbox"
                checked={autoUpdate}
                onChange={(e) => setAutoUpdate(e.target.checked)}
                className="mr-2"
              />
              <span className="text-sm text-gray-700">Auto-update sensors</span>
            </label>
          </div>
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">
              Update Interval (ms)
            </label>
            <select
              value={updateInterval}
              onChange={(e) => setUpdateInterval(parseInt(e.target.value))}
              className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
            >
              <option value={500}>500ms</option>
              <option value={1000}>1000ms</option>
              <option value={2000}>2000ms</option>
              <option value={5000}>5000ms</option>
            </select>
          </div>
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">
              Presets
            </label>
            <div className="flex space-x-2">
              <button
                onClick={() => loadPreset('idle')}
                className="px-3 py-2 bg-gray-600 text-white rounded text-sm hover:bg-gray-700"
              >
                Idle
              </button>
              <button
                onClick={() => loadPreset('moving')}
                className="px-3 py-2 bg-blue-600 text-white rounded text-sm hover:bg-blue-700"
              >
                Moving
              </button>
              <button
                onClick={() => loadPreset('calibration')}
                className="px-3 py-2 bg-green-600 text-white rounded text-sm hover:bg-green-700"
              >
                Calibration
              </button>
            </div>
          </div>
        </div>
        {lastUpdate && (
          <div className="mt-4 text-sm text-gray-600">
            Last sent: {lastUpdate.toLocaleTimeString()}
          </div>
        )}
      </div>

      {/* IMU Sensor */}
      <div className="bg-white rounded-lg shadow p-6">
        <h3 className="text-lg font-semibold text-gray-900 mb-4">📐 IMU Sensor</h3>
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4">
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">
              Accel X (m/s²)
            </label>
            <input
              type="number"
              step="0.01"
              value={sensorData.imu.accel_x}
              onChange={(e) => updateSensorValue('imu', 'accel_x', e.target.value)}
              className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
            />
          </div>
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">
              Accel Y (m/s²)
            </label>
            <input
              type="number"
              step="0.01"
              value={sensorData.imu.accel_y}
              onChange={(e) => updateSensorValue('imu', 'accel_y', e.target.value)}
              className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
            />
          </div>
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">
              Accel Z (m/s²)
            </label>
            <input
              type="number"
              step="0.01"
              value={sensorData.imu.accel_z}
              onChange={(e) => updateSensorValue('imu', 'accel_z', e.target.value)}
              className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
            />
          </div>
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">
              Temperature (°C)
            </label>
            <input
              type="number"
              step="0.1"
              value={sensorData.imu.temperature}
              onChange={(e) => updateSensorValue('imu', 'temperature', e.target.value)}
              className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
            />
          </div>
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">
              Gyro X (rad/s)
            </label>
            <input
              type="number"
              step="0.001"
              value={sensorData.imu.gyro_x}
              onChange={(e) => updateSensorValue('imu', 'gyro_x', e.target.value)}
              className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
            />
          </div>
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">
              Gyro Y (rad/s)
            </label>
            <input
              type="number"
              step="0.001"
              value={sensorData.imu.gyro_y}
              onChange={(e) => updateSensorValue('imu', 'gyro_y', e.target.value)}
              className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
            />
          </div>
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">
              Gyro Z (rad/s)
            </label>
            <input
              type="number"
              step="0.001"
              value={sensorData.imu.gyro_z}
              onChange={(e) => updateSensorValue('imu', 'gyro_z', e.target.value)}
              className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
            />
          </div>
        </div>
      </div>

      {/* GPS Sensor */}
      <div className="bg-white rounded-lg shadow p-6">
        <h3 className="text-lg font-semibold text-gray-900 mb-4">🛰️ GPS Sensor</h3>
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4">
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">
              Latitude (°)
            </label>
            <input
              type="number"
              step="0.000001"
              value={sensorData.gps.latitude}
              onChange={(e) => updateSensorValue('gps', 'latitude', e.target.value)}
              className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
            />
          </div>
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">
              Longitude (°)
            </label>
            <input
              type="number"
              step="0.000001"
              value={sensorData.gps.longitude}
              onChange={(e) => updateSensorValue('gps', 'longitude', e.target.value)}
              className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
            />
          </div>
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">
              Altitude (m)
            </label>
            <input
              type="number"
              step="0.1"
              value={sensorData.gps.altitude}
              onChange={(e) => updateSensorValue('gps', 'altitude', e.target.value)}
              className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
            />
          </div>
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">
              Heading (°)
            </label>
            <input
              type="number"
              step="0.1"
              min="0"
              max="360"
              value={sensorData.gps.heading}
              onChange={(e) => updateSensorValue('gps', 'heading', e.target.value)}
              className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
            />
          </div>
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">
              Speed (m/s)
            </label>
            <input
              type="number"
              step="0.1"
              min="0"
              value={sensorData.gps.speed}
              onChange={(e) => updateSensorValue('gps', 'speed', e.target.value)}
              className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
            />
          </div>
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">
              Satellites
            </label>
            <input
              type="number"
              min="0"
              max="20"
              value={sensorData.gps.satellites}
              onChange={(e) => updateSensorValue('gps', 'satellites', e.target.value)}
              className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
            />
          </div>
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">
              HDOP
            </label>
            <input
              type="number"
              step="0.01"
              min="0"
              value={sensorData.gps.hdop}
              onChange={(e) => updateSensorValue('gps', 'hdop', e.target.value)}
              className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
            />
          </div>
        </div>
      </div>

      {/* Battery Sensor */}
      <div className="bg-white rounded-lg shadow p-6">
        <h3 className="text-lg font-semibold text-gray-900 mb-4">🔋 Battery Monitor</h3>
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4">
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">
              Voltage (V)
            </label>
            <input
              type="number"
              step="0.01"
              min="0"
              value={sensorData.battery.voltage}
              onChange={(e) => updateSensorValue('battery', 'voltage', e.target.value)}
              className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
            />
          </div>
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">
              Current (A)
            </label>
            <input
              type="number"
              step="0.01"
              value={sensorData.battery.current}
              onChange={(e) => updateSensorValue('battery', 'current', e.target.value)}
              className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
            />
          </div>
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">
              Charge Level (%)
            </label>
            <input
              type="number"
              step="0.1"
              min="0"
              max="100"
              value={sensorData.battery.charge_level}
              onChange={(e) => updateSensorValue('battery', 'charge_level', e.target.value)}
              className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
            />
          </div>
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">
              Temperature (°C)
            </label>
            <input
              type="number"
              step="0.1"
              value={sensorData.battery.temperature}
              onChange={(e) => updateSensorValue('battery', 'temperature', e.target.value)}
              className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
            />
          </div>
        </div>
      </div>

      {/* Motor Sensors */}
      <div className="bg-white rounded-lg shadow p-6">
        <h3 className="text-lg font-semibold text-gray-900 mb-4">⚙️ Motor Encoders</h3>
        <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
          {/* Left Motor */}
          <div>
            <h4 className="font-medium text-gray-900 mb-3">Left Motor</h4>
            <div className="grid grid-cols-2 gap-4">
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  Position (rad)
                </label>
                <input
                  type="number"
                  step="0.01"
                  value={sensorData.motor_left.position}
                  onChange={(e) => updateSensorValue('motor_left', 'position', e.target.value)}
                  className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
                />
              </div>
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  Velocity (rad/s)
                </label>
                <input
                  type="number"
                  step="0.01"
                  value={sensorData.motor_left.velocity}
                  onChange={(e) => updateSensorValue('motor_left', 'velocity', e.target.value)}
                  className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
                />
              </div>
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  Current (A)
                </label>
                <input
                  type="number"
                  step="0.01"
                  value={sensorData.motor_left.current}
                  onChange={(e) => updateSensorValue('motor_left', 'current', e.target.value)}
                  className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
                />
              </div>
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  Temperature (°C)
                </label>
                <input
                  type="number"
                  step="0.01"
                  value={sensorData.motor_left.temperature}
                  onChange={(e) => updateSensorValue('motor_left', 'temperature', e.target.value)}
                  className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
                />
              </div>
            </div>
          </div>

          {/* Right Motor */}
          <div>
            <h4 className="font-medium text-gray-900 mb-3">Right Motor</h4>
            <div className="grid grid-cols-2 gap-4">
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  Position (rad)
                </label>
                <input
                  type="number"
                  step="0.01"
                  value={sensorData.motor_right.position}
                  onChange={(e) => updateSensorValue('motor_right', 'position', e.target.value)}
                  className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
                />
              </div>
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  Velocity (rad/s)
                </label>
                <input
                  type="number"
                  step="0.01"
                  value={sensorData.motor_right.velocity}
                  onChange={(e) => updateSensorValue('motor_right', 'velocity', e.target.value)}
                  className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
                />
              </div>
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  Current (A)
                </label>
                <input
                  type="number"
                  step="0.01"
                  value={sensorData.motor_right.current}
                  onChange={(e) => updateSensorValue('motor_right', 'current', e.target.value)}
                  className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
                />
              </div>
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  Temperature (°C)
                </label>
                <input
                  type="number"
                  step="0.01"
                  value={sensorData.motor_right.temperature}
                  onChange={(e) => updateSensorValue('motor_right', 'temperature', e.target.value)}
                  className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
                />
              </div>
            </div>
          </div>
        </div>
      </div>

      {/* Environmental Sensors */}
      <div className="bg-white rounded-lg shadow p-6">
        <h3 className="text-lg font-semibold text-gray-900 mb-4">🌡️ Environmental Sensors</h3>
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4">
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">
              CPU Temperature (°C)
            </label>
            <input
              type="number"
              step="0.1"
              value={sensorData.environment.cpu_temp}
              onChange={(e) => updateSensorValue('environment', 'cpu_temp', e.target.value)}
              className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
            />
          </div>
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">
              Ambient Temperature (°C)
            </label>
            <input
              type="number"
              step="0.1"
              value={sensorData.environment.ambient_temp}
              onChange={(e) => updateSensorValue('environment', 'ambient_temp', e.target.value)}
              className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
            />
          </div>
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">
              Humidity (%)
            </label>
            <input
              type="number"
              step="0.1"
              min="0"
              max="100"
              value={sensorData.environment.humidity}
              onChange={(e) => updateSensorValue('environment', 'humidity', e.target.value)}
              className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
            />
          </div>
          <div>
            <label className="block text-sm font-medium text-gray-700 mb-1">
              Pressure (hPa)
            </label>
            <input
              type="number"
              step="0.01"
              value={sensorData.environment.pressure}
              onChange={(e) => updateSensorValue('environment', 'pressure', e.target.value)}
              className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
            />
          </div>
        </div>
      </div>
    </div>
  );
};

export default SensorDataSimulator;



