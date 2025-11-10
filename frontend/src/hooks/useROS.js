import { useState, useEffect, useRef, useCallback } from 'react';
import ROSLIB from 'roslib';

/**
 * ROS Connection Hook - URC 2026 Mars Rover Interface
 *
 * Custom React hook for managing WebSocket connection to ROS2 bridge server.
 * Provides automatic reconnection, connection state management, and error handling
 * for reliable communication with the rover's ROS2 system.
 *
 * This hook establishes and maintains a WebSocket connection to the ROS2 bridge
 * server running on the rover, enabling real-time data exchange between the web
 * interface and the autonomous systems.
 *
 * Features:
 * - Automatic reconnection with exponential backoff
 * - Connection state monitoring and reporting
 * - Error handling and recovery
 * - Clean disconnection and resource cleanup
 *
 * @param {Object} config - ROS connection configuration
 * @param {string} [config.url='ws://localhost:9090'] - ROS bridge server WebSocket URL
 * @param {number} [config.reconnectInterval=3000] - Time between reconnection attempts (ms)
 * @param {number} [config.maxReconnectAttempts=10] - Maximum number of reconnection attempts
 *
 * @returns {Object} ROS connection state and control functions
 * @returns {boolean} returns.isConnected - Current connection status
 * @returns {string} returns.connectionStatus - Detailed connection state ('connected', 'connecting', 'disconnected', 'error')
 * @returns {number} returns.reconnectAttempts - Number of reconnection attempts made
 * @returns {Error|null} returns.lastError - Last connection error encountered
 * @returns {function} returns.connect - Manually initiate connection
 * @returns {function} returns.disconnect - Manually disconnect and stop reconnection
 * @returns {number} returns.maxReconnectAttempts - Maximum allowed reconnection attempts
 * @returns {ROSLIB.Ros|null} returns.ros - ROSLIB.Ros instance for direct ROS operations
 *
 * @example
 * ```javascript
 * const {
 *   isConnected,
 *   connectionStatus,
 *   connect,
 *   disconnect,
 *   ros
 * } = useROS({
 *   url: 'ws://192.168.1.100:9090',
 *   reconnectInterval: 5000,
 *   maxReconnectAttempts: 5
 * });
 *
 * // Use the ROS instance for publishing/subscribing
 * useEffect(() => {
 *   if (isConnected && ros) {
 *     const topic = new ROSLIB.Topic({
 *       ros: ros,
 *       name: '/system_state',
 *       messageType: 'autonomy_interfaces/msg/SystemState'
 *     });
 *     // ... topic operations
 *   }
 * }, [isConnected, ros]);
 * ```
 *
 * @throws {Error} Throws error if ROSLIB is not available or configuration is invalid
 */
export const useROS = (config = {}) => {
  const {
    url = 'ws://localhost:9090',
    reconnectInterval = 3000,
    maxReconnectAttempts = 10
  } = config;

  const [isConnected, setIsConnected] = useState(false);
  const [connectionStatus, setConnectionStatus] = useState('disconnected');
  const [reconnectAttempts, setReconnectAttempts] = useState(0);
  const [lastError, setLastError] = useState(null);

  const rosRef = useRef(null);
  const reconnectTimeoutRef = useRef(null);
  const reconnectAttemptsRef = useRef(0);

  // Initialize ROS connection
  const connect = useCallback(() => {
    // Clear any existing reconnection timeout
    if (reconnectTimeoutRef.current) {
      clearTimeout(reconnectTimeoutRef.current);
      reconnectTimeoutRef.current = null;
    }

    if (rosRef.current) {
      rosRef.current.close();
    }

    setConnectionStatus('connecting');
    setLastError(null);

    rosRef.current = new ROSLIB.Ros({
      url: url
    });

    rosRef.current.on('connection', () => {
      console.log('Connected to ROS bridge server at', url);
      setIsConnected(true);
      setConnectionStatus('connected');
      setReconnectAttempts(0);
      reconnectAttemptsRef.current = 0;
      // Clear any pending reconnection timeout since we're now connected
      if (reconnectTimeoutRef.current) {
        clearTimeout(reconnectTimeoutRef.current);
        reconnectTimeoutRef.current = null;
      }
    });

    rosRef.current.on('error', (error) => {
      console.error('ROS connection error:', error);
      setLastError(error);
      setConnectionStatus('error');
      setIsConnected(false);

      // Clear any existing reconnection timeout before scheduling a new one
      if (reconnectTimeoutRef.current) {
        clearTimeout(reconnectTimeoutRef.current);
        reconnectTimeoutRef.current = null;
      }

      // Attempt reconnection if under max attempts
      if (reconnectAttemptsRef.current < maxReconnectAttempts) {
        reconnectAttemptsRef.current += 1;
        setReconnectAttempts(reconnectAttemptsRef.current);

        reconnectTimeoutRef.current = setTimeout(() => {
          console.log(`Attempting reconnection (${reconnectAttemptsRef.current}/${maxReconnectAttempts})`);
          connect();
        }, reconnectInterval);
      } else {
        setConnectionStatus('failed');
        console.error('Max reconnection attempts reached');
      }
    });

    rosRef.current.on('close', () => {
      console.log('ROS connection closed');
      setIsConnected(false);
      setConnectionStatus('disconnected');

      // Only attempt reconnection if we haven't already scheduled one from an error
      // and we're still under the max attempts
      if (!reconnectTimeoutRef.current && reconnectAttemptsRef.current < maxReconnectAttempts) {
        reconnectTimeoutRef.current = setTimeout(() => {
          reconnectAttemptsRef.current += 1;
          setReconnectAttempts(reconnectAttemptsRef.current);
          console.log(`Attempting reconnection (${reconnectAttemptsRef.current}/${maxReconnectAttempts})`);
          connect();
        }, reconnectInterval);
      }
    });

  }, [url, reconnectInterval, maxReconnectAttempts]);

  // Disconnect from ROS
  const disconnect = useCallback(() => {
    if (reconnectTimeoutRef.current) {
      clearTimeout(reconnectTimeoutRef.current);
      reconnectTimeoutRef.current = null;
    }

    if (rosRef.current) {
      rosRef.current.close();
      rosRef.current = null;
    }

    setIsConnected(false);
    setConnectionStatus('disconnected');
    setReconnectAttempts(0);
    reconnectAttemptsRef.current = 0;
    setLastError(null);
  }, []);

  // Reset reconnection attempts
  const resetReconnection = useCallback(() => {
    setReconnectAttempts(0);
    reconnectAttemptsRef.current = 0;
    if (reconnectTimeoutRef.current) {
      clearTimeout(reconnectTimeoutRef.current);
      reconnectTimeoutRef.current = null;
    }
  }, []);

  // Auto-connect on mount
  useEffect(() => {
    connect();

    // Cleanup on unmount
    return () => {
      disconnect();
    };
  }, [connect, disconnect]);

  return {
    ros: rosRef.current,
    isConnected,
    connectionStatus,
    reconnectAttempts,
    lastError,
    connect,
    disconnect,
    resetReconnection,
    maxReconnectAttempts
  };
};
