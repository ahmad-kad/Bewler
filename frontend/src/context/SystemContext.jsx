import React, { createContext, useContext, useState, useEffect, useCallback } from 'react';
import { useROS } from '../hooks/useROS';
import { useStateMachine } from '../hooks/useStateMachine';

/**
 * System Context Provider
 *
 * Manages global system state, telemetry, and mission data for the entire application.
 * Provides context-aware data to all components based on current system state.
 */
const SystemContext = createContext(null);

export const useSystemContext = () => {
  const context = useContext(SystemContext);
  if (!context) {
    throw new Error('useSystemContext must be used within SystemContextProvider');
  }
  return context;
};

export const SystemContextProvider = ({ children }) => {
  // ROS connection
  const { ros, isConnected, connectionStatus } = useROS();

  // State machine
  const {
    currentState,
    currentSubstate,
    requestStateTransition,
    isTransitioning
  } = useStateMachine(ros);

  // System telemetry
  const [telemetry, setTelemetry] = useState({
    battery: 85,
    gps: { satellites: 12, hdop: 1.2, position: { lat: 38.406, lon: -110.792 } },
    speed: 0.0,
    temperature: 28,
    timestamp: Date.now()
  });

  // System status
  const [systemStatus, setSystemStatus] = useState({
    safety: 'ready',
    navigation: 'ok',
    vision: 'ready',
    can: 'mock', // Always mock for now
    websocket: isConnected ? 'connected' : 'disconnected'
  });

  // Active mission
  const [activeMission, setActiveMission] = useState(null);

  // Alerts and warnings
  const [alerts, setAlerts] = useState([]);

  // Error count for debug tab badge
  const [errorCount, setErrorCount] = useState(0);

  // Test status for testing tab badge
  const [runningTests, setRunningTests] = useState(0);

  // Update telemetry periodically (mock data for now)
  useEffect(() => {
    const interval = setInterval(() => {
      setTelemetry(prev => ({
        ...prev,
        battery: Math.max(0, prev.battery - Math.random() * 0.1),
        gps: {
          ...prev.gps,
          satellites: 10 + Math.floor(Math.random() * 5),
          hdop: 1.0 + Math.random() * 1.5
        },
        speed: currentState === 'AUTONOMOUS' ? 0.3 + Math.random() * 0.4 : 0.0,
        temperature: 25 + Math.random() * 10,
        timestamp: Date.now()
      }));
    }, 1000);

    return () => clearInterval(interval);
  }, [currentState]);

  // Update alerts based on system state
  useEffect(() => {
    const newAlerts = [];

    if (telemetry.gps.hdop > 2.0) {
      newAlerts.push({
        id: 'gps_drift',
        type: 'warning',
        message: 'GPS drift detected',
        component: 'navigation',
        timestamp: Date.now()
      });
    }

    if (telemetry.battery < 20) {
      newAlerts.push({
        id: 'low_battery',
        type: 'error',
        message: 'Low battery warning',
        component: 'power',
        timestamp: Date.now()
      });
    }

    if (systemStatus.navigation === 'degraded') {
      newAlerts.push({
        id: 'nav_degraded',
        type: 'warning',
        message: 'Navigation system degraded',
        component: 'navigation',
        timestamp: Date.now()
      });
    }

    setAlerts(newAlerts);
    setErrorCount(newAlerts.filter(a => a.type === 'error').length);
  }, [telemetry, systemStatus]);

  // Get system state badge info
  const getStateBadge = useCallback(() => {
    const stateConfig = {
      'BOOT': { label: 'BOOT', color: 'blue', emoji: '🔵' },
      'IDLE': { label: 'IDLE', color: 'green', emoji: '🟢' },
      'AUTONOMOUS': { label: 'AUTONOMOUS', color: 'cyan', emoji: '🔵' },
      'TELEOPERATION': { label: 'TELEOP', color: 'yellow', emoji: '🟡' },
      'SAFESTOP': { label: 'SAFE STOP', color: 'orange', emoji: '🟠' },
      'SAFETY': { label: 'SAFETY', color: 'red', emoji: '🔴' },
      'SHUTDOWN': { label: 'SHUTDOWN', color: 'gray', emoji: '⚫' }
    };

    return stateConfig[currentState] || { label: currentState, color: 'gray', emoji: '⚪' };
  }, [currentState]);

  // Emergency stop handler
  const handleEmergencyStop = useCallback(async () => {
    try {
      await requestStateTransition('SAFETY', 'Emergency stop activated from UI');
    } catch (error) {
      console.error('Emergency stop failed:', error);
    }
  }, [requestStateTransition]);

  const value = {
    // Connection
    ros,
    isConnected,
    connectionStatus,

    // State
    currentState,
    currentSubstate,
    requestStateTransition,
    isTransitioning,
    getStateBadge,

    // Telemetry
    telemetry,
    setTelemetry,

    // System status
    systemStatus,
    setSystemStatus,

    // Mission
    activeMission,
    setActiveMission,

    // Alerts
    alerts,
    setAlerts,
    errorCount,

    // Testing
    runningTests,
    setRunningTests,

    // Actions
    handleEmergencyStop
  };

  return (
    <SystemContext.Provider value={value}>
      {children}
    </SystemContext.Provider>
  );
};
