import { useState, useEffect, useCallback } from 'react';
import { SAFETY_TOPICS, SERVICE_TYPES, MESSAGE_TYPES } from '../config/rosTopics';

/**
 * Custom hook for ROS safety system integration
 *
 * Provides safety monitoring and control capabilities including:
 * - Real-time safety status monitoring
 * - Active alerts tracking
 * - System health monitoring
 * - Emergency status monitoring
 * - Safety service calls (emergency stop, recovery, etc.)
 *
 * @param {Object} ros - ROS connection object
 * @returns {Object} Safety system state and control functions
 */
export const useSafetySystem = (ros) => {
  // Safety state
  const [safetyStatus, setSafetyStatus] = useState(null);
  const [activeAlerts, setActiveAlerts] = useState([]);
  const [systemHealth, setSystemHealth] = useState(null);
  const [emergencyStatus, setEmergencyStatus] = useState(null);
  const [watchdogStatus, setWatchdogStatus] = useState(null);
  const [sensorHealth, setSensorHealth] = useState(null);

  // Test results
  const [lastTestResult, setLastTestResult] = useState(null);

  // Subscribe to safety topics when ROS is connected
  useEffect(() => {
    if (!ros || !ros.isConnected) return;

    const subscriptions = [];

    try {
      // Safety status from state machine
      const safetyStatusSub = ros.subscribe(
        SAFETY_TOPICS.SAFETY_STATUS,
        (msg) => {
          try {
            setSafetyStatus(msg);
          } catch (error) {
            console.error('Error parsing safety status:', error);
          }
        },
        10
      );
      subscriptions.push(safetyStatusSub);

      // Safety violations from watchdog
      const violationsSub = ros.subscribe(
        SAFETY_TOPICS.SAFETY_VIOLATIONS,
        (msg) => {
          try {
            setSafetyStatus(prev => ({ ...prev, violations: msg }));
          } catch (error) {
            console.error('Error parsing safety violations:', error);
          }
        },
        10
      );
      subscriptions.push(violationsSub);

      // Redundant safety status
      const redundantSub = ros.subscribe(
        SAFETY_TOPICS.REDUNDANT_SAFETY_STATUS,
        (msg) => {
          try {
            setSafetyStatus(prev => ({ ...prev, redundant: msg }));
          } catch (error) {
            console.error('Error parsing redundant safety status:', error);
          }
        },
        10
      );
      subscriptions.push(redundantSub);

      // Dashboard status (JSON string)
      const dashboardSub = ros.subscribe(
        SAFETY_TOPICS.DASHBOARD_STATUS,
        (msg) => {
          try {
            const data = JSON.parse(msg.data);
            setSafetyStatus(prev => ({ ...prev, dashboard: data }));
          } catch (error) {
            console.error('Error parsing dashboard status:', error);
          }
        },
        10
      );
      subscriptions.push(dashboardSub);

      // Active alerts (JSON string)
      const alertsSub = ros.subscribe(
        SAFETY_TOPICS.ACTIVE_ALERTS,
        (msg) => {
          try {
            const data = JSON.parse(msg.data);
            setActiveAlerts(data.active_alerts || []);
          } catch (error) {
            console.error('Error parsing active alerts:', error);
          }
        },
        10
      );
      subscriptions.push(alertsSub);

      // System health (JSON string)
      const healthSub = ros.subscribe(
        SAFETY_TOPICS.SYSTEM_HEALTH,
        (msg) => {
          try {
            const data = JSON.parse(msg.data);
            setSystemHealth(data);
          } catch (error) {
            console.error('Error parsing system health:', error);
          }
        },
        10
      );
      subscriptions.push(healthSub);

      // Watchdog status (JSON string)
      const watchdogSub = ros.subscribe(
        SAFETY_TOPICS.WATCHDOG_STATUS,
        (msg) => {
          try {
            const data = JSON.parse(msg.data);
            setWatchdogStatus(data);
          } catch (error) {
            console.error('Error parsing watchdog status:', error);
          }
        },
        10
      );
      subscriptions.push(watchdogSub);

      // Sensor health (JSON string)
      const sensorSub = ros.subscribe(
        SAFETY_TOPICS.SENSOR_HEALTH,
        (msg) => {
          try {
            const data = JSON.parse(msg.data);
            setSensorHealth(data);
          } catch (error) {
            console.error('Error parsing sensor health:', error);
          }
        },
        10
      );
      subscriptions.push(sensorSub);

      // Emergency status (JSON string)
      const emergencySub = ros.subscribe(
        SAFETY_TOPICS.EMERGENCY_STATUS,
        (msg) => {
          try {
            const data = JSON.parse(msg.data);
            setEmergencyStatus(data);
          } catch (error) {
            console.error('Error parsing emergency status:', error);
          }
        },
        10
      );
      subscriptions.push(emergencySub);

    } catch (error) {
      console.error('Error setting up safety subscriptions:', error);
    }

    // Cleanup subscriptions on unmount or ROS disconnect
    return () => {
      subscriptions.forEach(sub => {
        try {
          sub.unsubscribe();
        } catch (error) {
          console.error('Error unsubscribing from safety topic:', error);
        }
      });
    };
  }, [ros]);

  // Safety service calls
  const callSafetyService = useCallback(async (serviceName, serviceType, request) => {
    if (!ros || !ros.isConnected) {
      throw new Error('ROS not connected');
    }

    return new Promise((resolve, reject) => {
      const service = ros.service(serviceName, serviceType);

      service.callService(request,
        (result) => resolve(result),
        (error) => reject(error)
      );
    });
  }, [ros]);

  // Software Emergency Stop
  const triggerSoftwareEstop = useCallback(async (operatorId, reason) => {
    try {
      const result = await callSafetyService(
        SAFETY_TOPICS.SOFTWARE_ESTOP,
        SERVICE_TYPES.SOFTWARE_ESTOP,
        {
          operator_id: operatorId,
          reason: reason || 'Frontend safety test',
          acknowledge_criticality: true,
          force_immediate: false
        }
      );

      setLastTestResult({
        test: 'software_estop',
        success: result.success,
        message: result.message,
        details: result
      });

      return {
        success: result.success,
        message: result.message,
        details: result
      };
    } catch (error) {
      setLastTestResult({
        test: 'software_estop',
        success: false,
        message: error.message,
        details: error
      });
      throw error;
    }
  }, [callSafetyService]);

  // Safety Recovery
  const recoverFromSafety = useCallback(async (recoveryMethod, operatorId) => {
    try {
      const result = await callSafetyService(
        SAFETY_TOPICS.RECOVER_FROM_SAFETY,
        SERVICE_TYPES.RECOVER_FROM_SAFETY,
        {
          recovery_method: recoveryMethod || 'AUTO',
          operator_id: operatorId || 'frontend_test',
          acknowledge_risks: true
        }
      );

      setLastTestResult({
        test: 'safety_recovery',
        success: result.success,
        message: result.message,
        details: result
      });

      return {
        success: result.success,
        message: result.message,
        details: result
      };
    } catch (error) {
      setLastTestResult({
        test: 'safety_recovery',
        success: false,
        message: error.message,
        details: error
      });
      throw error;
    }
  }, [callSafetyService]);

  // Manual Recovery with steps
  const recoverFromSafetyManual = useCallback(async (operatorId, completedSteps) => {
    try {
      const result = await callSafetyService(
        SAFETY_TOPICS.RECOVER_FROM_SAFETY,
        SERVICE_TYPES.RECOVER_FROM_SAFETY,
        {
          recovery_method: 'MANUAL_GUIDED',
          operator_id: operatorId || 'frontend_test',
          acknowledge_risks: true,
          completed_steps: completedSteps || []
        }
      );

      setLastTestResult({
        test: 'manual_recovery',
        success: result.success,
        message: result.message,
        details: result
      });

      return {
        success: result.success,
        message: result.message,
        details: result
      };
    } catch (error) {
      setLastTestResult({
        test: 'manual_recovery',
        success: false,
        message: error.message,
        details: error
      });
      throw error;
    }
  }, [callSafetyService]);

  // Generic test runner
  const runSafetyTest = useCallback(async (testId) => {
    switch (testId) {
      case 'software_estop':
        return await triggerSoftwareEstop('frontend_test', `Safety test: ${testId}`);

      case 'safety_recovery':
        return await recoverFromSafety('AUTO', 'frontend_test');

      case 'manual_recovery':
        return await recoverFromSafetyManual('frontend_test', ['acknowledged_risks']);

      case 'watchdog_monitoring':
        // Check if watchdog is publishing data
        if (watchdogStatus) {
          return {
            success: true,
            message: 'Watchdog monitoring active',
            details: watchdogStatus
          };
        } else {
          return {
            success: false,
            message: 'No watchdog data received',
            details: null
          };
        }

      case 'redundant_safety':
        // Check redundant safety status
        if (safetyStatus?.redundant) {
          return {
            success: true,
            message: 'Redundant safety system active',
            details: safetyStatus.redundant
          };
        } else {
          return {
            success: false,
            message: 'No redundant safety data received',
            details: null
          };
        }

      case 'sensor_health':
        // Check sensor health
        if (sensorHealth) {
          const healthySensors = Object.values(sensorHealth.sensors || {}).filter(s => s.healthy).length;
          const totalSensors = Object.keys(sensorHealth.sensors || {}).length;

          return {
            success: healthySensors > 0,
            message: `${healthySensors}/${totalSensors} sensors healthy`,
            details: sensorHealth
          };
        } else {
          return {
            success: false,
            message: 'No sensor health data received',
            details: null
          };
        }

      case 'system_diagnostics':
        // Check overall system health
        if (systemHealth) {
          const healthySystems = Object.values(systemHealth.systems || {}).filter(s => s.status === 'HEALTHY').length;
          const totalSystems = Object.keys(systemHealth.systems || {}).length;

          return {
            success: healthySystems === totalSystems,
            message: `${healthySystems}/${totalSystems} systems healthy`,
            details: systemHealth
          };
        } else {
          return {
            success: false,
            message: 'No system health data received',
            details: null
          };
        }

      case 'safety_state_transition':
        // This would require state machine integration
        return {
          success: false,
          message: 'Safety state transition test requires state machine integration',
          details: null
        };

      default:
        throw new Error(`Unknown safety test: ${testId}`);
    }
  }, [triggerSoftwareEstop, recoverFromSafety, recoverFromSafetyManual, watchdogStatus, safetyStatus, sensorHealth, systemHealth]);

  return {
    // State
    safetyStatus,
    activeAlerts,
    systemHealth,
    emergencyStatus,
    watchdogStatus,
    sensorHealth,
    lastTestResult,

    // Computed state
    isEmergencyActive: emergencyStatus?.emergency_active || false,
    safetyLevel: safetyStatus?.safety_level || 'UNKNOWN',
    isSafe: safetyStatus?.is_safe || false,

    // Actions
    triggerSoftwareEstop,
    recoverFromSafety,
    recoverFromSafetyManual,
    runSafetyTest,

    // Utilities
    callSafetyService
  };
};
