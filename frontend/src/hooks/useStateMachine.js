import { useState, useEffect, useRef } from 'react';
import { createSubscriber, createServiceClient, callService } from '../utils/rosbridge';
import { STATE_TOPICS, MESSAGE_TYPES, SERVICE_TYPES } from '../config/rosTopics';
import {
  SystemState,
  AutonomousSubstate,
  EquipmentServicingSubstate,
  CalibrationSubstate
} from '../config/stateDefinitions';

/**
 * Custom hook for subscribing to state machine ROS topics
 *
 * Provides real-time state machine data from ROS backend
 *
 * @param {Object} ros - ROS connection instance from useROS hook
 * @returns {Object} Current state machine status
 */
export const useStateMachine = (ros) => {
  const [currentState, setCurrentState] = useState(SystemState.BOOT);
  const [currentSubstate, setCurrentSubstate] = useState(AutonomousSubstate.NONE);
  const [currentSubSubstate, setCurrentSubSubstate] = useState(EquipmentServicingSubstate.NONE);
  const [currentCalibrationSubstate, setCurrentCalibrationSubstate] = useState(CalibrationSubstate.NONE);
  const [stateMetadata, setStateMetadata] = useState({});
  const [transitionHistory, setTransitionHistory] = useState([]);
  const [isTransitioning, setIsTransitioning] = useState(false);
  const [lastTransition, setLastTransition] = useState(null);

  // ROS subscribers
  const subscribersRef = useRef({});

  // Subscribe to state topics when ROS is connected
  useEffect(() => {
    if (!ros) return;

    console.log('Setting up state machine topic subscriptions...');

    // Current state subscriber
    subscribersRef.current.currentState = createSubscriber(
      ros,
      STATE_TOPICS.CURRENT_STATE,
      MESSAGE_TYPES.SYSTEM_STATE,
      (message) => {
        console.log('Received current state:', message);
        setCurrentState(message.current_state || SystemState.BOOT);
        setCurrentSubstate(message.autonomous_substate || AutonomousSubstate.NONE);
        setCurrentSubSubstate(message.equipment_servicing_substate || EquipmentServicingSubstate.NONE);
        setCurrentCalibrationSubstate(message.calibration_substate || CalibrationSubstate.NONE);
        setStateMetadata(message.metadata || {});
        setIsTransitioning(message.is_transitioning || false);
      }
    );

    // State transition subscriber
    subscribersRef.current.stateTransition = createSubscriber(
      ros,
      STATE_TOPICS.STATE_TRANSITION,
      MESSAGE_TYPES.STATE_TRANSITION,
      (message) => {
        console.log('Received state transition:', message);
        setLastTransition({
          fromState: message.from_state,
          toState: message.to_state,
          reason: message.reason,
          initiatedBy: message.initiated_by,
          timestamp: message.timestamp,
          success: message.success
        });

        // Update transition history
        setTransitionHistory(prev => {
          const newHistory = [{
            fromState: message.from_state,
            toState: message.to_state,
            reason: message.reason,
            timestamp: new Date().toISOString(),
            success: message.success
          }, ...prev].slice(0, 50); // Keep last 50 transitions
          return newHistory;
        });

        // Update state if transition was successful
        if (message.success) {
          setCurrentState(message.to_state);
          setIsTransitioning(false);
        }
      }
    );

    // Substate subscriber
    subscribersRef.current.substate = createSubscriber(
      ros,
      STATE_TOPICS.SUBSTATE,
      MESSAGE_TYPES.STRING,
      (message) => {
        console.log('Received substate update:', message);
        // Parse substate data (assuming format: "AUTONOMOUS:SCIENCE" or "CALIBRATION:SETUP")
        const parts = message.data.split(':');
        if (parts.length === 2) {
          const [type, substate] = parts;
          if (type === 'AUTONOMOUS') {
            setCurrentSubstate(substate);
          } else if (type === 'EQUIPMENT_SERVICING') {
            setCurrentSubSubstate(substate);
          } else if (type === 'CALIBRATION') {
            setCurrentCalibrationSubstate(substate);
          }
        }
      }
    );

    // Cleanup function
    return () => {
      console.log('Cleaning up state machine subscriptions...');
      Object.values(subscribersRef.current).forEach(subscriber => {
        if (subscriber && subscriber.unsubscribe) {
          subscriber.unsubscribe();
        }
      });
      subscribersRef.current = {};
    };
  }, [ros]);

  // Request current state (useful for initialization)
  const requestCurrentState = () => {
    if (!ros) return;

    // Call the get_current_state service
    ros.callService(
      STATE_TOPICS.GET_CURRENT_STATE_SERVICE,
      MESSAGE_TYPES.GET_SYSTEM_STATE,
      {},
      (result) => {
        console.log('Current state response:', result);
        if (result.current_state) {
          setCurrentState(result.current_state);
          setCurrentSubstate(result.autonomous_substate || AutonomousSubstate.NONE);
          setCurrentSubSubstate(result.equipment_servicing_substate || EquipmentServicingSubstate.NONE);
          setCurrentCalibrationSubstate(result.calibration_substate || CalibrationSubstate.NONE);
        }
      },
      (error) => {
        console.error('Failed to get current state:', error);
      }
    );
  };

  // Request state transition
  const requestStateTransition = async (targetState, reason = 'frontend_request', force = false) => {
    // Check if ROS is connected
    if (!ros || !ros.isConnected) {
      // Demo mode: simulate transition validation without ROS
      console.log('ROS not connected - running in demo mode');

      // Simulate network delay
      await new Promise(resolve => setTimeout(resolve, 500));

      // Store current state before transition
      const previousState = currentState;

      // Simulate successful transition for demo purposes
      const mockResult = {
        success: true,
        message: `Demo mode: ${previousState} → ${targetState} transition would succeed`,
        new_state: targetState,
        previous_state: previousState,
        timestamp: new Date().toISOString()
      };

      // Update local state to simulate the transition
      setCurrentState(targetState);
      setIsTransitioning(false); // Demo mode doesn't have real transitioning state

      // Simulate a state transition message
      const mockTransition = {
        from_state: previousState,
        to_state: targetState,
        reason: reason,
        initiated_by: 'frontend_demo',
        timestamp: new Date().toISOString(),
        success: true
      };

      // Update transition history
      setTransitionHistory(prev => {
        const newHistory = [{
          fromState: mockTransition.from_state,
          toState: mockTransition.to_state,
          reason: mockTransition.reason,
          timestamp: new Date().toISOString(),
          success: true
        }, ...prev].slice(0, 50); // Keep last 50 transitions
        return newHistory;
      });

      console.log('Demo transition result:', mockResult);
      return mockResult;
    }

    const serviceName = force ? STATE_TOPICS.FORCE_TRANSITION_SERVICE : STATE_TOPICS.CHANGE_STATE_SERVICE;

    try {
      const serviceClient = createServiceClient(ros, serviceName, SERVICE_TYPES.CHANGE_STATE);

      const request = {
        desired_state: targetState,
        reason: reason,
        initiated_by: 'frontend'
      };

      const result = await callService(serviceClient, request);

      console.log('State transition result:', result);
      if (result.success) {
        return result;
      } else {
        throw new Error(result.message || 'State transition failed');
      }
    } catch (error) {
      console.error('State transition service error:', error);
      throw error;
    }
  };

  return {
    // Current state data
    currentState,
    currentSubstate,
    currentSubSubstate,
    currentCalibrationSubstate,
    stateMetadata,
    isTransitioning,
    lastTransition,

    // History
    transitionHistory,

    // Actions
    requestCurrentState,
    requestStateTransition,

    // Status
    isSubscribed: Object.keys(subscribersRef.current).length > 0
  };
};
