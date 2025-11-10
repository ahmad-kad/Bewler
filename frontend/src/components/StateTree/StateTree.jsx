import React, { useState, useMemo } from 'react';
import StateNode from './StateNode';
import TransitionArrow from './TransitionArrow';
import { SystemState, StateTransitions, AutonomousSubstate, CalibrationSubstate } from '../../config/stateDefinitions';

/**
 * Enhanced StateTree Component - Mermaid-style flowchart visualization
 *
 * Displays the URC 2026 state machine as an interactive flowchart with:
 * - Professional flowchart layout (like Mermaid diagrams)
 * - Smooth curved transition arrows
 * - Enhanced visual indicators and animations
 * - Better HCI with intuitive interactions
 * - Clean, modern design
 */
const StateTree = ({
  currentState,
  currentSubstate,
  currentSubSubstate,
  currentCalibrationSubstate,
  onStateTransition,
  className = "",
  isTransitioning = false
}) => {
  const [hoveredState, setHoveredState] = useState(null);
  const [selectedState, setSelectedState] = useState(null);

  // Enhanced flowchart layout - arranged like a professional state diagram
  const statePositions = useMemo(() => ({
    // Top row - initial states
    [SystemState.BOOT]: { x: 150, y: 120 },
    [SystemState.CALIBRATION]: { x: 350, y: 120 },
    [SystemState.IDLE]: { x: 550, y: 120 },

    // Middle row - operational states
    [SystemState.TELEOPERATION]: { x: 250, y: 320 },
    [SystemState.AUTONOMOUS]: { x: 450, y: 320 },

    // Bottom row - terminal/error states
    [SystemState.SAFETY]: { x: 350, y: 520 },
    [SystemState.SHUTDOWN]: { x: 550, y: 520 },
  }), []);

  // Generate smooth curved transition paths
  const getArrowPath = (fromState, toState) => {
    const start = statePositions[fromState];
    const end = statePositions[toState];
    if (!start || !end) return '';

    const dx = end.x - start.x;
    const dy = end.y - start.y;

    // Create smooth curved paths for professional flowchart look
    const startX = start.x + 75;
    const startY = start.y + 30;
    const endX = end.x;
    const endY = end.y + 30;

    // Calculate control points for smooth curves
    if (Math.abs(dx) > Math.abs(dy)) {
      // Horizontal flow - create gentle S-curve
      const midX = startX + dx / 2;
      const control1X = startX + Math.abs(dx) * 0.3;
      const control2X = endX - Math.abs(dx) * 0.3;
      const controlY1 = startY + (dy > 0 ? 20 : -20);
      const controlY2 = endY + (dy > 0 ? -20 : 20);

      return `M${startX},${startY} C${control1X},${controlY1} ${control2X},${controlY2} ${endX},${endY}`;
    } else {
      // Vertical flow - create gentle curve
      const midY = startY + dy / 2;
      const controlX1 = startX + (dx > 0 ? 20 : -20);
      const controlX2 = endX + (dx > 0 ? -20 : 20);
      const controlY = startY + dy / 2;

      return `M${startX},${startY} C${controlX1},${startY} ${controlX2},${endY} ${endX},${endY}`;
    }
  };

  // Generate all transition paths with enhanced metadata
  const transitionPaths = useMemo(() => {
    const paths = [];
    Object.entries(StateTransitions).forEach(([fromState, toStates]) => {
      toStates.forEach((toState) => {
        paths.push({
          id: `${fromState}-${toState}`,
          path: getArrowPath(fromState, toState),
          fromState,
          toState,
          isActive: currentState === fromState && StateTransitions[fromState].includes(toState),
          isTransitioning: isTransitioning && currentState === fromState,
          isAvailable: StateTransitions[currentState]?.includes(toState)
        });
      });
    });
    return paths;
  }, [currentState, isTransitioning]);

  // Handle state node interactions
  const handleStateClick = (state) => {
    setSelectedState(state);
    if (onStateTransition) {
      const isValid = StateTransitions[currentState]?.includes(state);
      onStateTransition(state, isValid);
    }
  };

  // Get current state display info
  const getCurrentStateInfo = () => {
    const path = [currentState];
    if (currentState === SystemState.AUTONOMOUS && currentSubstate !== AutonomousSubstate.NONE) {
      path.push(currentSubstate);
    }
    if (currentState === SystemState.CALIBRATION && currentCalibrationSubstate !== CalibrationSubstate.NONE) {
      path.push(currentCalibrationSubstate);
    }
    return path.join(' → ');
  };

  const states = Object.values(SystemState);

  return (
    <div className={`relative w-full h-full bg-gradient-to-br from-slate-50 to-slate-100 dark:from-slate-900 dark:to-slate-800 rounded-xl overflow-hidden shadow-lg ${className}`}>
      {/* Professional header */}
      <div className="absolute top-0 left-0 right-0 z-20 bg-white/95 dark:bg-slate-800/95 backdrop-blur-sm border-b border-slate-200 dark:border-slate-700 p-4 shadow-sm">
        <div className="flex items-center justify-between">
          <div className="flex items-center gap-3">
            <div className="flex items-center gap-2">
              <div className={`w-4 h-4 rounded-full ${
                currentState === SystemState.BOOT ? 'bg-blue-500 shadow-lg shadow-blue-500/50' :
                currentState === SystemState.CALIBRATION ? 'bg-yellow-500 shadow-lg shadow-yellow-500/50' :
                currentState === SystemState.IDLE ? 'bg-green-500 shadow-lg shadow-green-500/50' :
                currentState === SystemState.TELEOPERATION ? 'bg-cyan-500 shadow-lg shadow-cyan-500/50' :
                currentState === SystemState.AUTONOMOUS ? 'bg-red-500 shadow-lg shadow-red-500/50' :
                currentState === SystemState.SAFETY ? 'bg-orange-500 shadow-lg shadow-orange-500/50' :
                'bg-gray-500 shadow-lg shadow-gray-500/50'
              } ${isTransitioning ? 'animate-pulse' : ''}`} />
              <div>
                <h3 className="font-bold text-slate-900 dark:text-slate-100 text-lg">State Machine</h3>
                <p className="text-sm text-slate-600 dark:text-slate-400 font-mono">{getCurrentStateInfo()}</p>
              </div>
            </div>
          </div>

          <div className="flex items-center gap-2">
            {isTransitioning && (
              <div className="flex items-center gap-2 px-3 py-1 bg-amber-100 dark:bg-amber-900/30 rounded-full">
                <div className="w-2 h-2 bg-amber-500 rounded-full animate-ping" />
                <span className="text-xs font-medium text-amber-800 dark:text-amber-200">Transitioning</span>
              </div>
            )}
            <div className="text-xs text-slate-500 dark:text-slate-400">
              Interactive Flowchart
            </div>
          </div>
        </div>
      </div>

      {/* Flowchart canvas */}
      <svg
        className="absolute inset-0 w-full h-full"
        style={{ paddingTop: '100px', paddingBottom: '80px' }}
        viewBox="0 0 800 600"
        preserveAspectRatio="xMidYMid meet"
      >
        {/* Subtle grid background */}
        <defs>
          <pattern id="flowchart-grid" width="40" height="40" patternUnits="userSpaceOnUse">
            <path d="M 40 0 L 0 0 0 40" fill="none" stroke="currentColor" strokeWidth="0.5" opacity="0.1"/>
          </pattern>

          {/* Enhanced arrow markers */}
          <marker id="arrowhead-default" markerWidth="12" markerHeight="8" refX="11" refY="4" orient="auto">
            <path d="M0,0 L12,4 L0,8 L3,4 Z" fill="#94a3b8" />
          </marker>
          <marker id="arrowhead-active" markerWidth="12" markerHeight="8" refX="11" refY="4" orient="auto">
            <path d="M0,0 L12,4 L0,8 L3,4 Z" fill="#3b82f6" />
          </marker>
          <marker id="arrowhead-available" markerWidth="12" markerHeight="8" refX="11" refY="4" orient="auto">
            <path d="M0,0 L12,4 L0,8 L3,4 Z" fill="#10b981" />
          </marker>
          <marker id="arrowhead-transitioning" markerWidth="12" markerHeight="8" refX="11" refY="4" orient="auto">
            <path d="M0,0 L12,4 L0,8 L3,4 Z" fill="#f59e0b">
              <animate attributeName="opacity" values="1;0.3;1" dur="1s" repeatCount="indefinite" />
            </path>
          </marker>
        </defs>

        {/* Grid background */}
        <rect width="100%" height="100%" fill="url(#flowchart-grid)" />

        {/* Transition arrows */}
        {transitionPaths.map((transition) => (
          <TransitionArrow
            key={transition.id}
            path={transition.path}
            isActive={transition.isActive}
            isAvailable={transition.isAvailable}
            isTransitioning={transition.isTransitioning}
          />
        ))}

        {/* State nodes */}
        {states.map((state) => (
          <StateNode
            key={state}
            state={state}
            position={statePositions[state]}
            isActive={currentState === state}
            isTransitioning={isTransitioning && currentState === state}
            isHovered={hoveredState === state}
            isSelected={selectedState === state}
            isAvailable={StateTransitions[currentState]?.includes(state)}
            onClick={() => handleStateClick(state)}
            onHover={setHoveredState}
            currentSubstate={state === SystemState.AUTONOMOUS ? currentSubstate : null}
            currentCalibrationSubstate={state === SystemState.CALIBRATION ? currentCalibrationSubstate : null}
          />
        ))}
      </svg>

      {/* Enhanced legend */}
      <div className="absolute bottom-0 left-0 right-0 bg-white/95 dark:bg-slate-800/95 backdrop-blur-sm border-t border-slate-200 dark:border-slate-700 p-3">
        <div className="flex flex-wrap items-center justify-between gap-4">
          <div className="flex flex-wrap gap-6 text-sm">
            <div className="flex items-center gap-2">
              <div className="w-4 h-4 rounded-full bg-blue-500 shadow-md" />
              <span className="text-slate-700 dark:text-slate-300 font-medium">Current</span>
            </div>
            <div className="flex items-center gap-2">
              <div className="w-4 h-4 rounded-full bg-green-500 shadow-md" />
              <span className="text-slate-700 dark:text-slate-300">Available</span>
            </div>
            <div className="flex items-center gap-2">
              <div className="w-4 h-4 rounded-full bg-slate-400 shadow-md" />
              <span className="text-slate-700 dark:text-slate-300">Inactive</span>
            </div>
            <div className="flex items-center gap-2">
              <div className="w-4 h-4 rounded-full bg-amber-500 shadow-md animate-pulse" />
              <span className="text-slate-700 dark:text-slate-300">Transitioning</span>
            </div>
          </div>

          <div className="text-xs text-slate-500 dark:text-slate-400">
            Click states to transition • Hover for details
          </div>
        </div>
      </div>

      {/* Enhanced tooltip */}
      {hoveredState && (
        <div className="absolute z-30 bg-white dark:bg-slate-800 rounded-lg shadow-xl border border-slate-200 dark:border-slate-700 p-4 max-w-sm"
             style={{
               left: statePositions[hoveredState]?.x + 100 || 100,
               top: statePositions[hoveredState]?.y + 50 || 50
             }}>
          <div className="flex items-center gap-2 mb-2">
            <div className={`w-3 h-3 rounded-full ${
              hoveredState === SystemState.BOOT ? 'bg-blue-500' :
              hoveredState === SystemState.CALIBRATION ? 'bg-yellow-500' :
              hoveredState === SystemState.IDLE ? 'bg-green-500' :
              hoveredState === SystemState.TELEOPERATION ? 'bg-cyan-500' :
              hoveredState === SystemState.AUTONOMOUS ? 'bg-red-500' :
              hoveredState === SystemState.SAFETY ? 'bg-orange-500' :
              'bg-gray-500'
            }`} />
            <h4 className="font-semibold text-slate-900 dark:text-slate-100">{hoveredState}</h4>
          </div>
          <p className="text-sm text-slate-600 dark:text-slate-400">
            {hoveredState === SystemState.BOOT && "Initial system state after power-on"}
            {hoveredState === SystemState.CALIBRATION && "Camera and sensor calibration workflow"}
            {hoveredState === SystemState.IDLE && "System ready, waiting for commands"}
            {hoveredState === SystemState.TELEOPERATION && "Manual remote control operation"}
            {hoveredState === SystemState.AUTONOMOUS && "AI-driven autonomous operation"}
            {hoveredState === SystemState.SAFETY && "Emergency safety mode"}
            {hoveredState === SystemState.SHUTDOWN && "System shutdown and cleanup"}
          </p>
          {StateTransitions[hoveredState] && StateTransitions[hoveredState].length > 0 && (
            <div className="mt-2 pt-2 border-t border-slate-200 dark:border-slate-700">
              <p className="text-xs text-slate-500 dark:text-slate-400">
                Can transition to: {StateTransitions[hoveredState].join(', ')}
              </p>
            </div>
          )}
        </div>
      )}
    </div>
  );
};

export default StateTree;
