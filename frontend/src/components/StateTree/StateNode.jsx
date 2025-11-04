import React from 'react';
import { SystemState, AutonomousSubstate, CalibrationSubstate } from '../../config/stateDefinitions';

/**
 * Enhanced StateNode Component - Professional flowchart node
 *
 * Renders a state as a modern rounded rectangle with enhanced visual indicators:
 * - Professional flowchart appearance
 * - Enhanced hover and selection states
 * - Smooth transitions and animations
 * - Status indicators and substate display
 */
const StateNode = ({
  state,
  position,
  isActive,
  isTransitioning,
  isHovered,
  isSelected,
  isAvailable,
  onClick,
  onHover,
  currentSubstate,
  currentCalibrationSubstate
}) => {
  // Enhanced color scheme for professional flowchart appearance
  const getNodeColors = () => {
    if (isActive) {
      return {
        bg: state === SystemState.BOOT ? '#3b82f6' :
            state === SystemState.CALIBRATION ? '#eab308' :
            state === SystemState.IDLE ? '#22c55e' :
            state === SystemState.TELEOPERATION ? '#06b6d4' :
            state === SystemState.AUTONOMOUS ? '#ef4444' :
            state === SystemState.SAFETY ? '#f97316' :
            '#6b7280',
        border: state === SystemState.BOOT ? '#1d4ed8' :
                state === SystemState.CALIBRATION ? '#ca8a04' :
                state === SystemState.IDLE ? '#16a34a' :
                state === SystemState.TELEOPERATION ? '#0891b2' :
                state === SystemState.AUTONOMOUS ? '#dc2626' :
                state === SystemState.SAFETY ? '#ea580c' :
                '#4b5563',
        text: '#ffffff',
        shadow: state === SystemState.BOOT ? '0 0 20px rgba(59, 130, 246, 0.5)' :
                state === SystemState.CALIBRATION ? '0 0 20px rgba(234, 179, 8, 0.5)' :
                state === SystemState.IDLE ? '0 0 20px rgba(34, 197, 94, 0.5)' :
                state === SystemState.TELEOPERATION ? '0 0 20px rgba(6, 182, 212, 0.5)' :
                state === SystemState.AUTONOMOUS ? '0 0 20px rgba(239, 68, 68, 0.5)' :
                state === SystemState.SAFETY ? '0 0 20px rgba(249, 115, 22, 0.5)' :
                '0 0 20px rgba(107, 114, 128, 0.5)'
      };
    } else if (isAvailable) {
      return {
        bg: '#10b981',
        border: '#059669',
        text: '#ffffff',
        shadow: '0 0 15px rgba(16, 185, 129, 0.3)'
      };
    } else {
      return {
        bg: '#f1f5f9',
        border: '#cbd5e1',
        text: '#475569',
        shadow: '0 2px 8px rgba(0, 0, 0, 0.1)'
      };
    }
  };

  const colors = getNodeColors();

  const handleMouseEnter = () => {
    if (onHover) onHover(state);
  };

  const handleMouseLeave = () => {
    if (onHover) onHover(null);
  };

  const handleClick = () => {
    if (isActive || isAvailable) {
      onClick(state);
    }
  };

  return (
    <g>
      {/* Main node */}
      <rect
        x={position.x}
        y={position.y}
        width="150"
        height="60"
        rx="12"
        fill={colors.bg}
        stroke={colors.border}
        strokeWidth="2"
        style={{
          filter: colors.shadow ? `drop-shadow(${colors.shadow})` : 'none',
          cursor: isActive || isAvailable ? 'pointer' : 'default',
          transition: 'all 0.3s ease',
          transform: isActive ? 'scale(1.05)' : isHovered ? 'scale(1.02)' : 'scale(1)',
          transformOrigin: `${position.x + 75}px ${position.y + 30}px`
        }}
        onMouseEnter={handleMouseEnter}
        onMouseLeave={handleMouseLeave}
        onClick={handleClick}
      />

      {/* State name */}
      <text
        x={position.x + 75}
        y={position.y + 25}
        textAnchor="middle"
        fill={colors.text}
        fontSize="12"
        fontWeight="600"
        style={{ pointerEvents: 'none', userSelect: 'none' }}
      >
        {state}
      </text>

      {/* Substate indicator */}
      {(isActive && state === SystemState.AUTONOMOUS && currentSubstate && currentSubstate !== AutonomousSubstate.NONE) && (
        <text
          x={position.x + 75}
          y={position.y + 42}
          textAnchor="middle"
          fill={colors.text}
          fontSize="9"
          fontWeight="500"
          opacity="0.9"
          style={{ pointerEvents: 'none', userSelect: 'none' }}
        >
          {currentSubstate}
        </text>
      )}

      {(isActive && state === SystemState.CALIBRATION && currentCalibrationSubstate && currentCalibrationSubstate !== CalibrationSubstate.NONE) && (
        <text
          x={position.x + 75}
          y={position.y + 42}
          textAnchor="middle"
          fill={colors.text}
          fontSize="9"
          fontWeight="500"
          opacity="0.9"
          style={{ pointerEvents: 'none', userSelect: 'none' }}
        >
          {currentCalibrationSubstate}
        </text>
      )}

      {/* Status indicator dot */}
      <circle
        cx={position.x + 135}
        cy={position.y + 10}
        r="4"
        fill={isActive ? '#ffffff' : isAvailable ? '#10b981' : '#94a3b8'}
        opacity="0.8"
      />

      {/* Transitioning animation overlay */}
      {isTransitioning && (
        <circle
          cx={position.x + 75}
          cy={position.y + 30}
          r="25"
          fill="none"
          stroke="#f59e0b"
          strokeWidth="2"
          opacity="0.6"
        >
          <animate attributeName="r" values="20;30;20" dur="1.5s" repeatCount="indefinite" />
          <animate attributeName="opacity" values="0.6;0.2;0.6" dur="1.5s" repeatCount="indefinite" />
        </circle>
      )}

      {/* Hover effect */}
      {isHovered && !isActive && (
        <rect
          x={position.x - 2}
          y={position.y - 2}
          width="154"
          height="64"
          rx="14"
          fill="none"
          stroke="#3b82f6"
          strokeWidth="2"
          strokeDasharray="5,5"
          opacity="0.7"
        />
      )}
    </g>
  );
};

export default StateNode;
