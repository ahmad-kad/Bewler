import React from 'react';

/**
 * Enhanced TransitionArrow Component - Professional flowchart arrows
 *
 * Renders smooth curved arrows with multiple visual states for the flowchart:
 * - Default: Subtle gray arrows
 * - Active: Bright blue arrows for current transitions
 * - Available: Green arrows for possible transitions
 * - Transitioning: Animated orange arrows with pulsing effects
 */
const TransitionArrow = ({
  path,
  isActive = false,
  isAvailable = false,
  isTransitioning = false
}) => {
  // Determine arrow appearance based on state
  const getArrowStyle = () => {
    if (isActive) {
      return {
        stroke: '#3b82f6',
        strokeWidth: 3,
        opacity: 1,
        markerEnd: 'url(#arrowhead-active)'
      };
    } else if (isAvailable) {
      return {
        stroke: '#10b981',
        strokeWidth: 2.5,
        opacity: 0.8,
        markerEnd: 'url(#arrowhead-available)'
      };
    } else if (isTransitioning) {
      return {
        stroke: '#f59e0b',
        strokeWidth: 3,
        opacity: 0.9,
        markerEnd: 'url(#arrowhead-transitioning)'
      };
    } else {
      return {
        stroke: '#94a3b8',
        strokeWidth: 2,
        opacity: 0.4,
        markerEnd: 'url(#arrowhead-default)'
      };
    }
  };

  const style = getArrowStyle();

  return (
    <g className="transition-arrow">
      {/* Main arrow path */}
      <path
        d={path}
        fill="none"
        stroke={style.stroke}
        strokeWidth={style.strokeWidth}
        opacity={style.opacity}
        style={{
          transition: 'all 0.3s ease-in-out',
          filter: isActive ? 'drop-shadow(0 0 4px rgba(59, 130, 246, 0.5))' :
                  isAvailable ? 'drop-shadow(0 0 3px rgba(16, 185, 129, 0.4))' :
                  isTransitioning ? 'drop-shadow(0 0 4px rgba(245, 158, 11, 0.5))' : 'none'
        }}
        markerEnd={style.markerEnd}
      />

      {/* Flow animation for active/transitioning states */}
      {(isActive || isTransitioning) && (
        <circle
          r="4"
          fill={isActive ? '#3b82f6' : '#f59e0b'}
          opacity="0.8"
        >
          <animateMotion
            dur={isTransitioning ? "1.5s" : "2s"}
            repeatCount="indefinite"
            path={path}
          />
        </circle>
      )}

      {/* Dashed line effect for available transitions */}
      {isAvailable && (
        <path
          d={path}
          fill="none"
          stroke="#10b981"
          strokeWidth="1"
          strokeDasharray="8,4"
          opacity="0.3"
          style={{
            animation: 'dash-flow 2s linear infinite'
          }}
        />
      )}
    </g>
  );
};

export default TransitionArrow;
