import { useState } from 'react';
import { SystemState, isValidTransition } from '../../config/stateDefinitions';

/**
 * StateControls Component - Manual state transition controls
 *
 * Provides UI controls for manually requesting state transitions:
 * - Dropdown to select target state
 * - Transition validation
 * - Force transition option
 * - Transition status feedback
 */
const StateControls = ({
  currentState,
  onStateTransition,
  isTransitioning = false,
  className = ""
}) => {
  const [selectedState, setSelectedState] = useState('');
  const [transitionReason, setTransitionReason] = useState('');
  const [forceTransition, setForceTransition] = useState(false);
  const [lastResult, setLastResult] = useState(null);

  // Available target states (excluding current state)
  const availableStates = Object.values(SystemState).filter(state => state !== currentState);

  // Check if selected transition is valid
  const isValidSelection = selectedState && isValidTransition(currentState, selectedState);
  const canTransition = !isTransitioning && selectedState && (isValidSelection || forceTransition);

  const handleStateTransition = async () => {
    if (!canTransition) return;

    try {
      setLastResult(null);
      const reason = transitionReason.trim() || (forceTransition ? 'forced_transition' : 'manual_request');

      await onStateTransition(selectedState, isValidSelection, reason, forceTransition);

      setLastResult({
        success: true,
        message: `Transition to ${selectedState} requested successfully`
      });

      // Clear form on success
      setTransitionReason('');
      setForceTransition(false);

    } catch (error) {
      setLastResult({
        success: false,
        message: error.message || 'Transition request failed'
      });
    }
  };

  const handleStateSelect = (state) => {
    setSelectedState(state);
    setLastResult(null); // Clear previous results
  };

  return (
    <div className={`bg-muted/50 border border-border rounded-lg p-4 ${className}`}>
      <h3 className="font-semibold text-sm text-primary mb-3">Manual State Control</h3>

      {/* Current State Display */}
      <div className="mb-4 p-2 bg-card border border-border rounded">
        <div className="text-xs text-muted-foreground">Current State</div>
        <div className="font-mono text-sm">{currentState}</div>
      </div>

      {/* Target State Selection */}
      <div className="mb-3">
        <label className="block text-xs font-medium text-muted-foreground mb-1">
          Target State
        </label>
        <select
          value={selectedState}
          onChange={(e) => handleStateSelect(e.target.value)}
          className="w-full px-3 py-2 bg-card border border-border rounded text-sm focus:outline-none focus:ring-2 focus:ring-primary"
          disabled={isTransitioning}
        >
          <option value="">Select target state...</option>
          {availableStates.map(state => (
            <option key={state} value={state}>
              {state}
            </option>
          ))}
        </select>
      </div>

      {/* Transition Reason */}
      <div className="mb-3">
        <label className="block text-xs font-medium text-muted-foreground mb-1">
          Reason (Optional)
        </label>
        <input
          type="text"
          value={transitionReason}
          onChange={(e) => setTransitionReason(e.target.value)}
          placeholder="manual_test"
          className="w-full px-3 py-2 bg-card border border-border rounded text-sm focus:outline-none focus:ring-2 focus:ring-primary"
          disabled={isTransitioning}
        />
      </div>

      {/* Force Transition Checkbox */}
      {selectedState && !isValidSelection && (
        <div className="mb-3 p-2 bg-yellow-500/10 border border-yellow-500/20 rounded">
          <label className="flex items-center space-x-2 text-xs">
            <input
              type="checkbox"
              checked={forceTransition}
              onChange={(e) => setForceTransition(e.target.checked)}
              className="rounded border-border"
            />
            <span className="text-yellow-700 dark:text-yellow-300">
              Force invalid transition (⚠️ Use with caution)
            </span>
          </label>
        </div>
      )}

      {/* Transition Validation Status */}
      {selectedState && (
        <div style={{
          marginBottom: '0.75rem',
          padding: '0.5rem',
          borderRadius: '0.25rem',
          fontSize: '0.75rem',
          backgroundColor: isValidSelection
            ? '#14532d'
            : forceTransition
              ? '#7c2d12'
              : '#7f1d1d',
          border: `1px solid ${isValidSelection
            ? '#16a34a'
            : forceTransition
              ? '#ea580c'
              : '#dc2626'}`,
          color: isValidSelection
            ? '#86efac'
            : forceTransition
              ? '#fed7aa'
              : '#fca5a5'
        }}>
          {isValidSelection ? (
            <>✓ Valid transition from {currentState} → {selectedState}</>
          ) : forceTransition ? (
            <>⚠ Force transition: {currentState} → {selectedState}</>
          ) : (
            <>✗ Invalid transition: {currentState} → {selectedState}</>
          )}
        </div>
      )}

      {/* Action Button */}
      <button
        onClick={handleStateTransition}
        disabled={!canTransition}
        style={{
          width: '100%',
          padding: '0.5rem 1rem',
          borderRadius: '0.375rem',
          fontWeight: '500',
          fontSize: '0.875rem',
          border: '1px solid transparent',
          cursor: canTransition ? 'pointer' : 'not-allowed',
          backgroundColor: canTransition
            ? forceTransition
              ? '#ea580c'
              : '#3b82f6'
            : '#6b7280',
          color: canTransition
            ? 'white'
            : '#9ca3af',
          opacity: canTransition ? 1 : 0.5
        }}
        onMouseOver={(e) => {
          if (canTransition) {
            e.target.style.backgroundColor = forceTransition ? '#dc2626' : '#2563eb';
          }
        }}
        onMouseOut={(e) => {
          if (canTransition) {
            e.target.style.backgroundColor = forceTransition ? '#ea580c' : '#3b82f6';
          }
        }}
      >
        {isTransitioning ? (
          <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'center', gap: '0.5rem' }}>
            <div style={{
              width: '1rem',
              height: '1rem',
              border: '2px solid currentColor',
              borderTopColor: 'transparent',
              borderRadius: '50%',
              animation: 'spin 1s linear infinite'
            }} />
            <span>Transitioning...</span>
          </div>
        ) : (
          `${forceTransition ? 'Force ' : ''}Transition to ${selectedState || 'State'}`
        )}
      </button>

      {/* Result Feedback */}
      {lastResult && (
        <div style={{
          marginTop: '0.75rem',
          padding: '0.5rem',
          borderRadius: '0.25rem',
          fontSize: '0.75rem',
          backgroundColor: lastResult.success ? '#14532d' : '#7f1d1d',
          border: `1px solid ${lastResult.success ? '#16a34a' : '#dc2626'}`,
          color: lastResult.success ? '#86efac' : '#fca5a5'
        }}>
          {lastResult.success ? '✓' : '✗'} {lastResult.message}
        </div>
      )}

      {/* Quick Actions */}
      <div style={{
        marginTop: '1rem',
        paddingTop: '0.75rem',
        borderTop: '1px solid #4b5563'
      }}>
        <div style={{
          fontSize: '0.75rem',
          fontWeight: '500',
          color: '#9ca3af',
          marginBottom: '0.5rem'
        }}>Quick Actions</div>
        <div style={{
          display: 'grid',
          gridTemplateColumns: '1fr 1fr',
          gap: '0.5rem'
        }}>
          <button
            onClick={() => handleStateSelect(SystemState.IDLE)}
            style={{
              padding: '0.25rem 0.5rem',
              backgroundColor: '#2563eb',
              color: 'white',
              border: 'none',
              borderRadius: '0.25rem',
              fontSize: '0.75rem',
              cursor: isTransitioning ? 'not-allowed' : 'pointer',
              opacity: isTransitioning ? 0.5 : 1
            }}
            onMouseOver={(e) => {
              if (!isTransitioning) e.target.style.backgroundColor = '#1d4ed8';
            }}
            onMouseOut={(e) => {
              if (!isTransitioning) e.target.style.backgroundColor = '#2563eb';
            }}
            disabled={isTransitioning}
          >
            To Idle
          </button>
          <button
            onClick={() => handleStateSelect(SystemState.AUTONOMOUS)}
            style={{
              padding: '0.25rem 0.5rem',
              backgroundColor: '#dc2626',
              color: 'white',
              border: 'none',
              borderRadius: '0.25rem',
              fontSize: '0.75rem',
              cursor: isTransitioning ? 'not-allowed' : 'pointer',
              opacity: isTransitioning ? 0.5 : 1
            }}
            onMouseOver={(e) => {
              if (!isTransitioning) e.target.style.backgroundColor = '#b91c1c';
            }}
            onMouseOut={(e) => {
              if (!isTransitioning) e.target.style.backgroundColor = '#dc2626';
            }}
            disabled={isTransitioning}
          >
            To Autonomous
          </button>
        </div>
      </div>
    </div>
  );
};

export default StateControls;
