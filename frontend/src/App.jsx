import { useState } from 'react';
import { useROS } from './hooks/useROS';
import { useStateMachine } from './hooks/useStateMachine';
import { useSafetySystem } from './hooks/useSafetySystem';
import { getStatusText } from './utils/rosbridge';
import { StateTransitions, SystemState } from './config/stateDefinitions';
import StateTree, { StateControls } from './components/StateTree';
import { SafetyDashboard } from './components/SafetyDashboard';
import { SafetyTestPanel } from './components/SafetyTestPanel';

/**
 * Main Application Component - URC 2026 Mars Rover Control Interface
 *
 * This is the root component of the React-based web interface for controlling
 * and monitoring the URC 2026 Mars Rover. It provides real-time visualization,
 * state machine control, and system monitoring capabilities.
 *
 * Features:
 * - Real-time ROS2 communication via WebSocket
 * - Hierarchical state machine visualization and control
 * - 3D rover and environment visualization
 * - System diagnostics and health monitoring
 * - Mission execution monitoring and control
 *
 * The application maintains connection to the rover's ROS2 system through
 * WebSocket bridge and provides both monitoring and control capabilities.
 *
 * @component
 * @returns {JSX.Element} The main application interface
 *
 * @example
 * // The App component is automatically rendered by React
 * // No props required - all state is managed internally
 * <App />
 */
function App() {
  const [activeTab, setActiveTab] = useState('state-machine');
  const [testRunning, setTestRunning] = useState(false);
  const [testResults, setTestResults] = useState([]);

  const {
    isConnected,
    connectionStatus,
    reconnectAttempts,
    lastError,
    connect,
    disconnect,
    maxReconnectAttempts,
    ros
  } = useROS({
    url: 'ws://localhost:9090',
    reconnectInterval: 3000,
    maxReconnectAttempts: 10
  });

  // State machine hook
  const {
    currentState,
    currentSubstate,
    currentSubSubstate,
    currentCalibrationSubstate,
    isTransitioning,
    lastTransition,
    requestStateTransition
  } = useStateMachine(ros);

  // Safety system hook
  const {
    safetyStatus,
    activeAlerts,
    systemHealth,
    emergencyStatus,
    runSafetyTest
  } = useSafetySystem(ros);

  // Simple state transition test function
  const testStateTransition = async (targetState) => {
    if (testRunning) return;

    setTestRunning(true);
    setTestResults([]);

    const addResult = (step, success, message) => {
      setTestResults(prev => [...prev, { step, success, message, timestamp: new Date() }]);
    };

    addResult(`Testing ${currentState} → ${targetState}`, false, 'Initiating transition...');

    try {
      // Check if transition is valid first
      const isValidTransition = StateTransitions[currentState]?.includes(targetState);

      if (!isValidTransition) {
        addResult(`Testing ${currentState} → ${targetState}`, false, `❌ Invalid transition: ${currentState} cannot go to ${targetState}`);
        setTestRunning(false);
        return;
      }

      // Attempt the transition
      await requestStateTransition(targetState, 'dashboard_test');

      // Check if we're in demo mode
      const isDemoMode = !ros || !ros.isConnected;

      // If we get here without error, transition was successful
      addResult(
        `Testing ${currentState} → ${targetState}`,
        true,
        `✅ ${isDemoMode ? 'Demo mode' : 'ROS'} transition successful: ${currentState} → ${targetState}`
      );

    } catch (error) {
      addResult(`Testing ${currentState} → ${targetState}`, false, `❌ Transition failed: ${error.message}`);
    } finally {
      setTestRunning(false);
    }
  };

  return (
    <div className="app-container">
      {/* Header */}
      <header className="app-header">
        <div className="header-left">
          <h1 className="app-title">URC 2026 Testing Interface</h1>
          <div className="connection-status">
            <div
              className={`status-dot ${
                isConnected ? 'status-connected' : 'status-disconnected'
              }`}
            />
            <span>{getStatusText(connectionStatus)}</span>
          </div>
          {isTransitioning && (
            <div className="connection-status">
              <div className="status-dot status-transitioning" />
              <span>Transitioning...</span>
            </div>
          )}
        </div>

        <div className="header-left">
          <span className="text-muted">Reconnect attempts: {reconnectAttempts}/{maxReconnectAttempts}</span>
          <div style={{ display: 'flex', gap: '0.5rem' }}>
            <button
              onClick={connect}
              className="btn btn-primary"
              disabled={isConnected}
            >
              Connect
            </button>
            <button
              onClick={disconnect}
              className="btn btn-destructive"
              disabled={!isConnected}
            >
              Disconnect
            </button>
          </div>
        </div>

        {lastTransition && (
          <div style={{
            marginTop: '0.5rem',
            padding: '0.5rem',
            backgroundColor: '#374151',
            border: '1px solid #4b5563',
            borderRadius: '0.25rem',
            fontSize: '0.75rem'
          }}>
            <span style={{ fontWeight: '500' }}>Last Transition:</span>{' '}
            {lastTransition.fromState} → {lastTransition.toState}
            {lastTransition.reason && ` (${lastTransition.reason})`}
            <span className={`ml-2 ${lastTransition.success ? 'text-success' : 'text-error'}`}>
              {lastTransition.success ? '✓' : '✗'}
            </span>
          </div>
        )}

        {lastError && (
          <div style={{
            marginTop: '0.5rem',
            padding: '0.5rem',
            backgroundColor: '#451a1a',
            border: '1px solid #7f1d1d',
            borderRadius: '0.25rem',
            fontSize: '0.875rem',
            color: '#fca5a5'
          }}>
            Connection Error: {lastError.message || 'Unknown error'}
          </div>
        )}
      </header>

      {/* Main Content */}
      <main style={{ flex: 1, padding: '1.5rem' }}>
        <div style={{ maxWidth: '1400px', margin: '0 auto' }}>
          {/* Tab Navigation */}
          <div className="tab-navigation" style={{
            display: 'flex',
            backgroundColor: '#2a2a2a',
            borderRadius: '0.5rem 0.5rem 0 0',
            border: '1px solid #404040',
            borderBottom: 'none',
            marginBottom: 0
          }}>
            <button
              onClick={() => setActiveTab('state-machine')}
              className={`tab-button ${activeTab === 'state-machine' ? 'active' : ''}`}
              style={{
                flex: 1,
                padding: '1rem',
                backgroundColor: activeTab === 'state-machine' ? '#1a1a1a' : 'transparent',
                color: activeTab === 'state-machine' ? '#60a5fa' : '#9ca3af',
                border: 'none',
                borderRadius: activeTab === 'state-machine' ? '0.5rem 0 0 0' : '0',
                cursor: 'pointer',
                fontSize: '1rem',
                fontWeight: activeTab === 'state-machine' ? '600' : '500',
                transition: 'all 0.2s ease',
                borderBottom: activeTab === 'state-machine' ? '2px solid #60a5fa' : 'none'
              }}
            >
              🎯 State Machine
            </button>
            <button
              onClick={() => setActiveTab('camera-feed')}
              className={`tab-button ${activeTab === 'camera-feed' ? 'active' : ''}`}
              style={{
                flex: 1,
                padding: '1rem',
                backgroundColor: activeTab === 'camera-feed' ? '#1a1a1a' : 'transparent',
                color: activeTab === 'camera-feed' ? '#60a5fa' : '#9ca3af',
                border: 'none',
                cursor: 'pointer',
                fontSize: '1rem',
                fontWeight: activeTab === 'camera-feed' ? '600' : '500',
                transition: 'all 0.2s ease',
                borderBottom: activeTab === 'camera-feed' ? '2px solid #60a5fa' : 'none'
              }}
            >
              📹 Camera Feed
            </button>
            <button
              onClick={() => setActiveTab('3d-visualization')}
              className={`tab-button ${activeTab === '3d-visualization' ? 'active' : ''}`}
              style={{
                flex: 1,
                padding: '1rem',
                backgroundColor: activeTab === '3d-visualization' ? '#1a1a1a' : 'transparent',
                color: activeTab === '3d-visualization' ? '#60a5fa' : '#9ca3af',
                border: 'none',
                cursor: 'pointer',
                fontSize: '1rem',
                fontWeight: activeTab === '3d-visualization' ? '600' : '500',
                transition: 'all 0.2s ease',
                borderBottom: activeTab === '3d-visualization' ? '2px solid #60a5fa' : 'none'
              }}
            >
              🌐 3D Visualization
            </button>
            <button
              onClick={() => setActiveTab('safety')}
              className={`tab-button ${activeTab === 'safety' ? 'active' : ''}`}
              style={{
                flex: 1,
                padding: '1rem',
                backgroundColor: activeTab === 'safety' ? '#1a1a1a' : 'transparent',
                color: activeTab === 'safety' ? '#60a5fa' : '#9ca3af',
                border: 'none',
                borderRadius: activeTab === 'safety' ? '0 0.5rem 0 0' : '0',
                cursor: 'pointer',
                fontSize: '1rem',
                fontWeight: activeTab === 'safety' ? '600' : '500',
                transition: 'all 0.2s ease',
                borderBottom: activeTab === 'safety' ? '2px solid #60a5fa' : 'none'
              }}
            >
              🛡️ Safety Testing
            </button>
          </div>

          {/* Tab Content */}
          <div className="tab-content" style={{
            minHeight: '600px',
            overflow: 'hidden'
          }}>
            {activeTab === 'state-machine' && (
              <div style={{ padding: '1.5rem', height: '100%' }}>
                {/* State Machine Panel - Full width now */}
                <div style={{
                  backgroundColor: '#2a2a2a',
                  border: '1px solid #404040',
                  borderRadius: '0.5rem',
                  padding: '1rem',
                  display: 'flex',
                  flexDirection: 'column',
                  height: '100%'
                }}>
                  <div className="panel-header">
                    <h2>State Machine Control</h2>
                  </div>
                  <div style={{ display: 'flex', flexDirection: 'column', gap: '1rem', height: '100%', flex: 1 }}>
                    {/* State Tree Visualization - Full width now */}
                    <div style={{ flex: 1, minHeight: '500px', position: 'relative' }}>
                      <StateTree
                        currentState={currentState}
                        currentSubstate={currentSubstate}
                        currentSubSubstate={currentSubSubstate}
                        currentCalibrationSubstate={currentCalibrationSubstate}
                        onStateTransition={async (targetState, isValid) => {
                          if (!isValid) {
                            console.warn(`Invalid transition requested: ${currentState} -> ${targetState}`);
                            return;
                          }

                          try {
                            console.log(`Requesting transition: ${currentState} -> ${targetState}`);
                            await requestStateTransition(targetState, 'frontend_request');
                            console.log('Transition request sent successfully');
                          } catch (error) {
                            console.error('Transition request failed:', error);
                          }
                        }}
                        className=""
                        isTransitioning={isTransitioning}
                      />
                    </div>

                    {/* State Controls */}
                    <div style={{ flexShrink: 0 }}>
                      <StateControls
                        currentState={currentState}
                        isTransitioning={isTransitioning}
                        onStateTransition={async (targetState, isValid, reason = 'frontend_request', force = false) => {
                          try {
                            console.log(`Requesting ${force ? 'forced ' : ''}transition: ${currentState} -> ${targetState}`);
                            await requestStateTransition(targetState, reason, force);
                            console.log('Transition request sent successfully');
                          } catch (error) {
                            console.error('Transition request failed:', error);
                            throw error; // Re-throw for StateControls to handle
                          }
                        }}
                      />
                    </div>
                  </div>
                </div>
              </div>
            )}

            {activeTab === 'camera-feed' && (
              <div style={{ padding: '1.5rem', height: '100%' }}>
                {/* Camera Feed Panel - Full width */}
                <div style={{
                  backgroundColor: '#2a2a2a',
                  border: '1px solid #404040',
                  borderRadius: '0.5rem',
                  padding: '1rem',
                  height: '100%'
                }}>
                  <div className="panel-header">
                    <h2>Camera Feed Stream</h2>
                  </div>
                  <div className="video-container" style={{ height: '500px', marginTop: '1rem' }}>
                    <div className="video-placeholder">
                      Video Stream
                      <br />
                      <small>Adaptive quality, bandwidth monitoring</small>
                      <br />
                      <small style={{ marginTop: '1rem', display: 'block', color: '#60a5fa' }}>
                        📡 Connect to ROS for live camera feed
                      </small>
                    </div>
                  </div>
                  <div style={{ marginTop: '1rem', padding: '1rem', backgroundColor: '#1a1a1a', borderRadius: '0.5rem' }}>
                    <h3 style={{ color: '#60a5fa', marginBottom: '0.5rem' }}>📊 Stream Statistics</h3>
                    <div className="camera-stats-grid">
                      <div className="camera-stat-item">
                        <div className="camera-stat-label">Resolution</div>
                        <div className="camera-stat-value" style={{ color: '#60a5fa' }}>1920x1080</div>
                      </div>
                      <div className="camera-stat-item">
                        <div className="camera-stat-label">Frame Rate</div>
                        <div className="camera-stat-value" style={{ color: '#22c55e' }}>30 FPS</div>
                      </div>
                      <div className="camera-stat-item">
                        <div className="camera-stat-label">Bandwidth</div>
                        <div className="camera-stat-value" style={{ color: '#eab308' }}>2.4 MB/s</div>
                      </div>
                      <div className="camera-stat-item">
                        <div className="camera-stat-label">Latency</div>
                        <div className="camera-stat-value" style={{ color: '#10b981' }}>45ms</div>
                      </div>
                    </div>
                  </div>
                </div>
              </div>
            )}

            {activeTab === '3d-visualization' && (
              <div style={{ padding: '1.5rem', height: '100%' }}>
                {/* 3D Visualization Panel - Full width */}
                <div style={{
                  backgroundColor: '#2a2a2a',
                  border: '1px solid #404040',
                  borderRadius: '0.5rem',
                  padding: '1rem',
                  height: '100%'
                }}>
                  <div className="panel-header">
                    <h2>3D Scene Visualization</h2>
                  </div>
                  <div className="scene-container" style={{ height: '500px', marginTop: '1rem' }}>
                    <div className="scene-placeholder">
                      3D Scene (Three.js)
                      <br />
                      <small>Map, navigation paths, robot pose</small>
                      <br />
                      <small style={{ marginTop: '1rem', display: 'block', color: '#60a5fa' }}>
                        🌐 Interactive 3D environment for mission planning
                      </small>
                    </div>
                  </div>
                  <div style={{ marginTop: '1rem', padding: '1rem', backgroundColor: '#1a1a1a', borderRadius: '0.5rem' }}>
                    <h3 style={{ color: '#60a5fa', marginBottom: '0.5rem' }}>🎮 3D Controls</h3>
                    <div className="viz-controls-grid">
                      <div className="viz-control-item">
                        <div className="viz-control-label">Navigation</div>
                        <div className="viz-control-value">Mouse: Rotate • Wheel: Zoom • Shift+Click: Pan</div>
                      </div>
                      <div className="viz-control-item">
                        <div className="viz-control-label">View Modes</div>
                        <div className="viz-control-value">Top • Side • Front • Isometric</div>
                      </div>
                      <div className="viz-control-item">
                        <div className="viz-control-label">Layers</div>
                        <div className="viz-control-value">Map • Robot • Path • Obstacles</div>
                      </div>
                    </div>
                  </div>
                </div>
              </div>
            )}

            {activeTab === 'safety' && (
              <div style={{ padding: '1.5rem', height: '100%' }}>
                <div className="safety-panels-container">
                  {/* Safety Dashboard */}
                  <div className="safety-dashboard-panel">
                    <SafetyDashboard
                      safetyStatus={safetyStatus}
                      activeAlerts={activeAlerts}
                      systemHealth={systemHealth}
                      emergencyStatus={emergencyStatus}
                      isConnected={isConnected}
                    />
                  </div>

                  {/* Safety Test Panel */}
                  <div className="safety-test-panel">
                    <SafetyTestPanel
                      onRunTest={runSafetyTest}
                      isConnected={isConnected}
                      currentState={currentState}
                    />
                  </div>
                </div>
              </div>
            )}
          </div>

          {/* State Transition Test Panel */}
          <div className="test-panel">
            <div className="panel-header" style={{ marginBottom: '1rem', display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
              <h2>State Transition Tests</h2>
              <div style={{ display: 'flex', alignItems: 'center', gap: '1rem', fontSize: '0.875rem' }}>
                <div style={{ display: 'flex', alignItems: 'center', gap: '0.5rem', color: '#9ca3af' }}>
                  <span>Current: <strong style={{ color: '#60a5fa' }}>{currentState}</strong></span>
                  {!isConnected && (
                    <span style={{
                      backgroundColor: '#eab308',
                      color: '#92400e',
                      padding: '0.125rem 0.5rem',
                      borderRadius: '0.25rem',
                      fontSize: '0.75rem',
                      fontWeight: '500'
                    }}>
                      DEMO MODE
                    </span>
                  )}
                </div>
                {testRunning && (
                  <div className="test-status-indicator">
                    <div className="test-status-dot" />
                    Testing...
                  </div>
                )}
              </div>
            </div>

            <div style={{ marginBottom: '1rem' }}>
              <p style={{ color: '#9ca3af', fontSize: '0.875rem', marginBottom: '0.5rem' }}>
                Click any button below to test transitioning from <strong style={{ color: '#60a5fa' }}>{currentState}</strong> to that state:
                {!isConnected && <span style={{ color: '#eab308', marginLeft: '0.5rem' }}>
                  (Demo mode - simulates transitions without ROS backend)
                </span>}
              </p>
              <div style={{ fontSize: '0.75rem', color: '#6b7280', marginTop: '0.25rem' }}>
                Available transitions: {StateTransitions[currentState]?.join(', ') || 'None'}
              </div>

              {/* Quick transition helper */}
              {(currentState === SystemState.BOOT || currentState === SystemState.CALIBRATION) && (
                <div style={{
                  marginTop: '0.75rem',
                  padding: '0.75rem',
                  backgroundColor: '#1e293b',
                  border: '1px solid #334155',
                  borderRadius: '0.375rem'
                }}>
                  <div style={{ fontSize: '0.875rem', color: '#60a5fa', fontWeight: '500', marginBottom: '0.5rem' }}>
                    💡 Tip: AUTONOMOUS and TELEOPERATION are only available from IDLE state
                  </div>
                  <button
                    onClick={() => testStateTransition(SystemState.IDLE)}
                    disabled={testRunning}
                    style={{
                      padding: '0.375rem 0.75rem',
                      backgroundColor: '#22c55e',
                      color: 'white',
                      border: 'none',
                      borderRadius: '0.25rem',
                      fontSize: '0.75rem',
                      cursor: testRunning ? 'not-allowed' : 'pointer',
                      opacity: testRunning ? 0.6 : 1
                    }}
                  >
                    🚀 Go to IDLE first
                  </button>
                </div>
              )}
            </div>

            <div className="test-grid">
              {Object.values(SystemState).map((targetState) => {
                const isCurrentState = targetState === currentState;
                const isValidTransition = StateTransitions[currentState]?.includes(targetState);
                const canTest = !isCurrentState && isValidTransition;

                return (
                  <button
                    key={targetState}
                    className="test-button"
                    onClick={() => canTest && testStateTransition(targetState)}
                    disabled={testRunning || !canTest}
                    style={{
                      opacity: (!canTest || testRunning) ? 0.5 : 1,
                      cursor: canTest && !testRunning ? 'pointer' : 'not-allowed',
                      backgroundColor: isCurrentState ? '#374151' : canTest ? '#1f2937' : '#111827'
                    }}
                  >
                    <div className="test-button-title">
                      {targetState === SystemState.BOOT && '🔄 Boot'}
                      {targetState === SystemState.CALIBRATION && '📐 Calibration'}
                      {targetState === SystemState.IDLE && '⏸️ Idle'}
                      {targetState === SystemState.TELEOPERATION && '🎮 Teleop'}
                      {targetState === SystemState.AUTONOMOUS && '🤖 Autonomous'}
                      {targetState === SystemState.SAFETY && '⚠️ Safety'}
                      {targetState === SystemState.SHUTDOWN && '🛑 Shutdown'}
                      {isCurrentState && ' (Current)'}
                    </div>
                    <div className="test-button-desc">
                      {isCurrentState && 'Already in this state'}
                      {!isValidTransition && !isCurrentState && 'Invalid transition'}
                      {canTest && `Test ${currentState} → ${targetState}`}
                    </div>
                  </button>
                );
              })}
            </div>

            {/* Latest Test Result */}
            {testResults.length > 0 && (
              <div style={{
                marginTop: '1.5rem',
                padding: '1rem',
                backgroundColor: '#1a1a1a',
                border: '1px solid #404040',
                borderRadius: '0.5rem'
              }}>
                <h3 style={{ color: '#60a5fa', marginBottom: '0.75rem', fontSize: '1rem' }}>
                  🧪 Latest Test Result
                </h3>
                {(() => {
                  const latestResult = testResults[testResults.length - 1];
                  return (
                    <div
                      style={{
                        display: 'flex',
                        alignItems: 'center',
                        gap: '0.75rem',
                        padding: '0.75rem',
                        backgroundColor: latestResult.success ? '#14532d' : '#7f1d1d',
                        borderRadius: '0.25rem',
                        border: `1px solid ${latestResult.success ? '#16a34a' : '#dc2626'}`,
                        fontSize: '0.875rem'
                      }}
                    >
                      <span style={{
                        fontSize: '1.25rem',
                        color: latestResult.success ? '#10b981' : '#ef4444'
                      }}>
                        {latestResult.success ? '✅' : '❌'}
                      </span>
                      <div style={{ flex: 1 }}>
                        <div style={{
                          fontWeight: '600',
                          color: latestResult.success ? '#86efac' : '#fca5a5',
                          marginBottom: '0.25rem'
                        }}>
                          {latestResult.step}
                        </div>
                        <div style={{
                          fontSize: '0.75rem',
                          color: latestResult.success ? '#6ee7b7' : '#f87171'
                        }}>
                          {latestResult.message}
                        </div>
                      </div>
                      <div style={{
                        fontSize: '0.75rem',
                        color: '#9ca3af',
                        textAlign: 'right'
                      }}>
                        {latestResult.timestamp.toLocaleTimeString()}
                      </div>
                    </div>
                  );
                })()}

                {testResults.length > 1 && (
                  <div style={{ marginTop: '1rem', textAlign: 'center' }}>
                    <span style={{ fontSize: '0.75rem', color: '#9ca3af' }}>
                      {testResults.length} total tests run
                    </span>
                  </div>
                )}
              </div>
            )}
          </div>
        </div>
      </main>

      {/* Footer */}
      <footer className="app-footer">
        <div>
          URC 2026 Autonomous Rover Testing Interface
        </div>
        <div className="footer-stats">
          <span>Bandwidth: <span className="text-success">0.0 KB/s</span></span>
          <span>FPS: <span className="text-success">60</span></span>
          <span>Latency: <span className="text-success">50ms</span></span>
        </div>
      </footer>
    </div>
  );
}

export default App;
