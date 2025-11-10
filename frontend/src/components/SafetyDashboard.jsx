import React from 'react';

/**
 * Safety Dashboard Component
 *
 * Displays comprehensive safety system monitoring including:
 * - Overall safety status
 * - Active safety alerts
 * - System health status
 * - Emergency status
 */
export const SafetyDashboard = ({
  safetyStatus,
  activeAlerts,
  systemHealth,
  emergencyStatus,
  isConnected
}) => {
  const getSafetyLevelColor = (level) => {
    switch (level?.toUpperCase()) {
      case 'NORMAL': return '#10b981'; // green
      case 'WARNING': return '#f59e0b'; // amber
      case 'CRITICAL': return '#f97316'; // orange
      case 'EMERGENCY': return '#ef4444'; // red
      default: return '#6b7280'; // gray
    }
  };

  const getAlertLevelColor = (level) => {
    switch (level?.toUpperCase()) {
      case 'INFO': return '#3b82f6'; // blue
      case 'WARNING': return '#f59e0b'; // amber
      case 'CRITICAL': return '#f97316'; // orange
      case 'EMERGENCY': return '#ef4444'; // red
      default: return '#6b7280'; // gray
    }
  };

  const getSystemStatusColor = (status) => {
    switch (status?.toUpperCase()) {
      case 'HEALTHY': return '#10b981'; // green
      case 'DEGRADED': return '#f59e0b'; // amber
      case 'FAILED': return '#ef4444'; // red
      case 'UNKNOWN': return '#6b7280'; // gray
      default: return '#6b7280'; // gray
    }
  };

  const formatTime = (timestamp) => {
    if (!timestamp) return 'Unknown';
    const date = new Date(timestamp * 1000);
    return date.toLocaleTimeString();
  };

  const formatAge = (timestamp) => {
    if (!timestamp) return 'Unknown';
    const now = Date.now() / 1000;
    const age = now - timestamp;
    if (age < 60) return `${Math.round(age)}s ago`;
    if (age < 3600) return `${Math.round(age / 60)}m ago`;
    return `${Math.round(age / 3600)}h ago`;
  };

  return (
    <div className="safety-dashboard">
      {/* Connection Status */}
      {!isConnected && (
        <div className="connection-warning">
          ⚠️ ROS connection required for live safety monitoring
        </div>
      )}

      <div className="safety-dashboard-grid">
        {/* Safety Status Panel */}
        <div className="safety-panel">
          <h3>🚨 Safety Status</h3>
          <div className="safety-status-overview">
            <div className="safety-indicator">
              <div
                className={`safety-dot ${safetyStatus?.is_safe ? 'safe' : 'unsafe'}`}
                style={{ backgroundColor: getSafetyLevelColor(safetyStatus?.safety_level) }}
              />
              <span className="safety-text">
                {safetyStatus?.is_safe ? 'SAFE' : 'UNSAFE'}
              </span>
            </div>
            <div className="safety-level">
              Level: <strong style={{ color: getSafetyLevelColor(safetyStatus?.safety_level) }}>
                {safetyStatus?.safety_level || 'UNKNOWN'}
              </strong>
            </div>
            {safetyStatus?.active_triggers?.length > 0 && (
              <div className="safety-triggers">
                <h4>Active Triggers ({safetyStatus.active_triggers.length}):</h4>
                <ul>
                  {safetyStatus.active_triggers.map((trigger, idx) => (
                    <li key={idx}>{trigger}</li>
                  ))}
                </ul>
              </div>
            )}
            {safetyStatus?.battery_level && (
              <div className="safety-metrics">
                <div className="metric">
                  <span>Battery:</span>
                  <span>{(safetyStatus.battery_level * 100).toFixed(1)}%</span>
                </div>
                {safetyStatus.temperature && (
                  <div className="metric">
                    <span>Temp:</span>
                    <span>{safetyStatus.temperature}°C</span>
                  </div>
                )}
              </div>
            )}
          </div>
        </div>

        {/* Active Alerts Panel */}
        <div className="safety-panel">
          <h3>⚠️ Active Alerts ({activeAlerts?.length || 0})</h3>
          <div className="alerts-container">
            {activeAlerts && activeAlerts.length > 0 ? (
              activeAlerts.slice(0, 10).map((alert) => (
                <div
                  key={alert.alert_id}
                  className="alert-item"
                  style={{ borderLeftColor: getAlertLevelColor(alert.level) }}
                >
                  <div className="alert-header">
                    <span className="alert-level" style={{ color: getAlertLevelColor(alert.level) }}>
                      {alert.level}
                    </span>
                    <span className="alert-source">{alert.source}</span>
                    <span className="alert-time">{formatTime(alert.timestamp)}</span>
                  </div>
                  <div className="alert-message">{alert.message}</div>
                  <div className="alert-context">
                    {alert.context?.trigger_type && (
                      <span>Type: {alert.context.trigger_type}</span>
                    )}
                  </div>
                </div>
              ))
            ) : (
              <div className="no-alerts">No active alerts</div>
            )}
          </div>
        </div>

        {/* System Health Panel */}
        <div className="safety-panel">
          <h3>💚 System Health</h3>
          <div className="health-overview">
            {systemHealth?.systems && (
              <>
                <div className="health-summary">
                  <div className="health-count healthy">
                    {Object.values(systemHealth.systems).filter(s => s.status === 'HEALTHY').length} Healthy
                  </div>
                  <div className="health-count degraded">
                    {Object.values(systemHealth.systems).filter(s => s.status === 'DEGRADED').length} Degraded
                  </div>
                  <div className="health-count failed">
                    {Object.values(systemHealth.systems).filter(s => s.status === 'FAILED').length} Failed
                  </div>
                </div>
                <div className="health-details">
                  {Object.entries(systemHealth.systems).map(([name, health]) => (
                    <div key={name} className="health-item">
                      <div className="system-name">{name.replace(/_/g, ' ')}</div>
                      <div className="system-status">
                        <span
                          className="status-indicator"
                          style={{ backgroundColor: getSystemStatusColor(health.status) }}
                        />
                        <span className="status-text">{health.status}</span>
                      </div>
                      <div className="health-score">
                        {(health.health_score * 100).toFixed(0)}%
                      </div>
                      <div className="last-update">
                        {formatAge(health.last_update)}
                      </div>
                    </div>
                  ))}
                </div>
              </>
            )}
          </div>
        </div>

        {/* Emergency Status Panel */}
        <div className="safety-panel">
          <h3>🚑 Emergency Status</h3>
          <div className="emergency-status">
            {emergencyStatus?.emergency_active ? (
              <div className="emergency-active">
                <div className="emergency-indicator">🚨 EMERGENCY ACTIVE</div>
                <div className="emergency-details">
                  <p>{emergencyStatus.message || 'Emergency stop is active'}</p>
                  <p>Recovery required to resume operations</p>
                </div>
              </div>
            ) : (
              <div className="emergency-normal">
                <div className="emergency-indicator normal">✅ Normal Operation</div>
                <p>All systems operating within safety parameters</p>
              </div>
            )}
          </div>
        </div>
      </div>
    </div>
  );
};
