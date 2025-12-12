import React, { useState } from 'react';
import { useSystemContext } from '../../context/SystemContext';
import { Search, Filter, X, AlertCircle } from 'lucide-react';
import { SystemState } from '../../config/stateDefinitions';

/**
 * DebugTab Component
 *
 * Multi-panel debug view with logs, state machine, topics, and performance.
 * Context-aware: highlights errors and relevant information.
 */
export const DebugTab = () => {
  const {
    currentState,
    requestStateTransition,
    isTransitioning,
    alerts,
    errorCount
  } = useSystemContext();

  const [activePanel, setActivePanel] = useState('logs');
  const [logLevel, setLogLevel] = useState('all');
  const [logComponent, setLogComponent] = useState('all');
  const [logs, setLogs] = useState([
    { time: '23:14:35', level: 'INFO', component: 'StateMachine', message: 'BOOT → IDLE' },
    { time: '23:14:36', level: 'WARN', component: 'Navigation', message: 'GPS HDOP: 2.5' },
    { time: '23:14:37', level: 'ERROR', component: 'Vision', message: 'Camera timeout' },
    { time: '23:14:38', level: 'DEBUG', component: 'CAN', message: 'Motor cmd sent' },
    { time: '23:14:39', level: 'INFO', component: 'Safety', message: 'All checks passed' }
  ]);

  // Mock ROS2 topics
  const topics = [
    { name: '/mission/cmds', rate: 0.5, size: '256B' },
    { name: '/odom', rate: 10, size: '1.2K' },
    { name: '/can/sensor', rate: 20, size: '512B' },
    { name: '/safety/status', rate: 1, size: '128B' },
    { name: '/vision/camera', rate: 0, size: '0B' }
  ];

  // State machine transitions
  const getValidTransitions = () => {
    const transitions = {
      [SystemState.IDLE]: [SystemState.AUTONOMOUS, SystemState.TELEOPERATION, SystemState.SAFETY],
      [SystemState.AUTONOMOUS]: [SystemState.IDLE, SystemState.TELEOPERATION, SystemState.SAFETY],
      [SystemState.TELEOPERATION]: [SystemState.IDLE, SystemState.AUTONOMOUS, SystemState.SAFETY],
      [SystemState.SAFETY]: [SystemState.IDLE, SystemState.SHUTDOWN]
    };
    return transitions[currentState] || [];
  };

  const getLogColor = (level) => {
    switch (level) {
      case 'ERROR':
        return 'text-red-400';
      case 'WARN':
        return 'text-yellow-400';
      case 'INFO':
        return 'text-blue-400';
      case 'DEBUG':
        return 'text-zinc-400';
      default:
        return 'text-zinc-300';
    }
  };

  const filteredLogs = logs.filter(log => {
    if (logLevel !== 'all' && log.level !== logLevel) return false;
    if (logComponent !== 'all' && log.component !== logComponent) return false;
    return true;
  });

  const panels = [
    { id: 'logs', label: 'Logs' },
    { id: 'state', label: 'State' },
    { id: 'topics', label: 'Topics' },
    { id: 'performance', label: 'Performance' },
    { id: 'network', label: 'Network' }
  ];

  return (
    <div className="p-4 space-y-4">
      {/* Panel tabs */}
      <div className="flex items-center gap-2 border-b border-zinc-800 pb-2">
        {panels.map(panel => (
          <button
            key={panel.id}
            onClick={() => setActivePanel(panel.id)}
            className={`px-3 py-1 text-xs rounded transition-colors ${
              activePanel === panel.id
                ? 'bg-zinc-800 text-zinc-200'
                : 'text-zinc-400 hover:text-zinc-200'
            }`}
          >
            {panel.label}
          </button>
        ))}
      </div>

      {/* Logs panel */}
      {activePanel === 'logs' && (
        <div className="space-y-4">
          <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
            <div className="flex items-center justify-between mb-4">
              <h3 className="text-sm font-semibold text-zinc-200">
                Logs (Context: Current State = {currentState})
              </h3>
              <div className="flex items-center gap-2">
                <div className="flex items-center gap-1 px-2 py-1 bg-zinc-800 rounded">
                  <Search className="w-3 h-3 text-zinc-400" />
                  <input
                    type="text"
                    placeholder="Search..."
                    className="bg-transparent border-0 text-xs text-zinc-200 placeholder-zinc-500 focus:outline-none"
                  />
                </div>
                <select
                  value={logLevel}
                  onChange={(e) => setLogLevel(e.target.value)}
                  className="px-2 py-1 bg-zinc-800 border border-zinc-700 rounded text-xs text-zinc-200"
                >
                  <option value="all">Level: All</option>
                  <option value="ERROR">ERROR</option>
                  <option value="WARN">WARN</option>
                  <option value="INFO">INFO</option>
                  <option value="DEBUG">DEBUG</option>
                </select>
                <select
                  value={logComponent}
                  onChange={(e) => setLogComponent(e.target.value)}
                  className="px-2 py-1 bg-zinc-800 border border-zinc-700 rounded text-xs text-zinc-200"
                >
                  <option value="all">Component: All</option>
                  <option value="StateMachine">StateMachine</option>
                  <option value="Navigation">Navigation</option>
                  <option value="Vision">Vision</option>
                  <option value="CAN">CAN</option>
                  <option value="Safety">Safety</option>
                </select>
                <span className="text-xs text-zinc-400">Last 100</span>
                <button className="px-2 py-1 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-xs rounded">
                  Clear
                </button>
              </div>
            </div>
            <div className="space-y-1 font-mono text-xs max-h-96 overflow-y-auto">
              {filteredLogs.map((log, idx) => (
                <div key={idx} className="flex items-start gap-3 py-1">
                  <span className="text-zinc-500">{log.time}</span>
                  <span className={`font-semibold ${getLogColor(log.level)}`}>
                    [{log.level}]
                  </span>
                  <span className="text-zinc-400">{log.component}:</span>
                  <span className="text-zinc-200 flex-1">{log.message}</span>
                </div>
              ))}
            </div>
          </div>
        </div>
      )}

      {/* State panel */}
      {activePanel === 'state' && (
        <div className="grid grid-cols-2 gap-4">
          <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
            <h3 className="text-sm font-semibold text-zinc-200 mb-4">State Machine</h3>
            <div className="space-y-3">
              <div>
                <div className="text-xs text-zinc-400">Current</div>
                <div className="text-lg font-bold text-green-400">{currentState}</div>
              </div>
              <div>
                <div className="text-xs text-zinc-400">Time in State</div>
                <div className="text-sm text-zinc-200">45.2s</div>
              </div>
              <div className="pt-3 border-t border-zinc-800">
                <div className="text-xs text-zinc-400 mb-2">Transitions:</div>
                <div className="space-y-1">
                  {getValidTransitions().map(state => (
                    <button
                      key={state}
                      onClick={() => requestStateTransition(state, 'Debug tab transition')}
                      disabled={isTransitioning}
                      className="block w-full text-left px-2 py-1 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-xs rounded transition-colors disabled:opacity-50"
                    >
                      → {state}
                    </button>
                  ))}
                </div>
              </div>
              <div className="pt-3 border-t border-zinc-800">
                <div className="text-xs text-zinc-400 mb-2">Preconditions:</div>
                <div className="space-y-1 text-xs">
                  <div className="flex items-center gap-2">
                    <span className="text-green-400">✓</span>
                    <span className="text-zinc-300">Boot Complete</span>
                  </div>
                  <div className="flex items-center gap-2">
                    <span className="text-green-400">✓</span>
                    <span className="text-zinc-300">Communication OK</span>
                  </div>
                  <div className="flex items-center gap-2">
                    <span className="text-yellow-400">⚠</span>
                    <span className="text-zinc-300">Calibration Pending</span>
                  </div>
                </div>
              </div>
            </div>
          </div>

          <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
            <h3 className="text-sm font-semibold text-zinc-200 mb-4">ROS2 Topics</h3>
            <div className="space-y-2">
              <div className="grid grid-cols-3 gap-2 text-xs text-zinc-400 border-b border-zinc-800 pb-2">
                <div>Topic</div>
                <div>Rate</div>
                <div>Size</div>
              </div>
              {topics.map((topic, idx) => (
                <div key={idx} className="grid grid-cols-3 gap-2 text-xs">
                  <div className="text-zinc-200 font-mono">{topic.name}</div>
                  <div className="text-zinc-400">{topic.rate} Hz</div>
                  <div className="text-zinc-400">{topic.size}</div>
                </div>
              ))}
            </div>
          </div>
        </div>
      )}

      {/* Performance panel */}
      {activePanel === 'performance' && (
        <div className="grid grid-cols-2 gap-4">
          <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
            <h3 className="text-sm font-semibold text-zinc-200 mb-4">Performance</h3>
            <div className="space-y-3 text-sm">
              <div className="flex items-center justify-between">
                <span className="text-zinc-400">CPU</span>
                <span className="text-zinc-200 font-medium">45%</span>
              </div>
              <div className="flex items-center justify-between">
                <span className="text-zinc-400">Memory</span>
                <span className="text-zinc-200 font-medium">60%</span>
              </div>
              <div className="flex items-center justify-between">
                <span className="text-zinc-400">Network</span>
                <span className="text-zinc-200 font-medium">100%</span>
              </div>
              <div className="flex items-center justify-between">
                <span className="text-zinc-400">Latency</span>
                <span className="text-zinc-200 font-medium">12ms</span>
              </div>
              <div className="flex items-center justify-between">
                <span className="text-zinc-400">Queue</span>
                <span className="text-zinc-200 font-medium">15/1000</span>
              </div>
              <div className="flex items-center justify-between">
                <span className="text-zinc-400">Messages/s</span>
                <span className="text-zinc-200 font-medium">248</span>
              </div>
            </div>
          </div>

          <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
            <h3 className="text-sm font-semibold text-zinc-200 mb-4">Alerts</h3>
            <div className="space-y-2">
              {alerts.length > 0 ? (
                alerts.map(alert => (
                  <div key={alert.id} className="flex items-center gap-2 text-xs">
                    <AlertCircle className={`w-4 h-4 ${
                      alert.type === 'error' ? 'text-red-400' : 'text-yellow-400'
                    }`} />
                    <span className="text-zinc-300">{alert.message}</span>
                  </div>
                ))
              ) : (
                <div className="text-xs text-zinc-500">No alerts</div>
              )}
            </div>
          </div>
        </div>
      )}

      {/* Topics panel */}
      {activePanel === 'topics' && (
        <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
          <h3 className="text-sm font-semibold text-zinc-200 mb-4">ROS2 Topics</h3>
          <div className="space-y-2">
            <div className="grid grid-cols-4 gap-2 text-xs text-zinc-400 border-b border-zinc-800 pb-2">
              <div>Topic</div>
              <div>Rate</div>
              <div>Size</div>
              <div>Status</div>
            </div>
            {topics.map((topic, idx) => (
              <div key={idx} className="grid grid-cols-4 gap-2 text-xs py-1 border-b border-zinc-800/50">
                <div className="text-zinc-200 font-mono">{topic.name}</div>
                <div className="text-zinc-400">{topic.rate} Hz</div>
                <div className="text-zinc-400">{topic.size}</div>
                <div className={topic.rate > 0 ? 'text-green-400' : 'text-red-400'}>
                  {topic.rate > 0 ? 'Active' : 'Inactive'}
                </div>
              </div>
            ))}
          </div>
        </div>
      )}

      {/* Network panel */}
      {activePanel === 'network' && (
        <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
          <h3 className="text-sm font-semibold text-zinc-200 mb-4">Network Status</h3>
          <div className="space-y-3 text-sm">
            <div className="flex items-center justify-between">
              <span className="text-zinc-400">WebSocket</span>
              <span className="text-green-400 font-medium">Connected</span>
            </div>
            <div className="flex items-center justify-between">
              <span className="text-zinc-400">Latency</span>
              <span className="text-zinc-200 font-medium">12ms</span>
            </div>
            <div className="flex items-center justify-between">
              <span className="text-zinc-400">Messages/s</span>
              <span className="text-zinc-200 font-medium">248</span>
            </div>
            <div className="flex items-center justify-between">
              <span className="text-zinc-400">Errors</span>
              <span className="text-red-400 font-medium">{errorCount}</span>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};
