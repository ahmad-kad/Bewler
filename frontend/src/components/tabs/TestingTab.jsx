import React, { useState } from 'react';
import { useSystemContext } from '../../context/SystemContext';
import { Shield, Navigation, Eye, Zap, Radio, CheckCircle2, XCircle, Loader2, AlertTriangle } from 'lucide-react';
import { SystemState } from '../../config/stateDefinitions';

/**
 * TestingTab Component
 *
 * Modular component testing interface with smart grouping.
 * Context-aware: shows only relevant tests based on system state.
 */
export const TestingTab = () => {
  const {
    currentState,
    systemStatus,
    setRunningTests
  } = useSystemContext();

  const [activeFilter, setActiveFilter] = useState('all');
  const [testResults, setTestResults] = useState({});

  // Component definitions
  const components = [
    {
      id: 'safety',
      name: 'Safety',
      icon: Shield,
      status: systemStatus.safety === 'ready' ? 'operational' : systemStatus.safety,
      tests: [
        { id: 'estop', name: 'E-Stop', status: 'passed' },
        { id: 'recovery', name: 'Recovery', status: 'passed' },
        { id: 'watchdog', name: 'Watchdog', status: 'running' },
        { id: 'thresholds', name: 'Thresholds', status: 'pending' }
      ],
      total: 6,
      passed: 4,
      failed: 0,
      running: 2
    },
    {
      id: 'navigation',
      name: 'Navigation',
      icon: Navigation,
      status: 'operational',
      tests: [
        { id: 'waypoint', name: 'Waypoint', status: 'passed' },
        { id: 'gps', name: 'GPS', status: 'passed' },
        { id: 'obstacles', name: 'Obstacles', status: 'passed' },
        { id: 'path_planning', name: 'Path Planning', status: 'passed' }
      ],
      total: 6,
      passed: 6,
      failed: 0,
      running: 0
    },
    {
      id: 'vision',
      name: 'Vision',
      icon: Eye,
      status: systemStatus.vision,
      tests: [
        { id: 'aruco', name: 'ArUco Detection', status: 'passed' },
        { id: 'object', name: 'Object Recognition', status: 'pending' }
      ],
      total: 4,
      passed: 2,
      failed: 0,
      running: 0
    },
    {
      id: 'can',
      name: 'CAN Bus',
      icon: Zap,
      status: 'mock',
      isMock: true,
      tests: [],
      total: 0,
      passed: 0,
      failed: 0,
      running: 0
    },
    {
      id: 'websocket',
      name: 'WebSocket',
      icon: Radio,
      status: systemStatus.websocket,
      tests: [
        { id: 'connection', name: 'Connection', status: 'passed' },
        { id: 'latency', name: 'Latency', status: 'passed' }
      ],
      total: 2,
      passed: 2,
      failed: 0,
      running: 0
    }
  ];

  const getStatusIcon = (status) => {
    switch (status) {
      case 'passed':
        return <CheckCircle2 className="w-4 h-4 text-green-400" />;
      case 'failed':
        return <XCircle className="w-4 h-4 text-red-400" />;
      case 'running':
        return <Loader2 className="w-4 h-4 text-blue-400 animate-spin" />;
      default:
        return <div className="w-4 h-4 rounded-full border-2 border-zinc-500" />;
    }
  };

  const getStatusColor = (status) => {
    switch (status) {
      case 'operational':
      case 'ready':
        return 'text-green-400';
      case 'mock':
        return 'text-yellow-400';
      case 'degraded':
        return 'text-yellow-400';
      case 'error':
        return 'text-red-400';
      default:
        return 'text-zinc-400';
    }
  };

  const filteredComponents = activeFilter === 'all'
    ? components
    : components.filter(c => c.id === activeFilter);

  // Update running tests count
  React.useEffect(() => {
    const running = components.reduce((sum, comp) => sum + comp.running, 0);
    setRunningTests(running);
  }, [components, setRunningTests]);

  // Show warning if mission is active
  const showMissionWarning = currentState === SystemState.AUTONOMOUS;

  return (
    <div className="p-4 space-y-4">
      {/* Filter tabs */}
      <div className="flex items-center gap-2 border-b border-zinc-800 pb-2">
        <button
          onClick={() => setActiveFilter('all')}
          className={`px-3 py-1 text-xs rounded transition-colors ${
            activeFilter === 'all'
              ? 'bg-zinc-800 text-zinc-200'
              : 'text-zinc-400 hover:text-zinc-200'
          }`}
        >
          All
        </button>
        {components.map(comp => (
          <button
            key={comp.id}
            onClick={() => setActiveFilter(comp.id)}
            className={`px-3 py-1 text-xs rounded transition-colors ${
              activeFilter === comp.id
                ? 'bg-zinc-800 text-zinc-200'
                : 'text-zinc-400 hover:text-zinc-200'
            }`}
          >
            {comp.name}
          </button>
        ))}
      </div>

      {/* Mission warning */}
      {showMissionWarning && (
        <div className="bg-yellow-900/20 border border-yellow-800 rounded p-3">
          <div className="flex items-center gap-2 text-sm text-yellow-400">
            <AlertTriangle className="w-4 h-4" />
            <span>Mission active - some tests unavailable</span>
          </div>
        </div>
      )}

      {/* Component cards */}
      <div className="space-y-4">
        {filteredComponents.map(component => {
          const Icon = component.icon;
          const statusColor = getStatusColor(component.status);

          return (
            <div key={component.id} className="bg-zinc-900 border border-zinc-800 rounded p-4">
              {/* Header */}
              <div className="flex items-center justify-between mb-4">
                <div className="flex items-center gap-3">
                  <Icon className="w-5 h-5 text-zinc-400" />
                  <div>
                    <h3 className="text-sm font-semibold text-zinc-200 flex items-center gap-2">
                      {component.name.toUpperCase()}
                      {component.isMock && (
                        <span className="text-xs text-yellow-400">(MOCK)</span>
                      )}
                    </h3>
                    <div className={`text-xs font-medium ${statusColor}`}>
                      {component.status === 'mock' ? 'Mock Data' : component.status.toUpperCase()}
                    </div>
                  </div>
                </div>
                <div className="text-xs text-zinc-400">
                  {component.status === 'operational' || component.status === 'ready' ? '🟢 Operational' : '🟡 ' + component.status}
                </div>
              </div>

              {/* CAN Bus special display */}
              {component.isMock && (
                <div className="mb-4 p-3 bg-yellow-900/10 border border-yellow-800/50 rounded">
                  <div className="text-xs text-yellow-400 mb-2">
                    ⚠️ Using simulated data - NOT REAL HARDWARE
                  </div>
                  <div className="grid grid-cols-2 gap-2 text-xs">
                    <div>
                      <div className="text-zinc-400">IMU</div>
                      <div className="text-zinc-200">[0.1, 0.2, 9.8] m/s²</div>
                    </div>
                    <div>
                      <div className="text-zinc-400">GPS</div>
                      <div className="text-zinc-200">38.406°, -110.792°</div>
                    </div>
                    <div>
                      <div className="text-zinc-400">Battery</div>
                      <div className="text-zinc-200">85% (25.2V)</div>
                    </div>
                    <div>
                      <div className="text-zinc-400">Motors</div>
                      <div className="text-zinc-200">0.0, 0.0 rad/s</div>
                    </div>
                  </div>
                  <div className="flex items-center gap-2 mt-3">
                    <button className="px-2 py-1 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-xs rounded">
                      Simulate Failure
                    </button>
                    <button className="px-2 py-1 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-xs rounded">
                      Adjust Params
                    </button>
                    <button className="px-2 py-1 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-xs rounded">
                      Details →
                    </button>
                  </div>
                </div>
              )}

              {/* Test summary */}
              {!component.isMock && (
                <>
                  <div className="grid grid-cols-4 gap-4 mb-4 text-xs">
                    <div>
                      <div className="text-zinc-400">Tests</div>
                      <div className="text-zinc-200 font-medium">{component.total} total</div>
                    </div>
                    <div>
                      <div className="text-zinc-400">Passed</div>
                      <div className="text-green-400 font-medium">{component.passed}</div>
                    </div>
                    <div>
                      <div className="text-zinc-400">Failed</div>
                      <div className="text-red-400 font-medium">{component.failed}</div>
                    </div>
                    <div>
                      <div className="text-zinc-400">Running</div>
                      <div className="text-blue-400 font-medium">{component.running}</div>
                    </div>
                  </div>

                  {/* Test list */}
                  <div className="flex flex-wrap gap-2 mb-4">
                    {component.tests.map(test => (
                      <div
                        key={test.id}
                        className="flex items-center gap-1.5 px-2 py-1 bg-zinc-800 rounded text-xs"
                      >
                        {getStatusIcon(test.status)}
                        <span className="text-zinc-300">{test.name}</span>
                      </div>
                    ))}
                  </div>

                  {/* Actions */}
                  <div className="flex items-center gap-2">
                    <button className="px-3 py-1.5 bg-cyan-600 hover:bg-cyan-700 text-white text-xs rounded">
                      Run All
                    </button>
                    <button className="px-3 py-1.5 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-xs rounded">
                      Export
                    </button>
                    <button className="px-3 py-1.5 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-xs rounded">
                      Details →
                    </button>
                  </div>
                </>
              )}
            </div>
          );
        })}
      </div>
    </div>
  );
};
