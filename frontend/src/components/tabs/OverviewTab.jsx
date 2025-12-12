import React from 'react';
import { useSystemContext } from '../../context/SystemContext';
import { SystemStatusCard } from '../cards/SystemStatusCard';
import { TelemetryCard } from '../cards/TelemetryCard';
import { MissionProgressCard } from '../cards/MissionProgressCard';
import { Play, TestTube, Settings, AlertTriangle } from 'lucide-react';
import { SystemState } from '../../config/stateDefinitions';

/**
 * OverviewTab Component
 *
 * Context-aware dashboard that adapts to system state (IDLE, AUTONOMOUS, ERROR).
 * Shows relevant information based on current state.
 */
export const OverviewTab = () => {
  const {
    currentState,
    telemetry,
    systemStatus,
    activeMission,
    alerts,
    requestStateTransition
  } = useSystemContext();

  // Context-aware system status based on state
  const getContextualSystems = () => {
    const baseSystems = {
      safety: systemStatus.safety,
      navigation: systemStatus.navigation,
      vision: systemStatus.vision
    };

    if (currentState === SystemState.AUTONOMOUS) {
      return {
        ...baseSystems,
        mission: 'active',
        slam: 'active'
      };
    }

    if (currentState === SystemState.SAFETY) {
      return {
        ...baseSystems,
        safety: 'active'
      };
    }

    return baseSystems;
  };

  // Render IDLE state layout
  if (currentState === SystemState.IDLE || currentState === SystemState.BOOT) {
    return (
      <div className="p-4 space-y-4">
        <div className="grid grid-cols-3 gap-4">
          {/* System Status */}
          <SystemStatusCard
            systems={getContextualSystems()}
            title="System Status"
          />

          {/* Quick Actions */}
          <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
            <h3 className="text-sm font-semibold text-zinc-200 mb-3">Quick Actions</h3>
            <div className="space-y-2">
              <button
                onClick={() => requestStateTransition(SystemState.AUTONOMOUS, 'Start mission from UI')}
                className="w-full px-3 py-2 bg-cyan-600 hover:bg-cyan-700 text-white text-sm rounded transition-colors flex items-center justify-center gap-2"
              >
                <Play className="w-4 h-4" />
                Start Mission
              </button>
              <button
                onClick={() => {/* Navigate to testing tab */}}
                className="w-full px-3 py-2 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-sm rounded transition-colors flex items-center justify-center gap-2"
              >
                <TestTube className="w-4 h-4" />
                Run Tests
              </button>
              <button
                onClick={() => requestStateTransition(SystemState.CALIBRATION, 'Calibration from UI')}
                className="w-full px-3 py-2 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-sm rounded transition-colors"
              >
                Calibrate
              </button>
            </div>
          </div>

          {/* Recent Activity */}
          <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
            <h3 className="text-sm font-semibold text-zinc-200 mb-3">Recent Activity</h3>
            <div className="text-sm text-zinc-400">
              <div className="mb-2">Last: Science Mission</div>
              <div className="text-green-400">Completed ✓</div>
              <div className="text-xs text-zinc-500 mt-1">2h ago</div>
            </div>
          </div>
        </div>

        <div className="grid grid-cols-3 gap-4">
          {/* Telemetry */}
          <TelemetryCard telemetry={telemetry} />

          {/* State Machine */}
          <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
            <h3 className="text-sm font-semibold text-zinc-200 mb-3">State Machine</h3>
            <div className="text-center py-4">
              <div className="text-2xl font-bold text-green-400 mb-2">{currentState}</div>
              <div className="text-xs text-zinc-400">Ready for:</div>
              <div className="text-xs text-zinc-300 mt-1">• Mission</div>
              <div className="text-xs text-zinc-300">• Teleop</div>
            </div>
          </div>

          {/* Mission Queue */}
          <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
            <h3 className="text-sm font-semibold text-zinc-200 mb-3">Mission Queue</h3>
            <div className="text-sm text-zinc-400 mb-3">Queue: Empty</div>
            <button className="w-full px-3 py-1.5 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-xs rounded transition-colors">
              + New
            </button>
          </div>
        </div>
      </div>
    );
  }

  // Render AUTONOMOUS state layout
  if (currentState === SystemState.AUTONOMOUS) {
    return (
      <div className="p-4 space-y-4">
        {/* Active Mission Banner */}
        {activeMission && (
          <div className="bg-cyan-900/20 border border-cyan-800 rounded p-4">
            <div className="flex items-center justify-between mb-2">
              <h2 className="text-base font-semibold text-cyan-400">
                Active Mission: {activeMission.name} ({Math.round(activeMission.progress)}% complete)
              </h2>
              <div className="flex items-center gap-2">
                <span className="text-xs text-zinc-400">ETA: {activeMission.eta}</span>
                <span className="text-xs text-zinc-400">Next: {activeMission.nextTask}</span>
                <button className="px-3 py-1 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-xs rounded">
                  Pause
                </button>
                <button className="px-3 py-1 bg-red-900/30 hover:bg-red-900/50 text-red-400 text-xs rounded">
                  Abort
                </button>
              </div>
            </div>
            <div className="text-sm text-zinc-300">
              Current Task: {activeMission.currentTask}
            </div>
          </div>
        )}

        <div className="grid grid-cols-3 gap-4">
          {/* Mission Context */}
          <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
            <h3 className="text-sm font-semibold text-zinc-200 mb-3">Mission Context</h3>
            <div className="space-y-2 text-sm">
              <div className="flex items-center justify-between">
                <span className="text-zinc-400">Waypoints</span>
                <span className="text-zinc-200 font-medium">{activeMission?.waypoints || '0/0'}</span>
              </div>
              <div className="flex items-center justify-between">
                <span className="text-zinc-400">Samples</span>
                <span className="text-zinc-200 font-medium">{activeMission?.samples || '0/0'}</span>
              </div>
              <div className="flex items-center justify-between">
                <span className="text-zinc-400">Analysis</span>
                <span className="text-zinc-200 font-medium">{activeMission?.analysis || '0/0'}</span>
              </div>
            </div>
          </div>

          {/* System Status */}
          <SystemStatusCard
            systems={getContextualSystems()}
            title="System Status"
          />

          {/* Telemetry */}
          <TelemetryCard telemetry={telemetry} />
        </div>

        <div className="grid grid-cols-3 gap-4">
          {/* Camera Feed */}
          <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
            <h3 className="text-sm font-semibold text-zinc-200 mb-3">Camera Feed</h3>
            <div className="bg-zinc-950 rounded aspect-video flex items-center justify-center">
              <div className="text-xs text-yellow-400">⚠️ MOCK</div>
            </div>
          </div>

          {/* Map View */}
          <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
            <h3 className="text-sm font-semibold text-zinc-200 mb-3">Map View</h3>
            <div className="bg-zinc-950 rounded aspect-video flex items-center justify-center">
              <div className="text-xs text-zinc-400">Rover Position & Path</div>
            </div>
          </div>

          {/* Alerts */}
          <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
            <h3 className="text-sm font-semibold text-zinc-200 mb-3">Alerts</h3>
            <div className="space-y-2">
              {alerts.length > 0 ? (
                alerts.map(alert => (
                  <div key={alert.id} className="flex items-center gap-2 text-xs">
                    <AlertTriangle className={`w-4 h-4 ${
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
      </div>
    );
  }

  // Render ERROR/SAFETY state layout
  if (currentState === SystemState.SAFETY) {
    return (
      <div className="p-4 space-y-4">
        {/* Safety Alert Banner */}
        <div className="bg-yellow-900/20 border border-yellow-800 rounded p-4">
          <div className="flex items-center gap-2 mb-2">
            <AlertTriangle className="w-5 h-5 text-yellow-400" />
            <h2 className="text-base font-semibold text-yellow-400">
              Safety Alert: {systemStatus.navigation === 'degraded' ? 'Navigation system degraded' : 'System in safety mode'}
            </h2>
          </div>
          <div className="text-sm text-zinc-300 mb-3">
            {systemStatus.navigation === 'degraded'
              ? 'GPS accuracy below threshold (HDOP: 2.5). Autonomous navigation limited.'
              : 'System has entered safety mode. Review status before continuing.'}
          </div>
          <div className="flex items-center gap-2">
            <button
              onClick={() => requestStateTransition(SystemState.TELEOPERATION, 'Switch to teleop from safety')}
              className="px-3 py-1.5 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-sm rounded"
            >
              Switch to Teleop
            </button>
            <button
              onClick={() => requestStateTransition(SystemState.IDLE, 'Continue with caution')}
              className="px-3 py-1.5 bg-yellow-900/30 hover:bg-yellow-900/50 text-yellow-400 text-sm rounded"
            >
              Continue with Caution
            </button>
          </div>
        </div>

        <div className="grid grid-cols-3 gap-4">
          {/* Affected Systems */}
          <SystemStatusCard
            systems={getContextualSystems()}
            title="Affected Systems"
          />

          {/* Recommended Actions */}
          <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
            <h3 className="text-sm font-semibold text-zinc-200 mb-3">Recommended Actions</h3>
            <div className="space-y-2">
              <button className="w-full px-3 py-2 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-sm rounded text-left">
                Switch Mode
              </button>
              <button className="w-full px-3 py-2 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-sm rounded text-left">
                Calibrate GPS
              </button>
              <button className="w-full px-3 py-2 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-sm rounded text-left">
                View Details
              </button>
            </div>
          </div>

          {/* System Status */}
          <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
            <h3 className="text-sm font-semibold text-zinc-200 mb-3">System Status</h3>
            <div className="space-y-2 text-sm">
              <div className="flex items-center justify-between">
                <span className="text-zinc-400">Overall</span>
                <span className="text-yellow-400 font-medium">DEGRADED</span>
              </div>
              <div className="flex items-center justify-between">
                <span className="text-zinc-400">Safety</span>
                <span className="text-green-400 font-medium">SAFE</span>
              </div>
              <div className="flex items-center justify-between">
                <span className="text-zinc-400">Capability</span>
                <span className="text-yellow-400 font-medium">LIMITED</span>
              </div>
            </div>
          </div>
        </div>
      </div>
    );
  }

  // Default layout for other states
  return (
    <div className="p-4 space-y-4">
      <div className="grid grid-cols-3 gap-4">
        <SystemStatusCard systems={getContextualSystems()} />
        <TelemetryCard telemetry={telemetry} />
        <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
          <h3 className="text-sm font-semibold text-zinc-200 mb-3">Current State</h3>
          <div className="text-lg font-bold text-zinc-200">{currentState}</div>
        </div>
      </div>
    </div>
  );
};
