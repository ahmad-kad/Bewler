import React from 'react';
import { useSystemContext } from '../context/SystemContext';
import { AlertTriangle } from 'lucide-react';

/**
 * TopBar Component
 *
 * Always-visible top bar with system state, critical telemetry, and emergency stop.
 * Minimal, information-dense design.
 */
export const TopBar = () => {
  const {
    currentState,
    getStateBadge,
    telemetry,
    isConnected,
    errorCount,
    runningTests,
    activeMission,
    handleEmergencyStop
  } = useSystemContext();

  const stateBadge = getStateBadge();

  return (
    <div className="bg-zinc-900 border-b border-zinc-800 px-4 py-2 flex items-center justify-between text-sm">
      {/* Left: System info */}
      <div className="flex items-center gap-4">
        <span className="font-semibold text-zinc-100">URC 2026</span>

        {/* System state badge */}
        <div className={`flex items-center gap-1.5 px-2 py-1 rounded ${
          stateBadge.color === 'green' ? 'bg-green-900/30 text-green-400' :
          stateBadge.color === 'blue' ? 'bg-blue-900/30 text-blue-400' :
          stateBadge.color === 'cyan' ? 'bg-cyan-900/30 text-cyan-400' :
          stateBadge.color === 'yellow' ? 'bg-yellow-900/30 text-yellow-400' :
          stateBadge.color === 'orange' ? 'bg-orange-900/30 text-orange-400' :
          stateBadge.color === 'red' ? 'bg-red-900/30 text-red-400' :
          'bg-zinc-800 text-zinc-400'
        }`}>
          <span>{stateBadge.emoji}</span>
          <span className="font-medium">{stateBadge.label}</span>
        </div>

        {/* Connection status */}
        <div className={`flex items-center gap-1 ${
          isConnected ? 'text-green-400' : 'text-red-400'
        }`}>
          <div className={`w-2 h-2 rounded-full ${
            isConnected ? 'bg-green-400' : 'bg-red-400'
          }`} />
          <span className="text-xs">{isConnected ? 'Connected' : 'Disconnected'}</span>
        </div>
      </div>

      {/* Center: Critical telemetry */}
      <div className="flex items-center gap-4 text-zinc-300">
        <div className="flex items-center gap-1.5">
          <span className="text-zinc-500">Battery:</span>
          <span className={`font-medium ${
            telemetry.battery > 50 ? 'text-green-400' :
            telemetry.battery > 20 ? 'text-yellow-400' :
            'text-red-400'
          }`}>
            {Math.round(telemetry.battery)}%
          </span>
        </div>

        <div className="flex items-center gap-1.5">
          <span className="text-zinc-500">GPS:</span>
          <span className="font-medium text-zinc-200">
            {telemetry.gps.satellites} sat
          </span>
        </div>

        {activeMission && (
          <div className="flex items-center gap-1.5">
            <span className="text-zinc-500">Mission:</span>
            <span className="font-medium text-cyan-400">
              {activeMission.name} ({Math.round(activeMission.progress)}%)
            </span>
          </div>
        )}
      </div>

      {/* Right: Emergency stop */}
      <div className="flex items-center gap-4">
        {errorCount > 0 && (
          <div className="flex items-center gap-1.5 text-yellow-400">
            <AlertTriangle className="w-4 h-4" />
            <span className="text-xs">{errorCount}</span>
          </div>
        )}

        <button
          onClick={handleEmergencyStop}
          className="px-4 py-1.5 bg-red-600 hover:bg-red-700 text-white font-semibold rounded transition-colors"
        >
          E-STOP
        </button>
      </div>
    </div>
  );
};
