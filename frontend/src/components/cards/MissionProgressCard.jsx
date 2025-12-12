import React from 'react';
import { Target, Pause, Square } from 'lucide-react';

/**
 * MissionProgressCard Component
 *
 * Displays active mission progress with controls.
 * Only shows when a mission is active.
 */
export const MissionProgressCard = ({ mission, onPause, onAbort, onViewDetails }) => {
  if (!mission) return null;

  return (
    <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
      <div className="flex items-center justify-between mb-3">
        <div className="flex items-center gap-2">
          <Target className="w-4 h-4 text-cyan-400" />
          <h3 className="text-sm font-semibold text-zinc-200">
            {mission.name} - {Math.round(mission.progress)}% Complete
          </h3>
        </div>
        <div className="flex items-center gap-2">
          <button
            onClick={onPause}
            className="px-3 py-1 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-xs rounded transition-colors"
          >
            <Pause className="w-3 h-3 inline mr-1" />
            Pause
          </button>
          <button
            onClick={onAbort}
            className="px-3 py-1 bg-red-900/30 hover:bg-red-900/50 text-red-400 text-xs rounded transition-colors"
          >
            <Square className="w-3 h-3 inline mr-1" />
            Abort
          </button>
        </div>
      </div>

      <div className="mb-3">
        <div className="text-xs text-zinc-400 mb-1">Current Task: {mission.currentTask}</div>
        <div className="flex items-center gap-4 text-xs text-zinc-400">
          <span>ETA: {mission.eta}</span>
          <span>Next: {mission.nextTask}</span>
        </div>
      </div>

      {/* Progress bar */}
      <div className="w-full bg-zinc-800 rounded-full h-2 mb-3">
        <div
          className="bg-cyan-500 h-2 rounded-full transition-all duration-300"
          style={{ width: `${mission.progress}%` }}
        />
      </div>

      {/* Mission breakdown */}
      <div className="grid grid-cols-3 gap-2 text-xs">
        <div>
          <div className="text-zinc-400">Waypoints</div>
          <div className="text-zinc-200 font-medium">{mission.waypoints}</div>
        </div>
        <div>
          <div className="text-zinc-400">Samples</div>
          <div className="text-zinc-200 font-medium">{mission.samples}</div>
        </div>
        <div>
          <div className="text-zinc-400">Analysis</div>
          <div className="text-zinc-200 font-medium">{mission.analysis}</div>
        </div>
      </div>

      {onViewDetails && (
        <button
          onClick={onViewDetails}
          className="mt-3 w-full px-3 py-1.5 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-xs rounded transition-colors"
        >
          View Details →
        </button>
      )}
    </div>
  );
};
