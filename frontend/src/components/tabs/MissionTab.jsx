import React, { useState } from 'react';
import { useSystemContext } from '../../context/SystemContext';
import { MissionProgressCard } from '../cards/MissionProgressCard';
import { TelemetryCard } from '../cards/TelemetryCard';
import { SystemStatusCard } from '../cards/SystemStatusCard';
import { Target, Plus, FileText, Play } from 'lucide-react';
import { SystemState } from '../../config/stateDefinitions';

/**
 * MissionTab Component
 *
 * Context-aware mission planning and execution interface.
 * Shows planner when IDLE, execution view when mission is active.
 */
export const MissionTab = () => {
  const {
    currentState,
    activeMission,
    setActiveMission,
    telemetry,
    systemStatus,
    requestStateTransition
  } = useSystemContext();

  const [missionType, setMissionType] = useState('science');
  const [priority, setPriority] = useState('normal');

  // Mission templates
  const missionTemplates = [
    { id: 'science', name: 'Science', icon: '🧪', description: 'Sample collection' },
    { id: 'delivery', name: 'Delivery', icon: '📦', description: 'Object move' },
    { id: 'equipment', name: 'Equipment', icon: '🔧', description: 'Maintenance' },
    { id: 'navigation', name: 'Navigation', icon: '🧭', description: 'Waypoint' }
  ];

  // Start a demo mission
  const handleStartMission = () => {
    const mission = {
      id: Date.now(),
      name: missionTemplates.find(t => t.id === missionType)?.name || 'Mission',
      type: missionType,
      progress: 0,
      currentTask: 'Initializing...',
      nextTask: 'Waypoint navigation',
      eta: '5m 00s',
      waypoints: '0/3',
      samples: '0/3',
      analysis: '0/3'
    };
    setActiveMission(mission);
    requestStateTransition(SystemState.AUTONOMOUS, 'Mission started from UI');
  };

  // If mission is active, show execution view
  if (activeMission && currentState === SystemState.AUTONOMOUS) {
    return (
      <div className="p-4 space-y-4">
        {/* Mission Header */}
        <div className="bg-cyan-900/20 border border-cyan-800 rounded p-4">
          <div className="flex items-center justify-between">
            <div>
              <h2 className="text-lg font-semibold text-cyan-400 flex items-center gap-2">
                <Target className="w-5 h-5" />
                {activeMission.name.toUpperCase()} MISSION - {Math.round(activeMission.progress)}% Complete
              </h2>
              <div className="flex items-center gap-4 mt-2 text-sm text-zinc-400">
                <span>Phase: Sample Collection</span>
                <span>ETA: {activeMission.eta}</span>
                <span>Next: {activeMission.nextTask}</span>
              </div>
            </div>
            <div className="flex items-center gap-2">
              <button className="px-3 py-1.5 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-sm rounded">
                Pause
              </button>
              <button className="px-3 py-1.5 bg-red-900/30 hover:bg-red-900/50 text-red-400 text-sm rounded">
                Abort
              </button>
            </div>
          </div>
        </div>

        <div className="grid grid-cols-3 gap-4">
          {/* Progress */}
          <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
            <h3 className="text-sm font-semibold text-zinc-200 mb-3">Progress</h3>
            <div className="space-y-3">
              <div>
                <div className="flex items-center justify-between text-xs mb-1">
                  <span className="text-zinc-400">Waypoints</span>
                  <span className="text-zinc-200">{activeMission.waypoints}</span>
                </div>
                <div className="w-full bg-zinc-800 rounded-full h-1.5">
                  <div className="bg-cyan-500 h-1.5 rounded-full" style={{ width: '67%' }} />
                </div>
              </div>
              <div>
                <div className="flex items-center justify-between text-xs mb-1">
                  <span className="text-zinc-400">Samples</span>
                  <span className="text-zinc-200">{activeMission.samples}</span>
                </div>
                <div className="w-full bg-zinc-800 rounded-full h-1.5">
                  <div className="bg-cyan-500 h-1.5 rounded-full" style={{ width: '33%' }} />
                </div>
              </div>
              <div>
                <div className="flex items-center justify-between text-xs mb-1">
                  <span className="text-zinc-400">Analysis</span>
                  <span className="text-zinc-200">{activeMission.analysis}</span>
                </div>
                <div className="w-full bg-zinc-800 rounded-full h-1.5">
                  <div className="bg-zinc-700 h-1.5 rounded-full" style={{ width: '0%' }} />
                </div>
              </div>
            </div>
          </div>

          {/* Details */}
          <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
            <h3 className="text-sm font-semibold text-zinc-200 mb-3">Details</h3>
            <div className="space-y-2 text-sm">
              <div>
                <div className="text-zinc-400">Current</div>
                <div className="text-zinc-200 font-medium">WP #2</div>
              </div>
              <div>
                <div className="text-zinc-400">Next</div>
                <div className="text-zinc-200 font-medium">{activeMission.nextTask}</div>
              </div>
              <div>
                <div className="text-zinc-400">Status</div>
                <div className="text-green-400 font-medium">Active</div>
              </div>
            </div>
          </div>

          {/* Controls */}
          <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
            <h3 className="text-sm font-semibold text-zinc-200 mb-3">Controls</h3>
            <div className="space-y-2">
              <button className="w-full px-3 py-2 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-sm rounded">
                Pause
              </button>
              <button className="w-full px-3 py-2 bg-red-900/30 hover:bg-red-900/50 text-red-400 text-sm rounded">
                Abort
              </button>
              <button className="w-full px-3 py-2 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-sm rounded">
                Details →
              </button>
            </div>
          </div>
        </div>

        <div className="grid grid-cols-3 gap-4">
          {/* Map View */}
          <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
            <h3 className="text-sm font-semibold text-zinc-200 mb-3">Map View</h3>
            <div className="bg-zinc-950 rounded aspect-video flex items-center justify-center">
              <div className="text-xs text-zinc-400">Interactive Map</div>
            </div>
            <div className="mt-2 text-xs text-zinc-400">
              • Current Position<br />
              • Waypoints<br />
              • Path
            </div>
          </div>

          {/* Camera */}
          <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
            <h3 className="text-sm font-semibold text-zinc-200 mb-3">Camera</h3>
            <div className="bg-zinc-950 rounded aspect-video flex items-center justify-center">
              <div className="text-xs text-yellow-400">⚠️ MOCK</div>
            </div>
          </div>

          {/* Telemetry */}
          <TelemetryCard telemetry={telemetry} />
        </div>
      </div>
    );
  }

  // Mission planning view (when IDLE)
  return (
    <div className="p-4 space-y-4">
      <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
        <h2 className="text-base font-semibold text-zinc-200 mb-4">Mission Planner</h2>

        <div className="grid grid-cols-3 gap-4 mb-4">
          <div>
            <label className="block text-xs text-zinc-400 mb-1">Type</label>
            <select
              value={missionType}
              onChange={(e) => setMissionType(e.target.value)}
              className="w-full px-3 py-2 bg-zinc-800 border border-zinc-700 rounded text-sm text-zinc-200"
            >
              {missionTemplates.map(t => (
                <option key={t.id} value={t.id}>{t.name}</option>
              ))}
            </select>
          </div>

          <div>
            <label className="block text-xs text-zinc-400 mb-1">Priority</label>
            <select
              value={priority}
              onChange={(e) => setPriority(e.target.value)}
              className="w-full px-3 py-2 bg-zinc-800 border border-zinc-700 rounded text-sm text-zinc-200"
            >
              <option value="normal">Normal</option>
              <option value="high">High</option>
              <option value="urgent">Urgent</option>
            </select>
          </div>

          <div className="flex items-end">
            <button
              onClick={handleStartMission}
              className="w-full px-4 py-2 bg-cyan-600 hover:bg-cyan-700 text-white text-sm rounded flex items-center justify-center gap-2"
            >
              <Play className="w-4 h-4" />
              Create
            </button>
          </div>
        </div>

        <div className="grid grid-cols-3 gap-4 text-xs text-zinc-400 mb-4">
          <div>Waypoints: (0)</div>
          <div>Parameters: (default)</div>
          <div className="text-green-400">Validation: ✓</div>
        </div>

        <div className="flex items-center gap-2">
          <button className="px-3 py-1.5 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-xs rounded flex items-center gap-2">
            <Plus className="w-3 h-3" />
            Add Waypoint
          </button>
          <button className="px-3 py-1.5 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-xs rounded flex items-center gap-2">
            <FileText className="w-3 h-3" />
            Load Template
          </button>
          <button className="px-3 py-1.5 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-xs rounded">
            Clear
          </button>
        </div>
      </div>

      {/* Available Missions */}
      <div>
        <h3 className="text-sm font-semibold text-zinc-200 mb-3">Available Missions</h3>
        <div className="grid grid-cols-4 gap-4">
          {missionTemplates.map(template => (
            <div
              key={template.id}
              className="bg-zinc-900 border border-zinc-800 rounded p-4 cursor-pointer hover:border-cyan-700 transition-colors"
              onClick={() => setMissionType(template.id)}
            >
              <div className="text-2xl mb-2">{template.icon}</div>
              <div className="text-sm font-semibold text-zinc-200 mb-1">{template.name}</div>
              <div className="text-xs text-zinc-400 mb-3">{template.description}</div>
              <button className="w-full px-3 py-1.5 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-xs rounded">
                Select
              </button>
            </div>
          ))}
        </div>
      </div>
    </div>
  );
};
