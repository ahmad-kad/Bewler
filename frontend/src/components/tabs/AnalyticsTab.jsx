import React, { useState } from 'react';
import { useSystemContext } from '../../context/SystemContext';
import { BarChart3, TrendingUp, AlertTriangle, CheckCircle2 } from 'lucide-react';

/**
 * AnalyticsTab Component
 * 
 * Data visualization and metrics dashboard.
 * Shows real-time and historical data.
 */
export const AnalyticsTab = () => {
  const {
    systemStatus,
    telemetry,
    alerts
  } = useSystemContext();

  const [activeView, setActiveView] = useState('realtime');

  // Mock mission statistics
  const missionStats = {
    total: 12,
    success: 11,
    failed: 1,
    avgDuration: '8m 32s',
    lastMission: { name: 'Science', time: '2h ago', status: 'success' }
  };

  const componentHealth = {
    safety: 100,
    navigation: 80,
    vision: 90,
    can: 100
  };

  const views = [
    { id: 'realtime', label: 'Real-time' },
    { id: 'historical', label: 'Historical' },
    { id: 'missions', label: 'Mission Reports' },
    { id: 'export', label: 'Export' }
  ];

  const getHealthColor = (value) => {
    if (value >= 90) return 'bg-green-500';
    if (value >= 70) return 'bg-yellow-500';
    return 'bg-red-500';
  };

  return (
    <div className="p-4 space-y-4">
      {/* View tabs */}
      <div className="flex items-center gap-2 border-b border-zinc-800 pb-2">
        {views.map(view => (
          <button
            key={view.id}
            onClick={() => setActiveView(view.id)}
            className={`px-3 py-1 text-xs rounded transition-colors ${
              activeView === view.id
                ? 'bg-zinc-800 text-zinc-200'
                : 'text-zinc-400 hover:text-zinc-200'
            }`}
          >
            {view.label}
          </button>
        ))}
      </div>

      {/* Real-time metrics */}
      {activeView === 'realtime' && (
        <div className="space-y-4">
          {/* Key metrics */}
          <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
            <h3 className="text-sm font-semibold text-zinc-200 mb-4">Real-time Metrics (Last 5 minutes)</h3>
            <div className="space-y-3">
              <div>
                <div className="flex items-center justify-between text-xs mb-1">
                  <span className="text-zinc-400">System Health</span>
                  <span className="text-zinc-200 font-medium">85%</span>
                </div>
                <div className="w-full bg-zinc-800 rounded-full h-2">
                  <div className="bg-green-500 h-2 rounded-full" style={{ width: '85%' }} />
                </div>
              </div>
              <div>
                <div className="flex items-center justify-between text-xs mb-1">
                  <span className="text-zinc-400">Mission Success</span>
                  <span className="text-zinc-200 font-medium">100% (3/3)</span>
                </div>
                <div className="w-full bg-zinc-800 rounded-full h-2">
                  <div className="bg-green-500 h-2 rounded-full" style={{ width: '100%' }} />
                </div>
              </div>
              <div className="flex items-center justify-between text-xs">
                <span className="text-zinc-400">Avg Response Time</span>
                <span className="text-zinc-200 font-medium">12ms</span>
              </div>
            </div>
          </div>

          <div className="grid grid-cols-2 gap-4">
            {/* Performance trends */}
            <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
              <h3 className="text-sm font-semibold text-zinc-200 mb-4">Performance Trends</h3>
              <div className="space-y-3">
                <div className="bg-zinc-950 rounded p-3 aspect-video flex items-center justify-center">
                  <div className="text-xs text-zinc-500">CPU Usage Chart</div>
                </div>
                <div className="bg-zinc-950 rounded p-3 aspect-video flex items-center justify-center">
                  <div className="text-xs text-zinc-500">Memory Usage Chart</div>
                </div>
                <div className="bg-zinc-950 rounded p-3 aspect-video flex items-center justify-center">
                  <div className="text-xs text-zinc-500">Network Latency Chart</div>
                </div>
              </div>
            </div>

            {/* Mission statistics */}
            <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
              <h3 className="text-sm font-semibold text-zinc-200 mb-4">Mission Statistics</h3>
              <div className="space-y-3 text-sm">
                <div className="flex items-center justify-between">
                  <span className="text-zinc-400">Total</span>
                  <span className="text-zinc-200 font-medium">{missionStats.total} missions</span>
                </div>
                <div className="flex items-center justify-between">
                  <span className="text-zinc-400">Success</span>
                  <span className="text-green-400 font-medium">
                    {missionStats.success} ({Math.round(missionStats.success / missionStats.total * 100)}%)
                  </span>
                </div>
                <div className="flex items-center justify-between">
                  <span className="text-zinc-400">Failed</span>
                  <span className="text-red-400 font-medium">
                    {missionStats.failed} ({Math.round(missionStats.failed / missionStats.total * 100)}%)
                  </span>
                </div>
                <div className="flex items-center justify-between">
                  <span className="text-zinc-400">Avg Duration</span>
                  <span className="text-zinc-200 font-medium">{missionStats.avgDuration}</span>
                </div>
                <div className="pt-3 border-t border-zinc-800">
                  <div className="text-xs text-zinc-400 mb-1">Last Mission</div>
                  <div className="flex items-center gap-2">
                    <CheckCircle2 className="w-4 h-4 text-green-400" />
                    <span className="text-zinc-200">{missionStats.lastMission.name}</span>
                    <span className="text-zinc-500 text-xs">({missionStats.lastMission.time})</span>
                  </div>
                </div>
              </div>
            </div>
          </div>

          <div className="grid grid-cols-2 gap-4">
            {/* Component health */}
            <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
              <h3 className="text-sm font-semibold text-zinc-200 mb-4">Component Health</h3>
              <div className="space-y-3">
                {Object.entries(componentHealth).map(([component, health]) => (
                  <div key={component}>
                    <div className="flex items-center justify-between text-xs mb-1">
                      <span className="text-zinc-400 capitalize">{component}</span>
                      <span className="text-zinc-200 font-medium">{health}%</span>
                    </div>
                    <div className="w-full bg-zinc-800 rounded-full h-2">
                      <div
                        className={`${getHealthColor(health)} h-2 rounded-full transition-all`}
                        style={{ width: `${health}%` }}
                      />
                    </div>
                  </div>
                ))}
              </div>
            </div>

            {/* Alerts & warnings */}
            <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
              <h3 className="text-sm font-semibold text-zinc-200 mb-4">Alerts & Warnings</h3>
              <div className="space-y-2">
                {alerts.length > 0 ? (
                  alerts.map(alert => (
                    <div key={alert.id} className="flex items-center gap-2 text-xs">
                      <AlertTriangle className={`w-4 h-4 ${
                        alert.type === 'error' ? 'text-red-400' : 'text-yellow-400'
                      }`} />
                      <span className="text-zinc-300">{alert.message}</span>
                      <span className="text-zinc-500 ml-auto">({alert.component})</span>
                    </div>
                  ))
                ) : (
                  <div className="flex items-center gap-2 text-xs text-zinc-500">
                    <CheckCircle2 className="w-4 h-4 text-green-400" />
                    <span>All systems nominal</span>
                  </div>
                )}
              </div>
            </div>
          </div>
        </div>
      )}

      {/* Historical view */}
      {activeView === 'historical' && (
        <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
          <h3 className="text-sm font-semibold text-zinc-200 mb-4">Historical Data</h3>
          <div className="text-sm text-zinc-400">
            Historical data visualization would go here.
          </div>
        </div>
      )}

      {/* Mission reports */}
      {activeView === 'missions' && (
        <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
          <h3 className="text-sm font-semibold text-zinc-200 mb-4">Mission Reports</h3>
          <div className="text-sm text-zinc-400">
            Mission reports would be listed here.
          </div>
        </div>
      )}

      {/* Export */}
      {activeView === 'export' && (
        <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
          <h3 className="text-sm font-semibold text-zinc-200 mb-4">Export Data</h3>
          <div className="space-y-2">
            <button className="w-full px-4 py-2 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-sm rounded text-left">
              Export Telemetry (CSV)
            </button>
            <button className="w-full px-4 py-2 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-sm rounded text-left">
              Export Mission Logs (JSON)
            </button>
            <button className="w-full px-4 py-2 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-sm rounded text-left">
              Export Performance Data (CSV)
            </button>
          </div>
        </div>
      )}
    </div>
  );
};




