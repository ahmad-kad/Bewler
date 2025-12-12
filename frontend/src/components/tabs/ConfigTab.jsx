import React, { useState } from 'react';
import { useSystemContext } from '../../context/SystemContext';
import { Save, RefreshCw } from 'lucide-react';

/**
 * ConfigTab Component
 *
 * System configuration interface.
 * Simple, clean configuration management.
 */
export const ConfigTab = () => {
  const {
    systemStatus,
    telemetry
  } = useSystemContext();

  const [config, setConfig] = useState({
    rosBridgeUrl: 'ws://localhost:9090',
    updateInterval: 1000,
    logLevel: 'INFO',
    autoReconnect: true
  });

  const handleSave = () => {
    // Save configuration
    console.log('Saving config:', config);
  };

  return (
    <div className="p-4 space-y-4">
      <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
        <h2 className="text-base font-semibold text-zinc-200 mb-4">System Configuration</h2>

        <div className="space-y-4">
          <div>
            <label className="block text-xs text-zinc-400 mb-1">ROS Bridge URL</label>
            <input
              type="text"
              value={config.rosBridgeUrl}
              onChange={(e) => setConfig({ ...config, rosBridgeUrl: e.target.value })}
              className="w-full px-3 py-2 bg-zinc-800 border border-zinc-700 rounded text-sm text-zinc-200"
            />
          </div>

          <div>
            <label className="block text-xs text-zinc-400 mb-1">Update Interval (ms)</label>
            <input
              type="number"
              value={config.updateInterval}
              onChange={(e) => setConfig({ ...config, updateInterval: parseInt(e.target.value) })}
              className="w-full px-3 py-2 bg-zinc-800 border border-zinc-700 rounded text-sm text-zinc-200"
            />
          </div>

          <div>
            <label className="block text-xs text-zinc-400 mb-1">Log Level</label>
            <select
              value={config.logLevel}
              onChange={(e) => setConfig({ ...config, logLevel: e.target.value })}
              className="w-full px-3 py-2 bg-zinc-800 border border-zinc-700 rounded text-sm text-zinc-200"
            >
              <option value="DEBUG">DEBUG</option>
              <option value="INFO">INFO</option>
              <option value="WARN">WARN</option>
              <option value="ERROR">ERROR</option>
            </select>
          </div>

          <div className="flex items-center gap-2">
            <input
              type="checkbox"
              checked={config.autoReconnect}
              onChange={(e) => setConfig({ ...config, autoReconnect: e.target.checked })}
              className="w-4 h-4"
            />
            <label className="text-sm text-zinc-300">Auto Reconnect</label>
          </div>

          <div className="flex items-center gap-2 pt-4 border-t border-zinc-800">
            <button
              onClick={handleSave}
              className="px-4 py-2 bg-cyan-600 hover:bg-cyan-700 text-white text-sm rounded flex items-center gap-2"
            >
              <Save className="w-4 h-4" />
              Save
            </button>
            <button className="px-4 py-2 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-sm rounded flex items-center gap-2">
              <RefreshCw className="w-4 h-4" />
              Reset
            </button>
          </div>
        </div>
      </div>
    </div>
  );
};
