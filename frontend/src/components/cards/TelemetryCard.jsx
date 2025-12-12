import React from 'react';

/**
 * TelemetryCard Component
 *
 * Displays critical telemetry data in a compact format.
 */
export const TelemetryCard = ({ telemetry, title = 'Telemetry' }) => {
  return (
    <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
      <h3 className="text-sm font-semibold text-zinc-200 mb-3">{title}</h3>
      <div className="space-y-2">
        <div className="flex items-center justify-between">
          <span className="text-sm text-zinc-400">Battery</span>
          <span className={`text-sm font-medium ${
            telemetry.battery > 50 ? 'text-green-400' :
            telemetry.battery > 20 ? 'text-yellow-400' :
            'text-red-400'
          }`}>
            {Math.round(telemetry.battery)}%
          </span>
        </div>

        <div className="flex items-center justify-between">
          <span className="text-sm text-zinc-400">Position</span>
          <span className="text-sm font-medium text-zinc-200">
            {telemetry.gps.position.lat.toFixed(3)}°, {telemetry.gps.position.lon.toFixed(3)}°
          </span>
        </div>

        <div className="flex items-center justify-between">
          <span className="text-sm text-zinc-400">Speed</span>
          <span className="text-sm font-medium text-zinc-200">
            {telemetry.speed.toFixed(1)} m/s
          </span>
        </div>

        <div className="flex items-center justify-between">
          <span className="text-sm text-zinc-400">Temperature</span>
          <span className="text-sm font-medium text-zinc-200">
            {Math.round(telemetry.temperature)}°C
          </span>
        </div>

        <div className="flex items-center justify-between">
          <span className="text-sm text-zinc-400">GPS Satellites</span>
          <span className="text-sm font-medium text-zinc-200">
            {telemetry.gps.satellites}
          </span>
        </div>
      </div>
    </div>
  );
};
