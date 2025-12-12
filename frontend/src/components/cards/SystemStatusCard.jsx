import React from 'react';
import { CheckCircle2, XCircle, AlertCircle, Loader2 } from 'lucide-react';

/**
 * SystemStatusCard Component
 *
 * Displays system status with color-coded indicators.
 * Context-aware: shows relevant systems based on current state.
 */
export const SystemStatusCard = ({ systems, title = 'System Status', showDetails = false }) => {
  const getStatusIcon = (status) => {
    switch (status) {
      case 'ok':
      case 'ready':
      case 'operational':
        return <CheckCircle2 className="w-4 h-4 text-green-400" />;
      case 'degraded':
      case 'warning':
        return <AlertCircle className="w-4 h-4 text-yellow-400" />;
      case 'error':
      case 'failed':
        return <XCircle className="w-4 h-4 text-red-400" />;
      case 'busy':
      case 'active':
        return <Loader2 className="w-4 h-4 text-blue-400 animate-spin" />;
      default:
        return <AlertCircle className="w-4 h-4 text-zinc-400" />;
    }
  };

  const getStatusColor = (status) => {
    switch (status) {
      case 'ok':
      case 'ready':
      case 'operational':
        return 'text-green-400';
      case 'degraded':
      case 'warning':
        return 'text-yellow-400';
      case 'error':
      case 'failed':
        return 'text-red-400';
      case 'busy':
      case 'active':
        return 'text-blue-400';
      default:
        return 'text-zinc-400';
    }
  };

  return (
    <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
      <h3 className="text-sm font-semibold text-zinc-200 mb-3">{title}</h3>
      <div className="space-y-2">
        {Object.entries(systems).map(([key, status]) => (
          <div key={key} className="flex items-center justify-between">
            <div className="flex items-center gap-2">
              {getStatusIcon(status)}
              <span className="text-sm text-zinc-300 capitalize">{key}</span>
            </div>
            <span className={`text-xs font-medium ${getStatusColor(status)}`}>
              {status.toUpperCase()}
            </span>
          </div>
        ))}
      </div>
    </div>
  );
};
