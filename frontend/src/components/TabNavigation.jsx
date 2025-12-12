import React from 'react';
import { useSystemContext } from '../context/SystemContext';
import { AlertTriangle, TestTube, Bug, BarChart3, Settings, LayoutDashboard, Target } from 'lucide-react';

/**
 * Tab Navigation Component
 *
 * Context-aware tab navigation with indicators for active missions, errors, and tests.
 */
export const TabNavigation = ({ activeTab, setActiveTab }) => {
  const {
    activeMission,
    errorCount,
    runningTests
  } = useSystemContext();

  const tabs = [
    { id: 'overview', label: 'Overview', icon: LayoutDashboard },
    { id: 'mission', label: 'Mission', icon: Target, badge: activeMission ? `${Math.round(activeMission.progress)}%` : null },
    { id: 'testing', label: 'Testing', icon: TestTube, badge: runningTests > 0 ? runningTests : null },
    { id: 'debug', label: 'Debug', icon: Bug, badge: errorCount > 0 ? errorCount : null },
    { id: 'analytics', label: 'Analytics', icon: BarChart3 },
    { id: 'config', label: 'Config', icon: Settings }
  ];

  return (
    <div className="bg-zinc-800 border-b border-zinc-700 flex items-center gap-1 px-2">
      {tabs.map(tab => {
        const Icon = tab.icon;
        const isActive = activeTab === tab.id;

        return (
          <button
            key={tab.id}
            onClick={() => setActiveTab(tab.id)}
            className={`
              relative flex items-center gap-2 px-4 py-2.5 text-sm font-medium rounded-t transition-colors
              ${isActive
                ? 'bg-zinc-900 text-zinc-100 border-t border-x border-zinc-700'
                : 'text-zinc-400 hover:text-zinc-200 hover:bg-zinc-800/50'
              }
            `}
          >
            <Icon className="w-4 h-4" />
            <span>{tab.label}</span>
            {tab.badge && (
              <span className={`
                px-1.5 py-0.5 text-xs rounded
                ${isActive
                  ? 'bg-zinc-700 text-zinc-200'
                  : 'bg-zinc-700/50 text-zinc-400'
                }
              `}>
                {tab.badge}
              </span>
            )}
          </button>
        );
      })}
    </div>
  );
};
