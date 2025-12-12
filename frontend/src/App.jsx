import React, { useState } from 'react';
import { SystemContextProvider } from './context/SystemContext';
import { TopBar } from './components/TopBar';
import { TabNavigation } from './components/TabNavigation';
import { OverviewTab } from './components/tabs/OverviewTab';
import { MissionTab } from './components/tabs/MissionTab';
import { TestingTab } from './components/tabs/TestingTab';
import { DebugTab } from './components/tabs/DebugTab';
import { AnalyticsTab } from './components/tabs/AnalyticsTab';
import { ConfigTab } from './components/tabs/ConfigTab';

/**
 * Main App Component
 *
 * Clean, modular, context-aware interface for URC 2026 rover control system.
 * Features:
 * - Tab-based navigation
 * - Context-aware UI that adapts to system state
 * - Information-dense displays
 * - Progressive disclosure
 */
function App() {
  const [activeTab, setActiveTab] = useState('overview');

  const renderTabContent = () => {
    switch (activeTab) {
      case 'overview':
        return <OverviewTab />;
      case 'mission':
        return <MissionTab />;
      case 'testing':
        return <TestingTab />;
      case 'debug':
        return <DebugTab />;
      case 'analytics':
        return <AnalyticsTab />;
      case 'config':
        return <ConfigTab />;
      default:
        return <OverviewTab />;
    }
  };

  return (
    <SystemContextProvider>
      <div className="h-screen w-screen flex flex-col bg-zinc-950 text-zinc-100 overflow-hidden">
        {/* Top bar - always visible */}
        <TopBar />

        {/* Tab navigation */}
        <TabNavigation activeTab={activeTab} setActiveTab={setActiveTab} />

        {/* Tab content - scrollable */}
        <div className="flex-1 overflow-y-auto">
          {renderTabContent()}
        </div>
      </div>
    </SystemContextProvider>
  );
}

export default App;
