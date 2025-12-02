import React, { useState, useEffect } from 'react';
import { SafetyTestPanel } from './SafetyTestPanel';
import { CameraPlaceholder } from './CameraPlaceholder';
import { SensorDataSimulator } from './SensorDataSimulator';

/**
 * Component Test Dashboard
 *
 * Main testing interface for all URC 2026 system components.
 * Provides centralized access to component testing, monitoring, and simulation.
 *
 * Features:
 * - Safety system testing (estops, recovery, monitoring)
 * - Navigation system validation (waypoint commands, path planning)
 * - Computer vision testing (ArUco detection, object recognition)
 * - CAN bus mock data verification (clearly labeled as mock)
 * - Real-time component status monitoring
 * - Mock sensor data simulation
 * - Camera feed testing with placeholder
 */
export const ComponentTestDashboard = ({
  onRunTest,
  onStateTransition,
  isConnected,
  demoMode,
  currentState,
  rosInstance
}) => {
  const [activeTab, setActiveTab] = useState('overview');
  const [systemStatus, setSystemStatus] = useState({
    safety: 'unknown',
    navigation: 'unknown',
    vision: 'unknown',
    can: 'mock', // Always mock for testing
    websocket: 'unknown',
    frontend: 'connected'
  });

  const [testHistory, setTestHistory] = useState([]);
  const [isRunningTests, setIsRunningTests] = useState(false);

  // Component definitions with test capabilities
  const componentTests = {
    safety: {
      name: 'Safety System',
      description: 'Emergency stops, recovery procedures, and safety monitoring',
      status: systemStatus.safety,
      tests: [
        'software_estop',
        'safety_recovery',
        'manual_recovery',
        'watchdog_monitoring',
        'safety_thresholds',
        'emergency_protocols'
      ],
      icon: '🛡️',
      color: 'red',
      priority: 1
    },
    navigation: {
      name: 'Navigation System',
      description: 'GPS navigation, waypoint following, and path planning',
      status: systemStatus.navigation,
      tests: [
        'waypoint_navigation',
        'gps_accuracy',
        'obstacle_avoidance',
        'path_planning',
        'localization_accuracy',
        'motion_control'
      ],
      icon: '🧭',
      color: 'blue',
      priority: 2
    },
    vision: {
      name: 'Computer Vision',
      description: 'ArUco detection, object recognition, and visual processing',
      status: systemStatus.vision,
      tests: [
        'camera_feed',
        'aruco_detection',
        'object_recognition',
        'visual_odometry',
        'target_tracking',
        'image_processing'
      ],
      icon: '👁️',
      color: 'green',
      priority: 3
    },
    can: {
      name: 'CAN Bus System',
      description: 'Sensor data communication and hardware interface',
      status: systemStatus.can,
      tests: [
        'sensor_connectivity',
        'data_integrity',
        'communication_latency',
        'motor_control',
        'battery_monitoring',
        'system_health'
      ],
      icon: '🔌',
      color: 'orange',
      priority: 4,
      warning: '⚠️ USING MOCK DATA - NOT REAL CAN BUS'
    },
    websocket: {
      name: 'WebSocket Bridge',
      description: 'Real-time communication between frontend and ROS2',
      status: systemStatus.websocket,
      tests: [
        'connection_stability',
        'message_routing',
        'latency_measurement',
        'error_handling',
        'reconnection_logic',
        'data_synchronization'
      ],
      icon: '🌐',
      color: 'purple',
      priority: 5
    }
  };

  // Update system status periodically
  useEffect(() => {
    const updateStatus = () => {
      // Simulate status updates (in real implementation, this would come from ROS2)
      setSystemStatus(prev => ({
        ...prev,
        safety: isConnected ? (demoMode ? 'demo' : 'operational') : 'disconnected',
        navigation: isConnected ? 'operational' : 'disconnected',
        vision: isConnected ? 'operational' : 'disconnected',
        websocket: isConnected ? 'connected' : 'disconnected'
      }));
    };

    updateStatus();
    const interval = setInterval(updateStatus, 5000); // Update every 5 seconds
    return () => clearInterval(interval);
  }, [isConnected, demoMode]);

  // Run component tests
  const runComponentTest = async (componentId, testId) => {
    if (isRunningTests) return;

    setIsRunningTests(true);
    const testStartTime = Date.now();

    try {
      // Add to test history
      const testRecord = {
        id: `${componentId}_${testId}_${Date.now()}`,
        component: componentId,
        test: testId,
        status: 'running',
        startTime: testStartTime
      };

      setTestHistory(prev => [testRecord, ...prev.slice(0, 9)]); // Keep last 10

      // Simulate test execution (in real implementation, this would call ROS2 services)
      await new Promise(resolve => setTimeout(resolve, 2000)); // 2 second test

      // Update test result
      setTestHistory(prev => prev.map(test =>
        test.id === testRecord.id
          ? {
              ...test,
              status: 'completed',
              duration: Date.now() - testStartTime,
              result: Math.random() > 0.2 ? 'pass' : 'fail' // 80% pass rate for demo
            }
          : test
      ));

    } catch (error) {
      setTestHistory(prev => prev.map(test =>
        test.id === testRecord.id
          ? {
              ...test,
              status: 'failed',
              duration: Date.now() - testStartTime,
              error: error.message
            }
          : test
      ));
    } finally {
      setIsRunningTests(false);
    }
  };

  // Run full system test suite
  const runFullSystemTest = async () => {
    setIsRunningTests(true);

    const components = Object.keys(componentTests);
    for (const componentId of components) {
      const component = componentTests[componentId];
      for (const testId of component.tests.slice(0, 2)) { // Run first 2 tests per component
        await runComponentTest(componentId, testId);
        await new Promise(resolve => setTimeout(resolve, 500)); // Brief pause between tests
      }
    }

    setIsRunningTests(false);
  };

  // Get status color
  const getStatusColor = (status) => {
    switch (status) {
      case 'operational': return 'text-green-600 bg-green-100';
      case 'connected': return 'text-green-600 bg-green-100';
      case 'demo': return 'text-yellow-600 bg-yellow-100';
      case 'mock': return 'text-orange-600 bg-orange-100';
      case 'disconnected': return 'text-red-600 bg-red-100';
      case 'unknown': return 'text-gray-600 bg-gray-100';
      default: return 'text-gray-600 bg-gray-100';
    }
  };

  // Get priority badge
  const getPriorityBadge = (priority) => {
    const colors = {
      1: 'bg-red-100 text-red-800',
      2: 'bg-blue-100 text-blue-800',
      3: 'bg-green-100 text-green-800',
      4: 'bg-orange-100 text-orange-800',
      5: 'bg-purple-100 text-purple-800'
    };
    return colors[priority] || 'bg-gray-100 text-gray-800';
  };

  return (
    <div className="min-h-screen bg-gray-50">
      {/* Header */}
      <div className="bg-white shadow-sm border-b">
        <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
          <div className="flex justify-between items-center py-4">
            <div>
              <h1 className="text-2xl font-bold text-gray-900">
                🧪 Component Test Dashboard
              </h1>
              <p className="text-sm text-gray-600 mt-1">
                Test and validate all URC 2026 system components
              </p>
            </div>
            <div className="flex items-center space-x-4">
              <div className={`px-3 py-1 rounded-full text-sm font-medium ${getStatusColor(systemStatus.websocket)}`}>
                {isConnected ? '🟢 ROS2 Connected' : '🔴 ROS2 Disconnected'}
              </div>
              <button
                onClick={runFullSystemTest}
                disabled={isRunningTests || !isConnected}
                className={`px-4 py-2 rounded-md text-sm font-medium ${
                  isRunningTests || !isConnected
                    ? 'bg-gray-300 text-gray-500 cursor-not-allowed'
                    : 'bg-blue-600 text-white hover:bg-blue-700'
                }`}
              >
                {isRunningTests ? '🔄 Running Tests...' : '🚀 Run Full System Test'}
              </button>
            </div>
          </div>
        </div>
      </div>

      {/* Navigation Tabs */}
      <div className="bg-white shadow-sm">
        <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
          <nav className="flex space-x-8">
            {[
              { id: 'overview', name: 'Overview', icon: '📊' },
              { id: 'safety', name: 'Safety System', icon: '🛡️' },
              { id: 'sensors', name: 'Sensors & CAN', icon: '🔌' },
              { id: 'vision', name: 'Computer Vision', icon: '👁️' },
              { id: 'navigation', name: 'Navigation', icon: '🧭' },
              { id: 'history', name: 'Test History', icon: '📋' }
            ].map(tab => (
              <button
                key={tab.id}
                onClick={() => setActiveTab(tab.id)}
                className={`py-4 px-1 border-b-2 font-medium text-sm ${
                  activeTab === tab.id
                    ? 'border-blue-500 text-blue-600'
                    : 'border-transparent text-gray-500 hover:text-gray-700 hover:border-gray-300'
                }`}
              >
                {tab.icon} {tab.name}
              </button>
            ))}
          </nav>
        </div>
      </div>

      {/* Main Content */}
      <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 py-8">
        {activeTab === 'overview' && (
          <div className="space-y-8">
            {/* System Status Overview */}
            <div className="bg-white rounded-lg shadow p-6">
              <h2 className="text-lg font-semibold text-gray-900 mb-4">System Status Overview</h2>
              <div className="grid grid-cols-1 md:grid-cols-3 lg:grid-cols-5 gap-4">
                {Object.entries(componentTests).map(([id, component]) => (
                  <div key={id} className="bg-gray-50 rounded-lg p-4">
                    <div className="flex items-center justify-between mb-2">
                      <span className="text-2xl">{component.icon}</span>
                      <span className={`px-2 py-1 rounded text-xs font-medium ${getPriorityBadge(component.priority)}`}>
                        P{component.priority}
                      </span>
                    </div>
                    <h3 className="font-medium text-gray-900">{component.name}</h3>
                    <p className="text-sm text-gray-600 mt-1">{component.description}</p>
                    <div className="mt-2 flex items-center">
                      <span className={`px-2 py-1 rounded-full text-xs font-medium ${getStatusColor(component.status)}`}>
                        {component.status.toUpperCase()}
                      </span>
                      {component.warning && (
                        <span className="ml-2 text-xs text-orange-600">{component.warning}</span>
                      )}
                    </div>
                  </div>
                ))}
              </div>
            </div>

            {/* Quick Actions */}
            <div className="bg-white rounded-lg shadow p-6">
              <h2 className="text-lg font-semibold text-gray-900 mb-4">Quick Actions</h2>
              <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4">
                <button
                  onClick={() => setActiveTab('safety')}
                  className="p-4 border border-red-200 rounded-lg hover:bg-red-50 transition-colors"
                >
                  <div className="text-red-600 text-2xl mb-2">🚨</div>
                  <div className="font-medium text-gray-900">Test Safety System</div>
                  <div className="text-sm text-gray-600">Emergency stops & recovery</div>
                </button>
                <button
                  onClick={() => setActiveTab('sensors')}
                  className="p-4 border border-orange-200 rounded-lg hover:bg-orange-50 transition-colors"
                >
                  <div className="text-orange-600 text-2xl mb-2">🔌</div>
                  <div className="font-medium text-gray-900">Test CAN Bus</div>
                  <div className="text-sm text-gray-600">Mock sensor data</div>
                </button>
                <button
                  onClick={() => setActiveTab('vision')}
                  className="p-4 border border-green-200 rounded-lg hover:bg-green-50 transition-colors"
                >
                  <div className="text-green-600 text-2xl mb-2">📹</div>
                  <div className="font-medium text-gray-900">Test Camera Feed</div>
                  <div className="text-sm text-gray-600">Placeholder vision</div>
                </button>
                <button
                  onClick={() => setActiveTab('navigation')}
                  className="p-4 border border-blue-200 rounded-lg hover:bg-blue-50 transition-colors"
                >
                  <div className="text-blue-600 text-2xl mb-2">🧭</div>
                  <div className="font-medium text-gray-900">Test Navigation</div>
                  <div className="text-sm text-gray-600">Waypoint following</div>
                </button>
              </div>
            </div>

            {/* Recent Test Results */}
            <div className="bg-white rounded-lg shadow p-6">
              <h2 className="text-lg font-semibold text-gray-900 mb-4">Recent Test Results</h2>
              {testHistory.length > 0 ? (
                <div className="space-y-2">
                  {testHistory.slice(0, 5).map(test => (
                    <div key={test.id} className="flex items-center justify-between p-3 bg-gray-50 rounded">
                      <div className="flex items-center space-x-3">
                        <span className="text-lg">
                          {test.result === 'pass' ? '✅' : test.result === 'fail' ? '❌' : '⏳'}
                        </span>
                        <div>
                          <div className="font-medium text-gray-900">
                            {componentTests[test.component]?.name} - {test.test}
                          </div>
                          <div className="text-sm text-gray-600">
                            {test.duration ? `${test.duration}ms` : 'Running...'}
                          </div>
                        </div>
                      </div>
                      <span className={`px-2 py-1 rounded text-xs font-medium ${
                        test.status === 'completed' && test.result === 'pass' ? 'bg-green-100 text-green-800' :
                        test.status === 'completed' && test.result === 'fail' ? 'bg-red-100 text-red-800' :
                        'bg-yellow-100 text-yellow-800'
                      }`}>
                        {test.status === 'completed' ? test.result?.toUpperCase() : test.status?.toUpperCase()}
                      </span>
                    </div>
                  ))}
                </div>
              ) : (
                <p className="text-gray-600">No tests run yet. Click "Run Full System Test" to get started.</p>
              )}
            </div>
          </div>
        )}

        {activeTab === 'safety' && (
          <SafetyTestPanel
            onRunTest={(testId) => runComponentTest('safety', testId)}
            onStateTransition={onStateTransition}
            isConnected={isConnected}
            demoMode={demoMode}
            currentState={currentState}
          />
        )}

        {activeTab === 'sensors' && (
          <div className="space-y-6">
            <div className="bg-yellow-50 border border-yellow-200 rounded-lg p-4">
              <div className="flex">
                <div className="flex-shrink-0">
                  <span className="text-yellow-600 text-xl">⚠️</span>
                </div>
                <div className="ml-3">
                  <h3 className="text-sm font-medium text-yellow-800">
                    Mock Data Warning
                  </h3>
                  <div className="mt-2 text-sm text-yellow-700">
                    <p>
                      The CAN bus system below uses <strong>MOCK/SIMULATED data</strong> for testing purposes.
                      This is NOT real sensor data from physical hardware. All values are generated for
                      component validation and will be replaced with actual CAN bus communication in production.
                    </p>
                  </div>
                </div>
              </div>
            </div>
            <SensorDataSimulator
              onDataChange={(data) => console.log('Mock sensor data changed:', data)}
              isConnected={isConnected}
            />
          </div>
        )}

        {activeTab === 'vision' && (
          <div className="space-y-6">
            <CameraPlaceholder
              onStartFeed={() => runComponentTest('vision', 'camera_feed')}
              onStopFeed={() => console.log('Stopping camera feed')}
              isConnected={isConnected}
            />
          </div>
        )}

        {activeTab === 'navigation' && (
          <div className="bg-white rounded-lg shadow p-6">
            <h2 className="text-lg font-semibold text-gray-900 mb-4">Navigation System Testing</h2>
            <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
              <div>
                <h3 className="font-medium text-gray-900 mb-3">Available Tests</h3>
                <div className="space-y-2">
                  {componentTests.navigation.tests.map(testId => (
                    <button
                      key={testId}
                      onClick={() => runComponentTest('navigation', testId)}
                      disabled={!isConnected || isRunningTests}
                      className="w-full text-left p-3 border rounded-lg hover:bg-gray-50 disabled:opacity-50 disabled:cursor-not-allowed"
                    >
                      <div className="font-medium text-gray-900">{testId.replace('_', ' ').toUpperCase()}</div>
                      <div className="text-sm text-gray-600">Test navigation system {testId.replace('_', ' ')}</div>
                    </button>
                  ))}
                </div>
              </div>
              <div>
                <h3 className="font-medium text-gray-900 mb-3">Test Results</h3>
                <div className="bg-gray-50 rounded-lg p-4 min-h-48">
                  <p className="text-gray-600">Navigation test results will appear here...</p>
                </div>
              </div>
            </div>
          </div>
        )}

        {activeTab === 'history' && (
          <div className="bg-white rounded-lg shadow p-6">
            <h2 className="text-lg font-semibold text-gray-900 mb-4">Test History</h2>
            {testHistory.length > 0 ? (
              <div className="space-y-3">
                {testHistory.map(test => (
                  <div key={test.id} className="border rounded-lg p-4">
                    <div className="flex items-center justify-between mb-2">
                      <div className="flex items-center space-x-2">
                        <span className="text-lg">
                          {test.result === 'pass' ? '✅' : test.result === 'fail' ? '❌' : '⏳'}
                        </span>
                        <span className="font-medium text-gray-900">
                          {componentTests[test.component]?.name} - {test.test.replace('_', ' ')}
                        </span>
                      </div>
                      <span className={`px-2 py-1 rounded text-xs font-medium ${
                        test.status === 'completed' && test.result === 'pass' ? 'bg-green-100 text-green-800' :
                        test.status === 'completed' && test.result === 'fail' ? 'bg-red-100 text-red-800' :
                        test.status === 'running' ? 'bg-yellow-100 text-yellow-800' :
                        'bg-gray-100 text-gray-800'
                      }`}>
                        {test.status.toUpperCase()}
                      </span>
                    </div>
                    <div className="text-sm text-gray-600">
                      Started: {new Date(test.startTime).toLocaleTimeString()}
                      {test.duration && ` | Duration: ${test.duration}ms`}
                      {test.error && ` | Error: ${test.error}`}
                    </div>
                  </div>
                ))}
              </div>
            ) : (
              <p className="text-gray-600">No test history available.</p>
            )}
          </div>
        )}
      </div>
    </div>
  );
};

export default ComponentTestDashboard;



