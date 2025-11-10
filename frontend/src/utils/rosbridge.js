import ROSLIB from 'roslib';

/**
 * ROS Bridge utilities for simplified ROS communication
 */

/**
 * Create a ROS Topic subscriber
 * @param {ROSLIB.Ros} ros - ROS connection instance
 * @param {string} topicName - Topic name
 * @param {string} messageType - ROS message type
 * @param {function} callback - Callback function for received messages
 * @returns {ROSLIB.Topic} Topic subscriber instance
 */
export const createSubscriber = (ros, topicName, messageType, callback) => {
  const topic = new ROSLIB.Topic({
    ros: ros,
    name: topicName,
    messageType: messageType
  });

  topic.subscribe(callback);
  return topic;
};

/**
 * Create a ROS Topic publisher
 * @param {ROSLIB.Ros} ros - ROS connection instance
 * @param {string} topicName - Topic name
 * @param {string} messageType - ROS message type
 * @returns {ROSLIB.Topic} Topic publisher instance
 */
export const createPublisher = (ros, topicName, messageType) => {
  return new ROSLIB.Topic({
    ros: ros,
    name: topicName,
    messageType: messageType
  });
};

/**
 * Create a ROS Service client
 * @param {ROSLIB.Ros} ros - ROS connection instance
 * @param {string} serviceName - Service name
 * @param {string} serviceType - ROS service type
 * @returns {ROSLIB.Service} Service client instance
 */
export const createServiceClient = (ros, serviceName, serviceType) => {
  return new ROSLIB.Service({
    ros: ros,
    name: serviceName,
    serviceType: serviceType
  });
};

/**
 * Call a ROS service
 * @param {ROSLIB.Service} serviceClient - Service client instance
 * @param {Object} request - Service request object
 * @returns {Promise} Promise that resolves with service response
 */
export const callService = (serviceClient, request) => {
  return new Promise((resolve, reject) => {
    serviceClient.callService(request, (response) => {
      resolve(response);
    }, (error) => {
      reject(error);
    });
  });
};

/**
 * Create ROS parameter client
 * @param {ROSLIB.Ros} ros - ROS connection instance
 * @param {string} paramName - Parameter name
 * @returns {ROSLIB.Param} Parameter instance
 */
export const createParam = (ros, paramName) => {
  return new ROSLIB.Param({
    ros: ros,
    name: paramName
  });
};

/**
 * Create common ROS message types
 */
export const createMessage = {
  /**
   * Create a String message
   * @param {string} data - String data
   * @returns {Object} ROS String message
   */
  string: (data) => ({ data }),

  /**
   * Create a PoseStamped message
   * @param {Object} pose - Pose data
   * @param {string} frameId - Frame ID
   * @returns {Object} ROS PoseStamped message
   */
  poseStamped: (pose, frameId = 'map') => ({
    header: {
      stamp: { sec: Math.floor(Date.now() / 1000), nanosec: 0 },
      frame_id: frameId
    },
    pose: pose
  }),

  /**
   * Create a Point message
   * @param {number} x - X coordinate
   * @param {number} y - Y coordinate
   * @param {number} z - Z coordinate
   * @returns {Object} ROS Point message
   */
  point: (x, y, z) => ({ x, y, z }),

  /**
   * Create a Quaternion message
   * @param {number} x - X component
   * @param {number} y - Y component
   * @param {number} z - Z component
   * @param {number} w - W component
   * @returns {Object} ROS Quaternion message
   */
  quaternion: (x = 0, y = 0, z = 0, w = 1) => ({ x, y, z, w }),

  /**
   * Create a Twist message for velocity commands
   * @param {number} linearX - Linear velocity in X
   * @param {number} angularZ - Angular velocity around Z
   * @returns {Object} ROS Twist message
   */
  twist: (linearX = 0, angularZ = 0) => ({
    linear: { x: linearX, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: angularZ }
  })
};

/**
 * Connection status utilities
 */
export const connectionStatus = {
  CONNECTED: 'connected',
  CONNECTING: 'connecting',
  DISCONNECTED: 'disconnected',
  ERROR: 'error',
  FAILED: 'failed'
};

/**
 * Get human-readable connection status
 * @param {string} status - Connection status
 * @returns {string} Human-readable status
 */
export const getStatusText = (status) => {
  switch (status) {
    case connectionStatus.CONNECTED:
      return 'Connected to ROS';
    case connectionStatus.CONNECTING:
      return 'Connecting to ROS...';
    case connectionStatus.DISCONNECTED:
      return 'Disconnected from ROS';
    case connectionStatus.ERROR:
      return 'Connection error';
    case connectionStatus.FAILED:
      return 'Failed to connect';
    default:
      return 'Unknown status';
  }
};
