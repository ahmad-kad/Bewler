/**
 * ROS2 Topic Configuration for URC Testing Frontend
 *
 * Defines all ROS2 topics used for communication between the frontend
 * and the rover's ROS2 system via rosbridge_server.
 */

// State Management Topics
export const STATE_TOPICS = {
  // Current state information
  CURRENT_STATE: '/state_machine/current_state',
  STATE_TRANSITION: '/state_machine/state_transition',
  SUBSTATE: '/state_machine/substate',

  // State control services
  CHANGE_STATE_SERVICE: '/state_machine/change_state',
  GET_CURRENT_STATE_SERVICE: '/state_machine/get_current_state',
  FORCE_TRANSITION_SERVICE: '/state_machine/force_transition'
};

// Navigation & SLAM Topics
export const NAVIGATION_TOPICS = {
  // Robot pose and odometry
  POSE_FUSED: '/slam/pose/fused',
  ODOMETRY: '/odom',
  TF_STATIC: '/tf_static',
  TF: '/tf',

  // Navigation planning
  PATH: '/navigation/path',
  GOAL: '/navigation/goal',
  CMD_VEL: '/cmd_vel',

  // SLAM data
  POINT_CLOUD: '/slam/point_cloud',
  OCCUPANCY_GRID: '/map',
  SLAM_STATUS: '/slam/system/status'
};

// Camera & Vision Topics
export const CAMERA_TOPICS = {
  // Raw camera feeds
  IMAGE_RAW: '/camera/image_raw',
  IMAGE_COMPRESSED: '/camera/image_raw/compressed',
  DEPTH_IMAGE: '/camera/depth/image_raw',
  CAMERA_INFO: '/camera/camera_info',

  // Vision processing results
  DETECTIONS: '/vision/detections',
  DEBUG_IMAGE: '/vision/debug_image',
  ARUCO_MARKERS: '/vision/aruco_markers'
};

// System Monitoring Topics
export const SYSTEM_TOPICS = {
  // ROS system status
  ROSOUT: '/rosout',
  DIAGNOSTICS: '/diagnostics',
  NODE_LIST: '/ros2/node/list',

  // Hardware status
  CPU_USAGE: '/system/cpu',
  MEMORY_USAGE: '/system/memory',
  NETWORK_STATS: '/system/network',

  // LED status
  LED_STATUS: '/led/status',
  LED_COMMAND: '/led/command'
};

// Mission Control Topics
export const MISSION_TOPICS = {
  // Mission status
  MISSION_STATUS: '/mission/status',
  MISSION_PROGRESS: '/mission/progress',

  // Science operations
  SCIENCE_STATUS: '/science/status',
  SAMPLE_STATUS: '/science/sample_status',

  // Equipment servicing
  SERVICING_STATUS: '/servicing/status',
  SERVICING_PROGRESS: '/servicing/progress'
};

// Safety System Topics
export const SAFETY_TOPICS = {
  // Safety status and monitoring
  SAFETY_STATUS: '/state_machine/safety_status',
  SAFETY_VIOLATIONS: '/safety/violations',
  REDUNDANT_SAFETY_STATUS: '/safety/redundant_status',

  // Safety dashboard topics
  DASHBOARD_STATUS: '/safety/dashboard_status',
  ACTIVE_ALERTS: '/safety/active_alerts',
  SYSTEM_HEALTH: '/safety/system_health',

  // Safety monitoring
  WATCHDOG_STATUS: '/safety/watchdog_status',
  SENSOR_HEALTH: '/safety/sensor_health',
  EMERGENCY_STATUS: '/safety/emergency_status',

  // Diagnostics
  SAFETY_DIAGNOSTICS: '/safety/dashboard_diagnostics',
  REDUNDANT_DIAGNOSTICS: '/safety/redundant_diagnostics',
  EMERGENCY_DIAGNOSTICS: '/safety/emergency_diagnostics'
};

// Telemetry Topics
export const TELEMETRY_TOPICS = {
  // GPS data
  GPS_FIX: '/gps/fix',
  GPS_STATUS: '/gps/status',

  // IMU data
  IMU_DATA: '/imu/data',
  IMU_MAGNETIC: '/imu/magnetic',

  // Battery and power
  BATTERY_STATUS: '/battery/status',
  POWER_CONSUMPTION: '/power/consumption'
};

// Topic message types (for reference)
export const MESSAGE_TYPES = {
  // Standard ROS2 message types
  STRING: 'std_msgs/msg/String',
  HEADER: 'std_msgs/msg/Header',
  BOOL: 'std_msgs/msg/Bool',
  INT32: 'std_msgs/msg/Int32',
  FLOAT64: 'std_msgs/msg/Float64',

  // Geometry messages
  POSE_STAMPED: 'geometry_msgs/msg/PoseStamped',
  POSE_WITH_COVARIANCE_STAMPED: 'geometry_msgs/msg/PoseWithCovarianceStamped',
  TWIST: 'geometry_msgs/msg/Twist',
  TRANSFORM_STAMPED: 'geometry_msgs/msg/TransformStamped',

  // Navigation messages
  PATH: 'nav_msgs/msg/Path',
  OCCUPANCY_GRID: 'nav_msgs/msg/OccupancyGrid',
  ODOMETRY: 'nav_msgs/msg/Odometry',

  // Sensor messages
  IMAGE: 'sensor_msgs/msg/Image',
  COMPRESSED_IMAGE: 'sensor_msgs/msg/CompressedImage',
  CAMERA_INFO: 'sensor_msgs/msg/CameraInfo',
  POINT_CLOUD2: 'sensor_msgs/msg/PointCloud2',

  // Custom URC messages
  SYSTEM_STATE: 'autonomy_interfaces/msg/SystemState',
  STATE_TRANSITION: 'autonomy_interfaces/msg/StateTransition',
  VISION_DETECTION: 'autonomy_interfaces/msg/VisionDetection',
  LED_COMMAND: 'autonomy_interfaces/msg/LedCommand',
  SAFETY_STATUS: 'autonomy_interfaces/msg/SafetyStatus'
};

// Service types
export const SERVICE_TYPES = {
  CHANGE_STATE: 'autonomy_interfaces/srv/ChangeState',
  GET_SYSTEM_STATE: 'autonomy_interfaces/srv/GetSystemState',
  SET_GOAL: 'nav2_msgs/srv/SetGoal',
  CANCEL_GOAL: 'nav2_msgs/srv/CancelGoal',
  SOFTWARE_ESTOP: 'autonomy_interfaces/srv/SoftwareEstop',
  RECOVER_FROM_SAFETY: 'autonomy_interfaces/srv/RecoverFromSafety',
  SAFESTOP_CONTROL: 'autonomy_interfaces/srv/SafestopControl'
};

// Quality of Service (QoS) profiles for different data types
export const QOS_PROFILES = {
  // Critical state data - reliable, keep last
  CRITICAL: {
    reliability: 'reliable',
    durability: 'transient_local',
    history: 'keep_last',
    depth: 1
  },

  // High-frequency sensor data - best effort, keep last
  SENSOR: {
    reliability: 'best_effort',
    durability: 'volatile',
    history: 'keep_last',
    depth: 10
  },

  // Navigation data - reliable, keep history
  NAVIGATION: {
    reliability: 'reliable',
    durability: 'volatile',
    history: 'keep_last',
    depth: 50
  },

  // Video streams - best effort for low latency
  VIDEO: {
    reliability: 'best_effort',
    durability: 'volatile',
    history: 'keep_last',
    depth: 1
  }
};
