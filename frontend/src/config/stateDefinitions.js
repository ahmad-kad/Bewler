/**
 * State Machine Definitions for URC 2026 Frontend
 *
 * Mirrors the Python states.py definitions for consistent state management
 * between frontend and backend.
 */

// System State Enum (mirrors SystemState from states.py)
export const SystemState = {
  BOOT: "BOOT",
  CALIBRATION: "CALIBRATION",
  IDLE: "IDLE",
  TELEOPERATION: "TELEOPERATION",
  AUTONOMOUS: "AUTONOMOUS",
  SAFETY: "SAFETY",
  SHUTDOWN: "SHUTDOWN"
};

// Autonomous Substate Enum (mirrors AutonomousSubstate from states.py)
export const AutonomousSubstate = {
  NONE: "NONE",
  SCIENCE: "SCIENCE",
  DELIVERY: "DELIVERY",
  EQUIPMENT_SERVICING: "EQUIPMENT_SERVICING",
  AUTONOMOUS_NAVIGATION: "AUTONOMOUS_NAVIGATION",
  FOLLOW_ME: "FOLLOW_ME"
};

// Equipment Servicing Substate Enum (mirrors EquipmentServicingSubstate from states.py)
export const EquipmentServicingSubstate = {
  NONE: "NONE",
  TRAVELING: "TRAVELING",
  SAMPLE_DELIVERY: "SAMPLE_DELIVERY",
  PANEL_OPERATIONS: "PANEL_OPERATIONS",
  AUTONOMOUS_TYPING: "AUTONOMOUS_TYPING",
  USB_CONNECTION: "USB_CONNECTION",
  FUEL_CONNECTION: "FUEL_CONNECTION",
  BUTTON_OPERATIONS: "BUTTON_OPERATIONS",
  COMPLETE: "COMPLETE"
};

// Calibration Substate Enum (mirrors CalibrationSubstate from states.py)
export const CalibrationSubstate = {
  NONE: "NONE",
  SETUP: "SETUP",
  INTRINSIC_CAPTURE: "INTRINSIC_CAPTURE",
  INTRINSIC_CALIBRATION: "INTRINSIC_CALIBRATION",
  INTRINSIC_VALIDATION: "INTRINSIC_VALIDATION",
  EXTRINSIC_SETUP: "EXTRINSIC_SETUP",
  EXTRINSIC_CAPTURE: "EXTRINSIC_CAPTURE",
  EXTRINSIC_CALIBRATION: "EXTRINSIC_CALIBRATION",
  EXTRINSIC_VALIDATION: "EXTRINSIC_VALIDATION",
  INTEGRATION_TEST: "INTEGRATION_TEST",
  COMPLETE: "COMPLETE"
};

// State Metadata (mirrors STATE_METADATA from states.py)
export const STATE_METADATA = {
  [SystemState.BOOT]: {
    state: SystemState.BOOT,
    allowedTransitions: [
      SystemState.CALIBRATION,
      SystemState.IDLE,
      SystemState.SAFETY,
      SystemState.SHUTDOWN,
    ],
    entryRequirements: [],
    exitRequirements: [],
    timeoutSeconds: 30.0,
    requiresCalibration: false,
    requiresSubsystems: [],
    description: "Initial system boot and initialization",
    autoTransition: SystemState.IDLE,
    color: "blue",
    displayName: "Boot"
  },

  [SystemState.CALIBRATION]: {
    state: SystemState.CALIBRATION,
    allowedTransitions: [
      SystemState.IDLE,
      SystemState.SAFETY,
      SystemState.SHUTDOWN,
    ],
    entryRequirements: ["boot_complete"],
    exitRequirements: [],
    timeoutSeconds: 300.0, // 5 minutes
    requiresCalibration: false,
    requiresSubsystems: ["camera", "navigation"],
    description: "Sensor and system calibration",
    autoTransition: null,
    color: "yellow",
    displayName: "Calibration"
  },

  [SystemState.IDLE]: {
    state: SystemState.IDLE,
    allowedTransitions: [
      SystemState.CALIBRATION,
      SystemState.TELEOPERATION,
      SystemState.AUTONOMOUS,
      SystemState.SAFETY,
      SystemState.SHUTDOWN,
    ],
    entryRequirements: ["boot_complete"],
    exitRequirements: [],
    timeoutSeconds: 0.0,
    requiresCalibration: false,
    requiresSubsystems: [],
    description: "Ready but not operating",
    autoTransition: null,
    color: "green",
    displayName: "Idle"
  },

  [SystemState.TELEOPERATION]: {
    state: SystemState.TELEOPERATION,
    allowedTransitions: [
      SystemState.IDLE,
      SystemState.AUTONOMOUS,
      SystemState.SAFETY,
      SystemState.SHUTDOWN,
    ],
    entryRequirements: [],
    exitRequirements: [],
    timeoutSeconds: 0.0,
    requiresCalibration: true,
    requiresSubsystems: ["navigation", "camera"],
    description: "Manual remote control",
    autoTransition: null,
    color: "cyan",
    displayName: "Teleoperation"
  },

  [SystemState.AUTONOMOUS]: {
    state: SystemState.AUTONOMOUS,
    allowedTransitions: [
      SystemState.IDLE,
      SystemState.SAFETY,
      SystemState.SHUTDOWN,
    ],
    entryRequirements: ["calibration_complete"],
    exitRequirements: [],
    timeoutSeconds: 0.0,
    requiresCalibration: true,
    requiresSubsystems: ["navigation", "slam", "computer_vision"],
    description: "Autonomous operation with substates",
    autoTransition: null,
    color: "red",
    displayName: "Autonomous",
    hasSubstates: true,
    substates: AutonomousSubstate
  },

  [SystemState.SAFETY]: {
    state: SystemState.SAFETY,
    allowedTransitions: [
      SystemState.IDLE,
      SystemState.SHUTDOWN,
    ],
    entryRequirements: [],
    exitRequirements: [],
    timeoutSeconds: 0.0,
    requiresCalibration: false,
    requiresSubsystems: [],
    description: "Safety/emergency state",
    autoTransition: null,
    color: "orange",
    displayName: "Safety"
  },

  [SystemState.SHUTDOWN]: {
    state: SystemState.SHUTDOWN,
    allowedTransitions: [], // Terminal state
    entryRequirements: [],
    exitRequirements: [],
    timeoutSeconds: 0.0,
    requiresCalibration: false,
    requiresSubsystems: [],
    description: "Graceful shutdown sequence",
    autoTransition: null,
    color: "gray",
    displayName: "Shutdown",
    isTerminal: true
  }
};

// Substate Metadata
export const SUBSTATE_METADATA = {
  [AutonomousSubstate.NONE]: {
    displayName: "None",
    description: "No specific mission active",
    color: "gray"
  },
  [AutonomousSubstate.SCIENCE]: {
    displayName: "Science",
    description: "Science mission",
    color: "purple"
  },
  [AutonomousSubstate.DELIVERY]: {
    displayName: "Delivery",
    description: "Delivery mission",
    color: "indigo"
  },
  [AutonomousSubstate.EQUIPMENT_SERVICING]: {
    displayName: "Equipment Servicing",
    description: "Equipment servicing mission",
    color: "teal",
    hasSubSubstates: true,
    subSubstates: EquipmentServicingSubstate
  },
  [AutonomousSubstate.AUTONOMOUS_NAVIGATION]: {
    displayName: "Autonomous Navigation",
    description: "Autonomous navigation mission",
    color: "green"
  },
  [AutonomousSubstate.FOLLOW_ME]: {
    displayName: "Follow Me",
    description: "Follow me mode using ArUco tag detection",
    color: "pink"
  }
};

// Equipment Servicing Substate Metadata
export const EQUIPMENT_SERVICING_METADATA = {
  [EquipmentServicingSubstate.NONE]: {
    displayName: "None",
    description: "Not in equipment servicing",
    color: "gray"
  },
  [EquipmentServicingSubstate.TRAVELING]: {
    displayName: "Traveling",
    description: "Traveling to lander",
    color: "blue"
  },
  [EquipmentServicingSubstate.SAMPLE_DELIVERY]: {
    displayName: "Sample Delivery",
    description: "Delivering sample cache",
    color: "yellow"
  },
  [EquipmentServicingSubstate.PANEL_OPERATIONS]: {
    displayName: "Panel Operations",
    description: "Opening panels/drawers",
    color: "orange"
  },
  [EquipmentServicingSubstate.AUTONOMOUS_TYPING]: {
    displayName: "Autonomous Typing",
    description: "Typing launch code",
    color: "red"
  },
  [EquipmentServicingSubstate.USB_CONNECTION]: {
    displayName: "USB Connection",
    description: "USB connection and data read",
    color: "purple"
  },
  [EquipmentServicingSubstate.FUEL_CONNECTION]: {
    displayName: "Fuel Connection",
    description: "Connecting fuel hose",
    color: "green"
  },
  [EquipmentServicingSubstate.BUTTON_OPERATIONS]: {
    displayName: "Button Operations",
    description: "Buttons, switches, knobs",
    color: "pink"
  },
  [EquipmentServicingSubstate.COMPLETE]: {
    displayName: "Complete",
    description: "Mission complete",
    color: "green"
  }
};

// Calibration Substate Metadata
export const CALIBRATION_METADATA = {
  [CalibrationSubstate.NONE]: {
    displayName: "None",
    description: "Not in calibration",
    color: "gray"
  },
  [CalibrationSubstate.SETUP]: {
    displayName: "Setup",
    description: "Environment setup and target preparation",
    color: "blue"
  },
  [CalibrationSubstate.INTRINSIC_CAPTURE]: {
    displayName: "Intrinsic Capture",
    description: "Capture images for intrinsic calibration",
    color: "cyan"
  },
  [CalibrationSubstate.INTRINSIC_CALIBRATION]: {
    displayName: "Intrinsic Calibration",
    description: "Compute intrinsic parameters",
    color: "green"
  },
  [CalibrationSubstate.INTRINSIC_VALIDATION]: {
    displayName: "Intrinsic Validation",
    description: "Validate intrinsic calibration quality",
    color: "yellow"
  },
  [CalibrationSubstate.EXTRINSIC_SETUP]: {
    displayName: "Extrinsic Setup",
    description: "Prepare for hand-eye calibration",
    color: "orange"
  },
  [CalibrationSubstate.EXTRINSIC_CAPTURE]: {
    displayName: "Extrinsic Capture",
    description: "Capture robot-camera pose pairs",
    color: "red"
  },
  [CalibrationSubstate.EXTRINSIC_CALIBRATION]: {
    displayName: "Extrinsic Calibration",
    description: "Compute hand-eye transformation",
    color: "purple"
  },
  [CalibrationSubstate.EXTRINSIC_VALIDATION]: {
    displayName: "Extrinsic Validation",
    description: "Validate extrinsic calibration",
    color: "pink"
  },
  [CalibrationSubstate.INTEGRATION_TEST]: {
    displayName: "Integration Test",
    description: "Test full vision-manipulation pipeline",
    color: "indigo"
  },
  [CalibrationSubstate.COMPLETE]: {
    displayName: "Complete",
    description: "Calibration complete, ready for autonomous ops",
    color: "green"
  }
};

// Utility functions

/**
 * Get state metadata by state name
 * @param {string} stateName - State name
 * @returns {Object|null} State metadata or null if not found
 */
export const getStateMetadata = (stateName) => {
  return STATE_METADATA[stateName] || null;
};

/**
 * Check if a state transition is valid
 * @param {string} fromState - Source state
 * @param {string} toState - Target state
 * @returns {boolean} True if transition is allowed
 */
export const isValidTransition = (fromState, toState) => {
  const metadata = getStateMetadata(fromState);
  return metadata ? metadata.allowedTransitions.includes(toState) : false;
};

/**
 * Get required subsystems for a state
 * @param {string} stateName - State name
 * @param {string} substateName - Optional substate name
 * @returns {Array} Array of required subsystem names
 */
export const getRequiredSubsystems = (stateName, substateName = null) => {
  const metadata = getStateMetadata(stateName);
  if (!metadata) return [];

  let subsystems = [...metadata.requiresSubsystems];

  // Add substate-specific requirements
  if (substateName && metadata.hasSubstates) {
    const substateMeta = SUBSTATE_METADATA[substateName];
    if (substateMeta?.hasSubSubstates) {
      // Add any sub-substate specific requirements here if needed
    }
  }

  return subsystems;
};

/**
 * Get display color for a state
 * @param {string} stateName - State name
 * @returns {string} CSS color class
 */
export const getStateColor = (stateName) => {
  const metadata = getStateMetadata(stateName);
  return metadata ? `state-${metadata.color}` : 'state-gray';
};

/**
 * Get hierarchical state path for display
 * @param {string} state - Main state
 * @param {string} substate - Autonomous substate (optional)
 * @param {string} subSubstate - Equipment servicing substate (optional)
 * @param {string} calibrationSubstate - Calibration substate (optional)
 * @returns {string} Formatted state path
 */
export const getStatePath = (state, substate = null, subSubstate = null, calibrationSubstate = null) => {
  const parts = [state];

  if (substate && substate !== 'NONE') {
    parts.push(substate);
  }

  if (subSubstate && subSubstate !== 'NONE') {
    parts.push(subSubstate);
  }

  if (calibrationSubstate && calibrationSubstate !== 'NONE') {
    parts.push(calibrationSubstate);
  }

  return parts.join(' → ');
};

// Export all enums and utilities
export default {
  SystemState,
  AutonomousSubstate,
  EquipmentServicingSubstate,
  CalibrationSubstate,
  STATE_METADATA,
  SUBSTATE_METADATA,
  EQUIPMENT_SERVICING_METADATA,
  CALIBRATION_METADATA,
  getStateMetadata,
  isValidTransition,
  getRequiredSubsystems,
  getStateColor,
  getStatePath
};

// State transition definitions (which states can transition to which others)
export const StateTransitions = {
  [SystemState.BOOT]: [SystemState.CALIBRATION, SystemState.IDLE, SystemState.SAFETY, SystemState.SHUTDOWN],
  [SystemState.CALIBRATION]: [SystemState.IDLE, SystemState.SAFETY, SystemState.SHUTDOWN],
  [SystemState.IDLE]: [SystemState.CALIBRATION, SystemState.TELEOPERATION, SystemState.AUTONOMOUS, SystemState.SAFETY, SystemState.SHUTDOWN],
  [SystemState.TELEOPERATION]: [SystemState.IDLE, SystemState.AUTONOMOUS, SystemState.SAFETY, SystemState.SHUTDOWN],
  [SystemState.AUTONOMOUS]: [SystemState.IDLE, SystemState.TELEOPERATION, SystemState.SAFETY, SystemState.SHUTDOWN],
  [SystemState.SAFETY]: [SystemState.IDLE, SystemState.TELEOPERATION, SystemState.SHUTDOWN],
  [SystemState.SHUTDOWN]: [] // Terminal state, no transitions
};
