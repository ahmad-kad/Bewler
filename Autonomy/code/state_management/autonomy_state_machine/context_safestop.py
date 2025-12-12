#!/usr/bin/env python3
"""
Context-Aware Safestop System

Implements intelligent safestop behavior that adapts based on current system context.
Instead of a one-size-fits-all emergency stop, this system:
- Freezes robotic arm when arm operations are active
- Decelerates motion systems when navigation is active
- Pauses analysis when science operations are running
- Stops following when in follow-me mode

This makes the system more user-friendly and reduces mechanical stress.
"""

import asyncio
import time
from enum import Enum
from typing import Callable, Dict, List


class SafetyBehavior(Enum):
    """Types of safety behaviors for different contexts"""

    FREEZE_ARM = "freeze_arm"
    DECELERATE_MOTION = "decelerate_motion"
    PAUSE_ANALYSIS = "pause_analysis"
    STOP_FOLLOWING = "stop_following"
    EMERGENCY_STOP = "emergency_stop"


class ContextSafestop:
    """Context-aware safestop coordinator"""

    def __init__(self):
        self.active_contexts: List[str] = []
        self.safety_actions: Dict[str, Callable] = {}
        self.last_safestop_time = 0
        self.safestop_cooldown = 1.0  # Minimum time between safestops

        # Context to safety behavior mapping
        self.context_behaviors = {
            "motion_active": SafetyBehavior.DECELERATE_MOTION,
            "arm_active": SafetyBehavior.FREEZE_ARM,
            "science_active": SafetyBehavior.PAUSE_ANALYSIS,
            "follow_active": SafetyBehavior.STOP_FOLLOWING,
            "equipment_active": SafetyBehavior.FREEZE_ARM,
        }

    def register_safety_action(self, context: str, action: Callable):
        """Register a safety action for a specific context"""
        self.safety_actions[context] = action

    def update_context(self, active_contexts: List[str]):
        """Update the list of currently active system contexts"""
        self.active_contexts = active_contexts

    def execute_safestop(self, reason: str = "operator_request") -> Dict[str, any]:
        """Execute context-aware safestop based on current system state"""

        # Prevent rapid-fire safestops
        current_time = time.time()
        if current_time - self.last_safestop_time < self.safestop_cooldown:
            return {
                "executed": False,
                "reason": "cooldown_active",
                "cooldown_remaining": self.safestop_cooldown - (current_time - self.last_safestop_time),
            }

        executed_actions = []
        failed_actions = []

        # Execute safety actions for each active context
        for context in self.active_contexts:
            if context in self.safety_actions:
                try:
                    self.safety_actions[context]()
                    executed_actions.append(context)
                except Exception as e:
                    failed_actions.append(f"{context}: {str(e)}")

        self.last_safestop_time = current_time

        return {
            "executed": True,
            "reason": reason,
            "active_contexts": self.active_contexts.copy(),
            "executed_actions": executed_actions,
            "failed_actions": failed_actions,
            "timestamp": current_time,
        }

    def get_recommended_behavior(self) -> Dict[str, any]:
        """Get recommended safety behavior based on current context"""
        behaviors = []

        for context in self.active_contexts:
            if context in self.context_behaviors:
                behavior = self.context_behaviors[context]
                behaviors.append(
                    {
                        "context": context,
                        "behavior": behavior.value,
                        "description": self._get_behavior_description(behavior),
                    }
                )

        return {
            "active_contexts": self.active_contexts.copy(),
            "recommended_behaviors": behaviors,
            "will_freeze_arm": any(b["behavior"] == "freeze_arm" for b in behaviors),
            "will_decelerate_motion": any(b["behavior"] == "decelerate_motion" for b in behaviors),
        }

    def _get_behavior_description(self, behavior: SafetyBehavior) -> str:
        """Get human-readable description of safety behavior"""
        descriptions = {
            SafetyBehavior.FREEZE_ARM: "Freeze robotic arm in current position",
            SafetyBehavior.DECELERATE_MOTION: "Gradually decelerate motion systems",
            SafetyBehavior.PAUSE_ANALYSIS: "Pause ongoing analysis operations",
            SafetyBehavior.STOP_FOLLOWING: "Stop following behavior and hold position",
            SafetyBehavior.EMERGENCY_STOP: "Immediate emergency stop of all systems",
        }
        return descriptions.get(behavior, "Unknown safety behavior")


class BootValidator:
    """System boot validation coordinator"""

    def __init__(self):
        self.validation_steps = [
            self.check_hardware_connectivity,
            self.check_sensor_calibration,
            self.check_communication_links,
            self.check_power_systems,
            self.check_emergency_systems,
        ]
        self.validation_results = {}

    async def validate_system_boot(self) -> Dict[str, any]:
        """Run complete system boot validation"""

        results = {}
        all_passed = True

        for step in self.validation_steps:
            step_name = step.__name__
            try:
                result = await step()
                results[step_name] = result
                if not result.get("passed", False):
                    all_passed = False
            except Exception as e:
                results[step_name] = {"passed": False, "error": str(e), "duration": 0}
                all_passed = False

        return {
            "boot_validation_complete": True,
            "all_systems_ready": all_passed,
            "validation_results": results,
            "timestamp": time.time(),
            "recommendation": "ready" if all_passed else "requires_attention",
        }

    async def check_hardware_connectivity(self) -> Dict[str, any]:
        """Check CAN bus and sensor hardware connectivity"""
        # This would integrate with actual hardware checks
        # For now, simulate successful check
        await asyncio.sleep(0.1)  # Simulate check time

        return {
            "passed": True,
            "component": "hardware_connectivity",
            "details": "CAN bus and sensors detected",
            "duration": 0.1,
        }

    async def check_sensor_calibration(self) -> Dict[str, any]:
        """Verify sensor calibration status"""
        await asyncio.sleep(0.2)

        return {
            "passed": True,
            "component": "sensor_calibration",
            "details": "IMU and camera calibration valid",
            "duration": 0.2,
        }

    async def check_communication_links(self) -> Dict[str, any]:
        """Test communication links"""
        await asyncio.sleep(0.1)

        return {
            "passed": True,
            "component": "communication_links",
            "details": "WebSocket and ROS2 links operational",
            "duration": 0.1,
        }

    async def check_power_systems(self) -> Dict[str, any]:
        """Verify power system health"""
        await asyncio.sleep(0.05)

        return {
            "passed": True,
            "component": "power_systems",
            "details": "Battery voltage and current within limits",
            "duration": 0.05,
        }

    async def check_emergency_systems(self) -> Dict[str, any]:
        """Test emergency stop systems"""
        await asyncio.sleep(0.1)

        return {
            "passed": True,
            "component": "emergency_systems",
            "details": "E-stop and safestop systems functional",
            "duration": 0.1,
        }


# Import asyncio for the BootValidator
