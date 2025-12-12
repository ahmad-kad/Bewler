#!/usr/bin/env python3
"""
LED Status Controller for URC 2026 Competition

Handles LED signaling for:
- 🔴 Red: Autonomous operation
- 🔵 Blue: Teleoperation (manual driving)
- 🟢 Flashing Green: Successful target arrival
- 🟡 Yellow: Calibration and boot states
- ⚪ White: Idle/ready state

Competition Requirements:
- LED status must be judge-visible from 50m distance
- Colors must clearly indicate rover operational mode
- Success indication requires flashing green pattern
- Integration with hierarchical state management system
"""

from enum import Enum

import rclpy
from autonomy.utilities import (
    NodeLogger,
    ROS2InterfaceRegistry,
    StateMachineNode,
    TimerManager,
    get_validated_parameter,
    safe_execute,
)
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


# LED color definitions
class LEDColor(Enum):
    RED = "red"
    BLUE = "blue"
    GREEN = "green"
    YELLOW = "yellow"
    WHITE = "white"
    OFF = "off"


# LED pattern definitions
class LEDPattern(Enum):
    SOLID = "solid"
    BLINK = "blink"
    FAST_BLINK = "fast_blink"
    FADE = "fade"
    PULSE = "pulse"


class LEDController(StateMachineNode):
    """
    ROS2 node for controlling competition LED status signaling.

    Subscribes to:
    - /state_machine/led_info: LED information from state machine
    - /state_machine/system_state: Current system state
    - /mission_status: Current mission progress state (legacy)

    Controls LED hardware to provide visual status indication.
    """

    def __init__(self):
        super().__init__("led_controller", "idle")

        # Initialize utilities
        self.logger = NodeLogger(self, "led_controller")
        self.interfaces = ROS2InterfaceRegistry()
        self.timers = TimerManager(self)

        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        # ROS2 subscriptions
        led_info_sub = self.create_subscription(String, "/state_machine/led_info", self.led_info_callback, qos_profile)
        self.interfaces.add_subscriber("led_info", led_info_sub)

        system_state_sub = self.create_subscription(
            String,
            "/state_machine/system_state",
            self.system_state_callback,
            qos_profile,
        )
        self.interfaces.add_subscriber("system_state", system_state_sub)

        # Legacy mission status subscription for backward compatibility
        mission_status_sub = self.create_subscription(
            String, "/mission_status", self.mission_status_callback, qos_profile
        )
        self.interfaces.add_subscriber("mission_status", mission_status_sub)

        # LED state variables
        self.current_led_info = "UNKNOWN_STATE"
        self.current_system_state = "UNKNOWN"
        self.current_mission_status = "pre_mission"
        self.current_pattern = LEDPattern.SOLID
        self.current_color = LEDColor.OFF

        # Load configuration with validation
        self.config = self._load_configuration()

        # Initialize LED hardware with error handling
        self.led_hardware = None
        self._initialize_led_hardware()

        # Setup flashing timer
        self.timers.create_named_timer("success_flash", 0.5, self._flash_timer_callback)

        self.logger.info("LED Controller initialized with state management integration")

    def _load_configuration(self) -> dict:
        """Load LED controller configuration with validation."""
        config = {}

        # Load validated parameters
        config["flash_on_duration"] = get_validated_parameter(
            self,
            "flash_on_duration",
            0.5,
            validator=lambda x: 0.1 <= x <= 2.0,
            error_msg="Flash on duration must be 0.1-2.0 seconds",
            logger=self.logger,
        )

        config["flash_off_duration"] = get_validated_parameter(
            self,
            "flash_off_duration",
            0.5,
            validator=lambda x: 0.1 <= x <= 2.0,
            error_msg="Flash off duration must be 0.1-2.0 seconds",
            logger=self.logger,
        )

        config["enable_hardware"] = get_validated_parameter(self, "enable_hardware", True, logger=self.logger)

        config["debug_mode"] = get_validated_parameter(self, "debug_mode", False, logger=self.logger)

        return config

    def _initialize_led_hardware(self) -> None:
        """
        Initialize LED hardware interface with type-safe error handling.

        TODO: Replace with actual hardware control (GPIO, PWM, etc.)
        """
        if not self.config["enable_hardware"]:
            self.logger.info("Hardware LED control disabled, using simulation mode")
            return

        result, error = safe_execute(LEDHardwareInterface)
        if error:
            self.logger.error("Failed to initialize LED hardware, using simulation mode", error=error)
            return

        self.led_hardware = result
        self.logger.info("LED hardware initialized successfully")

    def led_info_callback(self, msg: String) -> None:
        """
        Handle LED information from state machine.

        This is the primary method for LED control based on state management.
        """
        led_info = msg.data

        if self.config["debug_mode"]:
            self.logger.debug("LED info received", led_info=led_info)

        if led_info != self.current_led_info:
            self.current_led_info = led_info
            result, error = safe_execute(self.process_led_info, led_info)
            if error:
                self.logger.error("Failed to process LED info", error=error, led_info=led_info)

    def system_state_callback(self, msg: String) -> None:
        """
        Handle system state changes from state machine.

        This provides additional context for LED control decisions.
        """
        system_state = msg.data

        if self.config["debug_mode"]:
            self.logger.debug("System state changed", system_state=system_state)

        self.current_system_state = system_state

    def mission_status_callback(self, msg: String) -> None:
        """
        Handle mission status changes (legacy support).

        Competition Requirements:
        - 🟢 Flashing Green on successful target arrival
        """
        new_status = msg.data

        if self.config["debug_mode"]:
            self.logger.debug("Mission status changed", mission_status=new_status)

        self.current_mission_status = new_status

        # Check for success condition using state machine
        if self.current_state != "flashing_success":
            if new_status == "completed" or "arrived" in new_status:
                self._start_success_flashing()
            elif new_status in ["failed", "emergency", "idle"]:
                self._stop_success_flashing()

    def process_led_info(self, led_info: str) -> None:
        """
        Process LED information from state machine and set appropriate LED state.

        Args:
            led_info: LED information string from state machine
        """
        # Stop any ongoing flashing when processing new LED info
        self._stop_success_flashing()

        # Parse LED information and set appropriate color/pattern
        if led_info == "AUTONOMOUS_RED":
            self._set_led_state_safe(LEDColor.RED, LEDPattern.SOLID)
            self.transition_to("autonomous", f"LED info: {led_info}")

        elif led_info == "TELEOPERATION_BLUE":
            self._set_led_state_safe(LEDColor.BLUE, LEDPattern.SOLID)
            self.transition_to("teleoperation", f"LED info: {led_info}")

        elif led_info == "SAFETY_RED_BLINK":
            self._set_led_state_safe(LEDColor.RED, LEDPattern.FAST_BLINK)
            self.transition_to("safety", f"LED info: {led_info}")

        elif led_info == "BOOT_INITIALIZING":
            self._set_led_state_safe(LEDColor.YELLOW, LEDPattern.BLINK)
            self.transition_to("booting", f"LED info: {led_info}")

        elif led_info == "CALIBRATION_YELLOW":
            self._set_led_state_safe(LEDColor.YELLOW, LEDPattern.SOLID)
            self.transition_to("calibrating", f"LED info: {led_info}")

        elif led_info == "IDLE_GREEN":
            self._set_led_state_safe(LEDColor.GREEN, LEDPattern.SOLID)
            self.transition_to("idle", f"LED info: {led_info}")

        elif led_info == "SHUTDOWN_RED_FADE":
            self._set_led_state_safe(LEDColor.RED, LEDPattern.FADE)
            self.transition_to("shutdown", f"LED info: {led_info}")

        elif led_info == "WAYPOINT_SUCCESS":
            self._start_success_flashing()
            self.transition_to("flashing_success", f"LED info: {led_info}")

        elif led_info == "TRANSITION_IN_PROGRESS":
            self._set_led_state_safe(LEDColor.WHITE, LEDPattern.PULSE)
            self.transition_to("transitioning", f"LED info: {led_info}")

        elif led_info.startswith("ERROR_"):
            error_type = led_info.replace("ERROR_", "")
            self._set_led_state_safe(LEDColor.RED, LEDPattern.FAST_BLINK)
            self.logger.warn("Error LED pattern set", error_type=error_type, led_info=led_info)

        elif led_info.startswith("SUCCESS_"):
            success_type = led_info.replace("SUCCESS_", "")
            self._start_success_flashing()
            self.logger.info("Success LED pattern set", success_type=success_type, led_info=led_info)

        else:
            self.logger.warn("Unknown LED info received", led_info=led_info)
            self._set_led_state_safe(LEDColor.OFF, LEDPattern.SOLID)

    def _set_led_state_safe(self, color: LEDColor, pattern: LEDPattern) -> None:
        """
        Set LED to specified color and pattern with error handling.

        Args:
            color: LEDColor enum value
            pattern: LEDPattern enum value
        """
        try:
            self.current_color = color
            self.current_pattern = pattern

            if self.led_hardware:
                result, error = safe_execute(self.led_hardware.set_color_and_pattern, color, pattern)
                if error:
                    self.logger.error("Failed to set LED hardware state", error=error)
                    return
            else:
                # Software simulation
                self.logger.info("LED set (simulation)", color=color.value, pattern=pattern.value)

        except Exception as e:
            self.logger.error("Unexpected error setting LED state", error=e)

    def set_led_color(self, color: LEDColor) -> None:
        """
        Set LED to specified color (legacy method for backward compatibility).

        Args:
            color: LEDColor enum value
        """
        self._set_led_state_safe(color, LEDPattern.SOLID)

    def _start_success_flashing(self) -> None:
        """
        Start flashing green LED for success indication.

        Competition Requirements:
        - Flashing green pattern for target success
        - Must be clearly visible to judges
        """
        if self.current_state == "flashing_success":
            return

        self.transition_to("flashing_success", "Starting success flashing")
        self.logger.info("Started success flashing pattern")

    def _stop_success_flashing(self) -> None:
        """Stop flashing pattern."""
        if self.current_state != "flashing_success":
            return

        self.transition_to("idle", "Stopped success flashing")
        self._set_led_state_safe(LEDColor.OFF, LEDPattern.SOLID)
        self.logger.info("Stopped success flashing pattern")

    def _flash_timer_callback(self) -> None:
        """
        Timer callback for LED flashing pattern.

        Pattern: 1 Hz flashing (on/off every 0.5s)
        """
        if self.current_state != "flashing_success":
            return

        try:
            # Alternate between green and off
            if self.current_color == LEDColor.GREEN:
                self._set_led_state_safe(LEDColor.OFF, LEDPattern.SOLID)
            else:
                self._set_led_state_safe(LEDColor.GREEN, LEDPattern.SOLID)

        except Exception as e:
            self.logger.error("Error in flashing timer callback", error=e)
            self._stop_success_flashing()

    def destroy_node(self) -> None:
        """Clean up resources with proper timer management."""
        self.logger.info("Shutting down LED controller")

        # Stop any flashing
        self._stop_success_flashing()

        # Cancel all timers
        cancelled_count = self.timers.cancel_all()
        if cancelled_count > 0:
            self.logger.debug("Cancelled timers during shutdown", count=cancelled_count)

        # Log final state for debugging
        final_state = self.debug_state_info()
        self.logger.info("LED controller shutdown complete", **final_state)

        super().destroy_node()

    def debug_system_state(self) -> Dict[str, Any]:
        """Get comprehensive debug information about the LED controller."""
        return {
            "state_machine": self.debug_state_info(),
            "interfaces": self.interfaces.get_active_interfaces(),
            "timers": self.timers.get_timer_info(),
            "led_state": {
                "current_color": self.current_color.value if self.current_color else None,
                "current_pattern": self.current_pattern.value if self.current_pattern else None,
                "led_info": self.current_led_info,
                "system_state": self.current_system_state,
                "mission_status": self.current_mission_status,
            },
            "config": self.config,
            "hardware_available": self.led_hardware is not None,
        }


class LEDHardwareInterface:
    """
    Hardware interface for LED control.

    TODO: Implement actual hardware control based on available hardware.
    Options:
    - GPIO direct control (Raspberry Pi)
    - PWM control for brightness
    - Dedicated LED controller (Arduino, etc.)
    - Addressable LED strips
    """

    def __init__(self):
        # TODO: Initialize hardware interface
        # Examples:
        # - RPi.GPIO setup
        # - pigpio for PWM
        # - Serial communication to microcontroller
        pass

    def set_color_and_pattern(self, color: LEDColor, pattern: LEDPattern):
        """
        Set LED to specified color and pattern.

        Args:
            color: LEDColor enum value
            pattern: LEDPattern enum value
        """
        # TODO: Implement actual hardware control
        # Example GPIO implementation:
        """
        if color == LEDColor.RED:
            GPIO.output(RED_PIN, GPIO.HIGH)
            GPIO.output(BLUE_PIN, GPIO.LOW)
            GPIO.output(GREEN_PIN, GPIO.LOW)
        elif color == LEDColor.BLUE:
            GPIO.output(RED_PIN, GPIO.LOW)
            GPIO.output(BLUE_PIN, GPIO.HIGH)
            GPIO.output(GREEN_PIN, GPIO.LOW)
        elif color == LEDColor.GREEN:
            GPIO.output(RED_PIN, GPIO.LOW)
            GPIO.output(BLUE_PIN, GPIO.LOW)
            GPIO.output(GREEN_PIN, GPIO.HIGH)
        # etc.

        # Handle patterns
        if pattern == LEDPattern.BLINK:
            # Implement blinking logic
            pass
        elif pattern == LEDPattern.FAST_BLINK:
            # Implement fast blinking logic
            pass
        elif pattern == LEDPattern.FADE:
            # Implement fading logic with PWM
            pass
        elif pattern == LEDPattern.PULSE:
            # Implement pulsing logic
            pass
        """

    def set_color(self, color: LEDColor):
        """
        Set LED to specified color (legacy method).

        Args:
            color: LEDColor enum value
        """
        self.set_color_and_pattern(color, LEDPattern.SOLID)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    try:
        led_controller = LEDController()
        rclpy.spin(led_controller)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
