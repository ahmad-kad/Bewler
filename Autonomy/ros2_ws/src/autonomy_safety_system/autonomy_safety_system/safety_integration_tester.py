"""
Safety Integration Testing Framework.

Automated testing framework for safety system integration.
Simulates safety scenarios and validates system responses without hardware.
"""

import time
import json
import asyncio
from enum import Enum
from typing import Dict, List, Optional, Any, Callable
from dataclasses import dataclass, field
from concurrent.futures import ThreadPoolExecutor

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import BatteryState, Imu, Temperature
from autonomy_interfaces.msg import SafetyStatus


class TestScenario(Enum):
    """Available safety test scenarios."""

    BATTERY_CRITICAL = "BATTERY_CRITICAL"
    COMMUNICATION_LOSS = "COMMUNICATION_LOSS"
    THERMAL_WARNING = "THERMAL_WARNING"
    SENSOR_FAILURE = "SENSOR_FAILURE"
    OBSTACLE_CRITICAL = "OBSTACLE_CRITICAL"
    EMERGENCY_STOP = "EMERGENCY_STOP"
    SYSTEM_FAULT = "SYSTEM_FAULT"
    RECOVERY_TEST = "RECOVERY_TEST"
    CONSISTENCY_CHECK = "CONSISTENCY_CHECK"
    PERFORMANCE_LOAD = "PERFORMANCE_LOAD"


class TestPhase(Enum):
    """Test execution phases."""

    SETUP = "SETUP"
    EXECUTION = "EXECUTION"
    VALIDATION = "VALIDATION"
    CLEANUP = "CLEANUP"
    COMPLETE = "COMPLETE"


@dataclass
class TestResult:
    """Result of a safety test scenario."""

    scenario: TestScenario
    test_id: str
    start_time: float
    end_time: Optional[float] = None
    success: bool = False
    error_message: Optional[str] = None
    metrics: Dict[str, Any] = field(default_factory=dict)
    violations_detected: List[Dict[str, Any]] = field(default_factory=list)
    alerts_generated: List[Dict[str, Any]] = field(default_factory=list)
    system_responses: List[Dict[str, Any]] = field(default_factory=list)
    phase: TestPhase = TestPhase.SETUP


@dataclass
class SafetyTestScenario:
    """Definition of a safety test scenario."""

    scenario: TestScenario
    description: str
    setup_actions: List[Callable] = field(default_factory=list)
    trigger_actions: List[Callable] = field(default_factory=list)
    validation_checks: List[Callable] = field(default_factory=list)
    cleanup_actions: List[Callable] = field(default_factory=list)
    expected_duration: float = 30.0  # seconds
    expected_violations: List[str] = field(default_factory=list)
    expected_alerts: List[str] = field(default_factory=list)


class SafetyIntegrationTester(Node):
    """
    Safety Integration Testing Framework.

    Provides automated testing of safety system integration scenarios.
    Can run comprehensive safety validation without requiring hardware.
    """

    def __init__(self):
        super().__init__("safety_integration_tester")

        self.logger = self.get_logger()

        # Test state
        self.active_tests: Dict[str, TestResult] = {}
        self.completed_tests: List[TestResult] = []
        self.test_scenarios = self._define_test_scenarios()

        # Monitoring state
        self.observed_violations: List[Dict[str, Any]] = []
        self.observed_alerts: List[Dict[str, Any]] = []
        self.system_responses: List[Dict[str, Any]] = []
        self.baseline_measurements: Dict[str, Any] = {}

        # QoS profiles
        self.qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=10
        )

        # Thread pool for concurrent test execution
        self.executor = ThreadPoolExecutor(max_workers=4)

        # Setup publishers and subscribers
        self._setup_publishers()
        self._setup_subscribers()
        self._setup_timers()

        self.logger.info("Safety Integration Tester initialized")

    def _define_test_scenarios(self) -> Dict[TestScenario, SafetyTestScenario]:
        """Define all available safety test scenarios."""
        return {
            TestScenario.BATTERY_CRITICAL: SafetyTestScenario(
                scenario=TestScenario.BATTERY_CRITICAL,
                description="Test critical battery level detection and response",
                trigger_actions=[self._simulate_battery_critical],
                validation_checks=[self._validate_battery_critical_response],
                expected_violations=["battery_critical"],
                expected_alerts=["battery_critical_redundant"],
            ),
            TestScenario.COMMUNICATION_LOSS: SafetyTestScenario(
                scenario=TestScenario.COMMUNICATION_LOSS,
                description="Test communication loss detection and safety response",
                trigger_actions=[self._simulate_communication_loss],
                validation_checks=[self._validate_communication_loss_response],
                expected_violations=["communication_timeout"],
                expected_duration=45.0,
            ),
            TestScenario.THERMAL_WARNING: SafetyTestScenario(
                scenario=TestScenario.THERMAL_WARNING,
                description="Test thermal warning and critical temperature handling",
                trigger_actions=[self._simulate_thermal_warning],
                validation_checks=[self._validate_thermal_response],
                expected_violations=["temperature_critical_redundant"],
                expected_alerts=["thermal_warning"],
            ),
            TestScenario.SENSOR_FAILURE: SafetyTestScenario(
                scenario=TestScenario.SENSOR_FAILURE,
                description="Test sensor failure detection and system response",
                trigger_actions=[self._simulate_sensor_failure],
                validation_checks=[self._validate_sensor_failure_response],
                expected_violations=["imu_sensor_stuck"],
                expected_duration=20.0,
            ),
            TestScenario.EMERGENCY_STOP: SafetyTestScenario(
                scenario=TestScenario.EMERGENCY_STOP,
                description="Test emergency stop propagation and coordination",
                trigger_actions=[self._simulate_emergency_stop],
                validation_checks=[self._validate_emergency_stop_response],
                expected_violations=["emergency_active"],
                expected_duration=15.0,
            ),
            TestScenario.RECOVERY_TEST: SafetyTestScenario(
                scenario=TestScenario.RECOVERY_TEST,
                description="Test safety system recovery procedures",
                setup_actions=[self._setup_for_recovery_test],
                trigger_actions=[self._simulate_recovery_scenario],
                validation_checks=[self._validate_recovery_response],
                cleanup_actions=[self._cleanup_recovery_test],
                expected_duration=60.0,
            ),
            TestScenario.CONSISTENCY_CHECK: SafetyTestScenario(
                scenario=TestScenario.CONSISTENCY_CHECK,
                description="Test consistency between primary and redundant safety systems",
                trigger_actions=[self._simulate_consistency_scenario],
                validation_checks=[self._validate_consistency_response],
                expected_duration=25.0,
            ),
            TestScenario.PERFORMANCE_LOAD: SafetyTestScenario(
                scenario=TestScenario.PERFORMANCE_LOAD,
                description="Test safety system performance under load",
                trigger_actions=[self._simulate_performance_load],
                validation_checks=[self._validate_performance_response],
                expected_duration=45.0,
            ),
        }

    def _setup_publishers(self):
        """Setup ROS2 publishers for test control."""
        self.test_status_pub = self.create_publisher(String, "/safety/test_status", 10)

        self.test_control_pub = self.create_publisher(String, "/safety/test_control", 10)

        # Simulated sensor publishers for testing
        self.battery_sim_pub = self.create_publisher(BatteryState, "/battery/status", 10, qos_profile=self.qos_reliable)

        self.imu_sim_pub = self.create_publisher(Imu, "/imu/data", 10, qos_profile=self.qos_reliable)

        self.temperature_sim_pub = self.create_publisher(
            Temperature, "/temperature/data", 10, qos_profile=self.qos_reliable
        )

        self.emergency_sim_pub = self.create_publisher(Bool, "/safety/emergency_stop", 10)

    def _setup_subscribers(self):
        """Setup ROS2 subscribers for monitoring test responses."""
        # Safety system monitoring
        self.create_subscription(SafetyStatus, "/state_machine/safety_status", self._safety_status_callback, 10)

        self.create_subscription(SafetyStatus, "/safety/violations", self._violations_callback, 10)

        self.create_subscription(SafetyStatus, "/safety/redundant_status", self._redundant_safety_callback, 10)

        self.create_subscription(String, "/safety/active_alerts", self._alerts_callback, 10)

        self.create_subscription(String, "/safety/emergency_status", self._emergency_callback, 10)

        # Dashboard monitoring
        self.create_subscription(String, "/safety/dashboard_status", self._dashboard_callback, 10)

    def _setup_timers(self):
        """Setup test monitoring and control timers."""
        # Test status publishing
        self.status_timer = self.create_timer(1.0, self._publish_test_status)

        # Test progress monitoring
        self.monitor_timer = self.create_timer(2.0, self._monitor_test_progress)

        # Automatic test completion check
        self.completion_timer = self.create_timer(5.0, self._check_test_completion)

    async def run_test_scenario(self, scenario: TestScenario) -> TestResult:
        """Run a specific safety test scenario."""
        if scenario not in self.test_scenarios:
            raise ValueError(f"Unknown test scenario: {scenario}")

        test_scenario = self.test_scenarios[scenario]
        test_id = f"{scenario.value}_{int(time.time())}"

        # Create test result
        test_result = TestResult(scenario=scenario, test_id=test_id, start_time=time.time(), phase=TestPhase.SETUP)

        self.active_tests[test_id] = test_result
        self.logger.info(f"Starting test scenario: {scenario.value} ({test_id})")

        try:
            # Setup phase
            test_result.phase = TestPhase.SETUP
            await self._execute_test_phase(test_scenario.setup_actions, test_result)

            # Execution phase
            test_result.phase = TestPhase.EXECUTION
            await self._execute_test_phase(test_scenario.trigger_actions, test_result)

            # Wait for system response
            await asyncio.sleep(5.0)

            # Validation phase
            test_result.phase = TestPhase.VALIDATION
            await self._execute_validation_phase(test_scenario.validation_checks, test_result)

            # Cleanup phase
            test_result.phase = TestPhase.CLEANUP
            await self._execute_test_phase(test_scenario.cleanup_actions, test_result)

            # Mark as complete
            test_result.phase = TestPhase.COMPLETE
            test_result.end_time = time.time()
            test_result.success = True

            self.logger.info(f"Test scenario completed successfully: {test_id}")

        except Exception as e:
            test_result.success = False
            test_result.error_message = str(e)
            test_result.end_time = time.time()
            self.logger.error(f"Test scenario failed: {test_id} - {e}")

        finally:
            # Move to completed tests
            self.completed_tests.append(test_result)
            if test_id in self.active_tests:
                del self.active_tests[test_id]

        return test_result

    async def _execute_test_phase(self, actions: List[Callable], test_result: TestResult):
        """Execute a list of test actions."""
        for action in actions:
            try:
                if asyncio.iscoroutinefunction(action):
                    await action(test_result)
                else:
                    action(test_result)
            except Exception as e:
                self.logger.error(f"Test action failed: {action.__name__} - {e}")
                raise

    async def _execute_validation_phase(self, checks: List[Callable], test_result: TestResult):
        """Execute validation checks for test scenario."""
        for check in checks:
            try:
                if asyncio.iscoroutinefunction(check):
                    result = await check(test_result)
                else:
                    result = check(test_result)

                if not result:
                    raise AssertionError(f"Validation check failed: {check.__name__}")

            except Exception as e:
                self.logger.error(f"Validation check failed: {check.__name__} - {e}")
                raise

    # Test scenario implementations

    def _simulate_battery_critical(self, test_result: TestResult):
        """Simulate critical battery level."""
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = 10.5
        msg.current = 5.0
        msg.percentage = 8.0  # Critical level
        msg.temperature = 35.0

        self.battery_sim_pub.publish(msg)
        self.logger.info("Simulated critical battery level")

    def _simulate_communication_loss(self, test_result: TestResult):
        """Simulate communication loss by stopping heartbeat simulation."""
        # Stop publishing simulated data to trigger timeouts
        self.logger.info("Simulating communication loss (stopping data publication)")

        # This test relies on natural timeouts rather than explicit triggers
        # In a real scenario, this would test the system's response to lost communication

    def _simulate_thermal_warning(self, test_result: TestResult):
        """Simulate thermal warning condition."""
        msg = Temperature()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.temperature = 88.0  # Critical temperature
        msg.variance = 1.0

        self.temperature_sim_pub.publish(msg)
        self.logger.info("Simulated critical temperature")

    def _simulate_sensor_failure(self, test_result: TestResult):
        """Simulate sensor failure by publishing constant IMU data."""
        # Publish constant IMU readings to simulate stuck sensor
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 9.81

        # Publish same values multiple times to trigger stuck sensor detection
        for _ in range(15):
            self.imu_sim_pub.publish(msg)
            time.sleep(0.1)

        self.logger.info("Simulated stuck IMU sensor")

    def _simulate_emergency_stop(self, test_result: TestResult):
        """Simulate emergency stop trigger."""
        msg = Bool()
        msg.data = True

        self.emergency_sim_pub.publish(msg)
        self.logger.info("Simulated emergency stop trigger")

    def _setup_for_recovery_test(self, test_result: TestResult):
        """Setup for recovery test scenario."""
        # First trigger an emergency condition
        self._simulate_emergency_stop(test_result)
        time.sleep(2.0)  # Allow system to respond

    def _simulate_recovery_scenario(self, test_result: TestResult):
        """Simulate recovery from emergency state."""
        # In a real implementation, this would call the recovery service
        # For now, we simulate recovery by clearing emergency condition
        msg = Bool()
        msg.data = False
        self.emergency_sim_pub.publish(msg)
        self.logger.info("Simulated emergency recovery")

    def _simulate_consistency_scenario(self, test_result: TestResult):
        """Simulate scenario to test system consistency."""
        # Create a condition that should trigger both primary and redundant systems
        self._simulate_battery_critical(test_result)
        time.sleep(1.0)
        # Both systems should detect this and respond consistently

    def _simulate_performance_load(self, test_result: TestResult):
        """Simulate high-load performance test."""
        # Rapidly publish sensor data to test system performance
        start_time = time.time()

        while time.time() - start_time < 10.0:  # 10 second load test
            # Publish high-frequency sensor data
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.linear_acceleration.x = 0.1
            imu_msg.linear_acceleration.y = 0.2
            imu_msg.linear_acceleration.z = 9.81

            self.imu_sim_pub.publish(imu_msg)

            battery_msg = BatteryState()
            battery_msg.header.stamp = self.get_clock().now().to_msg()
            battery_msg.percentage = 75.0

            self.battery_sim_pub.publish(battery_msg)

            time.sleep(0.01)  # 100Hz publication rate

        self.logger.info("Completed performance load simulation")

    # Validation check implementations

    def _validate_battery_critical_response(self, test_result: TestResult) -> bool:
        """Validate response to battery critical scenario."""
        # Check that violations were detected
        battery_violations = [v for v in self.observed_violations if "battery" in v.get("violation_id", "").lower()]

        if len(battery_violations) == 0:
            test_result.error_message = "No battery violations detected"
            return False

        # Check that emergency response was triggered
        emergency_responses = [r for r in self.system_responses if r.get("emergency_active", False)]

        if len(emergency_responses) == 0:
            test_result.error_message = "No emergency response triggered"
            return False

        return True

    def _validate_communication_loss_response(self, test_result: TestResult) -> bool:
        """Validate response to communication loss."""
        # Check for communication-related violations
        comm_violations = [
            v
            for v in self.observed_violations
            if "communication" in v.get("violation_id", "").lower() or "timeout" in v.get("violation_id", "").lower()
        ]

        if len(comm_violations) == 0:
            test_result.error_message = "No communication violations detected"
            return False

        return True

    def _validate_thermal_response(self, test_result: TestResult) -> bool:
        """Validate response to thermal warning."""
        thermal_violations = [v for v in self.observed_violations if "temperature" in v.get("violation_id", "").lower()]

        if len(thermal_violations) == 0:
            test_result.error_message = "No thermal violations detected"
            return False

        return True

    def _validate_sensor_failure_response(self, test_result: TestResult) -> bool:
        """Validate response to sensor failure."""
        sensor_violations = [
            v
            for v in self.observed_violations
            if "sensor" in v.get("violation_id", "").lower() and "stuck" in v.get("violation_id", "").lower()
        ]

        if len(sensor_violations) == 0:
            test_result.error_message = "No sensor failure violations detected"
            return False

        return True

    def _validate_emergency_stop_response(self, test_result: TestResult) -> bool:
        """Validate emergency stop response."""
        emergency_responses = [r for r in self.system_responses if r.get("emergency_active", False)]

        if len(emergency_responses) == 0:
            test_result.error_message = "No emergency stop response detected"
            return False

        return True

    def _validate_recovery_response(self, test_result: TestResult) -> bool:
        """Validate recovery response."""
        # Check that emergency was initially triggered
        emergency_triggered = any(r.get("emergency_active", False) for r in self.system_responses)

        # Check that recovery was successful
        recovery_complete = any(r.get("recovery_complete", False) for r in self.system_responses)

        if not emergency_triggered:
            test_result.error_message = "Emergency was not properly triggered"
            return False

        if not recovery_complete:
            test_result.error_message = "Recovery was not completed"
            return False

        return True

    def _validate_consistency_response(self, test_result: TestResult) -> bool:
        """Validate consistency between safety systems."""
        # Check that both primary and redundant systems detected the issue
        primary_violations = [v for v in self.observed_violations if "state_machine" in v.get("source", "")]

        redundant_violations = [v for v in self.observed_violations if "redundant" in v.get("source", "")]

        if len(primary_violations) == 0:
            test_result.error_message = "Primary safety system did not detect issue"
            return False

        if len(redundant_violations) == 0:
            test_result.error_message = "Redundant safety system did not detect issue"
            return False

        return True

    def _validate_performance_response(self, test_result: TestResult) -> bool:
        """Validate performance under load."""
        # Check that system remained responsive during load test
        # This would check metrics like processing time, dropped messages, etc.
        # For now, just verify no critical failures occurred during test
        critical_violations = [
            v
            for v in self.observed_violations
            if "critical" in v.get("violation_id", "").lower() or "emergency" in v.get("violation_id", "").lower()
        ]

        if len(critical_violations) > 0:
            test_result.error_message = (
                f"Critical violations occurred during performance test: {len(critical_violations)}"
            )
            return False

        return True

    def _cleanup_recovery_test(self, test_result: TestResult):
        """Cleanup after recovery test."""
        # Reset emergency state
        msg = Bool()
        msg.data = False
        self.emergency_sim_pub.publish(msg)

    # Monitoring callbacks

    def _safety_status_callback(self, msg: SafetyStatus):
        """Monitor safety status updates."""
        self.observed_violations.append(
            {
                "timestamp": time.time(),
                "source": "state_machine",
                "violation_id": msg.trigger_type,
                "description": msg.trigger_description,
                "severity": msg.safety_level,
            }
        )

    def _violations_callback(self, msg: SafetyStatus):
        """Monitor safety violations."""
        self.observed_violations.append(
            {
                "timestamp": time.time(),
                "source": "safety_watchdog",
                "violation_id": msg.trigger_type,
                "description": msg.trigger_description,
                "severity": msg.safety_level,
            }
        )

    def _redundant_safety_callback(self, msg: SafetyStatus):
        """Monitor redundant safety system."""
        self.observed_violations.append(
            {
                "timestamp": time.time(),
                "source": "redundant_safety_monitor",
                "violation_id": msg.trigger_type,
                "description": msg.trigger_description,
                "severity": msg.safety_level,
            }
        )

    def _alerts_callback(self, msg: String):
        """Monitor active alerts."""
        try:
            alerts_data = json.loads(msg.data)
            self.observed_alerts.extend(alerts_data.get("active_alerts", []))
        except json.JSONDecodeError:
            pass

    def _emergency_callback(self, msg: String):
        """Monitor emergency status."""
        try:
            emergency_data = json.loads(msg.data)
            self.system_responses.append(emergency_data)
        except json.JSONDecodeError:
            pass

    def _dashboard_callback(self, msg: String):
        """Monitor dashboard status."""
        try:
            dashboard_data = json.loads(msg.data)
            self.system_responses.append(dashboard_data)
        except json.JSONDecodeError:
            pass

    def _publish_test_status(self):
        """Publish current test status."""
        status_data = {
            "timestamp": time.time(),
            "active_tests": len(self.active_tests),
            "completed_tests": len(self.completed_tests),
            "active_test_ids": list(self.active_tests.keys()),
            "recent_violations": len(self.observed_violations),
            "recent_alerts": len(self.observed_alerts),
        }

        status_msg = String()
        status_msg.data = json.dumps(status_data, indent=2)
        self.test_status_pub.publish(status_msg)

    def _monitor_test_progress(self):
        """Monitor progress of active tests."""
        for test_id, test_result in self.active_tests.items():
            elapsed_time = time.time() - test_result.start_time

            # Check for test timeouts
            if elapsed_time > 120.0:  # 2 minute timeout
                test_result.success = False
                test_result.error_message = f"Test timeout after {elapsed_time:.1f}s"
                test_result.end_time = time.time()

                self.logger.error(f"Test timeout: {test_id}")
                self.completed_tests.append(test_result)
                del self.active_tests[test_id]

    def _check_test_completion(self):
        """Check for completed tests and cleanup."""
        # This is handled in the test execution logic

    def get_test_results(self, limit: int = 10) -> List[TestResult]:
        """Get recent test results."""
        return self.completed_tests[-limit:]

    def clear_test_history(self):
        """Clear test history."""
        self.completed_tests.clear()
        self.observed_violations.clear()
        self.observed_alerts.clear()
        self.system_responses.clear()

    async def run_full_test_suite(self) -> Dict[str, TestResult]:
        """Run the complete safety test suite."""
        test_results = {}

        # Run all test scenarios sequentially
        for scenario in TestScenario:
            if scenario != TestScenario.RECOVERY_TEST:  # Skip recovery test for now
                self.logger.info(f"Running test scenario: {scenario.value}")
                result = await self.run_test_scenario(scenario)
                test_results[scenario.value] = result

                # Brief pause between tests
                await asyncio.sleep(5.0)

        return test_results


def main(args=None):
    """Main entry point for safety integration tester."""
    rclpy.init(args=args)

    # Create safety integration tester
    tester = SafetyIntegrationTester()

    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.logger.info("Safety Integration Tester shutting down")
    finally:
        tester.executor.shutdown(wait=True)
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
