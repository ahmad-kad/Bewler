#!/usr/bin/env python3
"""
URC 2026 Simulation Testing Runner

Executes comprehensive simulation tests including:
- Gazebo world integration tests
- Mission execution validation
- Performance benchmarking
- Failure mode testing

Usage:
    python3 run_simulation_tests.py [options]

Options:
    --world WORLD       Gazebo world to use (default: urc_desert_terrain)
    --test TEST         Specific test to run (default: all)
    --gui               Enable Gazebo GUI
    --verbose           Verbose output
    --report            Generate detailed report
"""

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Dict, Optional

# Add project paths
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))
sys.path.insert(0, str(project_root / 'Autonomy' / 'simulation'))
sys.path.insert(0, str(project_root / 'tests' / 'integration'))
sys.path.insert(0, str(project_root / 'tests' / 'performance'))


class SimulationTestRunner:
    """Comprehensive simulation testing runner."""

    def __init__(self, world: str = 'urc_desert_terrain', gui: bool = False, verbose: bool = False):
        self.world = world
        self.gui = gui
        self.verbose = verbose
        self.results = {}
        self.project_root = project_root

        # Test configuration
        self.test_worlds = [
            'urc_desert_terrain',
            'gps_denied_area',
            'urc_obstacles'
        ]

        self.test_scenarios = [
            'autonomous_waypoint_navigation',
            'gps_denied_slam',
            'dynamic_obstacle_avoidance',
            'endurance_slam_test',
            'sensor_failure_recovery'
        ]

    def log(self, message: str, level: str = 'INFO'):
        """Log message with timestamp."""
        timestamp = time.strftime('%H:%M:%S')
        if self.verbose or level in ['ERROR', 'WARNING']:
            print(f"[{timestamp}] {level}: {message}")

    def check_dependencies(self) -> bool:
        """Check if all required dependencies are available."""
        self.log("Checking dependencies...")

        # Check ROS2
        try:
            result = subprocess.run(['ros2', '--version'],
                                    capture_output=True, text=True, timeout=10)
            if result.returncode != 0:
                self.log("ROS2 not found or not properly configured", "ERROR")
                return False
            self.log(f"ROS2 version: {result.stdout.strip()}")
        except (subprocess.TimeoutExpired, FileNotFoundError):
            self.log("ROS2 not available", "ERROR")
            return False

        # Check Gazebo
        try:
            result = subprocess.run(['gz', '--version'],
                                    capture_output=True, text=True, timeout=10)
            if result.returncode != 0:
                self.log("Gazebo not found", "ERROR")
                return False
            self.log(f"Gazebo version: {result.stdout.strip()}")
        except (subprocess.TimeoutExpired, FileNotFoundError):
            self.log("Gazebo not available", "ERROR")
            return False

        # Check Python test modules
        required_modules = [
            'rclpy', 'numpy', 'matplotlib', 'unittest'
        ]

        for module in required_modules:
            try:
                __import__(module)
                self.log(f"✓ {module} available")
            except ImportError:
                self.log(f"✗ {module} not available", "ERROR")
                return False

        self.log("All dependencies satisfied")
        return True

    def run_gazebo_integration_test(self) -> Dict[str, Any]:
        """Test Gazebo world loading and basic functionality."""
        self.log("Running Gazebo integration test...")

        result = {
            'test': 'gazebo_integration',
            'status': 'unknown',
            'duration': 0,
            'details': {}
        }

        start_time = time.time()

        try:
            # Build simulation package if needed
            sim_dir = self.project_root / 'Autonomy' / 'simulation'
            if not (sim_dir / 'build').exists():
                self.log("Building simulation package...")
                cmd = ['colcon', 'build', '--packages-select', 'autonomy_simulation']
                env = os.environ.copy()
                env['AMENT_PREFIX_PATH'] = str(sim_dir / 'install')

                build_result = subprocess.run(
                    cmd,
                    cwd=str(sim_dir),
                    env=env,
                    capture_output=not self.verbose,
                    text=True,
                    timeout=300
                )

                if build_result.returncode != 0:
                    result['status'] = 'failed'
                    result['details']['error'] = 'Build failed'
                    result['details']['build_output'] = build_result.stderr
                    return result

            # Test Gazebo world loading
            self.log(f"Testing Gazebo world: {self.world}")

            # Launch Gazebo with test world
            launch_cmd = [
                'ros2', 'launch', 'autonomy_simulation', 'rover_gazebo.launch.py',
                f'world:={self.world}',
                f'gui:={"true" if self.gui else "false"}',
                'use_sim_time:=true'
            ]

            env = os.environ.copy()
            env['AMENT_PREFIX_PATH'] = str(sim_dir / 'install')

            # Launch in background for testing
            gazebo_proc = subprocess.Popen(
                launch_cmd,
                env=env,
                stdout=subprocess.PIPE if not self.verbose else None,
                stderr=subprocess.PIPE if not self.verbose else None
            )

            # Wait for Gazebo to start (check for ROS2 topics)
            time.sleep(10)

            # Check if Gazebo is running by checking topics
            topic_check = subprocess.run(
                ['ros2', 'topic', 'list'],
                env=env,
                capture_output=True,
                text=True,
                timeout=10
            )

            if topic_check.returncode == 0 and 'gazebo' in topic_check.stdout.lower():
                result['status'] = 'passed'
                result['details']['topics_found'] = len(topic_check.stdout.split('\n'))
                self.log("✓ Gazebo integration successful")
            else:
                result['status'] = 'failed'
                result['details']['error'] = 'Gazebo topics not found'
                result['details']['topics_output'] = topic_check.stdout

            # Clean up
            gazebo_proc.terminate()
            try:
                gazebo_proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                gazebo_proc.kill()

        except Exception as e:
            result['status'] = 'failed'
            result['details']['error'] = str(e)

        result['duration'] = time.time() - start_time
        return result

    def run_mission_tests(self) -> Dict[str, Any]:
        """Run comprehensive mission system tests."""
        self.log("Running mission system tests...")

        result = {
            'test': 'mission_system',
            'status': 'unknown',
            'duration': 0,
            'details': {}
        }

        start_time = time.time()

        try:
            # Import and run mission tests
            import unittest

            import test_mission_system

            # Create test suite
            suite = unittest.TestLoader().loadTestsFromModule(test_mission_system)
            runner = unittest.TextTestRunner(verbosity=0, stream=subprocess.DEVNULL)

            # Run tests
            test_result = runner.run(suite)

            result['details']['tests_run'] = test_result.testsRun
            result['details']['failures'] = len(test_result.failures)
            result['details']['errors'] = len(test_result.errors)

            if test_result.wasSuccessful():
                result['status'] = 'passed'
                self.log("✓ Mission system tests passed")
            else:
                result['status'] = 'failed'
                result['details']['failure_details'] = [
                    {'test': str(failure[0]), 'error': failure[1]}
                    for failure in test_result.failures + test_result.errors
                ]
                self.log("✗ Mission system tests failed")

        except Exception as e:
            result['status'] = 'failed'
            result['details']['error'] = str(e)

        result['duration'] = time.time() - start_time
        return result

    def run_performance_tests(self) -> Dict[str, Any]:
        """Run performance and load tests."""
        self.log("Running performance tests...")

        result = {
            'test': 'performance_load',
            'status': 'unknown',
            'duration': 0,
            'details': {}
        }

        start_time = time.time()

        try:
            # Import and run performance tests
            import unittest

            import test_performance_load

            # Create test suite
            suite = unittest.TestLoader().loadTestsFromModule(test_performance_load)
            runner = unittest.TextTestRunner(verbosity=0, stream=subprocess.DEVNULL)

            # Run tests
            test_result = runner.run(suite)

            result['details']['tests_run'] = test_result.testsRun
            result['details']['failures'] = len(test_result.failures)
            result['details']['errors'] = len(test_result.errors)

            if test_result.wasSuccessful():
                result['status'] = 'passed'
                self.log("✓ Performance tests passed")

                # Generate performance metrics
                test_instance = test_performance_load.PerformanceLoadTest()
                test_instance.setUp()

                # Run individual performance tests
                test_instance.test_message_throughput()
                test_instance.test_can_sensor_data_latency()
                test_instance.test_concurrent_operations()

                result['details']['metrics'] = test_instance.results
                test_instance.tearDown()

            else:
                result['status'] = 'failed'
                result['details']['failure_details'] = [
                    {'test': str(failure[0]), 'error': failure[1]}
                    for failure in test_result.failures + test_result.errors
                ]
                self.log("✗ Performance tests failed")

        except Exception as e:
            result['status'] = 'failed'
            result['details']['error'] = str(e)

        result['duration'] = time.time() - start_time
        return result

    def run_scenario_test(self, scenario: str) -> Dict[str, Any]:
        """Run a specific test scenario."""
        self.log(f"Running scenario test: {scenario}")

        result = {
            'test': f'scenario_{scenario}',
            'status': 'unknown',
            'duration': 0,
            'details': {}
        }

        start_time = time.time()

        try:
            scenario_path = self.project_root / 'Autonomy' / 'simulation' / 'test_scenarios' / f'{scenario}.py'

            if not scenario_path.exists():
                result['status'] = 'failed'
                result['details']['error'] = f'Scenario file not found: {scenario_path}'
                return result

            # Run the scenario script
            cmd = [sys.executable, str(scenario_path)]
            if self.verbose:
                cmd.append('--verbose')

            env = os.environ.copy()
            env['PYTHONPATH'] = str(self.project_root)

            scenario_result = subprocess.run(
                cmd,
                env=env,
                capture_output=not self.verbose,
                text=True,
                timeout=600  # 10 minute timeout
            )

            if scenario_result.returncode == 0:
                result['status'] = 'passed'
                self.log(f"✓ Scenario {scenario} completed successfully")
            else:
                result['status'] = 'failed'
                result['details']['return_code'] = scenario_result.returncode
                result['details']['stdout'] = scenario_result.stdout
                result['details']['stderr'] = scenario_result.stderr
                self.log(f"✗ Scenario {scenario} failed")

        except subprocess.TimeoutExpired:
            result['status'] = 'failed'
            result['details']['error'] = 'Test timed out'

        except Exception as e:
            result['status'] = 'failed'
            result['details']['error'] = str(e)

        result['duration'] = time.time() - start_time
        return result

    def run_all_tests(self) -> Dict[str, Any]:
        """Run all simulation tests."""
        self.log("Starting comprehensive simulation test suite...")

        all_results = {
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            'world': self.world,
            'tests': []
        }

        # Check dependencies first
        if not self.check_dependencies():
            all_results['status'] = 'failed'
            all_results['error'] = 'Dependencies not satisfied'
            return all_results

        # Run core integration tests
        tests = [
            ('gazebo_integration', self.run_gazebo_integration_test),
            ('mission_system', self.run_mission_tests),
            ('performance_load', self.run_performance_tests)
        ]

        # Add scenario tests
        for scenario in self.test_scenarios:
            tests.append((f'scenario_{scenario}', lambda s=scenario: self.run_scenario_test(s)))

        passed_tests = 0
        total_tests = len(tests)

        for test_name, test_func in tests:
            self.log(f"\n--- Running {test_name} ---")
            try:
                result = test_func()
                all_results['tests'].append(result)

                if result['status'] == 'passed':
                    passed_tests += 1
                    self.log(f"✅ {test_name}: PASSED ({result['duration']:.1f}s)")
                else:
                    self.log(f"❌ {test_name}: FAILED ({result['duration']:.1f}s)")

                if result['status'] == 'failed' and 'error' in result['details']:
                    self.log(f"   Error: {result['details']['error']}")

            except Exception as e:
                self.log(f"❌ {test_name}: EXCEPTION - {str(e)}")
                all_results['tests'].append({
                    'test': test_name,
                    'status': 'error',
                    'duration': 0,
                    'details': {'error': str(e)}
                })

        # Summary
        all_results['summary'] = {
            'total_tests': total_tests,
            'passed_tests': passed_tests,
            'failed_tests': total_tests - passed_tests,
            'success_rate': passed_tests / total_tests if total_tests > 0 else 0
        }

        if passed_tests == total_tests:
            all_results['status'] = 'passed'
            self.log("\n🎉 ALL TESTS PASSED!")
        elif passed_tests >= total_tests * 0.8:
            all_results['status'] = 'warning'
            self.log(f"\n⚠️ MOST TESTS PASSED ({passed_tests}/{total_tests})")
        else:
            all_results['status'] = 'failed'
            self.log(f"\n❌ MANY TESTS FAILED ({passed_tests}/{total_tests})")

        return all_results

    def generate_report(self, results: Dict[str, Any], output_file: Optional[str] = None):
        """Generate detailed test report."""
        if output_file is None:
            output_file = f"simulation_test_report_{int(time.time())}.json"

        report_path = self.project_root / output_file

        with open(report_path, 'w') as f:
            json.dump(results, f, indent=2, default=str)

        self.log(f"Test report saved to: {report_path}")

        # Print summary to console
        print("\n" + "=" * 60)
        print("SIMULATION TEST REPORT SUMMARY")
        print("=" * 60)
        print(f"Timestamp: {results['timestamp']}")
        print(f"World: {results['world']}")
        print(f"Overall Status: {results.get('status', 'unknown').upper()}")

        summary = results.get('summary', {})
        print("\nTest Results:")
        print(f"  Total Tests: {summary.get('total_tests', 0)}")
        print(f"  Passed: {summary.get('passed_tests', 0)}")
        print(f"  Failed: {summary.get('failed_tests', 0)}")
        print(".1%")

        print("\nDetailed Results:")
        for test in results.get('tests', []):
            status_icon = "✅" if test['status'] == 'passed' else "❌" if test['status'] == 'failed' else "⚠️"
            print(f"   {status_icon} {test['test']}: {test['status'].upper()} ({test['duration']:.1f}s)")
        print("=" * 60)


def main():
    parser = argparse.ArgumentParser(description='URC 2026 Simulation Testing Runner')
    parser.add_argument('--world', default='urc_desert_terrain',
                        choices=['urc_desert_terrain', 'gps_denied_area', 'urc_obstacles'],
                        help='Gazebo world to use for testing')
    parser.add_argument('--test', choices=['gazebo', 'mission', 'performance', 'all'],
                        default='all', help='Specific test to run')
    parser.add_argument('--gui', action='store_true', help='Enable Gazebo GUI')
    parser.add_argument('--verbose', '-v', action='store_true', help='Verbose output')
    parser.add_argument('--report', action='store_true', help='Generate detailed report')

    args = parser.parse_args()

    # Initialize test runner
    runner = SimulationTestRunner(
        world=args.world,
        gui=args.gui,
        verbose=args.verbose
    )

    print("🚀 URC 2026 Simulation Testing Suite")
    print(f"World: {args.world}")
    print(f"GUI: {'Enabled' if args.gui else 'Disabled'}")
    print(f"Test: {args.test}")
    print("-" * 40)

    try:
        if args.test == 'all':
            results = runner.run_all_tests()
        elif args.test == 'gazebo':
            results = {'tests': [runner.run_gazebo_integration_test()]}
        elif args.test == 'mission':
            results = {'tests': [runner.run_mission_tests()]}
        elif args.test == 'performance':
            results = {'tests': [runner.run_performance_tests()]}

        if args.report:
            runner.generate_report(results)

        # Exit with appropriate code
        if results.get('status') == 'passed':
            sys.exit(0)
        elif results.get('status') == 'warning':
            sys.exit(1)  # Warning treated as failure for CI/CD
        else:
            sys.exit(1)

    except KeyboardInterrupt:
        print("\n⚠️ Testing interrupted by user")
        sys.exit(130)
    except Exception as e:
        print(f"\n❌ Testing failed with exception: {e}")
        if args.verbose:
            import traceback
            traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
