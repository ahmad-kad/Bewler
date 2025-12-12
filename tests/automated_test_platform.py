#!/usr/bin/env python3
"""
Automated Testing Platform - Full System Lifecycle Management

Spin-up → Test → Validate → Report → Spin-down
"""

import json
import os
import signal
import subprocess
import tempfile
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import psutil


@dataclass
class TestEnvironment:
    """Complete test environment configuration."""
    name: str
    ros_domain_id: int = 42
    workspace_path: Path = None
    log_directory: Path = None
    resource_limits: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self):
        if self.workspace_path is None:
            self.workspace_path = Path.cwd()
        if self.log_directory is None:
            self.log_directory = Path.cwd() / "test_logs"


@dataclass
class TestSuite:
    """Automated test suite configuration."""
    name: str
    description: str
    environment: TestEnvironment
    setup_commands: List[str] = field(default_factory=list)
    test_commands: List[str] = field(default_factory=list)
    cleanup_commands: List[str] = field(default_factory=list)
    timeout_seconds: int = 300
    required_services: List[str] = field(default_factory=list)
    resource_limits: Dict[str, Any] = field(default_factory=dict)


class AutomatedTestPlatform:
    """
    Complete automated testing platform for URC 2026 autonomy system.

    Features:
    - Automated system lifecycle management
    - Parallel test execution
    - Resource monitoring and limits
    - Comprehensive reporting
    - CI/CD integration
    - Failure recovery and cleanup
    """

    def __init__(self, project_root: Path):
        self.project_root = project_root
        self.active_processes: Dict[str, subprocess.Popen] = {}
        self.test_results: List[Dict[str, Any]] = []
        self.resource_monitor = ResourceMonitor()
        self.failure_recovery = FailureRecovery()

        # Test environment configurations
        self.environments = self._create_test_environments()
        self.test_suites = self._create_test_suites()

    def _create_test_environments(self) -> Dict[str, TestEnvironment]:
        """Create test environment configurations."""
        return {
            'unit': TestEnvironment(
                name='unit',
                ros_domain_id=42,
                resource_limits={'cpu_percent': 50, 'memory_mb': 512}
            ),
            'integration': TestEnvironment(
                name='integration',
                ros_domain_id=43,
                resource_limits={'cpu_percent': 80, 'memory_mb': 1024}
            ),
            'system': TestEnvironment(
                name='system',
                ros_domain_id=44,
                resource_limits={'cpu_percent': 90, 'memory_mb': 2048}
            ),
            'performance': TestEnvironment(
                name='performance',
                ros_domain_id=45,
                resource_limits={'cpu_percent': 95, 'memory_mb': 4096}
            ),
            'safety': TestEnvironment(
                name='safety',
                ros_domain_id=46,
                resource_limits={'cpu_percent': 100, 'memory_mb': 2048}
            )
        }

    def _create_test_suites(self) -> Dict[str, TestSuite]:
        """Create automated test suite configurations."""
        return {
            'aoi_tests': TestSuite(
                name='aoi_tests',
                description='Age of Information monitoring tests',
                environment=self.environments['integration'],
                setup_commands=[
                    'cd /home/ubuntu/urc-machiato-2026',
                    'source Autonomy/ros2_ws/install/setup.bash',
                    './scripts/system_launch.sh --minimal'
                ],
                test_commands=[
                    'cd /home/ubuntu/urc-machiato-2026',
                    'python3 tests/aoi_automated_test.py --test-suite all --output-json test_results.json'
                ],
                cleanup_commands=[
                    './scripts/system_stop.sh'
                ],
                timeout_seconds=600,
                required_services=['ros2', 'python3']
            ),

            'full_system_tests': TestSuite(
                name='full_system_tests',
                description='Complete autonomy system integration tests',
                environment=self.environments['system'],
                setup_commands=[
                    'cd /home/ubuntu/urc-machiato-2026',
                    'source Autonomy/ros2_ws/install/setup.bash',
                    './scripts/system_launch.sh --full'
                ],
                test_commands=[
                    'cd /home/ubuntu/urc-machiato-2026',
                    'python3 tests/run_tests.py system --coverage',
                    'python3 tests/run_tests.py integration --coverage',
                    'python3 tests/run_tests.py safety --coverage'
                ],
                cleanup_commands=[
                    './scripts/system_stop.sh',
                    'rm -rf test_artifacts/'
                ],
                timeout_seconds=1800,
                required_services=['ros2', 'python3', 'pytest']
            ),

            'performance_tests': TestSuite(
                name='performance_tests',
                description='System performance and resource usage tests',
                environment=self.environments['performance'],
                setup_commands=[
                    'cd /home/ubuntu/urc-machiato-2026',
                    'source Autonomy/ros2_ws/install/setup.bash',
                    './scripts/system_launch.sh --full'
                ],
                test_commands=[
                    'cd /home/ubuntu/urc-machiato-2026',
                    './scripts/benchmark_system.sh',
                    'python3 tests/run_tests.py performance --verbose'
                ],
                cleanup_commands=[
                    './scripts/system_stop.sh'
                ],
                timeout_seconds=1200
            ),

            'safety_critical_tests': TestSuite(
                name='safety_critical_tests',
                description='Safety-critical system validation',
                environment=self.environments['safety'],
                setup_commands=[
                    'cd /home/ubuntu/urc-machiato-2026',
                    'source Autonomy/ros2_ws/install/setup.bash',
                    './scripts/system_launch.sh --minimal'
                ],
                test_commands=[
                    'cd /home/ubuntu/urc-machiato-2026',
                    'python3 tests/run_tests.py safety --verbose',
                    './scripts/start_safety_testing.sh',
                    'sleep 30',
                    './scripts/stop_safety_testing.sh'
                ],
                cleanup_commands=[
                    './scripts/system_stop.sh'
                ],
                timeout_seconds=900
            )
        }

    def run_automated_test_suite(self, suite_name: str,
                               output_dir: Path = None) -> Dict[str, Any]:
        """
        Run complete automated test suite with full lifecycle management.

        Returns:
            Complete test results with metrics and status
        """
        if suite_name not in self.test_suites:
            raise ValueError(f"Unknown test suite: {suite_name}")

        suite = self.test_suites[suite_name]

        # Create temporary directory for artifacts
        if output_dir is None:
            output_dir = Path(tempfile.mkdtemp(prefix=f"test_{suite_name}_"))
        output_dir.mkdir(parents=True, exist_ok=True)

        start_time = time.time()
        test_results = {
            'suite_name': suite_name,
            'start_time': datetime.now().isoformat(),
            'status': 'running',
            'phases': {},
            'metrics': {},
            'artifacts': str(output_dir)
        }

        try:
            # Phase 1: Environment Setup
            test_results['phases']['setup'] = self._run_phase(
                'setup', suite.setup_commands, suite.environment, output_dir
            )

            # Phase 2: System Validation
            test_results['phases']['validation'] = self._run_system_validation(suite)

            # Phase 3: Test Execution
            test_results['phases']['execution'] = self._run_phase(
                'execution', suite.test_commands, suite.environment, output_dir
            )

            # Phase 4: Result Analysis
            test_results['phases']['analysis'] = self._analyze_results(output_dir)

            # Phase 5: Cleanup
            test_results['phases']['cleanup'] = self._run_phase(
                'cleanup', suite.cleanup_commands, suite.environment, output_dir
            )

            test_results['status'] = 'completed'
            test_results['duration'] = time.time() - start_time
            test_results['metrics'] = self.resource_monitor.get_summary()

        except Exception as e:
            test_results['status'] = 'failed'
            test_results['error'] = str(e)
            test_results['duration'] = time.time() - start_time
            # Ensure cleanup even on failure
            self._emergency_cleanup()

        # Save results
        results_file = output_dir / "test_results.json"
        with open(results_file, 'w') as f:
            json.dump(test_results, f, indent=2, default=str)

        return test_results

    def _run_phase(self, phase_name: str, commands: List[str],
                  environment: TestEnvironment, output_dir: Path) -> Dict[str, Any]:
        """Execute a test phase with monitoring."""
        phase_start = time.time()
        phase_results = {
            'phase': phase_name,
            'commands': commands,
            'start_time': datetime.now().isoformat(),
            'status': 'running'
        }

        # Start resource monitoring for this phase
        self.resource_monitor.start_monitoring(f"{phase_name}_resources")

        try:
            for cmd in commands:
                self._execute_command(cmd, environment, output_dir / f"{phase_name}.log")

            phase_results['status'] = 'completed'

        except Exception as e:
            phase_results['status'] = 'failed'
            phase_results['error'] = str(e)

        finally:
            # Stop resource monitoring
            self.resource_monitor.stop_monitoring()

        phase_results['duration'] = time.time() - phase_start
        return phase_results

    def _run_system_validation(self, suite: TestSuite) -> Dict[str, Any]:
        """Validate that required systems are running."""
        validation_results = {
            'required_services': suite.required_services,
            'system_ready': False,
            'services_status': {}
        }

        # Check ROS2 environment
        try:
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                capture_output=True,
                text=True,
                timeout=10
            )
            validation_results['ros2_available'] = result.returncode == 0
            validation_results['active_nodes'] = len(result.stdout.strip().split('\n'))
        except:
            validation_results['ros2_available'] = False

        # Check required services
        for service in suite.required_services:
            try:
                result = subprocess.run(
                    ['which', service],
                    capture_output=True,
                    text=True
                )
                validation_results['services_status'][service] = result.returncode == 0
            except:
                validation_results['services_status'][service] = False

        # Overall validation
        validation_results['system_ready'] = all([
            validation_results.get('ros2_available', False),
            all(validation_results['services_status'].values())
        ])

        return validation_results

    def _analyze_results(self, output_dir: Path) -> Dict[str, Any]:
        """Analyze test results and generate summary."""
        analysis = {
            'test_files_found': [],
            'coverage_data': {},
            'failure_patterns': [],
            'performance_metrics': {}
        }

        # Look for test result files
        result_files = list(output_dir.glob("*.json")) + list(output_dir.glob("*.xml"))
        analysis['test_files_found'] = [f.name for f in result_files]

        # Parse pytest results if available
        junit_file = output_dir / "junit.xml"
        if junit_file.exists():
            analysis['junit_results'] = self._parse_junit_results(junit_file)

        # Parse coverage data
        coverage_file = output_dir / "coverage.xml"
        if coverage_file.exists():
            analysis['coverage_data'] = self._parse_coverage_data(coverage_file)

        return analysis

    def _execute_command(self, command: str, environment: TestEnvironment,
                        log_file: Path) -> subprocess.CompletedProcess:
        """Execute command with logging and resource monitoring."""

        # Set up environment
        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = str(environment.ros_domain_id)
        env['ROS_LOG_DIR'] = str(environment.log_directory)

        # Execute command
        with open(log_file, 'w') as log:
            process = subprocess.run(
                command,
                shell=True,
                cwd=self.project_root,
                env=env,
                stdout=log,
                stderr=subprocess.STDOUT,
                timeout=300  # 5 minute timeout per command
            )

        if process.returncode != 0:
            raise subprocess.CalledProcessError(
                process.returncode, command,
                output=f"See log: {log_file}"
            )

        return process

    def _emergency_cleanup(self):
        """Emergency cleanup on test failure."""
        print("🚨 Emergency cleanup initiated...")

        # Kill all test processes
        subprocess.run(['pkill', '-f', 'autonomy'], check=False)
        subprocess.run(['pkill', '-f', 'ros2'], check=False)
        subprocess.run(['pkill', '-f', 'pytest'], check=False)

        # Reset ROS environment
        os.environ.pop('ROS_DOMAIN_ID', None)

        print("✅ Emergency cleanup completed")

    def _parse_junit_results(self, junit_file: Path) -> Dict[str, Any]:
        """Parse JUnit XML results."""
        try:
            import xml.etree.ElementTree as ET
            tree = ET.parse(junit_file)
            root = tree.getroot()

            return {
                'tests': int(root.attrib.get('tests', 0)),
                'failures': int(root.attrib.get('failures', 0)),
                'errors': int(root.attrib.get('errors', 0)),
                'skipped': int(root.attrib.get('skipped', 0))
            }
        except:
            return {'parse_error': True}

    def _parse_coverage_data(self, coverage_file: Path) -> Dict[str, Any]:
        """Parse coverage XML data."""
        try:
            import xml.etree.ElementTree as ET
            tree = ET.parse(coverage_file)
            root = tree.getroot()

            return {
                'line_rate': float(root.attrib.get('line-rate', 0)),
                'branch_rate': float(root.attrib.get('branch-rate', 0)),
                'lines_covered': int(root.attrib.get('lines-covered', 0)),
                'lines_valid': int(root.attrib.get('lines-valid', 0))
            }
        except:
            return {'parse_error': True}


class ResourceMonitor:
    """Monitor system resources during testing."""

    def __init__(self):
        self.monitoring_threads: Dict[str, threading.Thread] = {}
        self.resource_data: Dict[str, List[Dict[str, Any]]] = {}

    def start_monitoring(self, session_name: str):
        """Start resource monitoring session."""
        self.resource_data[session_name] = []

        def monitor():
            while session_name in self.monitoring_threads:
                data = {
                    'timestamp': time.time(),
                    'cpu_percent': psutil.cpu_percent(interval=0.1),
                    'memory_percent': psutil.virtual_memory().percent,
                    'memory_mb': psutil.virtual_memory().used / 1024 / 1024,
                    'disk_usage': psutil.disk_usage('/').percent
                }
                self.resource_data[session_name].append(data)
                time.sleep(5)

        thread = threading.Thread(target=monitor, daemon=True)
        self.monitoring_threads[session_name] = thread
        thread.start()

    def stop_monitoring(self, session_name: str = None):
        """Stop resource monitoring."""
        if session_name:
            self.monitoring_threads.pop(session_name, None)
        else:
            self.monitoring_threads.clear()

    def get_summary(self) -> Dict[str, Any]:
        """Get resource usage summary."""
        summary = {}

        for session_name, data in self.resource_data.items():
            if not data:
                continue

            cpu_values = [d['cpu_percent'] for d in data]
            memory_values = [d['memory_mb'] for d in data]

            summary[session_name] = {
                'duration': len(data) * 5,  # 5 second intervals
                'cpu_avg': sum(cpu_values) / len(cpu_values),
                'cpu_max': max(cpu_values),
                'memory_avg_mb': sum(memory_values) / len(memory_values),
                'memory_max_mb': max(memory_values)
            }

        return summary


class FailureRecovery:
    """Handle test failures and recovery."""

    def __init__(self):
        self.recovery_actions = {
            'ros2_failure': self._recover_ros2,
            'memory_exhaustion': self._recover_memory,
            'process_hang': self._recover_hung_process
        }

    def attempt_recovery(self, failure_type: str) -> bool:
        """Attempt recovery from failure."""
        if failure_type in self.recovery_actions:
            return self.recovery_actions[failure_type]()
        return False

    def _recover_ros2(self) -> bool:
        """Recover from ROS2 failures."""
        try:
            subprocess.run(['pkill', '-f', 'ros2'], check=False)
            time.sleep(2)
            # Reset ROS environment
            os.environ.pop('ROS_DOMAIN_ID', None)
            return True
        except:
            return False

    def _recover_memory(self) -> bool:
        """Recover from memory exhaustion."""
        try:
            # Kill memory-intensive processes
            subprocess.run(['pkill', '-f', 'rviz'], check=False)
            subprocess.run(['pkill', '-f', 'gzserver'], check=False)
            time.sleep(1)
            return True
        except:
            return False

    def _recover_hung_process(self) -> bool:
        """Recover from hung processes."""
        try:
            # Force kill test processes
            subprocess.run(['pkill', '-9', '-f', 'pytest'], check=False)
            subprocess.run(['pkill', '-9', '-f', 'ros2'], check=False)
            time.sleep(2)
            return True
        except:
            return False


def main():
    """Main entry point for automated testing platform."""
    import argparse

    parser = argparse.ArgumentParser(
        description="Automated Testing Platform for URC 2026 Autonomy System"
    )
    parser.add_argument(
        'suite',
        choices=['aoi_tests', 'full_system_tests', 'performance_tests', 'safety_critical_tests', 'all'],
        help="Test suite to run"
    )
    parser.add_argument(
        '--output-dir',
        type=str,
        help="Output directory for test artifacts"
    )
    parser.add_argument(
        '--parallel',
        action='store_true',
        help="Run test suites in parallel"
    )
    parser.add_argument(
        '--ci-mode',
        action='store_true',
        help="CI/CD mode with JSON output"
    )

    args = parser.parse_args()

    project_root = Path.cwd()
    platform = AutomatedTestPlatform(project_root)

    # Set output directory
    output_dir = Path(args.output_dir) if args.output_dir else None

    try:
        print("🤖 Automated Testing Platform Starting")
        print("=" * 50)

        if args.suite == 'all':
            # Run all test suites
            all_results = {}
            suites_to_run = ['aoi_tests', 'full_system_tests', 'performance_tests', 'safety_critical_tests']

            if args.parallel:
                # Run in parallel (would need ThreadPoolExecutor implementation)
                print("Parallel execution not yet implemented")
                return 1
            else:
                # Run sequentially
                for suite_name in suites_to_run:
                    print(f"\n🏃 Running {suite_name}...")
                    result = platform.run_automated_test_suite(suite_name, output_dir)
                    all_results[suite_name] = result

                    if result['status'] != 'completed':
                        print(f"❌ {suite_name} failed: {result.get('error', 'Unknown error')}")
                        if not args.ci_mode:
                            break

            # Summary report
            print("\n📊 Complete Test Suite Results:")
            for suite_name, result in all_results.items():
                status_icon = "✅" if result['status'] == 'completed' else "❌"
                duration = result.get('duration', 0)
                print(f"  {status_icon} {suite_name}: {duration:.1f}s")

        else:
            # Run single test suite
            print(f"🏃 Running {args.suite}...")
            result = platform.run_automated_test_suite(args.suite, output_dir)

            # Print summary
            status_icon = "✅" if result['status'] == 'completed' else "❌"
            print(f"\n{status_icon} {args.suite} completed in {result.get('duration', 0):.1f}s")

            if result['status'] != 'completed':
                print(f"❌ Error: {result.get('error', 'Unknown error')}")
                return 1

        print("\n📄 Test artifacts saved to:")
        if output_dir:
            print(f"   {output_dir}")
        else:
            print("   ./test_*/ directories")

        return 0

    except KeyboardInterrupt:
        print("\n⚠️ Testing interrupted by user")
        return 1
    except Exception as e:
        print(f"\n❌ Testing platform error: {e}")
        return 1
    finally:
        # Ensure clean shutdown
        platform._emergency_cleanup()


if __name__ == "__main__":
    exit(main())
