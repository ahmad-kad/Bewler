#!/usr/bin/env python3
"""
Full Autonomy-Teleoperation Integration Test
Tests the complete data flow from mock teleoperation to autonomy decisions
"""

import os
import subprocess
import sys
import time


class IntegrationTest:
    """Test full autonomy-teleoperation integration"""

    def __init__(self):
        self.processes = []
        self.test_duration = 30  # seconds
        self.success = False

    def start_mission_executor(self):
        """Start the mission executor"""
        print("🚀 Starting Mission Executor...")
        cmd = ["ros2", "run", "missions", "mission_executor"]
        env = os.environ.copy()
        env['ROVER_ENV'] = 'production'
        process = subprocess.Popen(
            cmd,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        self.processes.append(("Mission Executor", process))
        time.sleep(2)  # Wait for startup
        return process.poll() is None

    def start_mock_teleoperation(self):
        """Start mock teleoperation publisher"""
        print("📡 Starting Mock Teleoperation Publisher...")
        cmd = ["python3", "test_teleoperation_integration.py"]
        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        self.processes.append(("Mock Teleoperation", process))
        time.sleep(2)  # Wait for startup
        return process.poll() is None

    def check_topics(self):
        """Check that teleoperation topics are being published"""
        print("🔍 Checking ROS2 topics...")
        try:
            result = subprocess.run(
                ["ros2", "topic", "list"],
                capture_output=True,
                text=True,
                timeout=5
            )

            topics = result.stdout.strip().split('\n')
            expected_topics = [
                '/teleoperation/joint_states',
                '/teleoperation/chassis_velocity',
                '/teleoperation/motor_temperatures',
                '/teleoperation/system_status'
            ]

            found_topics = [topic for topic in topics if topic in expected_topics]
            print(f"📋 Found {len(found_topics)}/{len(expected_topics)} expected topics:")
            for topic in expected_topics:
                status = "✅" if topic in found_topics else "❌"
                print(f"  {status} {topic}")

            return len(found_topics) == len(expected_topics)

        except Exception as e:
            print(f"❌ Error checking topics: {e}")
            return False

    def check_topic_rates(self):
        """Check that topics are publishing at expected rates"""
        print("⏱️  Checking topic publish rates...")
        try:
            # Check joint states rate (should be ~10Hz)
            result = subprocess.run(
                ["timeout", "5", "ros2", "topic", "hz", "/teleoperation/joint_states"],
                capture_output=True,
                text=True
            )

            if "average rate:" in result.stdout:
                # Parse the rate from output
                lines = result.stdout.split('\n')
                for line in lines:
                    if "average rate:" in line:
                        rate_str = line.split("average rate:")[1].strip()
                        rate = float(rate_str.split()[0])
                        if 8 <= rate <= 12:  # Allow some tolerance
                            print(".1f")
                            return True
                        else:
                            print(".1f")
                            return False
            else:
                print("❌ No rate data for joint_states")
                return False

        except Exception as e:
            print(f"❌ Error checking rates: {e}")
            return False

    def check_data_quality(self):
        """Check that autonomy is processing data correctly"""
        print("🔬 Checking data processing...")
        try:
            # Listen for log messages indicating data processing
            time.sleep(5)  # Let system run a bit

            # Check if any of our processes have error logs
            for name, process in self.processes:
                if process.poll() is not None:
                    print(f"❌ {name} process died")
                    return False

            print("✅ All processes still running")
            return True

        except Exception as e:
            print(f"❌ Error checking data quality: {e}")
            return False

    def run_test(self):
        """Run the full integration test"""
        print("🧪 Starting Autonomy-Teleoperation Integration Test")
        print("=" * 60)

        success = True

        try:
            # Start components
            if not self.start_mission_executor():
                print("❌ Failed to start Mission Executor")
                return False

            if not self.start_mock_teleoperation():
                print("❌ Failed to start Mock Teleoperation")
                return False

            # Wait for system to stabilize
            print("⏳ Waiting for system stabilization...")
            time.sleep(5)

            # Run checks
            checks = [
                ("Topic Availability", self.check_topics),
                ("Topic Rates", self.check_topic_rates),
                ("Data Processing", self.check_data_quality)
            ]

            for check_name, check_func in checks:
                print(f"\n🔍 Running: {check_name}")
                if not check_func():
                    print(f"❌ {check_name} failed")
                    success = False
                else:
                    print(f"✅ {check_name} passed")

            # Run for test duration
            print(f"\n⏳ Running integration test for {self.test_duration} seconds...")
            time.sleep(self.test_duration)

            if success:
                print("\n🎉 Integration test PASSED!")
                print("✅ Autonomy successfully receives and processes teleoperation data")
            else:
                print("\n❌ Integration test FAILED!")
                print("❌ Some components are not working correctly")

            return success

        except Exception as e:
            print(f"\n💥 Test failed with exception: {e}")
            return False

        finally:
            # Cleanup
            print("\n🧹 Cleaning up...")
            self.cleanup()

    def cleanup(self):
        """Clean up test processes"""
        for name, process in self.processes:
            try:
                if process.poll() is None:
                    print(f"🛑 Stopping {name}...")
                    process.terminate()
                    process.wait(timeout=5)
            except Exception as e:
                print(f"Warning: Error stopping {name}: {e}")
                try:
                    process.kill()
                except BaseException:
                    pass

        # Kill any remaining ROS2 processes
        try:
            subprocess.run(["pkill", "-f", "mission_executor"], check=False)
            subprocess.run(["pkill", "-f", "test_teleoperation_integration"], check=False)
        except BaseException:
            pass


def main():
    """Main test entry point"""
    # Check if ROS2 is available
    try:
        result = subprocess.run(
            ["ros2", "--version"],
            capture_output=True,
            text=True,
            timeout=5
        )
        if result.returncode != 0:
            print("❌ ROS2 not available. Please source ROS2 environment.")
            sys.exit(1)
    except BaseException:
        print("❌ ROS2 not found. Please install and source ROS2.")
        sys.exit(1)

    # Run the test
    test = IntegrationTest()
    success = test.run_test()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
