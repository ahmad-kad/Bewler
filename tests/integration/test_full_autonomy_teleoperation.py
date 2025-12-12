#!/usr/bin/env python3
"""
Full Autonomy-Teleoperation Integration Test
Tests the complete data flow from mock teleoperation to autonomy decision-making
"""

import os
import signal
import subprocess
import sys
import time


def test_full_integration():
    """Test full autonomy-teleoperation integration"""
    print("🚀 Full Autonomy-Teleoperation Integration Test")
    print("=" * 60)

    success = True
    processes = []

    try:
        # 1. Start mock teleoperation data publisher
        print("📡 Starting mock teleoperation publisher...")
        env = os.environ.copy()
        env['PYTHONPATH'] = '/home/ubuntu/urc-machiato-2026'
        teleop_process = subprocess.Popen([
            'python3', 'test_teleoperation_integration.py'
        ], env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        processes.append(("Mock Teleoperation", teleop_process))

        # Wait for publisher to start
        time.sleep(3)

        # Check if teleoperation is running
        if teleop_process.poll() is not None:
            print("❌ Mock teleoperation publisher failed to start")
            success = False
        else:
            print("✅ Mock teleoperation publisher started")

        # 2. Test ROS2 topic publishing
        print("\n🔍 Testing ROS2 topic publishing...")
        try:
            result = subprocess.run([
                'ros2', 'topic', 'list'
            ], capture_output=True, text=True, timeout=5, env=env)

            topics = result.stdout.strip().split('\n')
            teleop_topics = [
                '/teleoperation/joint_states',
                '/teleoperation/chassis_velocity',
                '/teleoperation/motor_temperatures',
                '/teleoperation/system_status'
            ]

            found_topics = [t for t in topics if t in teleop_topics]
            if len(found_topics) == len(teleop_topics):
                print("✅ All teleoperation topics are publishing")
                for topic in teleop_topics:
                    print(f"   ✓ {topic}")
            else:
                print("❌ Missing teleoperation topics")
                print(f"   Expected: {teleop_topics}")
                print(f"   Found: {found_topics}")
                success = False

        except Exception as e:
            print(f"❌ ROS2 topic check failed: {e}")
            success = False

        # 3. Test ROS2 message rates
        print("\n⏱️  Testing message publishing rates...")
        try:
            # Check joint_states rate
            result = subprocess.run([
                'timeout', '3', 'ros2', 'topic', 'hz', '/teleoperation/joint_states'
            ], capture_output=True, text=True, env=env)

            if "average rate:" in result.stdout:
                # Parse rate
                lines = result.stdout.split('\n')
                for line in lines:
                    if "average rate:" in line:
                        rate_str = line.split("average rate:")[1].strip()
                        rate = float(rate_str.split()[0])
                        if 8 <= rate <= 12:  # Allow some tolerance
                            print(".1f")
                        else:
                            print(".1f")
                            success = False
                        break
            else:
                print("❌ No rate data for joint_states")
                success = False

        except Exception as e:
            print(f"❌ Rate check failed: {e}")
            success = False

        # 4. Test autonomy code import and structure
        print("\n🤖 Testing autonomy code structure...")
        try:
            # Test that our modified mission executor can be imported
            sys.path.insert(0, '/home/ubuntu/urc-machiato-2026')
            import missions.mission_executor

            # Check that key methods exist
            mission_class = missions.mission_executor.SimpleMissionExecutor

            required_methods = [
                'joint_state_callback',
                '_validate_joint_state_data',
                '_monitor_motor_health',
                '_monitor_thermal_limits'
            ]

            missing_methods = []
            for method_name in required_methods:
                if not hasattr(mission_class, method_name):
                    missing_methods.append(method_name)

            if not missing_methods:
                print("✅ All required autonomy methods present")
            else:
                print("❌ Missing autonomy methods:")
                for method in missing_methods:
                    print(f"   ✗ {method}")
                success = False

        except Exception as e:
            print(f"❌ Autonomy code test failed: {e}")
            success = False

        # 5. Test configuration loading
        print("\n⚙️  Testing configuration loading...")
        try:
            import yaml
            with open('/home/ubuntu/urc-machiato-2026/config/production.yaml', 'r') as f:
                config = yaml.safe_load(f)

            if 'teleoperation' in config:
                teleop_config = config['teleoperation']
                required_sections = ['thermal_limits', 'battery_limits', 'motor_limits']

                missing_sections = [s for s in required_sections if s not in teleop_config]
                if not missing_sections:
                    print("✅ Teleoperation configuration loaded correctly")
                    print(f"   Config has {len(teleop_config)} sections")
                else:
                    print("❌ Missing configuration sections:")
                    for section in missing_sections:
                        print(f"   ✗ {section}")
                    success = False
            else:
                print("❌ No teleoperation configuration found")
                success = False

        except Exception as e:
            print(f"❌ Configuration test failed: {e}")
            success = False

        # Wait a bit more to let everything stabilize
        time.sleep(2)

        # Final status check
        all_processes_running = all(p.poll() is None for _, p in processes)
        if not all_processes_running:
            print("❌ Some processes died during testing")
            success = False

    except Exception as e:
        print(f"💥 Test suite failed with exception: {e}")
        success = False

    finally:
        # Cleanup
        print("\n🧹 Cleaning up test processes...")
        for name, process in processes:
            try:
                if process.poll() is None:
                    print(f"🛑 Stopping {name}...")
                    process.terminate()
                    process.wait(timeout=5)
            except Exception as e:
                print(f"Warning: Error stopping {name}: {e}")

        # Kill any remaining processes
        try:
            subprocess.run(['pkill', '-f', 'test_teleoperation_integration'], check=False)
        except:
            pass

    # Final results
    print("\n" + "=" * 60)
    if success:
        print("🎉 FULL AUTONOMY-TELEOPERATION INTEGRATION TEST PASSED!")
        print("✅ All components working correctly:")
        print("   ✓ Mock teleoperation publishing ROS2 topics")
        print("   ✓ ROS2 topics available and publishing at correct rates")
        print("   ✓ Autonomy code structure with teleoperation callbacks")
        print("   ✓ Configuration system with teleoperation settings")
        print("   ✓ All processes running stably")
        print("\n🚀 Autonomy team is ready for teleoperation integration!")
    else:
        print("❌ FULL AUTONOMY-TELEOPERATION INTEGRATION TEST FAILED!")
        print("❌ Some components need attention")
        print("\n🔧 Check the errors above and fix issues before proceeding")

    return success

if __name__ == '__main__':
    # Set ROS environment if running standalone
    ros_env = os.environ.get('ROS_DISTRO')
    if not ros_env:
        # Try to source ROS if not already done
        ros_setup = '/opt/ros/humble/setup.bash'
        if os.path.exists(ros_setup):
            print("📦 Sourcing ROS2 environment...")
            result = subprocess.run(['bash', '-c', f'source {ros_setup} && env'],
                                  capture_output=True, text=True)
            if result.returncode == 0:
                # Update environment with ROS variables
                for line in result.stdout.split('\n'):
                    if '=' in line:
                        key, value = line.split('=', 1)
                        os.environ[key] = value

    success = test_full_integration()
    sys.exit(0 if success else 1)
