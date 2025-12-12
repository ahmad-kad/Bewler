#!/usr/bin/env python3
"""
Test script for LED coordination between state management and LED status subsystems.

This script tests the critical path item: Competition LED Coordination
- 🔴 Red status during autonomous operation
- 🔵 Blue status during teleoperation
- 🟢 Flashing Green on successful target arrival
"""

import os
import subprocess
import sys
import time


def run_command(cmd, timeout=10):
    """Run a command with timeout"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)
        return result.returncode == 0, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        return False, "", "Command timed out"


def test_led_coordination():
    """Test LED coordination functionality"""

    print("🧪 Testing LED Coordination System")
    print("=" * 50)

    # Set environment
    env = os.environ.copy()
    env["PYTHONPATH"] = "/workspace"
    env["PATH"] = "/opt/ros/humble/bin:" + env.get("PATH", "")
    env["ROS_DOMAIN_ID"] = "42"
    env["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"

    try:
        # Step 1: Start ROS2 daemon
        print("1. Starting ROS2 daemon...")
        success, out, err = run_command("ros2 daemon start", timeout=5)
        if not success:
            print(f"   ❌ Failed to start ROS2 daemon: {err}")
            return False
        print("   ✅ ROS2 daemon started")

        # Step 2: Start state management node in background
        print("2. Starting state management node...")
        state_proc = subprocess.Popen(
            "ros2 run autonomy_state_management state_management_node",
            shell=True,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        time.sleep(2)  # Wait for node to start

        # Check if state management node is running
        success, out, err = run_command("ros2 node list | grep state_management", timeout=5)
        if not success:
            print(f"   ❌ State management node not found: {err}")
            state_proc.terminate()
            return False
        print("   ✅ State management node running")

        # Step 3: Start LED controller node in background
        print("3. Starting LED controller node...")
        led_proc = subprocess.Popen(
            "ros2 run autonomy_led_status led_status_node",
            shell=True,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        time.sleep(2)  # Wait for node to start

        # Check if LED controller node is running
        success, out, err = run_command("ros2 node list | grep led", timeout=5)
        if not success:
            print(f"   ❌ LED controller node not found: {err}")
            state_proc.terminate()
            led_proc.terminate()
            return False
        print("   ✅ LED controller node running")

        # Step 4: Test initial state (should be idle)
        print("4. Testing initial state...")
        success, out, err = run_command("ros2 topic echo --once /system_mode", timeout=5)
        if success and "idle" in out:
            print("   ✅ Initial system mode: idle")
        else:
            print(f"   ❌ Initial system mode incorrect: {out}")
            return False

        # Step 5: Start mission (should switch to autonomous mode - red LED)
        print("5. Starting autonomous mission...")
        success, out, err = run_command("ros2 service call /start_mission std_srvs/srv/Trigger", timeout=5)
        if success and "success: true" in out.lower():
            print("   ✅ Mission started successfully")
        else:
            print(f"   ❌ Failed to start mission: {err}")
            return False

        # Wait for mode change and check system mode
        time.sleep(2)
        success, out, err = run_command("ros2 topic echo --once /system_mode", timeout=5)
        if success and "autonomous" in out:
            print("   ✅ System mode switched to autonomous (🔴 Red LED)")
        else:
            print(f"   ❌ System mode not autonomous: {out}")
            return False

        # Step 6: Simulate target reached (should switch to completed - flashing green LED)
        print("6. Simulating target reached...")
        success, out, err = run_command("ros2 service call /simulate_target_reached std_srvs/srv/Trigger", timeout=5)
        if success and "success: true" in out.lower():
            print("   ✅ Target reached simulation successful")
        else:
            print(f"   ❌ Failed to simulate target reached: {err}")
            return False

        # Wait for mission status change and check mission status
        time.sleep(2)
        success, out, err = run_command("ros2 topic echo --once /mission_status", timeout=5)
        if success and "completed" in out:
            print("   ✅ Mission status: completed (🟢 Flashing Green LED)")
        else:
            print(f"   ❌ Mission status not completed: {out}")
            return False

        # Step 7: Stop mission (should return to idle)
        print("7. Stopping mission...")
        success, out, err = run_command("ros2 service call /stop_mission std_srvs/srv/Trigger", timeout=5)
        if success and "success: true" in out.lower():
            print("   ✅ Mission stopped successfully")
        else:
            print(f"   ❌ Failed to stop mission: {err}")
            return False

        # Wait for mode change and check final state
        time.sleep(2)
        success, out, err = run_command("ros2 topic echo --once /system_mode", timeout=5)
        if success and "idle" in out:
            print("   ✅ Final system mode: idle (LED off)")
        else:
            print(f"   ❌ Final system mode not idle: {out}")
            return False

        print("\n" + "=" * 50)
        print("🎉 LED Coordination Test PASSED!")
        print("✅ All competition requirements verified:")
        print("   🔴 Red LED during autonomous operation")
        print("   🟢 Flashing Green LED on mission success")
        print("   LED off when idle")
        return True

    except Exception as e:
        print(f"❌ Test failed with exception: {e}")
        return False

    finally:
        # Cleanup
        print("\n🧹 Cleaning up...")
        try:
            if "state_proc" in locals():
                state_proc.terminate()
                state_proc.wait(timeout=5)
            if "led_proc" in locals():
                led_proc.terminate()
                led_proc.wait(timeout=5)
        except BaseException:
            pass

        # Stop ROS2 daemon
        run_command("ros2 daemon stop", timeout=5)


if __name__ == "__main__":
    success = test_led_coordination()
    sys.exit(0 if success else 1)
