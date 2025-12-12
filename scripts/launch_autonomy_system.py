#!/usr/bin/env python3
"""
URC 2026 Autonomy System Launcher
Launches all ROS2 nodes for system integration testing
"""

import signal
import subprocess
import sys
import time
from pathlib import Path


class AutonomySystemLauncher:
    """Launcher for URC 2026 autonomy system components."""

    def __init__(self):
        self.processes = []
        self.nodes_launched = []

        # Node launch configurations
        self.node_configs = {
            'state_machine': {
                'cmd': ['ros2', 'run', 'autonomy_state_management', 'state_machine_director'],
                'name': 'State Machine Director'
            },
            'safety_watchdog': {
                'cmd': ['ros2', 'run', 'autonomy_safety_system', 'safety_watchdog'],
                'name': 'Safety Watchdog'
            },
            'computer_vision': {
                'cmd': ['ros2', 'run', 'autonomy_computer_vision', 'computer_vision_node'],
                'name': 'Computer Vision Node'
            },
            'navigation': {
                'cmd': ['ros2', 'run', 'autonomy_navigation', 'navigation_node'],
                'name': 'Navigation Node'
            },
            'sensor_bridge': {
                'cmd': ['ros2', 'run', 'autonomy_sensor_bridge', 'sensor_bridge'],
                'name': 'Sensor Bridge'
            },
            'led_status': {
                'cmd': ['ros2', 'run', 'autonomy_led_status', 'led_controller'],
                'name': 'LED Status Controller'
            }
        }

    def launch_node(self, node_key):
        """Launch a single ROS2 node."""
        if node_key not in self.node_configs:
            print(f" Unknown node: {node_key}")
            return False

        config = self.node_configs[node_key]
        print(f" Launching {config['name']}...")

        try:
            process = subprocess.Popen(
                config['cmd'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )

            # Wait a moment for startup
            time.sleep(2)

            # Check if process is still running
            if process.poll() is None:
                self.processes.append(process)
                self.nodes_launched.append(node_key)
                print(f" {config['name']} launched successfully")
                return True
            else:
                stdout, stderr = process.communicate()
                print(f" {config['name']} failed to start:")
                print(f"stdout: {stdout}")
                print(f"stderr: {stderr}")
                return False

        except Exception as e:
            print(f" Failed to launch {config['name']}: {e}")
            return False

    def launch_system(self, nodes_to_launch=None):
        """Launch the complete autonomy system."""
        if nodes_to_launch is None:
            # Launch in dependency order
            nodes_to_launch = ['safety_watchdog', 'state_machine', 'computer_vision',
                             'navigation', 'sensor_bridge', 'led_status']

        print(" LAUNCHING URC 2026 AUTONOMY SYSTEM")
        print("=" * 50)

        success_count = 0
        for node in nodes_to_launch:
            if self.launch_node(node):
                success_count += 1
            time.sleep(1)  # Brief pause between launches

        print(f"\n Launch Results: {success_count}/{len(nodes_to_launch)} nodes started")

        if success_count == len(nodes_to_launch):
            print(" ALL SYSTEMS GO - Autonomy system operational!")
        elif success_count >= 3:  # Core systems running
            print(" PARTIAL SUCCESS - Core systems operational, some nodes failed")
        else:
            print(" LAUNCH FAILURE - Critical systems not operational")

        return success_count

    def shutdown(self):
        """Shutdown all launched processes."""
        print("\n Shutting down autonomy system...")

        for process in self.processes:
            try:
                process.terminate()
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                print(" Force killing process...")
                process.kill()

        self.processes.clear()
        self.nodes_launched.clear()
        print(" All processes terminated")

def main():
    """Main launcher function."""
    launcher = AutonomySystemLauncher()

    def signal_handler(sig, frame):
        print("\n Shutdown signal received...")
        launcher.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        success_count = launcher.launch_system()

        if success_count > 0:
            print("\n⏳ System running... Press Ctrl+C to shutdown")

            # Keep running until interrupted
            while True:
                time.sleep(1)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f" Launch error: {e}")
    finally:
        launcher.shutdown()

if __name__ == '__main__':
    main()
