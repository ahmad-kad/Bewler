#!/usr/bin/env python3
"""
CAN Mock System Demo - Shows WebSocket communication with mock CAN data

This script demonstrates:
1. CAN mock simulator providing realistic sensor data
2. Priority-based message routing
3. WebSocket communication for testing

Run this to see how the system works without real CAN hardware.
"""

import asyncio
import json
import threading
import time

import websockets

from bridges.can_mock_simulator import CANBusMockSimulator


async def websocket_client_demo():
    """Demonstrate WebSocket client connecting to CAN mock system"""
    print("🔌 Connecting to CAN Mock WebSocket server...")

    try:
        async with websockets.connect("ws://localhost:8766") as websocket:
            print("✅ Connected to CAN Mock Simulator")

            # Receive welcome message
            welcome = await websocket.recv()
            welcome_data = json.loads(welcome)
            print(f"📨 Welcome: {welcome_data.get('message', 'Unknown')}")
            print(f"⚠️  {welcome_data.get('mock_warning', 'Mock system active')}")

            # Request different sensor readings
            sensors_to_test = ['imu', 'gps', 'battery', 'motor_left', 'environment']

            for sensor in sensors_to_test:
                # Send sensor request
                request = {
                    'type': 'sensor_request',
                    'sensor': sensor,
                    'timestamp': time.time()
                }

                await websocket.send(json.dumps(request))
                print(f"📤 Requested {sensor} data")

                # Receive response
                response = await websocket.recv()
                data = json.loads(response)

                if 'data' in data:
                    print(f"📨 {sensor.upper()}: {data['data']}")
                    print(f"   Mock: {data.get('mock', 'Unknown')}")
                else:
                    print(f"❌ Error for {sensor}: {data}")

                # Small delay between requests
                await asyncio.sleep(0.5)

            # Test motor command
            print("\n🔧 Testing motor command...")
            motor_cmd = {
                'type': 'motor_command',
                'motor': 'motor_left',
                'velocity': 1.5,
                'timestamp': time.time()
            }

            await websocket.send(json.dumps(motor_cmd))
            print("📤 Sent motor command: left motor at 1.5 rad/s")

            # Receive acknowledgment
            ack = await websocket.recv()
            ack_data = json.loads(ack)
            print(f"📨 Ack: {ack_data}")

            await asyncio.sleep(2)  # Let motor command take effect

            # Request motor data again to see the change
            motor_request = {
                'type': 'sensor_request',
                'sensor': 'motor_left',
                'timestamp': time.time()
            }

            await websocket.send(json.dumps(motor_request))
            motor_response = await websocket.recv()
            motor_data = json.loads(motor_response)

            if 'data' in motor_data:
                velocity = motor_data['data'].get('velocity', 'unknown')
                print(f"🔄 Motor left velocity after command: {velocity} rad/s")

    except Exception as e:
        print(f"❌ WebSocket demo failed: {e}")
        print("💡 Make sure the CAN mock simulator is running:")
        print("   python3 bridges/can_mock_simulator.py")


async def demo_priority_routing():
    """Demonstrate priority-based message routing"""
    print("\n🔄 Testing Priority Message Routing...")

    from bridges.priority_message_router import MessagePriority, PriorityMessageRouter

    router = PriorityMessageRouter(max_queue_size=20)

    # Create messages with different priorities
    messages = [
        {'type': 'telemetry', 'sensor': 'temp', 'value': 25.5},  # LOW
        {'type': 'imu_data', 'accel': [0, 0, 9.81]},           # NORMAL
        {'type': 'calibration_command', 'action': 'start'},     # HIGH
        {'type': 'safety_trigger', 'reason': 'obstacle'},       # CRITICAL
        {'type': 'navigation_command', 'waypoint': [10, 5]},    # HIGH
    ]

    print("📤 Enqueuing messages with different priorities...")

    for msg in messages:
        priority = router.determine_priority(msg)
        router.enqueue_message(msg, 'demo_client')
        print(f"   {msg['type']} → {priority.name}")

    # Process messages in priority order
    print("\n📨 Processing messages in priority order:")

    while True:
        message = router.dequeue_message()
        if message is None:
            break

        priority = router.determine_priority(message)
        print(f"   ✅ {message['type']} (Priority: {priority.name})")

    # Show final statistics
    status = router.get_queue_status()
    print("\n📊 Final Statistics:")
    print(f"   Messages processed: {status['stats']['messages_processed']}")
    print(f"   Messages dropped: {status['stats']['messages_dropped']}")
    print(f"   Priority distribution: {status['priority_breakdown']}")


def run_mock_simulator():
    """Run the CAN mock simulator in a separate thread"""
    print("🚗 Starting CAN Mock Simulator in background...")

    simulator = CANBusMockSimulator()

    async def run_server():
        await simulator.start_websocket_server()

    # Run in background thread
    def run_async():
        asyncio.run(run_server())

    thread = threading.Thread(target=run_async, daemon=True)
    thread.start()

    # Give it time to start
    time.sleep(1)

    return simulator


async def main():
    """Main demo function"""
    print("🎮 URC 2026 CAN Mock System Demo")
    print("=" * 50)

    # Start CAN mock simulator
    simulator = run_mock_simulator()

    try:
        # Demonstrate priority routing
        await demo_priority_routing()

        # Demonstrate WebSocket communication
        await websocket_client_demo()

        print("\n🎉 Demo completed successfully!")
        print("\n💡 Key Features Demonstrated:")
        print("   ✅ Mock CAN sensor data with realistic values")
        print("   ✅ Priority-based message routing")
        print("   ✅ WebSocket communication for testing")
        print("   ✅ Motor command simulation")
        print("   ✅ Real-time data updates")
        print("   ⚠️  All data is SIMULATED - NOT REAL HARDWARE")

    except KeyboardInterrupt:
        print("\n🛑 Demo interrupted by user")

    except Exception as e:
        print(f"\n❌ Demo failed with error: {e}")

    finally:
        if simulator:
            simulator.stop()
            print("🛑 CAN Mock Simulator stopped")


if __name__ == "__main__":
    asyncio.run(main())
