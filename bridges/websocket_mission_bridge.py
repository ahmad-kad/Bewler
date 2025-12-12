#!/usr/bin/env python3
"""
WebSocket Mission Bridge - Routes commands from WebSocket to ROS2

This bridge subscribes to WebSocket messages and publishes them to ROS2 topics
that the mission executor can consume. Includes CAN bus mock data integration
and priority-based message routing.

Usage: Commands come from WebSocket clients and get published to /mission/commands
CAN mock data requests are served from the integrated simulator.
"""

import asyncio
import json
import threading
import time
from typing import Any, Dict, Optional

import rclpy
import websockets
from rclpy.node import Node
from std_msgs.msg import String

# Import our new components
from .can_mock_simulator import CANBusMockSimulator
from .priority_message_router import MessagePriority, PriorityMessageRouter


class WebSocketMissionBridge(Node):
    """
    WebSocket to ROS2 Bridge for Mission Commands with CAN Mock Integration

    Listens for WebSocket connections and forwards mission commands to ROS2.
    Includes CAN bus mock data simulation and priority-based message routing.
    """

    def __init__(self):
        super().__init__('websocket_mission_bridge')

        # Initialize components
        self.can_simulator = CANBusMockSimulator()
        self.message_router = PriorityMessageRouter(max_queue_size=100)

        # Publishers for different message types
        self.mission_cmd_pub = self.create_publisher(String, '/mission/commands', 10)
        self.can_data_pub = self.create_publisher(String, '/can/sensor_data', 10)
        self.system_status_pub = self.create_publisher(String, '/system/status', 10)

        # Subscribers for CAN data requests
        self.can_request_sub = self.create_subscription(
            String, '/can/data_request', self.handle_can_request, 10)

        # WebSocket server settings
        self.websocket_port = 8765
        self.websocket_host = '0.0.0.0'

        # Status tracking
        self.connected_clients = 0
        self.messages_processed = 0

        # Start WebSocket server in separate thread
        self.websocket_thread = threading.Thread(target=self.run_websocket_server)
        self.websocket_thread.daemon = True
        self.websocket_thread.start()

        # Start message processing
        self.processing_thread = threading.Thread(target=self.process_message_queue)
        self.processing_thread.daemon = True
        self.processing_thread.start()

        self.get_logger().info(f'WebSocket Mission Bridge with CAN Mock started on {self.websocket_host}:{self.websocket_port}')
        self.get_logger().warning('WARNING: CAN data is MOCK/SIMULATED - NOT REAL HARDWARE')

    def handle_can_request(self, msg):
        """Handle CAN data requests from ROS2 nodes"""
        try:
            request = json.loads(msg.data)
            sensor_type = request.get('sensor', 'unknown')

            # Get mock data from simulator
            mock_data = self.can_simulator.get_mock_reading(sensor_type)

            # Publish to ROS2
            response_msg = String()
            response_msg.data = json.dumps(mock_data)
            self.can_data_pub.publish(response_msg)

            self.get_logger().info(f'Published CAN mock data for sensor: {sensor_type}')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid CAN request JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error handling CAN request: {e}')

    def process_message_queue(self):
        """Process messages from priority queue"""
        while rclpy.ok():
            try:
                message = self.message_router.dequeue_message()
                if message:
                    self.process_message(message)
                    self.messages_processed += 1
                else:
                    # Small delay when queue is empty
                    time.sleep(0.01)
            except Exception as e:
                self.get_logger().error(f'Error processing message queue: {e}')

    def process_message(self, message: Dict[str, Any]):
        """Process a single message based on type"""
        message_type = message.get('type', 'unknown')

        if message_type in ['mission_command', 'waypoint_command', 'start_mission']:
            # Mission commands - publish to ROS2
            ros_msg = String()
            ros_msg.data = json.dumps(message)
            self.mission_cmd_pub.publish(ros_msg)
            self.get_logger().info(f'Published mission command: {message_type}')

        elif message_type in ['can_data', 'sensor_data']:
            # CAN/sensor data - already published by handle_can_request
            pass

        elif message_type == 'system_status':
            # System status updates
            ros_msg = String()
            ros_msg.data = json.dumps(message)
            self.system_status_pub.publish(ros_msg)

        else:
            self.get_logger().warning(f'Unknown message type: {message_type}')

    def run_websocket_server(self):
        """Run WebSocket server in asyncio event loop"""
        asyncio.run(self.websocket_main())

    async def websocket_main(self):
        """Main WebSocket server loop"""
        async def handler(websocket, path):
            client_id = f"client_{id(websocket)}"
            self.connected_clients += 1

            try:
                self.get_logger().info(f'WebSocket client connected: {client_id}')

                # Send welcome message with mock data warning
                await websocket.send(json.dumps({
                    'type': 'welcome',
                    'client_id': client_id,
                    'message': 'Connected to URC WebSocket Bridge',
                    'mock_warning': 'CAN data is MOCK/SIMULATED - NOT REAL HARDWARE',
                    'capabilities': ['mission_commands', 'can_data_requests', 'system_status']
                }))

                async for message in websocket:
                    try:
                        # Parse incoming message
                        data = json.loads(message)

                        # Add client identification
                        data['client_id'] = client_id
                        data['websocket_received'] = time.time()

                        # Route message through priority queue
                        self.message_router.enqueue_message(data, client_id)

                        # Send immediate acknowledgment
                        await websocket.send(json.dumps({
                            'type': 'ack',
                            'message_id': data.get('id', 'unknown'),
                            'status': 'queued',
                            'priority': str(self.message_router.determine_priority(data)),
                            'queue_size': self.message_router.get_queue_status()['queue_size']
                        }))

                        self.get_logger().info(f'Enqueued message from {client_id}: {data.get("type", "unknown")}')

                    except json.JSONDecodeError:
                        await websocket.send(json.dumps({
                            'type': 'error',
                            'error': 'Invalid JSON format'
                        }))
                    except Exception as e:
                        self.get_logger().error(f'Error processing message from {client_id}: {e}')
                        await websocket.send(json.dumps({
                            'type': 'error',
                            'error': str(e)
                        }))

            except websockets.exceptions.ConnectionClosed:
                self.connected_clients -= 1
                self.get_logger().info(f'WebSocket client disconnected: {client_id}')

        # Start WebSocket server
        server = await websockets.serve(handler, self.websocket_host, self.websocket_port)
        self.get_logger().info(f'WebSocket server started on {self.websocket_host}:{self.websocket_port}')

        # Keep server running
        await server.wait_closed()

    def publish_mission_command(self, command_data: Dict[str, Any]):
        """Publish mission command to ROS2 topic"""
        msg = String()
        msg.data = json.dumps(command_data)
        self.mission_cmd_pub.publish(msg)


def main():
    """Main entry point"""
    rclpy.init()
    try:
        node = WebSocketMissionBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
