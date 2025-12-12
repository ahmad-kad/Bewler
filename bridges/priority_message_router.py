#!/usr/bin/env python3
"""
Priority-Based Message Router for WebSocket Bridges

Routes messages based on priority levels to ensure critical systems
(safety, calibration) are not starved by lower-priority traffic.
"""

import asyncio
import heapq
import json
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional


class MessagePriority(Enum):
    """Message priority levels"""
    CRITICAL = 1    # Safety systems, emergency stops
    HIGH = 2        # Calibration, navigation commands
    NORMAL = 3      # Sensor data, telemetry
    LOW = 4         # Status updates, diagnostics


@dataclass(order=True)
class PrioritizedMessage:
    """Message with priority for queue ordering"""
    priority: int
    timestamp: float
    message: Dict[str, Any] = field(compare=False)
    source: str = field(compare=False)

    def __post_init__(self):
        # Make priority the first comparison key (lowest number = highest priority)
        object.__setattr__(self, 'priority', self.priority)


class PriorityMessageRouter:
    """Routes messages based on priority with QoS guarantees"""

    PRIORITY_MAPPINGS = {
        # Safety critical (highest priority)
        'safety_trigger': MessagePriority.CRITICAL,
        'emergency_stop': MessagePriority.CRITICAL,
        'estop': MessagePriority.CRITICAL,

        # System control (high priority)
        'calibration_command': MessagePriority.HIGH,
        'navigation_command': MessagePriority.HIGH,
        'motor_command': MessagePriority.HIGH,
        'arm_command': MessagePriority.HIGH,

        # Sensor data (normal priority)
        'imu_data': MessagePriority.NORMAL,
        'gps_data': MessagePriority.NORMAL,
        'camera_data': MessagePriority.NORMAL,
        'battery_data': MessagePriority.NORMAL,

        # Diagnostics (low priority)
        'status_update': MessagePriority.LOW,
        'telemetry': MessagePriority.LOW,
        'diagnostic': MessagePriority.LOW,
    }

    def __init__(self, max_queue_size: int = 100):
        self.max_queue_size = max_queue_size
        self.message_queue: List[PrioritizedMessage] = []
        self.processing_stats = {
            'messages_processed': 0,
            'messages_dropped': 0,
            'priority_distribution': {p: 0 for p in MessagePriority},
            'avg_processing_time': 0.0
        }

    def determine_priority(self, message: Dict[str, Any]) -> MessagePriority:
        """Determine message priority based on content"""
        message_type = message.get('type', '')

        # Direct mapping
        if message_type in self.PRIORITY_MAPPINGS:
            return self.PRIORITY_MAPPINGS[message_type]

        # Content-based priority
        if 'safety' in message_type.lower() or 'emergency' in message_type.lower():
            return MessagePriority.CRITICAL

        if 'calibration' in message_type.lower() or 'command' in message_type.lower():
            return MessagePriority.HIGH

        if 'data' in message_type.lower() or 'sensor' in message_type.lower():
            return MessagePriority.NORMAL

        # Default to low priority
        return MessagePriority.LOW

    def enqueue_message(self, message: Dict[str, Any], source: str = "unknown") -> bool:
        """Add message to priority queue"""
        priority = self.determine_priority(message)
        timestamp = time.time()

        prioritized_msg = PrioritizedMessage(
            priority=priority.value,
            timestamp=timestamp,
            message=message,
            source=source
        )

        # Add to priority queue (heapq maintains order)
        heapq.heappush(self.message_queue, prioritized_msg)

        # Enforce queue size limit (drop lowest priority messages)
        if len(self.message_queue) > self.max_queue_size:
            # Remove lowest priority message (last in heap)
            dropped = heapq.heappop(self.message_queue)
            self.processing_stats['messages_dropped'] += 1
            print(f"⚠️  Dropped low-priority message from {dropped.source}")

        return True

    def dequeue_message(self) -> Optional[Dict[str, Any]]:
        """Get next highest priority message"""
        if not self.message_queue:
            return None

        start_time = time.time()
        prioritized_msg = heapq.heappop(self.message_queue)

        # Update statistics
        self.processing_stats['messages_processed'] += 1
        self.processing_stats['priority_distribution'][MessagePriority(prioritized_msg.priority)] += 1

        processing_time = time.time() - start_time
        self.processing_stats['avg_processing_time'] = (
            (self.processing_stats['avg_processing_time'] *
             (self.processing_stats['messages_processed'] - 1) +
             processing_time) / self.processing_stats['messages_processed']
        )

        return prioritized_msg.message

    def get_queue_status(self) -> Dict[str, Any]:
        """Get current queue status and statistics"""
        priority_counts = {}
        for msg in self.message_queue:
            priority = MessagePriority(msg.priority)
            priority_counts[priority] = priority_counts.get(priority, 0) + 1

        return {
            'queue_size': len(self.message_queue),
            'max_queue_size': self.max_queue_size,
            'priority_breakdown': {str(k): v for k, v in priority_counts.items()},
            'stats': self.processing_stats.copy(),
            'oldest_message_age': (
                time.time() - min((msg.timestamp for msg in self.message_queue), default=time.time())
                if self.message_queue else 0
            )
        }

    def clear_queue(self, priority_filter: Optional[MessagePriority] = None):
        """Clear messages from queue (optionally by priority)"""
        if priority_filter is None:
            self.message_queue.clear()
        else:
            self.message_queue = [
                msg for msg in self.message_queue
                if MessagePriority(msg.priority) != priority_filter
            ]
            heapq.heapify(self.message_queue)  # Restore heap property

    async def process_messages(self, message_handler, poll_interval: float = 0.01):
        """Continuously process messages from queue"""
        print("🔄 Priority Message Router: Starting message processing...")

        while True:
            message = self.dequeue_message()
            if message:
                try:
                    await message_handler(message)
                except Exception as e:
                    print(f"❌ Error processing message: {e}")
                    # Could implement retry logic here

            await asyncio.sleep(poll_interval)


class CANMessageRouter(PriorityMessageRouter):
    """Specialized router for CAN bus messages"""

    CAN_PRIORITY_MAPPINGS = {
        # CAN message IDs to priority mapping
        0x100: MessagePriority.HIGH,   # Motor commands
        0x101: MessagePriority.NORMAL,  # Motor feedback
        0x200: MessagePriority.NORMAL,  # IMU data
        0x201: MessagePriority.NORMAL,  # GPS data
        0x300: MessagePriority.LOW,    # Battery status
        0x400: MessagePriority.CRITICAL,  # Safety triggers
        0x500: MessagePriority.HIGH,   # Calibration data
    }

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.can_stats = {
            'can_messages_received': 0,
            'can_messages_processed': 0,
            'protocol_errors': 0
        }

    def determine_priority(self, message: Dict[str, Any]) -> MessagePriority:
        """CAN-specific priority determination"""
        # Check for CAN-specific priority
        can_id = message.get('can_id')
        if can_id in self.CAN_PRIORITY_MAPPINGS:
            return self.CAN_PRIORITY_MAPPINGS[can_id]

        # Fall back to general priority determination
        return super().determine_priority(message)

    def route_can_message(self, can_id: int, data: bytes, source: str = "can_bus") -> bool:
        """Route incoming CAN message"""
        self.can_stats['can_messages_received'] += 1

        try:
            # Decode CAN message (simplified)
            message = {
                'type': 'can_data',
                'can_id': can_id,
                'data': data.hex(),  # Convert bytes to hex string
                'source': source,
                'timestamp': time.time(),
                'mock': True  # Flag for testing
            }

            return self.enqueue_message(message, source)

        except Exception as e:
            self.can_stats['protocol_errors'] += 1
            print(f"❌ CAN protocol error: {e}")
            return False


# Example usage and testing
async def example_message_handler(message: Dict[str, Any]):
    """Example message handler for testing"""
    print(f"📨 Processing: {message.get('type', 'unknown')} from {message.get('source', 'unknown')}")

    # Simulate processing delay based on priority
    priority = message.get('priority', MessagePriority.NORMAL)
    delay = 0.001 * priority.value  # Lower priority = longer processing
    await asyncio.sleep(delay)


async def demo_priority_router():
    """Demonstrate priority message routing"""
    router = PriorityMessageRouter(max_queue_size=50)

    # Add messages with different priorities
    messages = [
        {'type': 'safety_trigger', 'data': 'emergency'},
        {'type': 'telemetry', 'sensor': 'temp', 'value': 25.5},
        {'type': 'navigation_command', 'command': 'move_forward'},
        {'type': 'diagnostic', 'component': 'camera', 'status': 'ok'},
        {'type': 'imu_data', 'accel': [0, 0, 9.81]},
        {'type': 'calibration_command', 'action': 'start'},
    ]

    print("📤 Adding messages to priority queue...")
    for msg in messages:
        router.enqueue_message(msg, "demo_source")

    print("📊 Queue status:", json.dumps(router.get_queue_status(), indent=2))

    # Process messages (they should come out in priority order)
    print("🔄 Processing messages in priority order...")
    await router.process_messages(example_message_handler)

    # Stop after a few seconds
    await asyncio.sleep(2)


if __name__ == "__main__":
    print("🚀 Priority Message Router Demo")
    asyncio.run(demo_priority_router())
