#!/usr/bin/env python3
"""
Non-Invasive Monitoring System

Event-driven, configurable, and resource-aware monitoring for robotics systems.
Designed to not impact real-time performance while providing comprehensive observability.
"""

import threading
import time
from collections import deque
from typing import Dict, Any, List, Optional, Callable, Set
from dataclasses import dataclass, field
from enum import Enum
import structlog
import uuid

logger = structlog.get_logger(__name__)


class MonitoringEvent(Enum):
    """Types of monitoring events that can trigger data collection."""
    DETECTION = "detection"
    FAILURE = "failure"
    EMERGENCY = "emergency"
    PERFORMANCE_DEGRADATION = "performance_degradation"
    SYSTEM_STATE_CHANGE = "system_state_change"
    MISSION_PROGRESS = "mission_progress"
    HEALTH_CHECK = "health_check"


class SamplingRate(Enum):
    """Sampling rate configurations."""
    HIGH = "high"      # Collect all events (for debugging)
    MEDIUM = "medium"  # Collect important events only
    LOW = "low"        # Collect critical events only
    OFF = "off"        # No collection


@dataclass
class MonitoringConfig:
    """Configuration for the monitoring system."""
    sampling_rate: SamplingRate = SamplingRate.MEDIUM
    max_buffer_size: int = 1000
    flush_interval: float = 30.0  # seconds
    enable_async_processing: bool = True
    event_filters: Set[MonitoringEvent] = field(default_factory=lambda: {
        MonitoringEvent.FAILURE,
        MonitoringEvent.EMERGENCY,
        MonitoringEvent.DETECTION,
        MonitoringEvent.PERFORMANCE_DEGRADATION
    })
    performance_monitoring: bool = True
    memory_limit_mb: int = 50


@dataclass
class MonitoringEventData:
    """Data structure for monitoring events."""
    event_id: str
    event_type: MonitoringEvent
    timestamp: float
    source: str
    data: Dict[str, Any]
    correlation_id: str
    processing_time: Optional[float] = None


class NonInvasiveMonitor:
    """
    Non-invasive monitoring system that collects data only on specific events.

    Features:
    - Event-driven collection (no continuous polling)
    - Configurable sampling rates
    - Asynchronous processing to avoid blocking
    - Resource-aware with memory limits
    - Performance monitoring for the monitoring system itself
    """

    def __init__(self, config: MonitoringConfig):
        self.config = config
        self.event_buffer: deque = deque(maxlen=config.max_buffer_size)
        self.event_handlers: Dict[MonitoringEvent, List[Callable]] = {}
        self.is_running = False
        self.flush_thread: Optional[threading.Thread] = None
        self.lock = threading.RLock()

        # Performance tracking for the monitoring system itself
        self.monitoring_overhead = {
            'events_processed': 0,
            'processing_time_total': 0.0,
            'memory_usage': 0,
            'last_performance_check': time.time()
        }

        # Initialize event handlers
        for event_type in MonitoringEvent:
            self.event_handlers[event_type] = []

        logger.info("Non-invasive monitoring system initialized",
                   sampling_rate=config.sampling_rate.value,
                   max_buffer_size=config.max_buffer_size)

    def start(self):
        """Start the monitoring system."""
        if self.is_running:
            return

        self.is_running = True
        if self.config.enable_async_processing:
            self.flush_thread = threading.Thread(target=self._async_flush_worker,
                                               daemon=True,
                                               name="monitoring-flush")
            self.flush_thread.start()

        logger.info("Non-invasive monitoring system started")

    def stop(self):
        """Stop the monitoring system."""
        if not self.is_running:
            return

        self.is_running = False
        if self.flush_thread and self.flush_thread.is_alive():
            self.flush_thread.join(timeout=5.0)

        # Final flush
        self._flush_events()
        logger.info("Non-invasive monitoring system stopped")

    def register_event_handler(self, event_type: MonitoringEvent,
                              handler: Callable[[MonitoringEventData], None]):
        """
        Register a handler for specific event types.

        Args:
            event_type: Type of event to handle
            handler: Function to call when event occurs
        """
        with self.lock:
            if event_type not in self.event_handlers:
                self.event_handlers[event_type] = []
            self.event_handlers[event_type].append(handler)

            logger.debug("Event handler registered",
                        event_type=event_type.value,
                        handler_count=len(self.event_handlers[event_type]))

    def record_event(self, event_type: MonitoringEvent, source: str,
                    data: Dict[str, Any], correlation_id: Optional[str] = None) -> bool:
        """
        Record a monitoring event (non-blocking).

        Args:
            event_type: Type of event being recorded
            source: Component that generated the event
            data: Event-specific data
            correlation_id: Optional correlation ID for tracing

        Returns:
            True if event was recorded, False if filtered out or system disabled
        """
        # Check if monitoring is enabled for this event type
        if self.config.sampling_rate == SamplingRate.OFF:
            return False

        # Apply event filtering based on sampling rate
        if not self._should_record_event(event_type):
            return False

        # Create event data
        event_data = MonitoringEventData(
            event_id=str(uuid.uuid4()),
            event_type=event_type,
            timestamp=time.time(),
            source=source,
            data=data,
            correlation_id=correlation_id or str(uuid.uuid4())
        )

        # Record processing start time for performance monitoring
        start_time = time.time()

        # Add to buffer (thread-safe)
        with self.lock:
            self.event_buffer.append(event_data)

        # Update performance metrics
        processing_time = time.time() - start_time
        self.monitoring_overhead['events_processed'] += 1
        self.monitoring_overhead['processing_time_total'] += processing_time

        # Trigger immediate handlers if configured for high sampling
        if self.config.sampling_rate == SamplingRate.HIGH:
            self._trigger_handlers(event_data)

        return True

    def _should_record_event(self, event_type: MonitoringEvent) -> bool:
        """Determine if an event should be recorded based on configuration."""
        # Always record critical events regardless of sampling rate
        critical_events = {MonitoringEvent.EMERGENCY, MonitoringEvent.FAILURE}

        if event_type in critical_events:
            return True

        # Apply sampling rate filtering
        if self.config.sampling_rate == SamplingRate.HIGH:
            return True
        elif self.config.sampling_rate == SamplingRate.MEDIUM:
            return event_type in self.config.event_filters
        elif self.config.sampling_rate == SamplingRate.LOW:
            # Only critical events (already handled above)
            return False

        return False

    def _trigger_handlers(self, event_data: MonitoringEventData):
        """Trigger registered event handlers."""
        handlers = self.event_handlers.get(event_data.event_type, [])
        for handler in handlers:
            try:
                # Call handler in separate thread to avoid blocking
                if self.config.enable_async_processing:
                    threading.Thread(target=handler, args=(event_data,),
                                   daemon=True, name=f"event-handler-{event_data.event_id}").start()
                else:
                    handler(event_data)
            except Exception as e:
                logger.error("Event handler failed",
                           event_id=event_data.event_id,
                           handler=str(handler),
                           error=str(e))

    def _async_flush_worker(self):
        """Background worker for periodic event flushing."""
        while self.is_running:
            time.sleep(self.config.flush_interval)
            try:
                self._flush_events()
            except Exception as e:
                logger.error("Event flush failed", error=str(e))

    def _flush_events(self):
        """Flush accumulated events to handlers."""
        events_to_process = []

        with self.lock:
            # Get all events from buffer
            while self.event_buffer:
                events_to_process.append(self.event_buffer.popleft())

        # Process events outside the lock
        for event_data in events_to_process:
            self._trigger_handlers(event_data)

        if events_to_process:
            logger.debug("Events flushed",
                        event_count=len(events_to_process),
                        buffer_remaining=len(self.event_buffer))

    def get_monitoring_stats(self) -> Dict[str, Any]:
        """Get monitoring system performance statistics."""
        current_time = time.time()
        time_elapsed = current_time - self.monitoring_overhead['last_performance_check']

        stats = {
            'events_processed': self.monitoring_overhead['events_processed'],
            'buffer_size': len(self.event_buffer),
            'buffer_capacity': self.config.max_buffer_size,
            'sampling_rate': self.config.sampling_rate.value,
            'avg_processing_time_ms': (
                (self.monitoring_overhead['processing_time_total'] /
                 max(self.monitoring_overhead['events_processed'], 1)) * 1000
            ),
            'events_per_second': (
                self.monitoring_overhead['events_processed'] / max(time_elapsed, 1.0)
            ),
            'is_running': self.is_running
        }

        # Update last check time
        self.monitoring_overhead['last_performance_check'] = current_time

        return stats

    def update_config(self, new_config: MonitoringConfig):
        """Update monitoring configuration."""
        old_rate = self.config.sampling_rate
        self.config = new_config

        if old_rate != new_config.sampling_rate:
            logger.info("Monitoring sampling rate changed",
                       old_rate=old_rate.value,
                       new_rate=new_config.sampling_rate.value)


# Global monitoring instance
_monitoring_instance: Optional[NonInvasiveMonitor] = None
_monitoring_lock = threading.Lock()


def get_monitor() -> NonInvasiveMonitor:
    """Get the global monitoring instance."""
    global _monitoring_instance

    with _monitoring_lock:
        if _monitoring_instance is None:
            # Default configuration - can be overridden
            config = MonitoringConfig()
            _monitoring_instance = NonInvasiveMonitor(config)
            _monitoring_instance.start()

    return _monitoring_instance


def record_detection(source: str, detection_data: Dict[str, Any],
                    correlation_id: Optional[str] = None):
    """Record a detection event."""
    monitor = get_monitor()
    return monitor.record_event(
        MonitoringEvent.DETECTION,
        source,
        detection_data,
        correlation_id
    )


def record_failure(source: str, failure_data: Dict[str, Any],
                  correlation_id: Optional[str] = None):
    """Record a failure event."""
    monitor = get_monitor()
    return monitor.record_event(
        MonitoringEvent.FAILURE,
        source,
        failure_data,
        correlation_id
    )


def record_emergency(source: str, emergency_data: Dict[str, Any],
                    correlation_id: Optional[str] = None):
    """Record an emergency event."""
    monitor = get_monitor()
    return monitor.record_event(
        MonitoringEvent.EMERGENCY,
        source,
        emergency_data,
        correlation_id
    )


def record_performance_degradation(source: str, perf_data: Dict[str, Any],
                                  correlation_id: Optional[str] = None):
    """Record a performance degradation event."""
    monitor = get_monitor()
    return monitor.record_event(
        MonitoringEvent.PERFORMANCE_DEGRADATION,
        source,
        perf_data,
        correlation_id
    )


# Convenience functions for common monitoring patterns
def monitor_function_performance(func_name: str, execution_time: float,
                               threshold_ms: float = 100.0):
    """Monitor function performance and record if it exceeds threshold."""
    if execution_time > (threshold_ms / 1000.0):
        record_performance_degradation(
            "function_monitor",
            {
                "function": func_name,
                "execution_time_ms": execution_time * 1000,
                "threshold_ms": threshold_ms,
                "exceeded_by_ms": (execution_time * 1000) - threshold_ms
            }
        )


def monitor_data_quality(source: str, quality_metrics: Dict[str, Any]):
    """Monitor data quality and record issues."""
    # Check for quality issues
    issues = []
    if quality_metrics.get('consecutive_failures', 0) > 5:
        issues.append("high_consecutive_failures")

    if quality_metrics.get('data_age_seconds', 0) > 5.0:
        issues.append("stale_data")

    if issues:
        record_failure(
            source,
            {
                "type": "data_quality_issue",
                "issues": issues,
                "metrics": quality_metrics
            }
        )


