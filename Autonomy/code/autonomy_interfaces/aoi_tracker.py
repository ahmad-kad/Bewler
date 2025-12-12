#!/usr/bin/env python3
"""
AoI (Age of Information) Tracking Infrastructure

Minimal-data AoI monitoring for real-time systems.
Provides efficient tracking with minimal memory and CPU overhead.
"""

import time
from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional, Tuple

import numpy as np


class FreshnessStatus(Enum):
    """AoI freshness categories."""

    FRESH = "FRESH"
    ACCEPTABLE = "ACCEPTABLE"
    STALE = "STALE"
    CRITICAL = "CRITICAL"


@dataclass
class AOIConfig:
    """Configuration for AoI monitoring."""

    acceptable_threshold: float = 1.0  # seconds
    optimal_threshold: float = 0.2  # seconds
    critical_threshold: float = 5.0  # seconds
    history_window: int = 32  # samples to keep
    update_interval: float = 1.0  # seconds between status updates


class AOITracker:
    """
    Minimal AoI tracker for individual sensors.

    Tracks age of information with minimal memory footprint.
    """

    def __init__(self, config: AOIConfig = None):
        self.config = config or AOIConfig()

        # Core tracking (minimal memory)
        self.last_timestamp: Optional[float] = None
        self.aoi_history: deque = deque(maxlen=self.config.history_window)

        # Statistics (computed on demand)
        self._last_stats_update = 0.0
        self._cached_stats = None

    def update(self, timestamp: float) -> float:
        """
        Update tracker with new timestamp.

        Args:
            timestamp: Sensor timestamp (seconds)

        Returns:
            Current AoI in seconds
        """
        current_time = time.time()
        aoi = current_time - timestamp

        # Update tracking
        self.last_timestamp = timestamp
        self.aoi_history.append(aoi)

        return aoi

    def get_current_aoi(self) -> float:
        """Get current age of information."""
        if self.last_timestamp is None:
            return float("inf")
        return time.time() - self.last_timestamp

    def get_freshness_status(self) -> FreshnessStatus:
        """Get current freshness status."""
        aoi = self.get_current_aoi()

        if aoi <= self.config.optimal_threshold:
            return FreshnessStatus.FRESH
        elif aoi <= self.config.acceptable_threshold:
            return FreshnessStatus.ACCEPTABLE
        elif aoi <= self.config.critical_threshold:
            return FreshnessStatus.STALE
        else:
            return FreshnessStatus.CRITICAL

    def get_quality_score(self) -> float:
        """Get quality score based on AoI (0.0-1.0)."""
        aoi = self.get_current_aoi()

        if aoi <= self.config.optimal_threshold:
            return 1.0
        elif aoi <= self.config.acceptable_threshold:
            return 0.8
        elif aoi <= self.config.critical_threshold:
            return 0.4
        else:
            return 0.0

    def get_statistics(self) -> Dict[str, float]:
        """Get AoI statistics (computed on demand)."""
        if not self.aoi_history:
            return {
                "current_aoi": float("inf"),
                "average_aoi": float("inf"),
                "max_aoi": float("inf"),
                "min_aoi": float("inf"),
                "freshness_ratio": 0.0,
            }

        current_time = time.time()
        current_aoi = self.get_current_aoi()

        # Calculate statistics
        recent_aois = list(self.aoi_history)
        average_aoi = sum(recent_aois) / len(recent_aois)

        # Freshness ratio (samples within acceptable threshold)
        fresh_count = sum(1 for a in recent_aois if a <= self.config.acceptable_threshold)
        freshness_ratio = fresh_count / len(recent_aois)

        return {
            "current_aoi": current_aoi,
            "average_aoi": average_aoi,
            "max_aoi": max(recent_aois),
            "min_aoi": min(recent_aois),
            "freshness_ratio": freshness_ratio,
        }


class SharedAOIBuffer:
    """
    Shared circular buffer for multiple sensors.

    Minimal memory usage: 4KB for 16 sensors × 32 samples.
    """

    def __init__(self, max_sensors: int = 16, history_size: int = 32):
        # Pre-allocated numpy arrays (4KB total memory)
        self.aoi_history = np.zeros((max_sensors, history_size), dtype=np.float32)
        self.timestamp_history = np.zeros((max_sensors, history_size), dtype=np.uint32)
        self.write_indices = np.zeros(max_sensors, dtype=np.uint8)

        # Sensor mapping
        self.sensor_ids: Dict[str, int] = {}
        self.next_sensor_id = 0
        self.max_sensors = max_sensors

    def update_aoi(self, sensor_name: str, aoi: float) -> bool:
        """
        Update AoI for sensor.

        Returns:
            True if update successful, False if buffer full
        """
        # Get or assign sensor ID
        if sensor_name not in self.sensor_ids:
            if self.next_sensor_id >= self.max_sensors:
                return False  # Buffer full
            self.sensor_ids[sensor_name] = self.next_sensor_id
            self.next_sensor_id += 1

        sensor_idx = self.sensor_ids[sensor_name]
        write_idx = int(self.write_indices[sensor_idx])

        # Update circular buffer
        self.aoi_history[sensor_idx, write_idx] = aoi
        self.timestamp_history[sensor_idx, write_idx] = int(time.time())

        # Advance write index
        self.write_indices[sensor_idx] = (write_idx + 1) % self.aoi_history.shape[1]

        return True

    def get_sensor_aoi(self, sensor_name: str) -> Optional[float]:
        """Get current AoI for sensor."""
        if sensor_name not in self.sensor_ids:
            return None

        sensor_idx = self.sensor_ids[sensor_name]
        write_idx = int(self.write_indices[sensor_idx])

        # Get most recent AoI (accounting for circular buffer)
        if write_idx == 0:
            recent_idx = self.aoi_history.shape[1] - 1
        else:
            recent_idx = write_idx - 1

        aoi = self.aoi_history[sensor_idx, recent_idx]

        # Check if valid (non-zero)
        return aoi if aoi > 0 else None

    def get_sensor_history(self, sensor_name: str, max_samples: int = 10) -> List[Tuple[float, float]]:
        """
        Get recent AoI history for sensor.

        Returns:
            List of (timestamp, aoi) tuples
        """
        if sensor_name not in self.sensor_ids:
            return []

        sensor_idx = self.sensor_ids[sensor_name]
        write_idx = int(self.write_indices[sensor_idx])

        history = []
        for i in range(min(max_samples, self.aoi_history.shape[1])):
            # Calculate index in circular buffer
            idx = (write_idx - 1 - i) % self.aoi_history.shape[1]

            aoi = self.aoi_history[sensor_idx, idx]
            timestamp = self.timestamp_history[sensor_idx, idx]

            if aoi > 0 and timestamp > 0:
                history.append((timestamp, aoi))

        return history

    def get_system_stats(self) -> Dict[str, float]:
        """Get system-wide AoI statistics."""
        if not self.sensor_ids:
            return {"average_aoi": 0.0, "fresh_sensors": 0, "total_sensors": 0}

        current_aois = []
        for sensor_name in self.sensor_ids.keys():
            aoi = self.get_sensor_aoi(sensor_name)
            if aoi is not None:
                current_aois.append(aoi)

        if not current_aois:
            return {
                "average_aoi": 0.0,
                "fresh_sensors": 0,
                "total_sensors": len(self.sensor_ids),
            }

        average_aoi = sum(current_aois) / len(current_aois)
        fresh_threshold = 1.0  # 1 second
        fresh_count = sum(1 for a in current_aois if a <= fresh_threshold)

        return {
            "average_aoi": average_aoi,
            "fresh_sensors": fresh_count,
            "total_sensors": len(self.sensor_ids),
            "freshness_ratio": fresh_count / len(self.sensor_ids),
        }


class NetworkAwareAOIAssessor:
    """
    Network-aware AOI assessment with transport-specific characteristics.
    """

    def __init__(self):
        # Transport characteristics
        self.transport_profiles = {
            "SERIAL": {
                "base_latency": 0.030,  # 30ms serial base
                "jitter_range": (0.005, 0.020),  # 5-20ms jitter
                "congestion_threshold": 0.100,  # 100ms indicates congestion
                "bandwidth_mbps": 1.0,  # 1 Mbps typical
                "reliability": 0.95,  # 95% reliable
            },
            "CAN": {
                "base_latency": 0.010,  # 10ms CAN base
                "jitter_range": (0.001, 0.005),  # 1-5ms jitter
                "congestion_threshold": 0.050,  # 50ms indicates congestion
                "bandwidth_mbps": 1.0,  # 1 Mbps typical
                "reliability": 0.98,  # 98% reliable
            },
            "ETHERNET": {
                "base_latency": 0.005,  # 5ms ethernet base
                "jitter_range": (0.001, 0.003),  # 1-3ms jitter
                "congestion_threshold": 0.020,  # 20ms indicates congestion
                "bandwidth_mbps": 100.0,  # 100 Mbps typical
                "reliability": 0.999,  # 99.9% reliable
            },
            "LOCAL": {
                "base_latency": 0.001,  # 1ms local base
                "jitter_range": (0.0001, 0.001),  # 0.1-1ms jitter
                "congestion_threshold": 0.010,  # 10ms indicates congestion
                "bandwidth_mbps": 1000.0,  # Local bus speeds
                "reliability": 0.9999,  # Near 100% reliable
            },
        }

        # Quality profiles per sensor type
        self.quality_profiles = {
            "imu": {
                "optimal": 0.05,  # 50ms
                "acceptable": 0.1,  # 100ms
                "degraded": 0.2,  # 200ms
                "unusable": 0.5,  # 500ms
            },
            "camera": {
                "optimal": 0.1,  # 100ms
                "acceptable": 0.2,  # 200ms
                "degraded": 0.5,  # 500ms
                "unusable": 1.0,  # 1s
            },
            "lidar": {
                "optimal": 0.1,  # 100ms
                "acceptable": 0.15,  # 150ms
                "degraded": 0.3,  # 300ms
                "unusable": 0.6,  # 600ms
            },
            "gps": {
                "optimal": 0.5,  # 500ms
                "acceptable": 1.0,  # 1s
                "degraded": 2.0,  # 2s
                "unusable": 5.0,  # 5s
            },
            "slam_pose": {
                "optimal": 0.2,  # 200ms
                "acceptable": 0.5,  # 500ms
                "degraded": 1.0,  # 1s
                "unusable": 2.0,  # 2s
            },
        }

        # Network health tracking
        self.network_history = {}
        self.congestion_events = []

    def get_transport_type(self, sensor_name: str) -> str:
        """Determine transport type based on sensor name."""
        sensor_lower = sensor_name.lower()

        if "camera" in sensor_lower or "depth" in sensor_lower:
            return "SERIAL"
        elif "imu" in sensor_lower or "motor" in sensor_lower or "can" in sensor_lower:
            return "CAN"
        elif "gps" in sensor_lower or "network" in sensor_lower:
            return "ETHERNET"
        else:
            return "LOCAL"

    def assess_network_health(self, sensor_name: str, measured_aoi: float) -> dict:
        """Assess network health for a sensor."""
        transport = self.get_transport_type(sensor_name)
        profile = self.transport_profiles[transport]

        # Calculate expected vs measured latency
        expected_latency = profile["base_latency"] + sum(profile["jitter_range"]) / 2
        network_latency = max(0, measured_aoi - 0.001)  # Subtract local processing

        # Detect congestion
        congestion_detected = network_latency > profile["congestion_threshold"]
        congestion_factor = network_latency / expected_latency if expected_latency > 0 else 1.0

        # Track congestion events
        if congestion_detected:
            self.congestion_events.append(
                {
                    "sensor": sensor_name,
                    "transport": transport,
                    "timestamp": time.time(),
                    "latency": network_latency,
                    "factor": congestion_factor,
                }
            )

            # Keep only recent events (last 100)
            self.congestion_events = self.congestion_events[-100:]

        return {
            "transport_type": transport,
            "network_latency": network_latency,
            "transport_latency": profile["base_latency"],
            "congestion_detected": congestion_detected,
            "congestion_factor": min(congestion_factor, 5.0),  # Cap at 5x
            "expected_latency": expected_latency,
            "bandwidth_efficiency": profile["bandwidth_mbps"],
            "transport_reliability": profile["reliability"],
        }

    def assess_quality(self, sensor_type: str, aoi: float) -> Tuple[float, str]:
        """
        Assess measurement quality based on AoI.

        Returns:
            Tuple of (quality_score, quality_category)
        """
        profile = self.quality_profiles.get(sensor_type, self.quality_profiles["imu"])

        if aoi <= profile["optimal"]:
            return 1.0, "OPTIMAL"
        elif aoi <= profile["acceptable"]:
            return 0.8, "ACCEPTABLE"
        elif aoi <= profile["degraded"]:
            return 0.5, "DEGRADED"
        elif aoi <= profile["unusable"]:
            return 0.2, "POOR"
        else:
            return 0.0, "UNUSABLE"
