#!/usr/bin/env python3
"""
URC Challenge Simulation Test

Comprehensive test covering all major URC 2026 challenges:
- Waypoint Navigation
- Follow Me
- Equipment Service
- Science Operations
- Terrain Traversal
- Long Distance Navigation
- Emergency Scenarios
"""

import json
import math
import time
from dataclasses import asdict, dataclass
from enum import Enum
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray


class URCChallenge(Enum):
    """URC 2026 Challenge Types."""
    WAYPOINT_NAVIGATION = "waypoint_navigation"
    FOLLOW_ME = "follow_me"
    EQUIPMENT_SERVICE = "equipment_service"
    SCIENCE_OPERATIONS = "science_operations"
    TERRAIN_TRAVERSAL = "terrain_traversal"
    LONG_DISTANCE = "long_distance"
    EMERGENCY_RESPONSE = "emergency_response"


class ChallengePhase(Enum):
    """Challenge execution phases."""
    SETUP = "setup"
    EXECUTION = "execution"
    COMPLETION = "completion"
    FAILURE = "failure"


@dataclass
class ChallengeMetrics:
    """Metrics for challenge performance."""
    challenge_type: URCChallenge
    start_time: float
    end_time: Optional[float] = None
    success: bool = False
    distance_traveled: float = 0.0
    waypoints_completed: int = 0
    obstacles_avoided: int = 0
    precision_score: float = 0.0  # 0-1 accuracy score
    time_score: float = 0.0  # Time efficiency score
    safety_violations: int = 0
    emergency_actions: int = 0


@dataclass
class URCChallengeResult:
    """Complete URC challenge test result."""
    challenge_name: str
    duration_minutes: float
    overall_success: bool
    challenges_completed: int
    total_score: float
    safety_rating: str
    performance_rating: str
    recommendations: List[str]


class URCChallengeSimulator(Node):
    """Comprehensive URC 2026 challenge simulator."""

    def __init__(self):
        super().__init__("urc_challenge_simulator")

        self.logger = self.get_logger()

        # Test configuration
        self.test_duration = 2400.0  # 40 minutes total
        self.challenge_duration = 300.0  # 5 minutes per challenge

        # Challenge tracking
        self.current_challenge = None
        self.challenge_metrics = []
        self.challenge_sequence = self.define_challenge_sequence()
        self.challenge_index = 0

        # Navigation state
        self.current_position = (0.0, 0.0, 0.0)
        self.target_waypoints = []
        self.waypoint_index = 0
        self.follow_target = None

        # Performance tracking
        self.total_distance = 0.0
        self.start_position = None
        self.last_position = None
        self.obstacles_detected = 0
        self.safety_violations = 0

        # QoS profiles
        self.qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, self.qos_reliable
        )

        self.laser_sub = self.create_subscription(
            LaserScan, "/scan", self.laser_callback, self.qos_reliable
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.challenge_status_pub = self.create_publisher(String, "/challenge_status", 10)

        # Visualization
        self.marker_pub = self.create_publisher(MarkerArray, "/challenge_markers", 10)

        # Test timers
        self.challenge_timer = self.create_timer(self.challenge_duration, self.next_challenge)
        self.navigation_timer = self.create_timer(0.5, self.update_navigation)
        self.monitor_timer = self.create_timer(2.0, self.monitor_performance)

        self.start_time = time.time()
        self.logger.info("URC Challenge Simulator initialized")
        self.logger.info(f"Test duration: {self.test_duration/60:.1f} minutes")
        self.logger.info(f"Challenges to test: {len(self.challenge_sequence)}")

        # Start first challenge
        self.start_challenge()

    def define_challenge_sequence(self) -> List[URCChallenge]:
        """Define the sequence of URC challenges to test."""
        return [
            URCChallenge.WAYPOINT_NAVIGATION,
            URCChallenge.TERRAIN_TRAVERSAL,
            URCChallenge.FOLLOW_ME,
            URCChallenge.EQUIPMENT_SERVICE,
            URCChallenge.SCIENCE_OPERATIONS,
            URCChallenge.LONG_DISTANCE,
            URCChallenge.EMERGENCY_RESPONSE
        ]

    def start_challenge(self):
        """Start the next challenge in sequence."""
        if self.challenge_index >= len(self.challenge_sequence):
            self.complete_test()
            return

        self.current_challenge = self.challenge_sequence[self.challenge_index]
        challenge_start = time.time()

        # Initialize challenge metrics
        metrics = ChallengeMetrics(
            challenge_type=self.current_challenge,
            start_time=challenge_start
        )

        self.challenge_metrics.append(metrics)

        # Setup challenge-specific parameters
        self.setup_challenge(self.current_challenge)

        self.logger.info(f" STARTING CHALLENGE: {self.current_challenge.value}")
        self.publish_challenge_status("STARTED", f"Challenge {self.challenge_index + 1}/7")

    def setup_challenge(self, challenge: URCChallenge):
        """Setup specific challenge parameters."""
        if challenge == URCChallenge.WAYPOINT_NAVIGATION:
            # 5-waypoint course with precision requirements
            self.target_waypoints = [
                (50.0, 25.0, 0.0),
                (100.0, -50.0, 0.0),
                (-50.0, 75.0, 0.0),
                (25.0, -25.0, 0.0),
                (0.0, 0.0, 0.0)  # Return to start
            ]
            self.publish_waypoint_markers()

        elif challenge == URCChallenge.TERRAIN_TRAVERSAL:
            # Rough terrain with obstacles
            self.target_waypoints = [
                (30.0, 30.0, 0.0),
                (60.0, 10.0, 0.0),
                (90.0, 40.0, 0.0)
            ]

        elif challenge == URCChallenge.FOLLOW_ME:
            # Follow moving target
            self.follow_target = (20.0, 0.0, 0.0)
            self.target_waypoints = []  # Dynamic following

        elif challenge == URCChallenge.EQUIPMENT_SERVICE:
            # Precision manipulation task
            self.target_waypoints = [
                (15.0, 15.0, 0.0),  # Equipment location
                (15.0, 17.0, 0.0),  # Service position
            ]

        elif challenge == URCChallenge.SCIENCE_OPERATIONS:
            # Sample collection simulation
            self.target_waypoints = [
                (40.0, -20.0, 0.0),  # Sample site 1
                (70.0, -10.0, 0.0),  # Sample site 2
                (60.0, 20.0, 0.0),   # Analysis station
            ]

        elif challenge == URCChallenge.LONG_DISTANCE:
            # Extended navigation (simulated)
            self.target_waypoints = [
                (200.0, 100.0, 0.0),
                (400.0, -200.0, 0.0),
                (600.0, 300.0, 0.0),
                (800.0, 0.0, 0.0),
            ]

        elif challenge == URCChallenge.EMERGENCY_RESPONSE:
            # Emergency scenario with failures
            self.target_waypoints = [
                (25.0, -25.0, 0.0),  # Emergency location
            ]

    def odom_callback(self, msg):
        """Handle odometry updates."""
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )

        # Track total distance
        if self.last_position:
            distance = math.sqrt(
                (self.current_position[0] - self.last_position[0])**2 +
                (self.current_position[1] - self.last_position[1])**2
            )
            self.total_distance += distance

            # Update current challenge metrics
            if self.challenge_metrics:
                self.challenge_metrics[-1].distance_traveled += distance

        self.last_position = self.current_position

        # Check waypoint completion
        self.check_waypoint_completion()

    def laser_callback(self, msg):
        """Handle LiDAR obstacle detection."""
        # Simple obstacle detection
        ranges = np.array(msg.ranges)
        min_range = np.min(ranges[np.isfinite(ranges)])

        if min_range < 2.0:  # Obstacle within 2m
            self.obstacles_detected += 1
            if self.challenge_metrics:
                self.challenge_metrics[-1].obstacles_avoided += 1

    def check_waypoint_completion(self):
        """Check if current waypoint is reached."""
        if not self.target_waypoints or self.waypoint_index >= len(self.target_waypoints):
            return

        current_wp = self.target_waypoints[self.waypoint_index]
        distance_to_wp = math.sqrt(
            (self.current_position[0] - current_wp[0])**2 +
            (self.current_position[1] - current_wp[1])**2
        )

        if distance_to_wp < 2.0:  # Within 2m of waypoint
            self.waypoint_index += 1
            if self.challenge_metrics:
                self.challenge_metrics[-1].waypoints_completed += 1

            self.logger.info(f" Waypoint {self.waypoint_index} reached")

            if self.waypoint_index >= len(self.target_waypoints):
                self.complete_current_challenge(success=True)
            else:
                self.publish_next_waypoint()

    def publish_next_waypoint(self):
        """Publish the next waypoint goal."""
        if self.waypoint_index < len(self.target_waypoints):
            wp = self.target_waypoints[self.waypoint_index]

            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = wp[0]
            goal.pose.position.y = wp[1]
            goal.pose.position.z = wp[2]
            goal.pose.orientation.w = 1.0

            self.goal_pub.publish(goal)
            self.logger.info(f" Published waypoint {self.waypoint_index + 1}: ({wp[0]}, {wp[1]})")

    def publish_waypoint_markers(self):
        """Publish visualization markers for waypoints."""
        marker_array = MarkerArray()

        for i, wp in enumerate(self.target_waypoints):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = wp[0]
            marker.pose.position.y = wp[1]
            marker.pose.position.z = wp[2]
            marker.pose.orientation.w = 1.0

            # Color: Green for start/complete, Blue for intermediate
            if i == 0:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif i == len(self.target_waypoints) - 1:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            marker.color.a = 0.8

            marker.scale.x = 2.0
            marker.scale.y = 2.0
            marker.scale.z = 0.1

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

    def update_navigation(self):
        """Update navigation commands based on current challenge."""
        if not self.current_challenge:
            return

        if self.current_challenge == URCChallenge.FOLLOW_ME:
            # Follow moving target logic
            if self.follow_target:
                # Move toward follow target
                dx = self.follow_target[0] - self.current_position[0]
                dy = self.follow_target[1] - self.current_position[1]
                distance = math.sqrt(dx*dx + dy*dy)

                if distance > 2.0:  # Maintain 2m following distance
                    cmd = Twist()
                    cmd.linear.x = min(0.5, distance * 0.1)  # Proportional speed
                    cmd.angular.z = math.atan2(dy, dx) * 0.5
                    self.cmd_vel_pub.publish(cmd)
        else:
            # Standard waypoint navigation
            if self.target_waypoints and self.waypoint_index < len(self.target_waypoints):
                # Simple proportional control toward next waypoint
                wp = self.target_waypoints[self.waypoint_index]
                dx = wp[0] - self.current_position[0]
                dy = wp[1] - self.current_position[1]
                distance = math.sqrt(dx*dx + dy*dy)

                if distance > 0.5:  # Not yet at waypoint
                    cmd = Twist()
                    cmd.linear.x = min(1.0, distance * 0.2)  # Speed proportional to distance
                    cmd.angular.z = math.atan2(dy, dx) * 0.8  # Turn toward waypoint
                    self.cmd_vel_pub.publish(cmd)

    def monitor_performance(self):
        """Monitor overall test performance."""
        elapsed = time.time() - self.start_time

        # Publish status update
        status_msg = String()
        status_msg.data = json.dumps({
            'elapsed_minutes': elapsed / 60,
            'current_challenge': self.current_challenge.value if self.current_challenge else None,
            'challenge_index': self.challenge_index,
            'total_distance': self.total_distance,
            'obstacles_avoided': self.obstacles_detected,
            'waypoints_completed': sum(m.waypoints_completed for m in self.challenge_metrics)
        })
        self.challenge_status_pub.publish(status_msg)

    def next_challenge(self):
        """Advance to the next challenge."""
        if self.current_challenge:
            self.complete_current_challenge(success=False)  # Timeout = failure

    def complete_current_challenge(self, success: bool = False):
        """Complete the current challenge."""
        if self.challenge_metrics:
            self.challenge_metrics[-1].end_time = time.time()
            self.challenge_metrics[-1].success = success

            duration = self.challenge_metrics[-1].end_time - self.challenge_metrics[-1].start_time
            self.logger.info(f" CHALLENGE COMPLETE: {self.current_challenge.value} - "
                           f"{'SUCCESS' if success else 'TIMEOUT'} ({duration:.1f}s)")

        self.challenge_index += 1

        # Small delay between challenges
        time.sleep(2)

        if self.challenge_index < len(self.challenge_sequence):
            self.start_challenge()
        else:
            self.complete_test()

    def complete_test(self):
        """Complete the entire URC challenge test."""
        end_time = time.time()
        total_duration = (end_time - self.start_time) / 60  # minutes

        # Calculate final results
        successful_challenges = sum(1 for m in self.challenge_metrics if m.success)
        total_score = successful_challenges / len(self.challenge_sequence)

        # Safety rating based on violations
        total_violations = sum(m.safety_violations for m in self.challenge_metrics)
        if total_violations == 0:
            safety_rating = "EXCELLENT"
        elif total_violations < 3:
            safety_rating = "GOOD"
        else:
            safety_rating = "NEEDS_IMPROVEMENT"

        # Performance rating
        if total_score >= 0.9:
            performance_rating = "EXCELLENT"
        elif total_score >= 0.7:
            performance_rating = "GOOD"
        else:
            performance_rating = "NEEDS_IMPROVEMENT"

        result = URCChallengeResult(
            challenge_name="URC 2026 Complete Challenge Suite",
            duration_minutes=total_duration,
            overall_success=successful_challenges == len(self.challenge_sequence),
            challenges_completed=successful_challenges,
            total_score=total_score,
            safety_rating=safety_rating,
            performance_rating=performance_rating,
            recommendations=self.generate_recommendations()
        )

        # Save comprehensive results
        test_data = {
            'summary': asdict(result),
            'challenge_details': [asdict(m) for m in self.challenge_metrics],
            'overall_metrics': {
                'total_distance_traveled': self.total_distance,
                'total_obstacles_avoided': self.obstacles_detected,
                'total_safety_violations': total_violations,
                'average_challenge_duration': total_duration / len(self.challenge_sequence)
            }
        }

        with open("/tmp/urc_challenge_test_results.json", "w") as f:
            json.dump(test_data, f, indent=2)

        # Publish final status
        final_msg = String()
        final_msg.data = f"URC_CHALLENGE_TEST_COMPLETE|{successful_challenges}/{len(self.challenge_sequence)}|Score:{total_score:.2f}"
        self.challenge_status_pub.publish(final_msg)

        self.print_final_results(result, test_data)

    def generate_recommendations(self) -> List[str]:
        """Generate recommendations based on test results."""
        recommendations = []

        successful_challenges = sum(1 for m in self.challenge_metrics if m.success)
        success_rate = successful_challenges / len(self.challenge_metrics)

        if success_rate < 0.8:
            recommendations.append("Improve navigation accuracy and waypoint reaching")

        total_violations = sum(m.safety_violations for m in self.challenge_metrics)
        if total_violations > 0:
            recommendations.append("Enhance safety systems and violation prevention")

        avg_precision = np.mean([m.precision_score for m in self.challenge_metrics])
        if avg_precision < 0.8:
            recommendations.append("Improve position estimation and control precision")

        if not recommendations:
            recommendations.append("All systems performing well - ready for competition")

        return recommendations

    def print_final_results(self, result: URCChallengeResult, test_data: Dict):
        """Print comprehensive final results."""
        print("\n" + "="*80)
        print("URC 2026 COMPLETE CHALLENGE TEST RESULTS")
        print("="*80)
        print(f"Test Duration: {result.duration_minutes:.1f} minutes")
        print(f"Challenges Completed: {result.challenges_completed}/7")
        print(f"Safety Rating: {result.safety_rating}")
        print(f"Performance Rating: {result.performance_rating}")
        print(f"Overall Success: {' YES' if result.overall_success else ' NO'}")

        print("\nCHALLENGE DETAILS:")
        challenge_names = {
            URCChallenge.WAYPOINT_NAVIGATION: "Waypoint Navigation",
            URCChallenge.TERRAIN_TRAVERSAL: "Terrain Traversal",
            URCChallenge.FOLLOW_ME: "Follow Me",
            URCChallenge.EQUIPMENT_SERVICE: "Equipment Service",
            URCChallenge.SCIENCE_OPERATIONS: "Science Operations",
            URCChallenge.LONG_DISTANCE: "Long Distance",
            URCChallenge.EMERGENCY_RESPONSE: "Emergency Response"
        }

        for i, metrics in enumerate(self.challenge_metrics):
            challenge_name = challenge_names.get(metrics.challenge_type, str(metrics.challenge_type))
            status = "" if metrics.success else ""
            duration = metrics.end_time - metrics.start_time if metrics.end_time else 0
            print(f"  {status} {challenge_name}: {duration:.1f}s, "
                  f"{metrics.waypoints_completed} waypoints, "
                  f"{metrics.obstacles_avoided} obstacles avoided")

        print("\nOVERALL METRICS:")
        print(f"Obstacles Avoided: {self.obstacles_detected}")
        print(f"Safety Violations: {sum(m.safety_violations for m in self.challenge_metrics)}")

        print("\nRECOMMENDATIONS:")
        for rec in result.recommendations:
            print(f"  • {rec}")

        print("="*80)
        print("\nURC 2026 AUTONOMY SYSTEM TEST COMPLETE!")
        print("Results saved to /tmp/urc_challenge_test_results.json")


def main():
    """Main test function."""
    rclpy.init()
    simulator = URCChallengeSimulator()

    try:
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        simulator.complete_test()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
