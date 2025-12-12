#!/usr/bin/env python3
"""
Arm Controller Module

Handles robotic arm control, motion planning with MoveIt, and trajectory execution
for autonomous keyboard typing.
"""

import random
import time
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Tuple

import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ArmState(Enum):
    """Arm operational states."""

    IDLE = "idle"
    MOVING = "moving"
    PRESSING = "pressing"
    ERROR = "error"


class SimpleInverseKinematics:
    """
    Simplified inverse kinematics solver for 6-DOF robotic arm.

    This is a placeholder implementation using forward kinematics + numerical IK.
    In production, would use analytical IK or MoveIt's built-in solvers.
    """

    def __init__(self):
        """Initialize IK solver with arm parameters."""
        # Standard 6-DOF arm DH parameters (approximate)
        self.link_lengths = [0.0, 0.3, 0.3, 0.0, 0.0, 0.0]  # meters
        self.joint_limits = [
            (-np.pi, np.pi),  # Joint 1 (shoulder pan)
            (-np.pi / 2, np.pi / 2),  # Joint 2 (shoulder lift)
            (-np.pi, np.pi),  # Joint 3 (elbow)
            (-np.pi, np.pi),  # Joint 4 (wrist 1)
            (-np.pi, np.pi),  # Joint 5 (wrist 2)
            (-np.pi, np.pi),  # Joint 6 (wrist 3)
        ]

    def forward_kinematics(self, joint_angles: np.ndarray) -> np.ndarray:
        """
        Calculate end-effector position from joint angles.

        Args:
            joint_angles: Array of 6 joint angles in radians

        Returns:
            3D end-effector position [x, y, z]
        """
        # Simplified FK: assumes arm configuration with known link lengths
        # In practice, would use DH parameters and matrix multiplication

        # Extract angles
        q1, q2, q3, q4, q5, q6 = joint_angles

        # Simple reach calculation (placeholder)
        # Real implementation would use proper forward kinematics
        reach = self.link_lengths[1] * np.cos(q1 + q2) + self.link_lengths[2] * np.cos(q1 + q2 + q3)

        x = reach * np.cos(q1)
        y = reach * np.sin(q1)
        z = self.link_lengths[1] * np.sin(q1 + q2) + self.link_lengths[2] * np.sin(q1 + q2 + q3)

        return np.array([x, y, z])

    def inverse_kinematics(
        self,
        target_pos: np.ndarray,
        target_orientation: Optional[np.ndarray] = None,
        initial_guess: Optional[np.ndarray] = None,
    ) -> Optional[np.ndarray]:
        """
        Solve inverse kinematics numerically.

        Args:
            target_pos: Target 3D position [x, y, z]
            target_orientation: Target orientation (optional)
            initial_guess: Initial joint angle guess for optimization

        Returns:
            Joint angles that achieve target, or None if unsolvable
        """
        # Use simple numerical approach: search around initial guess
        if initial_guess is None:
            initial_guess = np.zeros(6)

        best_angles = initial_guess.copy()
        best_error = float("inf")

        # Grid search around initial guess
        search_step = 0.2  # radians
        for i in range(10):  # 10 iterations
            for j in range(6):  # 6 joints
                for delta in [-search_step, 0, search_step]:
                    test_angles = best_angles.copy()
                    test_angles[j] = np.clip(
                        best_angles[j] + delta,
                        self.joint_limits[j][0],
                        self.joint_limits[j][1],
                    )

                    # Calculate FK and error
                    fk_pos = self.forward_kinematics(test_angles)
                    error = np.linalg.norm(fk_pos - target_pos)

                    if error < best_error:
                        best_error = error
                        best_angles = test_angles.copy()

        # Check if solution is within acceptable tolerance (1cm)
        if best_error < 0.01:
            return best_angles
        else:
            return None


class ArmController:
    """
    High-level arm controller for autonomous typing.

    Manages motion planning, trajectory execution, and force control
    for pressing keyboard keys.
    """

    def __init__(self, node: Node):
        """
        Initialize arm controller.

        Args:
            node: ROS 2 node for pub/sub
        """
        self.node = node
        self.logger = node.get_logger()

        # Arm configuration
        self.arm_joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        self.arm_reach = 0.8  # meters
        self.arm_base_height = 0.0  # relative to ground

        # State tracking
        self.current_joint_positions: Optional[np.ndarray] = None
        self.current_state = ArmState.IDLE

        # IK solver
        self.ik_solver = SimpleInverseKinematics()

        # Publishers and subscribers
        self.joint_command_pub = node.create_publisher(JointTrajectory, "arm/joint_command", 10)

        self.joint_state_sub = node.create_subscription(JointState, "arm/joint_states", self._joint_state_callback, 10)

        # Trajectory execution parameters
        self.trajectory_speed = 0.5  # 0-1 scale for speed
        self.trajectory_accel = 0.3  # 0-1 scale for acceleration

        self.logger.info("Arm controller initialized")

    def _joint_state_callback(self, msg: JointState) -> None:
        """Update current joint positions from feedback."""
        # Match joints by name
        if len(msg.name) == len(self.arm_joint_names):
            self.current_joint_positions = np.array(msg.position)

    def move_to_position(self, target_xyz: np.ndarray, duration: float = 3.0) -> bool:
        """
        Move arm end-effector to target XYZ position.

        Args:
            target_xyz: Target position [x, y, z]
            duration: Time allowed for movement in seconds

        Returns:
            True if movement succeeded, False otherwise
        """
        try:
            self.current_state = ArmState.MOVING

            # Solve inverse kinematics
            self.logger.info(f"Solving IK for target: {target_xyz}")
            target_angles = self.ik_solver.inverse_kinematics(
                target_xyz,
                initial_guess=(self.current_joint_positions if self.current_joint_positions is not None else None),
            )

            if target_angles is None:
                self.logger.error("IK solver failed")
                self.current_state = ArmState.ERROR
                return False

            # Plan trajectory
            trajectory = self._plan_trajectory(target_angles, duration)

            # Execute trajectory
            self.joint_command_pub.publish(trajectory)

            self.logger.info(f"Trajectory published, waiting {duration}s for execution")
            time.sleep(duration)

            self.current_state = ArmState.IDLE
            return True

        except Exception as e:
            self.logger.error(f"Move to position failed: {e}")
            self.current_state = ArmState.ERROR
            return False

    def press_key(
        self,
        key_world_pos: np.ndarray,
        approach_height: float = 0.02,
        press_depth: float = 0.015,
        dwell_time: float = 0.1,
    ) -> bool:
        """
        Execute key press motion (approach -> press -> retract).

        Args:
            key_world_pos: XY position of key center in world frame [x, y]
            approach_height: Height above keyboard for approach
            press_depth: Depth to press key
            dwell_time: Time to hold key pressed

        Returns:
            True if press succeeded, False otherwise
        """
        try:
            self.current_state = ArmState.PRESSING

            self.logger.info(f"Starting key press at {key_world_pos}")

            # Approach phase (move above key)
            approach_pos = np.array([key_world_pos[0], key_world_pos[1], key_world_pos[2] + approach_height])

            self.logger.info(f"Phase 1: Approaching {approach_pos}")
            if not self.move_to_position(approach_pos, duration=1.0):
                return False

            # Press phase (move down to key)
            press_pos = np.array([key_world_pos[0], key_world_pos[1], key_world_pos[2] - press_depth])

            self.logger.info(f"Phase 2: Pressing {press_pos}")
            if not self.move_to_position(press_pos, duration=0.5):
                return False

            # Dwell at bottom of press
            self.logger.info(f"Phase 3: Dwelling for {dwell_time}s")
            time.sleep(dwell_time)

            # Retract phase (move back up)
            self.logger.info(f"Phase 4: Retracting to {approach_pos}")
            if not self.move_to_position(approach_pos, duration=0.5):
                return False

            self.logger.info("Key press completed successfully")
            self.current_state = ArmState.IDLE
            return True

        except Exception as e:
            self.logger.error(f"Key press failed: {e}")
            self.current_state = ArmState.ERROR
            return False

    def _plan_trajectory(self, target_angles: np.ndarray, duration: float, num_waypoints: int = 10) -> JointTrajectory:
        """
        Plan a trajectory from current position to target.

        Args:
            target_angles: Target joint angles
            duration: Total movement time in seconds
            num_waypoints: Number of trajectory waypoints

        Returns:
            JointTrajectory message
        """
        if self.current_joint_positions is None:
            self.logger.warn("Current joint positions unknown, using zeros")
            current = np.zeros(6)
        else:
            current = self.current_joint_positions.copy()

        # Create trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = self.arm_joint_names

        # Generate waypoints (linear interpolation in joint space)
        for i in range(num_waypoints):
            t = i / (num_waypoints - 1)
            interpolated = current + t * (target_angles - current)

            point = JointTrajectoryPoint()
            point.positions = interpolated.tolist()
            point.velocities = [0.0] * 6
            point.accelerations = [0.0] * 6
            point.time_from_start.sec = int(duration * t)
            point.time_from_start.nanosec = int((duration * t % 1) * 1e9)

            trajectory.points.append(point)

        return trajectory

    def get_state(self) -> ArmState:
        """Get current arm state."""
        return self.current_state

    def is_ready(self) -> bool:
        """Check if arm is ready for commands."""
        return self.current_joint_positions is not None and self.current_state == ArmState.IDLE


# Standalone testing
if __name__ == "__main__":
    # Test IK solver
    print("Testing Inverse Kinematics Solver:")
    ik = SimpleInverseKinematics()

    # Test target position
    target = np.array([0.5, 0.0, 0.3])
    print(f"  Target position: {target}")

    # Solve IK
    angles = ik.inverse_kinematics(target)
    if angles is not None:
        print(f"  Solved angles: {angles}")

        # Verify with FK
        fk_result = ik.forward_kinematics(angles)
        print(f"  FK verification: {fk_result}")
        print(f"  Error: {np.linalg.norm(fk_result - target):.4f}m")
    else:
        print("  IK solver failed")


# =============================================================================
# RRT* Implementation for Optimal Arm Trajectory Planning
# =============================================================================


@dataclass
class RRTNode:
    """Node in RRT* tree"""

    configuration: np.ndarray  # Joint angles [6]
    cost: float = 0.0
    parent: Optional["RRTNode"] = None

    def distance_to(self, other: "RRTNode") -> float:
        """Joint space distance to another node"""
        return np.linalg.norm(self.configuration - other.configuration)


class RRTStarPlanner:
    """
    RRT* algorithm for optimal joint trajectory planning.

    Features:
    - Asymptotically optimal paths in joint space
    - Rewiring for continuous optimization
    - Collision checking in configuration space
    - Joint limit enforcement
    """

    def __init__(self, joint_limits: Dict[str, Tuple[float, float]]):
        # Joint limits from arm configuration
        self.joint_limits = joint_limits
        self.num_joints = len(joint_limits)

        # RRT* parameters
        self.max_iterations = 1000
        self.goal_bias = 0.15  # Probability of sampling goal
        self.rewire_radius = 0.8  # Joint space rewiring radius (radians)
        self.step_size = 0.1  # Max step size in joint space (radians)
        self.goal_tolerance = 0.01  # Goal tolerance (radians)

        # Collision checking
        self.collision_checker = None  # Will be set externally

        # Tree storage
        self.nodes: List[RRTNode] = []
        self.goal_nodes: List[RRTNode] = []

    def plan_trajectory(
        self, start_config: np.ndarray, goal_configs: List[np.ndarray], max_time: float = 5.0
    ) -> Optional[List[np.ndarray]]:
        """
        Plan optimal trajectory from start to goal configurations

        Args:
            start_config: Starting joint configuration [6]
            goal_configs: List of valid goal configurations (from IK)
            max_time: Maximum planning time in seconds

        Returns:
            Optimal joint trajectory or None if no path found
        """
        if not self._is_valid_configuration(start_config):
            self.logger.error("Invalid start configuration")
            return None

        # Initialize tree with start node
        self.nodes = [RRTNode(start_config.copy())]

        # Add goal nodes for multi-goal planning
        for goal_config in goal_configs:
            if self._is_valid_configuration(goal_config):
                goal_node = RRTNode(goal_config.copy(), cost=float("inf"))
                self.goal_nodes.append(goal_node)

        if not self.goal_nodes:
            self.logger.error("No valid goal configurations")
            return None

        start_time = time.time()

        for iteration in range(self.max_iterations):
            # Check time limit
            if time.time() - start_time > max_time:
                break

            # Sample random configuration
            q_rand = self._sample_configuration()

            # Find nearest node in tree
            q_nearest = self._nearest_neighbor(q_rand)

            # Steer toward random configuration
            q_new = self._steer(q_nearest, q_rand)

            # Check if new configuration is valid
            if not self._is_valid_configuration(q_new.configuration):
                continue

            # Find nearby nodes for rewiring
            q_near = self._near_nodes(q_new, self.rewire_radius)

            # Choose best parent (minimizes cost)
            q_min = q_nearest
            c_min = q_nearest.cost + q_nearest.distance_to(q_new)

            for q_near_node in q_near:
                if self._collision_free(q_near_node, q_new):
                    cost_via_near = q_near_node.cost + q_near_node.distance_to(q_new)
                    if cost_via_near < c_min:
                        q_min = q_near_node
                        c_min = cost_via_near

            # Add node to tree
            q_new.cost = c_min
            q_new.parent = q_min
            self.nodes.append(q_new)

            # Rewire nearby nodes if q_new provides better path
            for q_near_node in q_near:
                if q_near_node != q_min and self._collision_free(q_new, q_near_node):
                    cost_through_new = q_new.cost + q_new.distance_to(q_near_node)
                    if cost_through_new < q_near_node.cost:
                        q_near_node.parent = q_new
                        q_near_node.cost = cost_through_new
                        self._propagate_cost_improvements(q_near_node)

            # Check if we can reach any goal
            for goal_node in self.goal_nodes:
                if self._collision_free(q_new, goal_node) and q_new.distance_to(goal_node) < self.goal_tolerance:
                    # Connect to goal
                    goal_node.parent = q_new
                    goal_node.cost = q_new.cost + q_new.distance_to(goal_node)
                    return self._extract_trajectory(goal_node)

        # Return best trajectory to nearest goal if no direct connection
        return self._find_best_trajectory_to_goals()

    def _sample_configuration(self) -> RRTNode:
        """Sample random configuration with goal bias"""
        if random.random() < self.goal_bias and self.goal_nodes:
            # Sample near a random goal
            goal = random.choice(self.goal_nodes)
            # Add small random perturbation
            perturbation = np.random.normal(0, 0.1, self.num_joints)
            config = np.clip(goal.configuration + perturbation, self._get_min_limits(), self._get_max_limits())
        else:
            # Sample uniformly in joint space
            config = np.random.uniform(self._get_min_limits(), self._get_max_limits(), self.num_joints)

        return RRTNode(config)

    def _nearest_neighbor(self, q: RRTNode) -> RRTNode:
        """Find nearest node in tree"""
        return min(self.nodes, key=lambda node: node.distance_to(q))

    def _steer(self, q_nearest: RRTNode, q_rand: RRTNode) -> RRTNode:
        """Steer from nearest node toward random configuration"""
        direction = q_rand.configuration - q_nearest.configuration
        distance = np.linalg.norm(direction)

        if distance <= self.step_size:
            return RRTNode(q_rand.configuration, parent=q_nearest)
        else:
            # Move step_size in direction
            direction_normalized = direction / distance
            new_config = q_nearest.configuration + direction_normalized * self.step_size
            return RRTNode(new_config, parent=q_nearest)

    def _near_nodes(self, q: RRTNode, radius: float) -> List[RRTNode]:
        """Find nodes within rewiring radius"""
        return [node for node in self.nodes if node.distance_to(q) <= radius]

    def _collision_free(self, q1: RRTNode, q2: RRTNode) -> bool:
        """Check if path between configurations is collision-free"""
        # Linear interpolation in joint space
        distance = q1.distance_to(q2)
        if distance == 0:
            return True

        num_checks = max(5, int(distance / 0.05))  # Check every 0.05 radians

        for i in range(num_checks + 1):
            t = i / num_checks
            config = q1.configuration + t * (q2.configuration - q1.configuration)

            if not self._is_valid_configuration(config):
                return False

            # Additional collision checking if available
            if self.collision_checker and not self.collision_checker(config):
                return False

        return True

    def _is_valid_configuration(self, config: np.ndarray) -> bool:
        """Check if joint configuration is within limits"""
        min_limits = self._get_min_limits()
        max_limits = self._get_max_limits()

        return np.all(config >= min_limits) and np.all(config <= max_limits)

    def _get_min_limits(self) -> np.ndarray:
        """Get minimum joint limits as array"""
        return np.array([self.joint_limits[name][0] for name in self.joint_limits])

    def _get_max_limits(self) -> np.ndarray:
        """Get maximum joint limits as array"""
        return np.array([self.joint_limits[name][1] for name in self.joint_limits])

    def _propagate_cost_improvements(self, node: RRTNode):
        """Propagate cost improvements through tree"""
        # Recursively update children costs
        for child in self._get_children(node):
            old_cost = child.cost
            new_cost = node.cost + node.distance_to(child)

            if new_cost < old_cost:
                child.cost = new_cost
                self._propagate_cost_improvements(child)

    def _get_children(self, node: RRTNode) -> List[RRTNode]:
        """Get all children of a node"""
        return [n for n in self.nodes if n.parent == node]

    def _extract_trajectory(self, goal_node: RRTNode) -> List[np.ndarray]:
        """Extract joint trajectory from tree"""
        trajectory = []
        current = goal_node

        while current is not None:
            trajectory.append(current.configuration.copy())
            current = current.parent

        trajectory.reverse()
        return trajectory

    def _find_best_trajectory_to_goals(self) -> Optional[List[np.ndarray]]:
        """Find best trajectory to any goal node"""
        best_trajectory = None
        best_cost = float("inf")

        for goal_node in self.goal_nodes:
            # Find closest tree node to goal
            closest_node = min(self.nodes, key=lambda node: node.distance_to(goal_node))

            if closest_node.distance_to(goal_node) < self.goal_tolerance * 2:
                # Try to connect
                if self._collision_free(closest_node, goal_node):
                    # Create temporary connection
                    temp_goal = RRTNode(
                        goal_node.configuration,
                        cost=closest_node.cost + closest_node.distance_to(goal_node),
                        parent=closest_node,
                    )

                    trajectory = self._extract_trajectory(temp_goal)
                    if len(trajectory) > 0 and temp_goal.cost < best_cost:
                        best_trajectory = trajectory
                        best_cost = temp_goal.cost

        return best_trajectory

    def set_collision_checker(self, checker_func):
        """Set external collision checking function"""
        self.collision_checker = checker_func

    def set_logger(self, logger):
        """Set logger for debugging"""
        self.logger = logger
