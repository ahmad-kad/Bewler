#!/usr/bin/env python3
"""
Path Planner - Global and local path planning for navigation.

Implements:
- A* global path planning
- Local obstacle avoidance
- Terrain-aware routing
- Multi-objective optimization

Author: URC 2026 Autonomy Team
"""

import math
from dataclasses import dataclass
from enum import Enum
from heapq import heappop, heappush
from typing import Dict, List, Optional, Tuple


class PathPlanningAlgorithm(Enum):
    """Available path planning algorithms"""

    ASTAR = "astar"
    DIJKSTRA = "dijkstra"
    RRT = "rrt"


@dataclass
class Node:
    """A* search node"""

    position: Tuple[int, int]
    g_cost: float  # Cost from start
    h_cost: float  # Heuristic cost to goal
    f_cost: float  # Total cost
    parent: Optional["Node"] = None

    @property
    def x(self) -> int:
        return self.position[0]

    @property
    def y(self) -> int:
        return self.position[1]


@dataclass
class PathSegment:
    """Path segment with properties"""

    start: Tuple[float, float]
    end: Tuple[float, float]
    length: float
    terrain_cost: float
    safety_margin: float


class PathPlanner:
    """
    Path planning system for autonomous navigation.

    Features:
    - Global path planning with A*
    - Local obstacle avoidance
    - Terrain cost integration
    - Multi-objective optimization
    """

    def __init__(self):
        self.algorithm = PathPlanningAlgorithm.ASTAR
        self.grid_resolution = 0.5  # meters
        self.max_search_iterations = 10000
        self.obstacle_inflation_radius = 1.0  # meters

        # Cost weights
        self.distance_weight = 1.0
        self.terrain_weight = 2.0
        self.safety_weight = 1.5

        # TODO: Initialize path planning parameters
        # - Map loading
        # - Cost map generation
        # - Algorithm parameters

    def initialize(self):
        """Initialize path planner"""
        # TODO: Load map data
        # Initialize cost maps
        # Set up algorithm parameters

    def plan_path(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        constraints: Optional[dict] = None,
    ) -> List[Tuple[float, float]]:
        """
        Plan path from start to goal

        Args:
            start: Starting position (x, y)
            goal: Goal position (x, y)
            constraints: Optional planning constraints

        Returns:
            List of waypoints forming the path
        """
        if self.algorithm == PathPlanningAlgorithm.ASTAR:
            return self._plan_astar(start, goal, constraints)
        elif self.algorithm == PathPlanningAlgorithm.DIJKSTRA:
            return self._plan_dijkstra(start, goal, constraints)
        else:
            # TODO: Implement other algorithms
            return []

    def _plan_astar(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        constraints: Optional[dict] = None,
    ) -> List[Tuple[float, float]]:
        """A* path planning implementation"""
        # TODO: Implement A* algorithm
        # - Grid discretization
        # - Open/closed set management
        # - Cost calculation with terrain
        # - Path reconstruction

        # Placeholder implementation
        return [start, goal]

    def _plan_dijkstra(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        constraints: Optional[dict] = None,
    ) -> List[Tuple[float, float]]:
        """Dijkstra path planning implementation"""
        # TODO: Implement Dijkstra algorithm
        return [start, goal]

    def get_terrain_cost(self, position: Tuple[float, float]) -> float:
        """Get terrain cost at position"""
        # TODO: Implement terrain cost lookup
        # - Query cost map
        # - Consider terrain type
        # - Factor in traversability
        return 1.0  # Placeholder

    def is_position_valid(self, position: Tuple[float, float]) -> bool:
        """Check if position is valid (no obstacles)"""
        # TODO: Implement obstacle checking
        # - Check against obstacle map
        # - Consider inflation radius
        # - Validate bounds
        return True  # Placeholder

    def smooth_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Smooth path to reduce sharp turns"""
        # TODO: Implement path smoothing
        # - Remove unnecessary waypoints
        # - Apply smoothing algorithms
        # - Maintain safety margins
        return path  # Placeholder

    def optimize_path(self, path: List[Tuple[float, float]], criteria: List[str] = None) -> List[Tuple[float, float]]:
        """Optimize path based on multiple criteria"""
        if not criteria:
            criteria = ["distance", "terrain", "safety"]

        # TODO: Implement multi-objective optimization
        # - Distance minimization
        # - Terrain cost reduction
        # - Safety margin maximization
        return path  # Placeholder

    def get_path_length(self, path: List[Tuple[float, float]]) -> float:
        """Calculate total path length"""
        if len(path) < 2:
            return 0.0

        total_length = 0.0
        for i in range(len(path) - 1):
            dx = path[i + 1][0] - path[i][0]
            dy = path[i + 1][1] - path[i][1]
            total_length += math.sqrt(dx * dx + dy * dy)

        return total_length

    def estimate_traversal_time(self, path: List[Tuple[float, float]], speed_profile: dict = None) -> float:
        """Estimate time to traverse path"""
        # TODO: Implement time estimation
        # - Consider terrain types
        # - Account for speed limits
        # - Factor in acceleration/deceleration
        length = self.get_path_length(path)
        avg_speed = speed_profile.get("average", 1.0) if speed_profile else 1.0
        return length / avg_speed

    def update_costmap(self, new_obstacles: List[Tuple[float, float, float]]):
        """Update costmap with new obstacle information"""
        # TODO: Implement costmap updates
        # - Add new obstacles
        # - Update inflation zones
        # - Replan affected paths if needed

    def reset_planner(self):
        """Reset planner state"""
        # TODO: Clear internal state
        # Reset costmaps
        # Clear cached paths

    def shutdown(self):
        """Shutdown path planner"""
        # TODO: Clean shutdown
        # Save state if needed
        # Release resources


# =============================================================================
# D* Lite Implementation for Dynamic Path Planning
# =============================================================================


@dataclass
class DStarNode:
    """Node for D* Lite priority queue"""

    position: Tuple[int, int]
    key: Tuple[float, float]  # (k1, k2) priority values

    def __lt__(self, other):
        return self.key < other.key


class DStarLite:
    """
    D* Lite algorithm for efficient replanning in dynamic environments.

    Key features:
    - Incremental replanning when map changes
    - Reverse search from goal to start
    - Maintains optimality while being computationally efficient
    """

    def __init__(self, grid_resolution: float = 0.5):
        # Priority queue for node updates
        self.U: List[DStarNode] = []

        # Cost estimates
        self.g: Dict[Tuple[int, int], float] = {}  # Current best cost from goal
        self.rhs: Dict[Tuple[int, int], float] = {}  # One-step lookahead cost

        # Grid parameters
        self.grid_resolution = grid_resolution
        self.obstacle_cost = float("inf")

        # Last start/goal for incremental updates
        self.s_start: Optional[Tuple[int, int]] = None
        self.s_goal: Optional[Tuple[int, int]] = None
        self.s_last: Optional[Tuple[int, int]] = None

        # Costmap (will be updated from SLAM)
        self.costmap: Dict[Tuple[int, int], float] = {}

        # Movement cost (diagonal moves allowed)
        self.sqrt2 = math.sqrt(2)

    def initialize(self, start: Tuple[float, float], goal: Tuple[float, float]):
        """Initialize D* Lite for new planning task"""
        # Convert to grid coordinates
        self.s_start = self._world_to_grid(start)
        self.s_goal = self._world_to_grid(goal)
        self.s_last = self.s_start

        # Clear data structures
        self.U.clear()
        self.g.clear()
        self.rhs.clear()

        # Initialize goal
        self.rhs[self.s_goal] = 0
        self.g[self.s_goal] = float("inf")

        # Insert goal into priority queue
        goal_node = DStarNode(self.s_goal, self._calculate_key(self.s_goal))
        heappush(self.U, goal_node)

        self._compute_shortest_path()

    def replan(self, new_costmap: Dict[Tuple[int, int], float]) -> List[Tuple[float, float]]:
        """
        Incremental replanning when costmap changes

        Args:
            new_costmap: Updated costmap from SLAM

        Returns:
            Updated path from current position to goal
        """
        # Update costmap
        self.costmap = new_costmap

        # Check for significant changes and replan
        if self._costmap_changed():
            self._update_changed_vertices()
            self._compute_shortest_path()

        return self._extract_path()

    def _costmap_changed(self) -> bool:
        """Check if costmap has significant changes requiring replan"""
        # TODO: Implement change detection logic
        # For now, assume changes require replan
        return True

    def _update_changed_vertices(self):
        """Update vertices affected by costmap changes"""
        # TODO: Implement vertex update logic based on SLAM changes
        # This would identify which grid cells changed and update their costs

    def _compute_shortest_path(self):
        """Main D* Lite algorithm - compute shortest path"""
        while self.U and (
            self.U[0].key < self._calculate_key(self.s_start) or self.rhs[self.s_start] != self.g[self.s_start]
        ):

            # Get node with lowest priority
            u = heappop(self.U)
            k_old = u.key
            k_new = self._calculate_key(u.position)

            if k_old < k_new:
                # Node priority increased, reinsert
                u.key = k_new
                heappush(self.U, u)
            elif self.g[u.position] > self.rhs[u.position]:
                # Locally overconsistent (rhs is better)
                self.g[u.position] = self.rhs[u.position]
                # Update all neighbors
                for neighbor in self._get_neighbors(u.position):
                    self._update_vertex(neighbor)
            else:
                # Locally underconsistent (g is better)
                self.g[u.position] = float("inf")
                # Update current node and all neighbors
                self._update_vertex(u.position)
                for neighbor in self._get_neighbors(u.position):
                    self._update_vertex(neighbor)

    def _update_vertex(self, u: Tuple[int, int]):
        """Update vertex in D* Lite algorithm"""
        if u != self.s_goal:
            # Calculate minimum cost through neighbors
            min_cost = float("inf")
            for neighbor in self._get_neighbors(u):
                cost = self.g.get(neighbor, float("inf")) + self._cost(u, neighbor)
                if cost < min_cost:
                    min_cost = cost
            self.rhs[u] = min_cost

        # Remove from priority queue if present
        self.U = [node for node in self.U if node.position != u]

        # Add to queue if locally inconsistent
        if self.g.get(u, float("inf")) != self.rhs.get(u, float("inf")):
            node = DStarNode(u, self._calculate_key(u))
            heappush(self.U, node)

    def _calculate_key(self, s: Tuple[int, int]) -> Tuple[float, float]:
        """Calculate priority key for node"""
        g_rhs_min = min(self.g.get(s, float("inf")), self.rhs.get(s, float("inf")))

        # Calculate heuristic from start to this node
        h_start = self._heuristic(self.s_start, s)

        return (g_rhs_min + h_start + self._heuristic(self.s_last, self.s_start), g_rhs_min)

    def _heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Euclidean distance heuristic"""
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return math.sqrt(dx * dx + dy * dy) * self.grid_resolution

    def _cost(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Cost of moving between adjacent cells"""
        # Check if either cell is an obstacle
        if self.costmap.get(a, 0) >= self.obstacle_cost:
            return float("inf")
        if self.costmap.get(b, 0) >= self.obstacle_cost:
            return float("inf")

        # Movement cost (diagonal moves cost more)
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])

        if dx == 1 and dy == 1:
            return self.sqrt2 * self.grid_resolution  # Diagonal
        else:
            return self.grid_resolution  # Cardinal

    def _get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get 8-connected neighbors"""
        x, y = pos
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                neighbor = (x + dx, y + dy)
                neighbors.append(neighbor)
        return neighbors

    def _world_to_grid(self, world_pos: Tuple[float, float]) -> Tuple[int, int]:
        """Convert world coordinates to grid coordinates"""
        x = int(world_pos[0] / self.grid_resolution)
        y = int(world_pos[1] / self.grid_resolution)
        return (x, y)

    def _grid_to_world(self, grid_pos: Tuple[int, int]) -> Tuple[float, float]:
        """Convert grid coordinates to world coordinates"""
        x = grid_pos[0] * self.grid_resolution
        y = grid_pos[1] * self.grid_resolution
        return (x, y)

    def _extract_path(self) -> List[Tuple[float, float]]:
        """Extract path from current position to goal"""
        if self.g.get(self.s_start, float("inf")) == float("inf"):
            return []  # No path found

        path = []
        current = self.s_start

        while current != self.s_goal:
            path.append(self._grid_to_world(current))

            # Find best neighbor
            best_neighbor = None
            best_cost = float("inf")

            for neighbor in self._get_neighbors(current):
                neighbor_cost = self.g.get(neighbor, float("inf")) + self._cost(current, neighbor)
                if neighbor_cost < best_cost:
                    best_cost = neighbor_cost
                    best_neighbor = neighbor

            if best_neighbor is None:
                break  # No valid path

            current = best_neighbor

        path.append(self._grid_to_world(self.s_goal))
        return path
