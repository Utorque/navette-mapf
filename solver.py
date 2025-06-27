"""
MAPF Solver using Space-Time A* with Prioritized Planning.
Handles collision avoidance for multiple robots.
"""

import heapq
from typing import List, Dict, Set, Tuple, Optional
from dataclasses import dataclass, field
from environment import Environment, Position, Robot

@dataclass
class AStarNode:
    """Node for A* search with time dimension."""
    position: Position
    time: int
    g_cost: float
    f_cost: float
    path: List[Position] = field(default_factory=list)
    
    def __lt__(self, other):
        return self.f_cost < other.f_cost

class SpaceTimeAStar:
    """Space-Time A* pathfinder with collision avoidance."""
    
    def __init__(self, environment: Environment):
        self.env = environment
    
    def find_path(self, start: Position, goal: Position, 
                  other_robot_paths: List[List[Position]] = None,
                  max_time: int = 50) -> List[Position]:
        """
        Find path from start to goal avoiding collisions with other robot paths.
        
        Args:
            start: Starting position
            goal: Goal position
            other_robot_paths: List of paths for other robots (indexed by time)
            max_time: Maximum time steps to search
            
        Returns:
            List of positions representing the path
        """
        if other_robot_paths is None:
            other_robot_paths = []
        
        # Priority queue for A* search
        open_set = []
        heapq.heappush(open_set, AStarNode(
            position=start,
            time=0,
            g_cost=0,
            f_cost=self.env.heuristic(start, goal),
            path=[]
        ))
        
        # Closed set to avoid revisiting states
        closed_set: Set[Tuple[Position, int]] = set()
        
        while open_set:
            current = heapq.heappop(open_set)
            
            # Check if we reached the goal
            if current.position == goal:
                return current.path + [current.position]
            
            # Skip if already visited this state
            state = (current.position, current.time)
            if state in closed_set:
                continue
            closed_set.add(state)
            
            # Don't search beyond max time
            if current.time >= max_time:
                continue
            
            # Explore neighboring positions
            for next_pos, move_cost in self.env.get_valid_moves(current.position):
                next_time = current.time + 1
                
                # Check for collisions with other robots
                if self._has_collision(current.position, next_pos, next_time, other_robot_paths):
                    continue
                
                # Skip if already visited this state
                next_state = (next_pos, next_time)
                if next_state in closed_set:
                    continue
                
                # Calculate costs
                g_cost = current.g_cost + move_cost
                h_cost = self.env.heuristic(next_pos, goal)
                f_cost = g_cost + h_cost
                
                # Add to open set
                heapq.heappush(open_set, AStarNode(
                    position=next_pos,
                    time=next_time,
                    g_cost=g_cost,
                    f_cost=f_cost,
                    path=current.path + [current.position]
                ))
        
        return []  # No path found
    
    def _has_collision(self, current_pos: Position, next_pos: Position, 
                      next_time: int, other_robot_paths: List[List[Position]]) -> bool:
        """
        Check for vertex and edge collisions with other robot paths.
        
        Args:
            current_pos: Current position
            next_pos: Next intended position
            next_time: Time step for the next position
            other_robot_paths: Paths of other robots
            
        Returns:
            True if there's a collision, False otherwise
        """
        for other_path in other_robot_paths:
            # Check vertex collision (same position at same time)
            if (next_time < len(other_path) and 
                other_path[next_time] == next_pos):
                return True
            
            # Check edge collision (robots swapping positions)
            if (next_time > 0 and 
                next_time - 1 < len(other_path) and 
                next_time < len(other_path)):
                
                other_prev = other_path[next_time - 1]
                other_next = other_path[next_time]
                
                # Check if robots are swapping positions
                if (current_pos == other_next and 
                    next_pos == other_prev):
                    return True
        
        return False

class MAPFSolver:
    """Multi-Agent Path Finding solver using prioritized planning."""
    
    def __init__(self, environment: Environment):
        self.env = environment
        self.pathfinder = SpaceTimeAStar(environment)
    
    def solve_multi_robot_path(self, robots: List[Robot], goals: Dict[int, Position]) -> Dict[int, List[Position]]:
        """
        Solve MAPF problem for multiple robots using prioritized planning.
        
        Args:
            robots: List of robots
            goals: Dictionary mapping robot_id to goal position
            
        Returns:
            Dictionary mapping robot_id to path
        """
        # Sort robots by priority (lower number = higher priority)
        sorted_robots = sorted(robots, key=lambda r: r.priority)
        
        paths: Dict[int, List[Position]] = {}
        robot_paths_list: List[List[Position]] = []
        
        for robot in sorted_robots:
            if robot.id not in goals:
                continue
            
            goal = goals[robot.id]
            
            # Find path avoiding already planned paths
            path = self.pathfinder.find_path(
                start=robot.position,
                goal=goal,
                other_robot_paths=robot_paths_list
            )
            
            if path:
                paths[robot.id] = path
                robot_paths_list.append(path)
            else:
                # No path found - robot stays in place
                paths[robot.id] = [robot.position]
        
        return paths
    
    def find_single_robot_path(self, robot: Robot, goal: Position, 
                              other_robot_paths: List[List[Position]] = None) -> List[Position]:
        """
        Find path for a single robot avoiding other robot paths.
        
        Args:
            robot: The robot to find path for
            goal: Goal position
            other_robot_paths: Paths of other robots to avoid
            
        Returns:
            List of positions representing the path
        """
        if other_robot_paths is None:
            other_robot_paths = []
        
        return self.pathfinder.find_path(
            start=robot.position,
            goal=goal,
            other_robot_paths=other_robot_paths
        )