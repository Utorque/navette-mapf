"""
Main simulator for MAPF simulation.
Handles the discrete simulation loop and robot coordination.
"""

import random
import time
from typing import List, Dict, Optional
from environment import Environment, Robot, Position, RobotStatus
from solver import MAPFSolver
from orders import OrderManager

class MAPFSimulator:
    """Main simulator class for MAPF simulation."""
    
    def __init__(self):
        self.env = Environment()
        self.mapf_solver = MAPFSolver(self.env)
        self.order_manager = OrderManager(self.env)
        
        # Simulation state
        self.is_running = False
        self.time_elapsed = 0
        self.last_update_time = 0
        self.update_interval = 1.0  # 1 second between updates
        
        # Initialize robots
        self.robots: List[Robot] = [
            Robot(
                id=1,
                position=Position(1, 0),  # Corridor position 0
                path=[],
                path_index=0,
                priority=1,
                status=RobotStatus.IDLE
            ),
            Robot(
                id=2,
                position=Position(1, 4),  # Corridor position 4
                path=[],
                path_index=0,
                priority=2,
                status=RobotStatus.IDLE
            )
        ]
    
    def update(self) -> None:
        """Main update method called each frame."""
        if not self.is_running:
            return
        
        current_time = time.time()
        if current_time - self.last_update_time >= self.update_interval:
            self._simulation_step()
            self.last_update_time = current_time
    
    def _simulation_step(self) -> None:
        """Execute one simulation step."""
        self.time_elapsed += 1
        
        # Generate random orders occasionally
        if random.random() < 0.1:  # 10% chance per second
            self.order_manager.generate_random_order(self.time_elapsed)
        
        # Assign orders to available robots
        self._assign_orders()
        
        # Move robots along their paths
        self._move_robots()
    
    def _assign_orders(self) -> None:
        """Assign pending orders to available robots."""
        pending_orders = self.order_manager.get_pending_orders()
        available_robots = [robot for robot in self.robots if robot.status == RobotStatus.IDLE]
        
        if not pending_orders or not available_robots:
            return
        
        # Process one order at a time (first come, first served)
        order = pending_orders[0]
        
        # Find the best robot for this order
        best_robot = self.order_manager.get_best_robot_for_order(order, available_robots)
        if not best_robot:
            return
        
        # Get pickup and delivery positions
        from_pos = self.env.find_room_position(order.from_room)
        to_pos = self.env.find_room_position(order.to_room)
        
        if not from_pos or not to_pos:
            return
        
        # Get paths of other active robots for collision avoidance
        other_robot_paths = self._get_other_robot_paths(best_robot.id)
        
        # Plan path: current -> pickup room -> destination room
        path_to_pickup = self.mapf_solver.find_single_robot_path(
            best_robot, from_pos, other_robot_paths
        )
        
        if not path_to_pickup:
            return  # No path found
        
        # Create temporary robot at pickup position to plan second leg
        temp_robot = Robot(
            id=best_robot.id,
            position=from_pos,
            path=[],
            path_index=0,
            priority=best_robot.priority,
            status=RobotStatus.MOVING
        )
        
        path_to_destination = self.mapf_solver.find_single_robot_path(
            temp_robot, to_pos, other_robot_paths
        )
        
        if not path_to_destination:
            return  # No path found
        
        # Combine paths (remove duplicate pickup position)
        full_path = path_to_pickup[:-1] + path_to_destination
        
        # Assign path to robot
        best_robot.path = full_path
        best_robot.path_index = 0
        best_robot.status = RobotStatus.MOVING
        best_robot.is_waiting = False
        
        # Assign order
        self.order_manager.assign_order_to_robot(order, best_robot, self.time_elapsed)
    
    def _get_other_robot_paths(self, exclude_robot_id: int) -> List[List[Position]]:
        """Get paths of other robots for collision avoidance."""
        other_paths = []
        
        for robot in self.robots:
            if robot.id == exclude_robot_id:
                continue
            
            if robot.status == RobotStatus.MOVING and robot.path:
                # Include current position as time 0, then remaining path
                current_pos = robot.position
                remaining_path = robot.path[robot.path_index:]
                full_path = [current_pos] + remaining_path
                other_paths.append(full_path)
            elif robot.status == RobotStatus.IDLE:
                # Stationary robot occupies its position
                stationary_path = [robot.position] * 20  # Block for 20 timesteps
                other_paths.append(stationary_path)
        
        return other_paths
    
    def _move_robots(self) -> None:
        """Move robots along their paths with collision avoidance."""
        # Calculate intended moves for all robots
        intended_moves = {}
        position_conflicts = {}
        
        for robot in self.robots:
            if robot.status == RobotStatus.MOVING and robot.path and robot.path_index < len(robot.path):
                next_pos = robot.path[robot.path_index]
                intended_moves[robot.id] = next_pos
                
                # Track position conflicts
                pos_key = (next_pos.row, next_pos.col)
                if pos_key not in position_conflicts:
                    position_conflicts[pos_key] = []
                position_conflicts[pos_key].append(robot)
        
        # Execute moves, handling conflicts by priority
        for robot in self.robots:
            if robot.id not in intended_moves:
                robot.is_waiting = False
                continue
            
            next_pos = intended_moves[robot.id]
            pos_key = (next_pos.row, next_pos.col)
            
            # Check for conflicts
            conflicting_robots = position_conflicts[pos_key]
            if len(conflicting_robots) > 1:
                # Find highest priority robot (lowest priority number)
                highest_priority_robot = min(conflicting_robots, key=lambda r: r.priority)
                
                if robot.id != highest_priority_robot.id:
                    # This robot has lower priority, so it waits
                    robot.is_waiting = True
                    continue
            
            # Move robot
            robot.position = next_pos
            robot.path_index += 1
            robot.is_waiting = False
            
            # Check if robot completed its path
            if robot.path_index >= len(robot.path):
                robot.path = []
                robot.path_index = 0
                robot.status = RobotStatus.IDLE
                
                # Complete the order
                order = self.order_manager.find_order_by_robot(robot.id)
                if order:
                    self.order_manager.complete_order(order, self.time_elapsed)
    
    def toggle_simulation(self) -> None:
        """Toggle simulation running state."""
        self.is_running = not self.is_running
        if self.is_running:
            self.last_update_time = time.time()
    
    def reset(self) -> None:
        """Reset the simulation to initial state."""
        self.is_running = False
        self.time_elapsed = 0
        self.last_update_time = 0
        
        # Reset robots
        self.robots = [
            Robot(
                id=1,
                position=Position(1, 0),
                path=[],
                path_index=0,
                priority=1,
                status=RobotStatus.IDLE
            ),
            Robot(
                id=2,
                position=Position(1, 4),
                path=[],
                path_index=0,
                priority=2,
                status=RobotStatus.IDLE
            )
        ]
        
        # Reset orders
        self.order_manager.reset()
    
    def add_manual_order(self) -> None:
        """Add a manual order."""
        self.order_manager.add_manual_order(self.time_elapsed)
    
    def get_robot_at_position(self, position: Position) -> Optional[Robot]:
        """Get robot at specific position."""
        for robot in self.robots:
            if robot.position == position:
                return robot
        return None
    
    def is_position_in_robot_path(self, position: Position, robot: Robot) -> bool:
        """Check if position is in robot's remaining path."""
        if not robot.path:
            return False
        
        remaining_path = robot.path[robot.path_index:]
        return position in remaining_path
    
    def get_path_step_number(self, position: Position, robot: Robot) -> int:
        """Get step number for position in robot's path."""
        if not robot.path:
            return 0
        
        remaining_path = robot.path[robot.path_index:]
        try:
            return remaining_path.index(position) + 1
        except ValueError:
            return 0