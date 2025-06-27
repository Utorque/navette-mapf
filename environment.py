"""
Environment module for MAPF simulation.
Defines the grid layout, robots, and basic structures.
"""

from typing import List, Tuple, Optional, NamedTuple
from dataclasses import dataclass
from enum import Enum

# Grid constants
GRID_WIDTH = 5
GRID_HEIGHT = 2
ROOMS = ['in', 'A', 'B', 'C', 'out']

class Position(NamedTuple):
    """Represents a position in the grid."""
    row: int
    col: int

class RobotStatus(Enum):
    """Robot status enumeration."""
    IDLE = "idle"
    MOVING = "moving"

class OrderStatus(Enum):
    """Order status enumeration."""
    PENDING = "pending"
    ASSIGNED = "assigned"
    COMPLETED = "completed"

@dataclass
class Robot:
    """Robot class with position, path, and status."""
    id: int
    position: Position
    path: List[Position]
    path_index: int
    priority: int
    status: RobotStatus
    is_waiting: bool = False
    
    @property
    def row(self) -> int:
        return self.position.row
    
    @property
    def col(self) -> int:
        return self.position.col

@dataclass
class Order:
    """Order class representing a pickup/delivery task."""
    id: int
    from_room: str
    to_room: str
    request_time: int
    status: OrderStatus
    assigned_robot: Optional[int] = None
    start_time: Optional[int] = None
    completion_time: Optional[int] = None

class Environment:
    """Environment class managing the grid layout and positions."""
    
    def __init__(self):
        self.width = GRID_WIDTH
        self.height = GRID_HEIGHT
        self.rooms = ROOMS
    
    def get_position_name(self, row: int, col: int) -> str:
        """Convert position to name."""
        if row == 0:
            return self.rooms[col]  # Room names
        return f"corridor-{col}"  # Corridor positions
    
    def find_room_position(self, room_name: str) -> Optional[Position]:
        """Find room position by name."""
        try:
            col = self.rooms.index(room_name)
            return Position(0, col)
        except ValueError:
            return None
    
    def is_valid_position(self, pos: Position) -> bool:
        """Check if position is valid in the grid."""
        return (0 <= pos.row < self.height and 
                0 <= pos.col < self.width)
    
    def get_valid_moves(self, pos: Position) -> List[Tuple[Position, int]]:
        """Get valid moves from a position with costs."""
        moves = []
        
        if pos.row == 1:  # In corridor
            # Horizontal movement in corridor
            if pos.col > 0:
                moves.append((Position(1, pos.col - 1), 1))  # left
            if pos.col < self.width - 1:
                moves.append((Position(1, pos.col + 1), 1))  # right
            # Access to room directly above
            moves.append((Position(0, pos.col), 1))  # up to room
            # Wait in place
            moves.append((Position(1, pos.col), 1))  # wait
            
        elif pos.row == 0:  # In room
            # Only move back down to corridor or wait
            moves.append((Position(1, pos.col), 1))  # down to corridor
            moves.append((Position(0, pos.col), 1))  # wait in room
        
        return moves
    
    def manhattan_distance(self, pos1: Position, pos2: Position) -> int:
        """Calculate Manhattan distance between two positions."""
        return abs(pos1.row - pos2.row) + abs(pos1.col - pos2.col)
    
    def heuristic(self, current: Position, goal: Position) -> int:
        """Heuristic function for A* pathfinding."""
        if goal.row == 0:  # Goal is a room
            corridor_steps = abs(current.col - goal.col)  # Horizontal corridor movement
            room_steps = 1 if current.row == 0 else 2  # Steps to access room
            return corridor_steps + room_steps
        return self.manhattan_distance(current, goal)