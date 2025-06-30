#!/usr/bin/env python3
"""
MAPF Demo with Space-Time A* and Proper Collision Detection
- No vertex conflicts (same position at same time)
- No edge conflicts (swapping positions)
- Priority-based planning with reservation table
"""

import pygame
import heapq
import random
from dataclasses import dataclass
from typing import List, Tuple, Set, Dict, Optional
from collections import defaultdict

# Constants
GRID_SIZE = 10
CELL_SIZE = 50
WINDOW_SIZE = GRID_SIZE * CELL_SIZE
FPS = 2
MAX_TIME = 100  # Maximum timesteps for planning

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (200, 200, 200)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)
YELLOW = (255, 255, 0)
PURPLE = (128, 0, 128)

@dataclass
class SpaceTimeNode:
    """Node in space-time search: position + time"""
    x: int
    y: int
    time: int
    
    def __hash__(self):
        return hash((self.x, self.y, self.time))
    
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.time == other.time
    
    def __lt__(self, other):
        # Define ordering for heapq: first by time, then by position
        if self.time != other.time:
            return self.time < other.time
        if self.x != other.x:
            return self.x < other.x
        return self.y < other.y

@dataclass
class Robot:
    id: int
    start: Tuple[int, int]
    goal: Tuple[int, int]
    pos: Tuple[int, int]
    path: List[Tuple[int, int]]
    color: Tuple[int, int, int]
    priority: int  # Lower number = higher priority

class ReservationTable:
    """Tracks reserved positions at each timestep to prevent conflicts"""
    def __init__(self):
        # vertex_table[time][(x,y)] = agent_id
        self.vertex_table: Dict[int, Dict[Tuple[int, int], int]] = defaultdict(dict)
        # edge_table[time][((x1,y1),(x2,y2))] = agent_id
        self.edge_table: Dict[int, Dict[Tuple[Tuple[int, int], Tuple[int, int]], int]] = defaultdict(dict)
    
    def is_vertex_free(self, x: int, y: int, time: int, agent_id: int) -> bool:
        """Check if position is free at given time"""
        if (x, y) in self.vertex_table[time]:
            return self.vertex_table[time][(x, y)] == agent_id
        return True
    
    def is_edge_free(self, from_pos: Tuple[int, int], to_pos: Tuple[int, int], 
                     time: int, agent_id: int) -> bool:
        """Check if edge traversal is free (no swap conflict)"""
        # Check if another agent is using the reverse edge at the same time
        reverse_edge = (to_pos, from_pos)
        if reverse_edge in self.edge_table[time]:
            return self.edge_table[time][reverse_edge] == agent_id
        return True
    
    def reserve_path(self, path: List[Tuple[int, int]], start_time: int, agent_id: int):
        """Reserve positions and edges along a path"""
        for i, pos in enumerate(path):
            t = start_time + i
            self.vertex_table[t][pos] = agent_id
            
            # Reserve edge from previous position
            if i > 0:
                prev_pos = path[i-1]
                edge = (prev_pos, pos)
                self.edge_table[t][edge] = agent_id
    
    def clear_agent_reservations(self, agent_id: int):
        """Clear all reservations for a specific agent"""
        # Clear vertex reservations
        for time_dict in self.vertex_table.values():
            to_remove = [pos for pos, aid in time_dict.items() if aid == agent_id]
            for pos in to_remove:
                del time_dict[pos]
        
        # Clear edge reservations
        for time_dict in self.edge_table.values():
            to_remove = [edge for edge, aid in time_dict.items() if aid == agent_id]
            for edge in to_remove:
                del time_dict[edge]

class SpaceTimeAStar:
    """Space-Time A* planner with collision avoidance"""
    def __init__(self, grid_size: int):
        self.grid_size = grid_size
    
    def heuristic(self, pos: Tuple[int, int], goal: Tuple[int, int]) -> int:
        """Manhattan distance heuristic"""
        return abs(pos[0] - goal[0]) + abs(pos[1] - goal[1])
    
    def get_neighbors(self, node: SpaceTimeNode) -> List[SpaceTimeNode]:
        """Get valid neighbors including wait action"""
        neighbors = []
        
        # Move actions
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            nx, ny = node.x + dx, node.y + dy
            if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                neighbors.append(SpaceTimeNode(nx, ny, node.time + 1))
        
        # Wait action
        neighbors.append(SpaceTimeNode(node.x, node.y, node.time + 1))
        
        return neighbors
    
    def plan(self, start: Tuple[int, int], goal: Tuple[int, int], 
             start_time: int, agent_id: int, reservation_table: ReservationTable) -> Optional[List[Tuple[int, int]]]:
        """Find collision-free path using Space-Time A*"""
        
        # Priority queue: (f_cost, g_cost, node, path)
        open_set = [(0, 0, SpaceTimeNode(start[0], start[1], start_time), [])]
        closed_set = set()
        
        while open_set and len(open_set) < 10000:  # Prevent infinite loops
            f_cost, g_cost, current, path = heapq.heappop(open_set)
            
            # Skip if already visited
            if current in closed_set:
                continue
            closed_set.add(current)
            
            # Check if reached goal
            if (current.x, current.y) == goal:
                return path + [(current.x, current.y)]
            
            # Expand neighbors
            for next_node in self.get_neighbors(current):
                # Skip if exceeds max time
                if next_node.time >= MAX_TIME:
                    continue
                
                # Skip if already visited
                if next_node in closed_set:
                    continue
                
                # Check vertex constraint
                if not reservation_table.is_vertex_free(next_node.x, next_node.y, 
                                                       next_node.time, agent_id):
                    continue
                
                # Check edge constraint (no swapping)
                current_pos = (current.x, current.y)
                next_pos = (next_node.x, next_node.y)
                if not reservation_table.is_edge_free(current_pos, next_pos, 
                                                     next_node.time, agent_id):
                    continue
                
                # Calculate costs
                new_g_cost = g_cost + 1
                h_cost = self.heuristic((next_node.x, next_node.y), goal)
                new_f_cost = new_g_cost + h_cost
                
                # Add to open set
                new_path = path + [(current.x, current.y)]
                heapq.heappush(open_set, (new_f_cost, new_g_cost, next_node, new_path))
        
        return None  # No path found

class MAPFSimulator:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))
        pygame.display.set_caption("MAPF with Correct Collision Detection")
        self.clock = pygame.time.Clock()
        
        # Create robots with different priorities
        self.robots = [
            Robot(0, (0, 0), (9, 9), (0, 0), [], BLUE, 0),
            Robot(1, (9, 0), (0, 9), (9, 0), [], RED, 1),
            Robot(2, (0, 9), (9, 0), (0, 9), [], GREEN, 2),
            Robot(3, (5, 5), (5, 0), (5, 5), [], PURPLE, 3)
        ]
        
        self.planner = SpaceTimeAStar(GRID_SIZE)
        self.reservation_table = ReservationTable()
        self.time_step = 0
        
        # Plan initial paths
        self.plan_all_paths()
    
    def plan_all_paths(self):
        """Plan paths for all robots using priority order"""
        # Clear all reservations
        self.reservation_table = ReservationTable()
        self.time_step = 0
        
        # Sort robots by priority (lower number = higher priority)
        sorted_robots = sorted(self.robots, key=lambda r: r.priority)
        
        for robot in sorted_robots:
            # Clear any existing path
            robot.path = []
            robot.pos = robot.start
            
            # Plan path with current reservations
            path = self.planner.plan(robot.start, robot.goal, 0, robot.id, self.reservation_table)
            
            if path:
                robot.path = path[1:]  # Exclude starting position
                # Reserve the path
                self.reservation_table.reserve_path([robot.start] + robot.path, 0, robot.id)
                print(f"Robot {robot.id}: Found path of length {len(robot.path)}")
            else:
                print(f"Robot {robot.id}: No collision-free path found!")
        
    def move_robots(self):
        """Move robots along their paths"""
        any_robot_moving = False
        
        # Track robots that moved out of the way last timestep
        if not hasattr(self, 'robots_waiting'):
            self.robots_waiting = {}
        
        # Move each robot based on their current state
        for robot in self.robots:
            if robot.id in self.robots_waiting:
                # Robot is waiting - check if it can return to goal
                wait_until = self.robots_waiting[robot.id]
                if self.time_step >= wait_until:
                    robot.pos = robot.goal
                    del self.robots_waiting[robot.id]
                    print(f"Robot {robot.id} returned to goal {robot.goal}")
                # else: stay in current temporary position
                
            elif robot.path and self.time_step < len(robot.path):
                # Robot is still following its path
                robot.pos = robot.path[self.time_step]
                any_robot_moving = True
                
            else:
                # Robot has reached goal and is not waiting - stay at goal
                robot.pos = robot.goal
        
        # Now check for potential collisions and move robots out of the way
        robots_at_goal = [r for r in self.robots if r.pos == r.goal and r.id not in self.robots_waiting]
        robots_moving = [r for r in self.robots if r.pos != r.goal and r.path]
        
        for blocking_robot in robots_at_goal:
            for moving_robot in robots_moving:
                future_steps = min(5, len(moving_robot.path) - self.time_step)
                for i in range(1, future_steps + 1):
                    if (self.time_step + i < len(moving_robot.path) and 
                        moving_robot.path[self.time_step + i] == blocking_robot.pos):
                        
                        if i == 1:  # Collision would happen in the next timestep
                            # Find an adjacent empty cell to move to
                            current_positions = {r.pos for r in self.robots}
                            for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                                new_x = blocking_robot.pos[0] + dx
                                new_y = blocking_robot.pos[1] + dy
                                new_pos = (new_x, new_y)
                                
                                if (0 <= new_x < GRID_SIZE and 0 <= new_y < GRID_SIZE and 
                                    new_pos not in current_positions):
                                    blocking_robot.pos = new_pos
                                    self.robots_waiting[blocking_robot.id] = self.time_step + 2
                                    print(f"Robot {blocking_robot.id} moved out of the way to {new_pos}")
                                    break
                            break
        
        self.time_step += 1
        
        if not any_robot_moving:
            self.generate_new_goals()
        
    def generate_new_goals(self):
        """Generate new random goals and replan"""
        occupied = {robot.pos for robot in self.robots}
        
        for robot in self.robots:
            # Find free positions
            free_positions = [(x, y) for x in range(GRID_SIZE) 
                            for y in range(GRID_SIZE) 
                            if (x, y) not in occupied]
            
            if free_positions:
                robot.start = robot.pos
                robot.goal = random.choice(free_positions)
                occupied.add(robot.goal)
        
        # Replan all paths
        self.plan_all_paths()
    
    def check_collisions(self):
        """Verify no collisions occurred (for debugging)"""
        positions = {}
        for robot in self.robots:
            if robot.pos in positions:
                print(f"COLLISION! Robots {positions[robot.pos]} and {robot.id} at {robot.pos}")
                return False
            positions[robot.pos] = robot.id
        return True
    
    def draw(self):
        """Draw the grid and robots"""
        self.screen.fill(WHITE)
        
        # Draw grid
        for x in range(GRID_SIZE):
            for y in range(GRID_SIZE):
                rect = pygame.Rect(x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE)
                pygame.draw.rect(self.screen, GRAY, rect, 1)
        
        # Draw goals
        for robot in self.robots:
            gx, gy = robot.goal
            center = (gx * CELL_SIZE + CELL_SIZE // 2, gy * CELL_SIZE + CELL_SIZE // 2)
            pygame.draw.circle(self.screen, robot.color, center, 8)
            pygame.draw.circle(self.screen, BLACK, center, 8, 2)
        
        # Draw future positions (faded)
        for robot in self.robots:
            if robot.path:
                for i in range(self.time_step, min(self.time_step + 5, len(robot.path))):
                    if i < len(robot.path):
                        px, py = robot.path[i]
                        center = (px * CELL_SIZE + CELL_SIZE // 2, 
                                py * CELL_SIZE + CELL_SIZE // 2)
                        # Faded color
                        fade = 100 + (i - self.time_step) * 30
                        color = tuple(min(255, c + fade) for c in robot.color)
                        pygame.draw.circle(self.screen, color, center, 5)
        
        # Draw robots
        for robot in self.robots:
            x, y = robot.pos
            center = (x * CELL_SIZE + CELL_SIZE // 2, y * CELL_SIZE + CELL_SIZE // 2)
            pygame.draw.circle(self.screen, robot.color, center, 18)
            pygame.draw.circle(self.screen, BLACK, center, 18, 2)
            
            # Draw robot ID
            font = pygame.font.Font(None, 24)
            text = font.render(str(robot.id), True, WHITE)
            text_rect = text.get_rect(center=center)
            self.screen.blit(text, text_rect)
        
        # Draw info
        font = pygame.font.Font(None, 20)
        info_text = f"Time: {self.time_step} | Space-Time A* with Priority Planning"
        text = font.render(info_text, True, BLACK)
        self.screen.blit(text, (10, 10))
        
        # Check and display collision status
        if self.check_collisions():
            status_text = "No Collisions"
            color = GREEN
        else:
            status_text = "COLLISION DETECTED!"
            color = RED
        
        text = font.render(status_text, True, color)
        self.screen.blit(text, (10, 30))
        
        pygame.display.flip()
    
    def run(self):
        """Main loop"""
        running = True
        paused = True
        
        print("MAPF with Proper Collision Detection")
        print("SPACE: Start/Pause | R: Replan | ESC: Exit")
        print("Using Space-Time A* with priority-based planning")
        
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_SPACE:
                        paused = not paused
                    elif event.key == pygame.K_r:
                        self.plan_all_paths()
            
            if not paused:
                self.move_robots()
            
            self.draw()
            self.clock.tick(FPS)
        
        pygame.quit()

if __name__ == "__main__":
    sim = MAPFSimulator()
    sim.run()