"""
Pygame renderer for MAPF simulation visualization.
Handles all graphics and user interface rendering.
"""

import pygame
import pygame.font
from typing import Tuple, List
from environment import Position, RobotStatus
from simulator import MAPFSimulator

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (128, 128, 128)
LIGHT_GRAY = (220, 220, 220)
DARK_GRAY = (64, 64, 64)
GREEN = (0, 255, 0)
LIGHT_GREEN = (144, 238, 144)
BLUE = (0, 0, 255)
LIGHT_BLUE = (173, 216, 230)
RED = (255, 0, 0)
YELLOW = (255, 255, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165, 0)

# Robot colors
ROBOT_COLORS = {
    1: BLUE,
    2: RED
}

class MAPFRenderer:
    """Pygame renderer for the MAPF simulation."""
    
    def __init__(self, simulator: MAPFSimulator):
        self.simulator = simulator
        
        # Display settings
        self.window_width = 1200
        self.window_height = 800
        self.screen = pygame.display.set_mode((self.window_width, self.window_height))
        pygame.display.set_caption("MAPF Simulation - Corridor + Rooms")
        
        # Grid settings
        self.grid_start_x = 50
        self.grid_start_y = 50
        self.cell_width = 120
        self.cell_height = 80
        self.room_height = 100
        
        # Fonts
        pygame.font.init()
        self.font_large = pygame.font.Font(None, 24)
        self.font_medium = pygame.font.Font(None, 20)
        self.font_small = pygame.font.Font(None, 16)
        self.font_tiny = pygame.font.Font(None, 12)
    
    def render(self) -> None:
        """Main render method."""
        self.screen.fill(WHITE)
        
        # Draw grid
        self._draw_grid()
        
        # Draw robots
        self._draw_robots()
        
        # Draw path indicators
        self._draw_path_indicators()
        
        # Draw UI panels
        self._draw_status_panel()
        self._draw_orders_panel()
        self._draw_robot_status_panel()
        
        # Draw instructions
        self._draw_instructions()
        
        pygame.display.flip()
    
    def _draw_grid(self) -> None:
        """Draw the corridor and rooms grid."""
        # Draw rooms (top row)
        for col in range(5):
            x = self.grid_start_x + col * self.cell_width
            y = self.grid_start_y
            
            # Room background
            room_rect = pygame.Rect(x, y, self.cell_width, self.room_height)
            pygame.draw.rect(self.screen, LIGHT_GREEN, room_rect)
            pygame.draw.rect(self.screen, BLACK, room_rect, 3)
            
            # Room label
            room_name = self.simulator.env.rooms[col]
            text = self.font_medium.render(room_name, True, BLACK)
            text_rect = text.get_rect(center=(x + self.cell_width // 2, y + 20))
            self.screen.blit(text, text_rect)
        
        # Draw corridor (bottom row)
        for col in range(5):
            x = self.grid_start_x + col * self.cell_width
            y = self.grid_start_y + self.room_height
            
            # Corridor background
            corridor_rect = pygame.Rect(x, y, self.cell_width, self.cell_height)
            pygame.draw.rect(self.screen, LIGHT_BLUE, corridor_rect)
            pygame.draw.rect(self.screen, GRAY, corridor_rect, 2)
            
            # Position number
            text = self.font_small.render(str(col), True, DARK_GRAY)
            self.screen.blit(text, (x + 5, y + 5))
    
    def _draw_robots(self) -> None:
        """Draw robots on the grid."""
        for robot in self.simulator.robots:
            self._draw_robot(robot)
    
    def _draw_robot(self, robot) -> None:
        """Draw a single robot."""
        pos = robot.position
        x = self.grid_start_x + pos.col * self.cell_width + self.cell_width // 2
        
        if pos.row == 0:  # Room
            y = self.grid_start_y + self.room_height // 2
            radius = 20
        else:  # Corridor
            y = self.grid_start_y + self.room_height + self.cell_height // 2
            radius = 16
        
        # Robot circle
        color = ROBOT_COLORS.get(robot.id, BLACK)
        pygame.draw.circle(self.screen, color, (x, y), radius)
        pygame.draw.circle(self.screen, BLACK, (x, y), radius, 2)
        
        # Robot ID
        text = self.font_medium.render(f"R{robot.id}", True, WHITE)
        text_rect = text.get_rect(center=(x, y))
        self.screen.blit(text, text_rect)
        
        # Waiting indicator
        if robot.is_waiting:
            pygame.draw.circle(self.screen, YELLOW, (x + 15, y - 15), 6)
    
    def _draw_path_indicators(self) -> None:
        """Draw path step indicators for robots."""
        for robot in self.simulator.robots:
            if not robot.path or robot.status == RobotStatus.IDLE:
                continue
            
            remaining_path = robot.path[robot.path_index:]
            color = ROBOT_COLORS.get(robot.id, BLACK)
            
            for i, pos in enumerate(remaining_path):
                # Skip current position
                if pos == robot.position:
                    continue
                
                x = self.grid_start_x + pos.col * self.cell_width + self.cell_width - 20
                
                if pos.row == 0:  # Room
                    y = self.grid_start_y + self.room_height - 20
                else:  # Corridor
                    y = self.grid_start_y + self.room_height + self.cell_height - 20
                
                # Path step circle
                pygame.draw.circle(self.screen, color, (x, y), 8)
                pygame.draw.circle(self.screen, BLACK, (x, y), 8, 1)
                
                # Step number
                step_text = self.font_tiny.render(str(i + 1), True, WHITE)
                step_rect = step_text.get_rect(center=(x, y))
                self.screen.blit(step_text, step_rect)
    
    def _draw_status_panel(self) -> None:
        """Draw the main status panel."""
        panel_x = self.grid_start_x + 5 * self.cell_width + 50
        panel_y = self.grid_start_y
        panel_width = 300
        
        # Time elapsed
        self._draw_status_box("Time Elapsed", f"{self.simulator.time_elapsed}s", 
                             panel_x, panel_y, panel_width, LIGHT_BLUE)
        
        # Active orders
        active_orders = len(self.simulator.order_manager.get_pending_orders()) + \
                       len(self.simulator.order_manager.get_assigned_orders())
        self._draw_status_box("Active Orders", str(active_orders), 
                             panel_x, panel_y + 80, panel_width, (255, 255, 200))
        
        # Completed orders
        completed_orders = len(self.simulator.order_manager.completed_orders)
        self._draw_status_box("Completed Orders", str(completed_orders), 
                             panel_x, panel_y + 160, panel_width, LIGHT_GREEN)
    
    def _draw_status_box(self, title: str, value: str, x: int, y: int, 
                        width: int, color: Tuple[int, int, int]) -> None:
        """Draw a status box with title and value."""
        height = 60
        
        # Background
        rect = pygame.Rect(x, y, width, height)
        pygame.draw.rect(self.screen, color, rect)
        pygame.draw.rect(self.screen, BLACK, rect, 2)
        
        # Title
        title_text = self.font_medium.render(title, True, BLACK)
        self.screen.blit(title_text, (x + 10, y + 10))
        
        # Value
        value_text = self.font_large.render(value, True, BLACK)
        self.screen.blit(value_text, (x + 10, y + 30))
    
    def _draw_orders_panel(self) -> None:
        """Draw the orders panel."""
        panel_x = self.grid_start_x
        panel_y = self.grid_start_y + self.room_height + self.cell_height + 50
        panel_width = 400
        panel_height = 200
        
        # Panel background
        panel_rect = pygame.Rect(panel_x, panel_y, panel_width, panel_height)
        pygame.draw.rect(self.screen, LIGHT_GRAY, panel_rect)
        pygame.draw.rect(self.screen, BLACK, panel_rect, 2)
        
        # Title
        title_text = self.font_medium.render("Current Orders", True, BLACK)
        self.screen.blit(title_text, (panel_x + 10, panel_y + 10))
        
        # Orders list
        y_offset = 40
        all_orders = (self.simulator.order_manager.get_pending_orders() + 
                     self.simulator.order_manager.get_assigned_orders())
        
        if not all_orders:
            no_orders_text = self.font_small.render("No active orders", True, GRAY)
            self.screen.blit(no_orders_text, (panel_x + 10, panel_y + y_offset))
        else:
            for i, order in enumerate(all_orders[:5]):  # Show max 5 orders
                order_y = panel_y + y_offset + i * 30
                
                # Order info
                order_text = f"#{order.id}: {order.from_room} → {order.to_room}"
                text = self.font_small.render(order_text, True, BLACK)
                self.screen.blit(text, (panel_x + 10, order_y))
                
                # Status
                if order.assigned_robot:
                    status_text = f"Robot {order.assigned_robot}"
                    status_color = ROBOT_COLORS.get(order.assigned_robot, BLACK)
                else:
                    status_text = "Pending"
                    status_color = ORANGE
                
                status = self.font_tiny.render(status_text, True, status_color)
                self.screen.blit(status, (panel_x + 250, order_y))
    
    def _draw_robot_status_panel(self) -> None:
        """Draw the robot status panel."""
        panel_x = self.grid_start_x + 450
        panel_y = self.grid_start_y + self.room_height + self.cell_height + 50
        panel_width = 300
        panel_height = 200
        
        # Panel background
        panel_rect = pygame.Rect(panel_x, panel_y, panel_width, panel_height)
        pygame.draw.rect(self.screen, LIGHT_GRAY, panel_rect)
        pygame.draw.rect(self.screen, BLACK, panel_rect, 2)
        
        # Title
        title_text = self.font_medium.render("Robot Status", True, BLACK)
        self.screen.blit(title_text, (panel_x + 10, panel_y + 10))
        
        # Robot info
        for i, robot in enumerate(self.simulator.robots):
            robot_y = panel_y + 40 + i * 60
            
            # Robot color indicator
            color = ROBOT_COLORS.get(robot.id, BLACK)
            pygame.draw.circle(self.screen, color, (panel_x + 20, robot_y + 10), 8)
            
            # Robot info
            robot_text = f"Robot {robot.id} (Priority {robot.priority})"
            text = self.font_small.render(robot_text, True, BLACK)
            self.screen.blit(text, (panel_x + 35, robot_y))
            
            # Location
            if robot.position.row == 0:
                location = f"Room {self.simulator.env.rooms[robot.position.col]}"
            else:
                location = f"Corridor {robot.position.col}"
            
            location_text = self.font_tiny.render(f"Location: {location}", True, BLACK)
            self.screen.blit(location_text, (panel_x + 35, robot_y + 15))
            
            # Status
            if robot.status == RobotStatus.IDLE:
                status = "Idle"
            elif robot.is_waiting:
                status = "Waiting"
            else:
                status = "Moving"
            
            status_text = self.font_tiny.render(f"Status: {status}", True, BLACK)
            self.screen.blit(status_text, (panel_x + 35, robot_y + 30))
            
            # Remaining steps
            if robot.path:
                remaining = len(robot.path) - robot.path_index
                steps_text = self.font_tiny.render(f"Steps left: {remaining}", True, BLACK)
                self.screen.blit(steps_text, (panel_x + 150, robot_y + 30))
    
    def _draw_instructions(self) -> None:
        """Draw control instructions."""
        instructions = [
            "Controls:",
            "SPACE - Start/Pause simulation",
            "R - Reset simulation", 
            "O - Add manual order",
            "ESC - Exit"
        ]
        
        start_y = self.window_height - 120
        for i, instruction in enumerate(instructions):
            color = BLACK if i == 0 else DARK_GRAY
            font = self.font_small if i == 0 else self.font_tiny
            text = font.render(instruction, True, color)
            self.screen.blit(text, (10, start_y + i * 15))