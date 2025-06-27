"""
Orders management for MAPF simulation.
Handles order generation, assignment, and tracking.
"""

import random
from typing import List, Optional
from environment import Environment, Order, Robot, OrderStatus, Position

class OrderManager:
    """Manages orders for the MAPF simulation."""
    
    def __init__(self, environment: Environment):
        self.env = environment
        self.orders: List[Order] = []
        self.completed_orders: List[Order] = []
        self.next_order_id = 1
    
    def generate_random_order(self, current_time: int) -> Order:
        """Generate a random order between two different rooms."""
        available_rooms = self.env.rooms.copy()
        from_room = random.choice(available_rooms)
        
        # Ensure to_room is different from from_room
        available_rooms.remove(from_room)
        to_room = random.choice(available_rooms)
        
        order = Order(
            id=self.next_order_id,
            from_room=from_room,
            to_room=to_room,
            request_time=current_time,
            status=OrderStatus.PENDING
        )
        
        self.orders.append(order)
        self.next_order_id += 1
        
        return order
    
    def add_manual_order(self, current_time: int) -> Order:
        """Add a manual order (wrapper for generate_random_order)."""
        return self.generate_random_order(current_time)
    
    def get_pending_orders(self) -> List[Order]:
        """Get all pending orders."""
        return [order for order in self.orders if order.status == OrderStatus.PENDING]
    
    def get_assigned_orders(self) -> List[Order]:
        """Get all assigned orders."""
        return [order for order in self.orders if order.status == OrderStatus.ASSIGNED]
    
    def assign_order_to_robot(self, order: Order, robot: Robot, current_time: int) -> None:
        """Assign an order to a robot."""
        order.status = OrderStatus.ASSIGNED
        order.assigned_robot = robot.id
        order.start_time = current_time
    
    def complete_order(self, order: Order, current_time: int) -> None:
        """Mark an order as completed."""
        if order.request_time is not None:
            order.completion_time = current_time - order.request_time
        order.status = OrderStatus.COMPLETED
        
        # Move to completed orders and remove from active orders
        self.completed_orders.append(order)
        self.orders.remove(order)
    
    def find_order_by_robot(self, robot_id: int) -> Optional[Order]:
        """Find the order assigned to a specific robot."""
        for order in self.orders:
            if order.assigned_robot == robot_id and order.status == OrderStatus.ASSIGNED:
                return order
        return None
    
    def get_best_robot_for_order(self, order: Order, available_robots: List[Robot]) -> Optional[Robot]:
        """
        Find the best robot for an order based on distance to pickup location.
        
        Args:
            order: The order to assign
            available_robots: List of available robots
            
        Returns:
            Best robot for the order, or None if no suitable robot found
        """
        if not available_robots:
            return None
        
        from_room_pos = self.env.find_room_position(order.from_room)
        if not from_room_pos:
            return None
        
        # Calculate distances for each robot
        robot_distances = []
        for robot in available_robots:
            # Distance = steps to get under pickup room + steps to enter room
            corridor_distance = abs(robot.col - from_room_pos.col)
            room_access_distance = 1 if robot.row == 1 else 2  # Already in corridor vs in room
            total_distance = corridor_distance + room_access_distance
            
            robot_distances.append((robot, total_distance))
        
        # Sort by distance, then by priority (lower priority number = higher priority)
        robot_distances.sort(key=lambda x: (x[1], x[0].priority))
        
        return robot_distances[0][0]
    
    def reset(self) -> None:
        """Reset all orders."""
        self.orders.clear()
        self.completed_orders.clear()
        self.next_order_id = 1
    
    def get_statistics(self) -> dict:
        """Get order statistics."""
        total_orders = len(self.completed_orders)
        if total_orders == 0:
            return {
                'total_completed': 0,
                'average_completion_time': 0,
                'pending_orders': len(self.get_pending_orders()),
                'assigned_orders': len(self.get_assigned_orders())
            }
        
        total_completion_time = sum(
            order.completion_time for order in self.completed_orders 
            if order.completion_time is not None
        )
        
        return {
            'total_completed': total_orders,
            'average_completion_time': total_completion_time / total_orders if total_orders > 0 else 0,
            'pending_orders': len(self.get_pending_orders()),
            'assigned_orders': len(self.get_assigned_orders())
        }