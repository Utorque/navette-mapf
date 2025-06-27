#!/usr/bin/env python3
"""
MAPF Simulation - Corridor + Rooms Environment
Main entry point for the Multi-Agent Path Finding simulation.
"""

import pygame
import sys
from simulator import MAPFSimulator
from renderer import MAPFRenderer

def main():
    """Main function to run the MAPF simulation."""
    # Initialize pygame
    pygame.init()
    
    # Create simulator and renderer
    simulator = MAPFSimulator()
    renderer = MAPFRenderer(simulator)
    
    # Main game loop
    clock = pygame.time.Clock()
    running = True
    
    print("MAPF Simulation Controls:")
    print("SPACE - Start/Pause simulation")
    print("R - Reset simulation")
    print("O - Add manual order")
    print("ESC - Exit")
    print()
    
    while running:
        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    simulator.toggle_simulation()
                elif event.key == pygame.K_r:
                    simulator.reset()
                elif event.key == pygame.K_o:
                    simulator.add_manual_order()
        
        # Update simulation
        simulator.update()
        
        # Render
        renderer.render()
        
        # Control frame rate
        clock.tick(60)  # 60 FPS for smooth animation
    
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()