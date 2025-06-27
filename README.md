# MAPF Simulation - Corridor + Rooms Environment

A Python implementation of Multi-Agent Path Finding (MAPF) simulation using Space-Time A* with Prioritized Planning. This simulation features two robots operating in a corridor-and-rooms environment, handling pickup and delivery orders while avoiding collisions.

## Features

- **Space-Time A* Pathfinding**: Advanced collision avoidance with vertex and edge conflict detection
- **Prioritized Planning**: Lower priority robots wait for higher priority ones during conflicts
- **Real-time Visualization**: Pygame-based graphical interface showing robots, paths, and orders
- **Order Management**: Automatic random order generation and manual order addition
- **Complete Collision Avoidance**: No same-position conflicts or robot collisions

## Environment Layout

```
┌────┬────┬────┬────┬────┐
│ in │ A  │ B  │ C  │out│  ← Rooms (pickup/dropoff)
├────┼────┼────┼────┼────┤
│ 0  │ 1  │ 2  │ 3  │ 4 │  ← Corridor (movement)
└────┴────┴────┴────┴────┘
```

- **Top Row (Green)**: Rooms - pickup and dropoff locations
- **Bottom Row (Blue)**: Corridor - movement pathway for robots
- **Movement**: Robots can only move horizontally in corridor and vertically to access rooms
- **Example Path (in→B)**: Up to "in" room → Down to corridor → Right 2 steps → Up to "B" room

## Installation

1. Ensure you have Python 3.7+ installed
2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

## Running the Simulation

```bash
python main.py
```

## Controls

- **SPACE**: Start/Pause simulation
- **R**: Reset simulation to initial state
- **O**: Add a manual random order
- **ESC**: Exit simulation

## File Structure

```
mapf_simulation/
├── main.py             # Entry point and main loop
├── environment.py      # Grid environment and data structures
├── solver.py           # Space-Time A* implementation
├── orders.py           # Order management and assignment
├── simulator.py        # Main simulation logic
├── renderer.py         # Pygame visualization
├── requirements.txt    # Python dependencies
└── README.md           # This file
```

## Architecture

### Core Components

1. **Environment**: Manages the 5×2 grid layout with rooms and corridor
2. **MAPFSolver**: Implements Space-Time A* with collision avoidance
3. **OrderManager**: Handles order generation, assignment, and completion tracking
4. **MAPFSimulator**: Main simulation loop coordinating all components
5. **MAPFRenderer**: Pygame-based visualization system

### Collision Avoidance

The simulation uses Space-Time A* with these collision types:

- **Vertex Conflicts**: Two robots at same position at same time
- **Edge Conflicts**: Two robots swapping positions (crossing paths)
- **Priority Resolution**: Higher priority robots (lower number) move first

### Robot Behavior

- **Robot 1**: Blue, Priority 1 (higher priority)
- **Robot 2**: Red, Priority 2 (lower priority)
- **Assignment**: Orders assigned to closest available robot
- **Pathfinding**: Two-stage planning (current→pickup→destination)

## Simulation Features

- **Real-time Statistics**: Time elapsed, active orders, completed orders
- **Visual Path Indicators**: Shows planned robot paths with step numbers
- **Waiting Indicators**: Yellow dots show robots waiting to avoid collisions
- **Order Tracking**: Complete order lifecycle from generation to completion
- **Automatic Order Generation**: Random orders generated during simulation

## Technical Details

- **Update Rate**: 1 second per simulation step
- **Path Planning**: A* with Manhattan distance heuristic
- **Priority System**: Deterministic conflict resolution
- **Maximum Search Time**: 50 timesteps to prevent infinite search
- **Random Order Rate**: 10% chance per second

## Customization

You can easily modify:

- Number of robots in `simulator.py`
- Grid dimensions in `environment.py`
- Robot priorities and starting positions
- Order generation rate and patterns
- Visualization colors and layout in `renderer.py`

## Dependencies

- **pygame**: Graphics and user interface
- **Python Standard Library**: All other functionality uses built-in modules

## License

This project is open source. Feel free to modify and distribute.