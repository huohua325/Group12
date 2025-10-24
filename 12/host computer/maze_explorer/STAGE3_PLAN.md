# Stage 3: Wall Detection and Motion Control

## Prerequisites

- ✅ Stage 1 Complete: BLE communication working
- ✅ Stage 2 Complete: SLAM map building working
- ✅ Can stably receive 500-point radar data
- ✅ Can save SLAM maps

---

## Goals

Implement radar-based wall detection and precise motion control for maze exploration preparation.

---

## Task List

### Task 3.1: Wall Detection Algorithm

**File**: `maze_explorer/perception/wall_detector.py`

**Function**: Detect surrounding walls based on 500-point radar data

**Principle**:
```
Maze cell: 600mm × 600mm
Robot position: Cell center
Detection basis: Radar ranging

Front (0°):     index 0 vicinity
Right (90°):    index 125 vicinity  
Back (180°):    index 250 vicinity
Left (270°):    index 375 vicinity

Wall threshold: ~300-400mm (half cell size)
```

**Code Structure**:
```python
class WallDetector:
    def __init__(self, cell_size=600, wall_threshold=400):
        """
        Args:
            cell_size: Cell size (mm)
            wall_threshold: Wall detection threshold (mm)
        """
        self.cell_size = cell_size
        self.threshold = wall_threshold
    
    def detect(self, distances_mm):
        """
        Detect surrounding walls
        
        Args:
            distances_mm: 500 distance values (mm)
        
        Returns:
            dict: {
                'front': bool,   # Wall in front
                'right': bool,   # Wall on right
                'back': bool,    # Wall behind
                'left': bool     # Wall on left
            }
        """
        # Front: index 0 vicinity (-10 to +10 degrees)
        front_indices = list(range(0, 14)) + list(range(486, 500))
        front_dists = [distances_mm[i] for i in front_indices if distances_mm[i] > 0]
        has_front_wall = min(front_dists) < self.threshold if front_dists else False
        
        # Right: index 125 vicinity (90° ±10 degrees)
        right_indices = range(111, 139)  # 80-100 degrees
        right_dists = [distances_mm[i] for i in right_indices if distances_mm[i] > 0]
        has_right_wall = min(right_dists) < self.threshold if right_dists else False
        
        # Back: index 250 vicinity (180° ±10 degrees)
        back_indices = range(236, 264)  # 170-190 degrees
        back_dists = [distances_mm[i] for i in back_indices if distances_mm[i] > 0]
        has_back_wall = min(back_dists) < self.threshold if back_dists else False
        
        # Left: index 375 vicinity (270° ±10 degrees)
        left_indices = range(361, 389)  # 260-280 degrees
        left_dists = [distances_mm[i] for i in left_indices if distances_mm[i] > 0]
        has_left_wall = min(left_dists) < self.threshold if left_dists else False
        
        return {
            'front': has_front_wall,
            'right': has_right_wall,
            'back': has_back_wall,
            'left': has_left_wall
        }
```

---

### Task 3.2: Motion Control Encapsulation

**File**: `maze_explorer/control/motion_controller.py`

**Function**: Encapsulate precise motion control

**Code Structure**:
```python
class MotionController:
    def __init__(self, comm, forward_time=2.0, turn_time=1.5):
        """
        Args:
            comm: BLERobotComm instance
            forward_time: Time needed to move forward 600mm (seconds)
            turn_time: Time needed to turn 90 degrees (seconds)
        """
        self.comm = comm
        self.forward_time = forward_time
        self.turn_time = turn_time
    
    def move_forward(self):
        """Move forward one cell (600mm)"""
        print("[MOTION] Moving forward one cell...")
        self.comm.forward()
        time.sleep(self.forward_time)
        self.comm.stop()
        time.sleep(0.5)  # Stabilization time
        print("[MOTION] Forward movement complete")
    
    def turn_left_90(self):
        """Turn left 90 degrees"""
        print("[MOTION] Turning left 90 degrees...")
        self.comm.turn_left()
        time.sleep(self.turn_time)
        self.comm.stop()
        time.sleep(0.5)
        print("[MOTION] Left turn complete")
    
    def turn_right_90(self):
        """Turn right 90 degrees"""
        print("[MOTION] Turning right 90 degrees...")
        self.comm.turn_right()
        time.sleep(self.turn_time)
        self.comm.stop()
        time.sleep(0.5)
        print("[MOTION] Right turn complete")
    
    def turn_180(self):
        """Turn around (180 degrees)"""
        print("[MOTION] Turning around...")
        self.turn_left_90()
        self.turn_left_90()
        print("[MOTION] Turn around complete")
```

---

### Task 3.3: Maze Grid Representation

**File**: `maze_explorer/strategy/maze_map.py`

**Function**: Represent 4×4 maze grid and wall information

**Code Structure**:
```python
class MazeCell:
    """Maze cell"""
    def __init__(self, row, col):
        self.row = row
        self.col = col
        self.walls = {
            'north': None,  # None=unknown, True=wall exists, False=no wall
            'east': None,
            'south': None,
            'west': None
        }
        self.visited = False

class MazeMap:
    """4×4 maze map"""
    def __init__(self, rows=4, cols=4):
        self.rows = rows
        self.cols = cols
        self.cells = [[MazeCell(r, c) for c in range(cols)] for r in range(rows)]
        
        # Robot current position
        self.robot_row = 0
        self.robot_col = 0
        self.robot_facing = 'north'  # north, east, south, west
    
    def update_walls(self, wall_detection):
        """
        Update wall information at current position
        
        Args:
            wall_detection: dict from WallDetector
                {'front': bool, 'right': bool, 'back': bool, 'left': bool}
        """
        cell = self.cells[self.robot_row][self.robot_col]
        
        # Convert directions based on orientation
        if self.robot_facing == 'north':
            cell.walls['north'] = wall_detection['front']
            cell.walls['east'] = wall_detection['right']
            cell.walls['south'] = wall_detection['back']
            cell.walls['west'] = wall_detection['left']
        # ... other orientation conversions
        
        cell.visited = True
    
    def get_available_directions(self):
        """Get feasible directions at current position"""
        cell = self.cells[self.robot_row][self.robot_col]
        available = []
        
        if not cell.walls['north']:
            available.append('north')
        if not cell.walls['east']:
            available.append('east')
        if not cell.walls['south']:
            available.append('south')
        if not cell.walls['west']:
            available.append('west')
        
        return available
    
    def print_map(self):
        """Print maze map (ASCII art)"""
        # Simple ASCII representation
        pass
```

---

### Task 3.4: Integration Testing

**File**: `maze_explorer/test_wall_detection.py`

**Function**: Test wall detection

```python
#!/usr/bin/env python3
"""Test wall detection"""

from communication.ble_comm import BLERobotComm
from perception.wall_detector import WallDetector
import config

def test_wall_detection():
    """Test wall detection"""
    detector = WallDetector(
        cell_size=config.CELL_SIZE,
        wall_threshold=350  # 350mm threshold
    )
    
    comm = BLERobotComm(config.BLE_ADDRESS)
    
    def on_lidar(data):
        distances = data['distances_mm']
        walls = detector.detect(distances)
        
        print(f"\nWall detection:")
        print(f"  Front: {'Wall' if walls['front'] else 'No wall'}")
        print(f"  Right: {'Wall' if walls['right'] else 'No wall'}")
        print(f"  Back: {'Wall' if walls['back'] else 'No wall'}")
        print(f"  Left: {'Wall' if walls['left'] else 'No wall'}")
    
    comm.on_lidar_frame = on_lidar
    
    if comm.connect():
        print("Receiving data (10 seconds)...")
        time.sleep(10)
        comm.disconnect()

if __name__ == '__main__':
    test_wall_detection()
```

---

## Acceptance Criteria

### Task 3.1: Wall Detection
- [ ] Can accurately detect front wall
- [ ] Can accurately detect right wall
- [ ] Can accurately detect back wall
- [ ] Can accurately detect left wall
- [ ] Threshold setting reasonable (300-400mm)

### Task 3.2: Motion Control
- [ ] Can precisely move forward 600mm
- [ ] Can precisely turn left 90 degrees
- [ ] Can precisely turn right 90 degrees
- [ ] Robot stops stably after action completion

### Task 3.3: Maze Map
- [ ] Can represent 4×4 cells
- [ ] Can record wall information
- [ ] Can record explored areas
- [ ] Can track robot position and orientation

---

## Development Order

1. First implement wall detection (based on radar data)
2. Then implement motion control (based on MODE commands)
3. Finally implement maze map (integrate information)

---

## Important Parameters

```python
# Cell size
CELL_SIZE = 600  # mm

# Wall threshold (needs actual measurement adjustment)
WALL_THRESHOLD = 350  # mm

# Motion time (needs actual measurement adjustment)
FORWARD_TIME = 2.0  # seconds
TURN_TIME = 1.5     # seconds

# Angle range (for wall detection)
ANGLE_TOLERANCE = 10  # degrees (±10 degree range)
```

---

**Ready to start Stage 3 development!**

