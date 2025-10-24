# Stage 3: Wall Detection and Wall Following

## Overview

Stage 3 implements wall detection and wall-following control based on lidar data, laying the foundation for maze exploration.

---

## Functional Modules

### 1. Wall Detection (`perception/wall_detector.py`)

**Function**: Identify walls in all directions based on 500-point lidar data

**Principle**:
- Lidar 500 points, ~0.72 degrees per point
- Front (0°): around index 0
- Right (90°): around index 125
- Back (180°): around index 250
- Left (270°): around index 375
- Wall detection threshold: 400mm (configurable)

**Usage Example**:
```python
from perception.wall_detector import WallDetector

# Create detector (threshold 400mm)
detector = WallDetector(cell_size=600, wall_threshold=400)

# Detect walls
walls = detector.detect(distances_mm)  # distances_mm is 500 distance values

# Return result
# {
#     'front': bool,       # Wall in front
#     'right': bool,       # Wall on right
#     'back': bool,        # Wall behind
#     'left': bool,        # Wall on left
#     'front_dist': float, # Minimum distance in front (mm)
#     'right_dist': float, # Minimum distance on right (mm)
#     'back_dist': float,  # Minimum distance behind (mm)
#     'left_dist': float   # Minimum distance on left (mm)
# }
```

---

### 2. Motion Control (`control/motion_controller.py`)

**Function**: Encapsulate precise motion control for the robot

**MotionController Class**:
```python
from control.motion_controller import MotionController

# Create controller
motion = MotionController(comm, forward_time=2.0, turn_time=1.5)

# Basic actions
motion.move_forward(cells=1)  # Move forward 1 cell (600mm)
motion.turn_left_90()         # Turn left 90 degrees
motion.turn_right_90()        # Turn right 90 degrees
motion.turn_180()             # Turn around
motion.stop()                 # Emergency stop
```

**WallFollower Class**:
```python
from control.motion_controller import WallFollower

# Create wall-following controller
follower = WallFollower(motion)

# Right-hand rule wall following
action = follower.follow_right_wall(walls)
# Returns: 'turn_right', 'forward', 'turn_left', or 'turn_around'

# Left-hand rule wall following
action = follower.follow_left_wall(walls)
```

---

## Test Scripts

### 1. Wall Detection Test (`test_wall_detection.py`)

**Function**: Test lidar data reception and wall recognition

**Running Method**:
```bash
cd maze_explorer
python test_wall_detection.py --time 30
```

**Parameters**:
- `-t, --time`: Test duration (seconds), default 30 seconds

**Output Example**:
```
[Frame #10] Wall detection results:
  Front: Wall (250mm) | Right: Open (800mm) | Back: Wall (300mm) | Left: Wall (280mm)

  Visualization (facing: front):
      ███
   ███  Car    
      ███
```

---

### 2. Wall Following Main Program (`main_wall_following.py`)

**Function**: Use right-hand rule or left-hand rule for maze exploration

**Running Method**:
```bash
cd maze_explorer

# Right-hand rule (default)
python main_wall_following.py --method right --steps 50

# Left-hand rule
python main_wall_following.py --method left --steps 50
```

**Parameters**:
- `-m, --method`: Wall-following method, `right` (right-hand rule) or `left` (left-hand rule)
- `-s, --steps`: Maximum steps, default 50

**Running Process**:
1. Connect to robot
2. Wait for lidar data to stabilize
3. Loop execution of wall-following strategy:
   - Detect walls in all directions
   - Select action based on strategy
   - Execute action
4. Reach maximum steps or manual interruption

---

## Preparation Before Use

### 1. Configure BLE Address

Edit `config.py`:
```python
BLE_ADDRESS = 'XX:XX:XX:XX:XX:XX'  # Replace with your robot's BLE address
```

### 2. Calibrate Motion Parameters

Edit `config.py`:
```python
FORWARD_TIME = 2.0   # Time to move forward 600mm (seconds), needs actual measurement adjustment
TURN_TIME = 1.5      # Time to turn 90 degrees (seconds), needs actual measurement adjustment
```

**Calibration Method**:
1. Place robot on flat ground
2. Measure actual forward distance (should be 600mm)
3. Measure actual turn angle (should be 90 degrees)
4. Adjust parameters until movement is accurate

### 3. Adjust Wall Detection Threshold

Edit parameters in `config.py` or code:
```python
# Wall detection threshold (mm)
# - For 600mm cells: recommend 300-400mm
# - For larger cells: adjust proportionally
wall_threshold = 400
```

---

## Wall Following Strategy Description

### Right-Hand Rule

**Decision Priority**:
1. **No wall on right** → Turn right and move forward
2. **No wall in front** → Move straight
3. **No wall on left** → Turn left and move forward
4. **Walls all around** → Turn around

**Characteristics**:
- Can guarantee traversal of all connected areas
- Suitable for simple mazes
- Easy to get stuck in loops

### Left-Hand Rule

**Decision Priority**:
1. **No wall on left** → Turn left and move forward
2. **No wall in front** → Move straight
3. **No wall on right** → Turn right and move forward
4. **Walls all around** → Turn around

**Characteristics**:
- Mirror symmetric with right-hand rule
- Choice depends on maze entrance position

---

## Known Limitations

1. **Motion Precision**:
   - Time-driven (not encoder closed-loop)
   - Accumulated error increases with steps
   - Requires regular calibration

2. **Wall Detection**:
   - Depends on lidar data quality
   - May be affected by environmental interference
   - Threshold needs environment-specific adjustment

3. **Wall Following Strategy**:
   - Simple strategy, may not be optimal path
   - Does not remember visited positions
   - May repeat in complex mazes

---

## Next Development Steps

- [ ] Integrate pose estimation (encoder + MPU)
- [ ] Implement maze map memory
- [ ] Improve path planning (A* algorithm)
- [ ] Add SLAM map integration with wall following

---

## Debugging Tips

### 1. View Raw Lidar Data

Run test script and observe distance values:
```bash
python test_wall_detection.py --time 60
```

### 2. Adjust Wall Threshold

Adjust threshold based on actual distance:
```python
# If misdetecting walls → Increase threshold
detector = WallDetector(wall_threshold=500)

# If missing walls → Decrease threshold
detector = WallDetector(wall_threshold=300)
```

### 3. Single-Step Motion Debugging

Add breakpoints or input confirmation in `main_wall_following.py`:
```python
# Wait for user confirmation each step
input("Press Enter to execute next step...")
```

---

## Troubleshooting

| Problem | Possible Cause | Solution |
|---------|----------------|----------|
| Connection Failed | Wrong BLE Address | Check `BLE_ADDRESS` in `config.py` |
| No Lidar Data | Communication Error | Restart robot, check Bluetooth connection |
| Inaccurate Wall Detection | Unsuitable Threshold | Adjust `wall_threshold` parameter |
| Inaccurate Movement Distance | Uncalibrated Parameters | Calibrate `FORWARD_TIME` and `TURN_TIME` |
| Robot Not Moving | Commands Not Effective | Check STM32 MODE command handling |

---

## Reference Documentation

- [Communication Protocol Description](../xxq/docs/硬件通信协议详细说明.md)
- [Lidar Usage Plan](../xxq/docs/激光雷达使用方案.md)
- [Configuration Management Specification](config.py)

