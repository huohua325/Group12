# Maze Explorer - Maze Exploration System

Intelligent maze exploration system based on RPLIDAR C1 lidar and BLE communication.

---

## Project Status

- [x] **Stage 1**: BLE Communication and Basic Control ✓
- [x] **Stage 2**: SLAM Map Construction ✓
- [x] **Stage 3**: Wall Detection and Wall Following (Enhanced) ✓
  - [x] Sequence number command-response mechanism
  - [x] Exit detection
  - [x] Path tracking and return
- [ ] **Stage 4**: Maze memory and path optimization

---

## Quick Start

### Step 1: Configure BLE Address
Edit `config.py`:
```python
BLE_ADDRESS = 'C4:25:01:20:02:8E'  # Replace with your robot address
```

### Step 2: Flash STM32 Firmware
```bash
cd xxq
# Compile and flash using STM32CubeIDE
```

### Step 3: Step-by-Step Testing

#### Step 1: Command ACK Test (Required) ⭐⭐⭐
```bash
cd maze_explorer
python test_command_ack.py
```

#### Step 2: Wall Detection Test (Required) ⭐⭐
```bash
python test_wall_detection_enhanced.py --time 30
```

#### Step 3: Complete Exploration
```bash
python main_maze_exploration.py --method right --steps 100
```

**Detailed Instructions**: See [QUICKSTART.md](QUICKSTART.md)

---

## Core Features

### 1. Reliable Command-Response Mechanism

**New CMD Command Format**: `CMD,seq,mode\n`
- seq: Sequence number (0-65535 cyclic)
- mode: 0=stop, 1=forward, 2=backward, 3=turn left, 4=turn right

**ACK Response Format**: `ACK,seq,status,time\n`
- status: OK or TIMEOUT
- time: Action duration

**Features**:
- Sequence number protection, avoiding command confusion
- Dual timeout protection (STM32 35s + Python configurable)
- Clear action completion feedback

### 2. Wall Detection

Based on 500-point lidar data to identify walls in all directions:
- Front (0°): around index 0
- Right (90°): around index 125
- Back (180°): around index 250
- Left (270°): around index 375
- Wall threshold: 400mm (configurable)

### 3. Exit Detection

Automatically detect wall openings (distance >1500mm)

### 4. Path Tracking and Return

- Record exploration path (position + orientation)
- Calculate return path (reverse sequence)
- Automatically return to starting point after exploration

---

## Hardware Requirements

- STM32F446 Smart Robot
- RPLIDAR C1 360-degree Laser Lidar
- HC-04 BLE Bluetooth Module
- Motors and Encoders

---

## Software Dependencies

```bash
pip install -r requirements.txt
```

Main dependencies:
- `bleak` - BLE communication
- `numpy` - Data processing
- `breezyslam` - SLAM mapping (optional)

---

## Usage Examples

### Example 1: Lidar Data Reception
```python
from communication.ble_comm import BLERobotComm

comm = BLERobotComm('C4:25:01:20:02:8E', verbose=True)

def on_lidar(data):
    distances = data['distances_mm']  # 500 distance values (mm)
    print(f"Received lidar frame: {sum(1 for d in distances if d > 0)}/500 valid points")

comm.on_lidar_frame = on_lidar
comm.connect()
```

### Example 2: Wall Detection
```python
from perception.wall_detector import WallDetector

detector = WallDetector(wall_threshold=400)
walls = detector.detect(distances_mm)
# Returns: {'front': bool, 'right': bool, 'back': bool, 'left': bool, ...}
```

### Example 3: Motion Control (with ACK)
```python
from control.motion_controller import MotionController

motion = MotionController(comm, timeout=10.0)

# All methods wait for action completion
if motion.move_forward():
    print("Forward successful")
    
if motion.turn_left_90():
    print("Turn left successful")
```

### Example 4: Complete Exploration
```python
from control.motion_controller import WallFollower
from strategy.path_tracker import PathTracker

follower = WallFollower(motion)
tracker = PathTracker()

# Execute wall following
action = follower.follow_right_wall(walls)
tracker.record_action(action)

# Return to starting point
return_path = tracker.get_return_path()
```

---

## Test Scripts

| Script | Function | Runtime |
|--------|----------|----------|
| `test_command_ack.py` | Command ACK test | 5 minutes |
| `test_wall_detection_enhanced.py` | Wall detection test | 30 seconds |
| `main_maze_exploration.py` | Complete exploration | 5-10 minutes |
| `test_slam_simple.py` | SLAM test | 1 minute |
| `tests/test_stage3.py` | Basic unit tests | 1 second |
| `tests/test_stage3_enhanced.py` | Enhanced unit tests | 1 second |

---

## Project Structure

```
maze_explorer/
├── communication/         # Communication module
│   └── ble_comm.py       # BLE communication (supports ACK and sequence numbers)
├── control/              # Control module
│   └── motion_controller.py  # Motion control (rewritten version)
├── perception/           # Perception module
│   ├── wall_detector.py  # Wall detection
│   └── exit_detector.py  # Exit detection
├── strategy/             # Strategy module
│   └── path_tracker.py   # Path tracking
├── slam/                 # SLAM module
│   └── breezy_slam.py    # BreezySLAM integration
├── tests/                # Unit tests
│   ├── test_stage3.py
│   └── test_stage3_enhanced.py
├── test_command_ack.py           # Command ACK test
├── test_wall_detection_enhanced.py  # Wall detection test
├── main_maze_exploration.py      # Complete exploration main program
├── config.py                     # Configuration file
├── QUICKSTART.md                 # Quick start guide
├── TEST_PROCEDURE.md             # Detailed test procedure
└── IMPLEMENTATION_COMPLETE.md    # Implementation completion report
```

---

## Configuration Parameters

Configure in `config.py`:

```python
# BLE Configuration
BLE_ADDRESS = 'C4:25:01:20:02:8E'

# Motion Parameters (Motor_Command API controls automatically)
FORWARD_TIME = 2.5   # Expected forward time (seconds), for display only
TURN_TIME = 1.8      # Expected turn time (seconds), for display only

# Detection Parameters
CELL_SIZE = 600           # Cell size (mm)
WALL_THRESHOLD = 400      # Wall detection threshold (mm)
EXIT_THRESHOLD = 1500     # Exit detection threshold (mm)
```

---

## Communication Protocol (Stage 3 Enhanced)

### Downstream Commands (Python → STM32)

**CMD Command**: `CMD,seq,mode\n`
- seq: Sequence number (0-65535)
- mode: 0=stop, 1=forward, 2=backward, 3=turn left, 4=turn right

**Example**: `CMD,15,1\n` (sequence number 15, forward)

### Upstream Response (STM32 → Python)

**ACK Response**: `ACK,seq,status,time\n`
- seq: Command sequence number
- status: OK or TIMEOUT
- time: Action duration (e.g., 2.35s)

**Examples**: 
- `ACK,15,OK,2.35s\n` (normal completion)
- `ACK,15,TIMEOUT,35.00s\n` (timeout)

**Lidar Data**: Binary frame (1012 bytes)
- Frame header: A5 5A 4C 44
- Data: 500 uint16 distance values (mm)

---

## Documentation

- **Quick Start**: [QUICKSTART.md](QUICKSTART.md) - 5-minute setup
- **Test Procedure**: [TEST_PROCEDURE.md](TEST_PROCEDURE.md) - Detailed test steps
- **Feature Description**: [STAGE3_ENHANCED_COMPLETE.md](STAGE3_ENHANCED_COMPLETE.md)
- **Implementation Report**: [IMPLEMENTATION_COMPLETE.md](IMPLEMENTATION_COMPLETE.md)
- **Stage 3 Plan**: [STAGE3_PLAN.md](STAGE3_PLAN.md) - Original plan (reference)

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Connection Failed | Check BLE address, run `python scripts/scan_ble.py` |
| No ACK Received | Re-flash STM32 firmware |
| Inaccurate Movement | Adjust PID parameters in motor.c |
| Wall Misdetection | Adjust `wall_threshold` parameter |

Detailed troubleshooting: See [TEST_PROCEDURE.md](TEST_PROCEDURE.md)

---

## Technical Architecture

```
STM32 Firmware Side                Python Side
┌─────────────────┐           ┌──────────────────────┐
│ Motor_Command   │           │ MotionController     │
│ - MoveForward   │           │ - Sequence tracking  │
│ - TurnLeft      │           │ - Event synchronization│
│ - TurnRight     │           │ - Timeout protection │
│ - Auto PID control│          └──────────────────────┘
│ - Auto stop when done│                ↑
└─────────────────┘                    │
         ↓                              │
┌─────────────────┐           ┌──────────────────────┐
│ ACK Response     │ BLE/UART  │ ACK Parsing           │
│ - Speed detection│←─────────→│ - Callback trigger    │
│ - Timeout protection│ 115200  │ - Status recording   │
└─────────────────┘           └──────────────────────┘
         ↓                              ↓
┌─────────────────┐           ┌──────────────────────┐
│ Lidar Data Send │           │ Wall Detection       │
│ - 500 points/frame│          │ - WallDetector       │
│ - 1012 bytes     │          │ - ExitDetector       │
│ - 1-2Hz         │          │ - PathTracker        │
└─────────────────┘           └──────────────────────┘
```

---

## Development Team

- STM32 Firmware: xxq team
- Python Algorithms: maze_explorer team

---

**🚀 Get Started Now**: See [QUICKSTART.md](QUICKSTART.md)
