# Quick Start Guide

## Preparation

### 1. Configure BLE Address
Edit `config.py` to set your robot's BLE address:
```python
BLE_ADDRESS = 'C4:25:01:20:02:8E'  # Replace with your address
```

### 2. Flash STM32 Firmware
```bash
cd xxq
# Open project with STM32CubeIDE
# Build -> Debug/Release
# Run -> Debug to flash to development board
```

---

## Step-by-Step Testing (Execute in Order)

### Step 1: Command ACK Test ⭐⭐⭐

**Purpose**: Verify command sending and ACK receiving mechanism

**Command**:
```bash
cd maze_explorer
python test_command_ack.py
```

**Preparation**:
- Place the robot on open ground
- Ensure at least 1 meter of space in front
- Ensure sufficient battery power

**Expected Results**:
```
Test 1: Stop command → Immediately receive ACK
Test 2: Forward command → Robot moves forward ~600mm → Receive ACK
Test 3: Turn left command → Robot turns left ~90 degrees → Receive ACK
Test 4: Turn right command → Robot turns right ~90 degrees → Receive ACK
```

**Checklist**:
- [ ] All commands receive ACK
- [ ] Sequence numbers are consecutive (1, 2, 3, 4...)
- [ ] Forward distance is accurate (600mm ±50mm)
- [ ] Turn angle is accurate (90 degrees ±5 degrees)

**Failure Handling**:
- If no ACK received → Check if firmware is latest
- If distance inaccurate → Adjust distance parameters in motor.c
- If angle inaccurate → Adjust angle parameters in motor.c

---

### Step 2: Wall Detection Test ⭐⭐

**Purpose**: Verify lidar data and wall recognition

**Command**:
```bash
cd maze_explorer
python test_wall_detection_enhanced.py --time 30
```

**Preparation**:
- Place the robot in the center of a cardboard box or maze cell
- Walls should be around (distance 300-400mm)

**Expected Results**:
```
[Frame #1-N] Wall detection results:
  Front: Wall (350mm) | Right: Wall (320mm) | Back: Wall (380mm) | Left: Wall (360mm)

Frame rate: 1-2 Hz
Distance statistics: Average 300-400mm
```

**Checklist**:
- [ ] Frame rate is stable (>1Hz)
- [ ] Distance measurement is accurate (±50mm from actual)
- [ ] Wall recognition is correct (all four directions)
- [ ] Data is stable (standard deviation <50mm)

**Failure Handling**:
- If frame rate too low → Check STM32 lidar code (radar.c)
- If distance inaccurate → Check lidar calibration
- If wall misdetection → Adjust `wall_threshold` parameter (default 400mm)

---

### Step 3: Simple Exploration Test ⭐

**Purpose**: Test wall-following exploration (small scale)

**Command**:
```bash
cd maze_explorer
python main_maze_exploration.py --method right --steps 5
```

**Preparation**:
- Build simple environment (e.g., L-shaped corridor)
- Ensure there is an exit (distance >1500mm in some direction)

**Expected Results**:
```
Step #1: Detect walls → Execute action → Receive ACK → Move to new position
Step #2: ...
...
Exit found!
Starting return to origin...
Return completed!
```

**Checklist**:
- [ ] Wall-following strategy is correct
- [ ] Action execution is accurate
- [ ] Exit detection is normal
- [ ] Path return is successful

---

## Complete Exploration (After Tests Pass)

```bash
cd maze_explorer
python main_maze_exploration.py --method right --steps 100
```

---

## Common Commands

### Run Unit Tests
```bash
# Basic functionality test
python tests/test_stage3.py

# Enhanced functionality test
python tests/test_stage3_enhanced.py
```

### View Help
```bash
python test_command_ack.py --help
python test_wall_detection_enhanced.py --help
python main_maze_exploration.py --help
```

---

## Debug Log Description

### STM32 Side Logs
- `[CMD] seq=X mode=Y ->` - Command received
- `[DEBUG] Action completed: seq=X time=Y.YYs` - Action completed
- `ACK,X,OK,Y.YYs` - Normal completion reply
- `ACK,X,TIMEOUT,Y.YYs` - Timeout completion reply

### Python Side Logs
- `[OK] ACK: seq=X status=OK Y.YYs` - Normal ACK received
- `[WARN] ACK: seq=X TIMEOUT Y.YYs` - Timeout ACK received
- `[MOTION] Forward completed` - Action successful
- `[MOTION] Forward failed` - Action failed

---

## Troubleshooting

### Q1: Connection Failed
```bash
# Scan BLE devices
python scripts/scan_ble.py
```

### Q2: Inaccurate Movement
- Check if ground is flat
- Check if wheels are slipping
- Adjust PID parameters (motor.c)

### Q3: No ACK Received
- Re-flash firmware
- Check serial communication
- Increase verbose logging

### Q4: Wall Misdetection
- Adjust `wall_threshold` (default 400mm)
- Check lidar data quality
- Improve test environment

---

## Parameter Adjustment

### Wall Detection Threshold
```python
# In config.py or code
wall_threshold = 400  # Increase → More strict wall detection
                      # Decrease → More lenient wall detection
```

### Exit Detection Threshold
```python
exit_threshold = 1500  # Increase → Harder to detect as exit
                       # Decrease → Easier to detect as exit
```

### Timeout Duration
```python
# motion_controller.py
timeout = 10.0  # Increase → More tolerant
                # Decrease → Fail faster
```

---

**Complete the three test steps in order to start full maze exploration!** 🎯

