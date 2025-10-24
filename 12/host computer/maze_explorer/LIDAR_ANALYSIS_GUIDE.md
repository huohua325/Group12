# Lidar Direction Distance Analysis Guide

## Script Description

`analyze_lidar_directions.py` - Real-time analysis of lidar distance data in 8 directions

---

## Running Methods

### Basic Run (default 60 seconds, update every second)
```bash
cd maze_explorer
python analyze_lidar_directions.py
```

### Custom Parameters
```bash
# Run 30 seconds, update every 0.5 seconds
python analyze_lidar_directions.py --time 30 --interval 0.5

# Run 120 seconds, update every 2 seconds
python analyze_lidar_directions.py --time 120 --interval 2.0
```

---

## Output Description

### 1. Real-time Data Display

Display every specified interval (default 1 second):

```
================================================================================
[Frame #15] Timestamp: 12345ms
================================================================================

Direction     Min       Avg       Max      Valid
--------------------------------------------------------------------------------
Front(0°)     280mm     320mm     350mm     25/28pts
Front-Right(45°) 450mm     480mm     510mm     12/14pts
Right(90°)    300mm     340mm     380mm     26/28pts
Back-Right(135°) 520mm     550mm     580mm     13/14pts
Back(180°)    290mm     330mm     370mm     27/28pts
Back-Left(225°) --        --        --         0/14pts
Left(270°)    310mm     350mm     390mm     28/28pts
Front-Left(315°) 480mm     500mm     520mm     14/14pts

Total valid points: 445/500

                    Visualization (based on average distance)
--------------------------------------------------------------------------------
                                   320
         350                                        480
                                  Robot
         500                                        340
                                   330
```

**Description**:
- **Min/Avg/Max**: Distance statistics for all points in that direction
- **Valid points**: Valid points/Total points (e.g., 25/28 means 25 out of 28 points are valid)
- **Visualization**: Numbers represent distance (mm), "Wall" represents distance < 400mm

---

### 2. Final Statistics

Display average values of all frames after completion:

```
================================================================================
Final Statistics (average of all frames)
================================================================================
Total frames: 60
Average frame rate: 2.00 Hz

Direction     Avg Distance  Std Dev    Min      Max
--------------------------------------------------------------------------------
Front(0°)        320mm       15mm    290mm   350mm
Front-Right(45°) 485mm       25mm    450mm   520mm
Right(90°)       338mm       18mm    300mm   380mm
Back-Right(135°) 548mm       22mm    520mm   580mm
Back(180°)       328mm       20mm    290mm   370mm
Back-Left(225°)    0mm        0mm      0mm     0mm
Left(270°)       348mm       16mm    310mm   390mm
Front-Left(315°) 498mm       12mm    480mm   520mm

================================================================================
Data Quality Assessment
================================================================================
Front(0°)    : Avg=  320mm StdDev=  15mm [Stable]
Right(90°)   : Avg=  338mm StdDev=  18mm [Stable]
Back(180°)   : Avg=  328mm StdDev=  20mm [Stable]
Left(270°)   : Avg=  348mm StdDev=  16mm [Stable]
```

**Stability Rating**:
- `[Stable]`: Standard deviation < 50mm, good data quality
- `[Moderately Stable]`: Standard deviation 50-100mm, usable
- `[Unstable]`: Standard deviation > 100mm, poor data quality

---

## Usage Scenarios

### Scenario 1: Environment Detection

**Purpose**: Check robot surroundings

**Operation**:
1. Place robot in cell center or cardboard box center
2. Run script for 30 seconds
3. Check distance in each direction

**Expected** (cardboard environment, distance about 30cm):
- Directions with walls: average distance 300-400mm
- Open directions: average distance > 1000mm

---

### Scenario 2: Lidar Calibration Verification

**Purpose**: Verify lidar distance measurement accuracy

**Operation**:
1. Place obstacles at known distances in each direction
2. Run script for 60 seconds
3. Compare actual distance with measured value

**Judgment Criteria**:
- Error < 50mm: Good accuracy
- Error 50-100mm: Usable
- Error > 100mm: Needs recalibration

---

### Scenario 3: Wall Detection Parameter Tuning

**Purpose**: Determine appropriate `wall_threshold` parameter

**Operation**:
1. Place robot in known environment
2. Run script for 60 seconds
3. Check average distance in wall directions
4. Set `wall_threshold` to that distance + 50mm

**Example**:
- Average distance in wall directions: 350mm
- Recommended `wall_threshold`: 400mm

---

## Data Interpretation

### 1. Direction Angle Mapping

| Direction | Angle | Index Range | Points |
|-----------|-------|-------------|--------|
| Front | 0° | 0-13, 486-499 | 28 |
| Front-Right | 45° | 55-68 | 14 |
| Right | 90° | 111-138 | 28 |
| Back-Right | 135° | 180-193 | 14 |
| Back | 180° | 236-263 | 28 |
| Back-Left | 225° | 305-318 | 14 |
| Left | 270° | 361-388 | 28 |
| Front-Left | 315° | 430-443 | 14 |

**Description**: 
- Four main directions (front, right, back, left) each occupy 28 points (~20 degree range)
- Four diagonal directions each occupy 14 points (~10 degree range)

---

### 2. Distance Data Analysis

#### Minimum Distance
- Distance to nearest obstacle in that direction
- Used to determine if there is a wall

#### Average Distance
- Average of all points in that direction
- Most stable reference value

#### Maximum Distance
- Farthest valid point in that direction
- Used to detect openings

#### Valid Point Count
- Points with data/Total points
- Used to judge data quality

---

### 3. Abnormal Situations

**Situation A**: Valid points = 0 in a direction
- **Cause**: That direction is beyond lidar detection range or no reflection
- **Impact**: Cannot determine wall status in that direction
- **Solution**: Adjust robot position or environment

**Situation B**: Standard deviation > 100mm in a direction
- **Cause**: Multiple obstacles at different distances in that direction, or unstable data
- **Impact**: Wall detection may be inaccurate
- **Solution**: Check environment or increase sampling time

**Situation C**: Average distance abnormally large/small
- **Cause**: Environment doesn't match expectations, or lidar malfunction
- **Impact**: Wall detection fails
- **Solution**: Recalibrate or check lidar

---

## Debugging Tips

### Tip 1: Extend Observation Time
```bash
python analyze_lidar_directions.py --time 120
```
Longer time can get more accurate averages

### Tip 2: Speed Up Updates
```bash
python analyze_lidar_directions.py --interval 0.5
```
Update every 0.5 seconds for more real-time

### Tip 3: Record Data
```bash
python analyze_lidar_directions.py --time 60 > lidar_log.txt
```
Save output to file

---

## Common Questions

### Q1: A direction always shows `--`
**A**: No valid data in that direction
- Check if lidar is rotating normally
- Check if that direction is beyond detection range
- Adjust robot position

### Q2: Distance values fluctuate violently
**A**: Environmental interference or unstable lidar
- Improve test environment (flat walls)
- Check if lidar is loose
- Increase sampling time

### Q3: All directions have large distances
**A**: Robot is in open environment
- This is normal, indicates no obstacles around
- Can be used to test exit detection

### Q4: Low frame rate
**A**: STM32 lidar processing issue
- Check radar.c code
- Look for TIMEOUT warnings

---

## Application Examples

### Example 1: Adjust Wall Detection Threshold

```bash
# 1. Place robot in cell center
# 2. Run analysis
python analyze_lidar_directions.py --time 30

# 3. Observe average distance in wall directions (e.g., 350mm)
# 4. Set threshold to average distance + 50mm
# 5. Edit wall_detector.py or config.py:
wall_threshold = 400  # 350 + 50
```

---

### Example 2: Verify Environment Setup

```bash
# 1. Build 4x4 maze (600mm cells)
# 2. Place robot in cell center
# 3. Run analysis
python analyze_lidar_directions.py --time 60

# 4. Check if surrounding distances are about 300mm (600/2=300)
# 5. If distance deviates significantly, adjust maze size
```

---

**Best tool for quick diagnosis of robot surroundings!** 🔍

