# LiDAR Raw Data Passthrough - Quick Start

> 5-minute getting started with radar data passthrough

---

## 📦 File List

```
xxq/
├── Core/Src/hardware/
│   ├── radar.h                    ✅ Updated (added passthrough API)
│   └── radar.c                    ✅ Updated (implemented passthrough logic)
├── Core/Src/
│   └── main.c                     ✅ Updated (main loop integration)
├── lidar_raw_receiver.py          ✅ Added (USB serial reception)
├── lidar_visualizer.py            ✅ Added (USB serial visualization)
├── lidar_raw_receiver_ble.py      ✅ Added (Bluetooth BLE reception)
├── lidar_visualizer_ble.py        ✅ Added (Bluetooth BLE visualization)
└── docs/
    └── LiDAR_Raw_Data_Passthrough_Solution.md ✅ Added (complete documentation)
```

---

## 🚀 Quick Start (3 Steps)

### Step 1: Compile and Flash STM32

```bash
# Ensure code is updated
cd xxq
make clean
make

# Use ST-Link or other tools to flash
# Or click "Build" then "Run" in STM32CubeIDE
```

### Step 2: Test Connection

#### Method A: USB Serial Connection

**Windows:**
```bash
python lidar_raw_receiver.py --port COM3 --frames 5
```

**Linux/Mac:**
```bash
python lidar_raw_receiver.py --port /dev/ttyUSB0 --frames 5
```

#### Method B: Bluetooth BLE Connection (Recommended)

```bash
python lidar_raw_receiver_ble.py --frames 5
```

Or specify BLE address:
```bash
python lidar_raw_receiver_ble.py --address "XX:XX:XX:XX:XX:XX" --frames 5
```

**Expected Output**:
```
[INFO] Connected to COM3 @ 460800 bps
[INFO] Starting to receive radar data (target 5 frames)...

============================================================
Frame #1
============================================================
LidarFrame(type=0x01, seq=42, time=12345ms, points=523)
Valid points: 487/523

First 5 valid points:
  1. LidarPoint(angle=0.72°, dist=1234.5mm, q=35)
  2. LidarPoint(angle=1.44°, dist=1189.0mm, q=42)
  ...
```

### Step 3: Real-time Visualization (Recommended)

#### USB Serial Version:
```bash
python lidar_visualizer.py --port COM3
```

#### Bluetooth BLE Version (Recommended):
```bash
python lidar_visualizer_ble.py
```

Will open a window showing:
- Left: Polar coordinate view (circular)
- Right: Cartesian coordinate top view
- Top: Real-time statistics (frame rate, point count, success rate)

Close the window to stop.

---

## 📊 Core Features

| Feature           | Description                              |
|----------------|-----------------------------------|
| **Data Format**   | Raw 5 bytes (angle_q6 + distance_q2)|
| **Transmission Rate**   | 2Hz (send one rotation every 500ms)            |
| **Bandwidth Usage**   | ~5KB/s (8.8% of 460800 baud rate)     |
| **Reliability**     | 8+8 byte dual header/trailer + CRC16 checksum  |
| **Compatibility**     | Does not conflict with existing text commands (CMD/ACK/ODO)|

---

## 🔧 API Quick Reference

### C Language (STM32)

```c
// Check if should send (returns 1 every 500ms)
uint8_t LIDAR_ShouldSendFrame(void);

// Pack and send raw 5-byte data
int LIDAR_PackAndSendRawFrame(const PackPoint_t *points, uint16_t n_points);
// Returns: 0=success, -1=UART busy, -3=zero points
```

### Python (Upper-level)

#### USB Serial Version:
```python
from lidar_raw_receiver import LidarRawReceiver

# Create receiver
receiver = LidarRawReceiver('COM3', 460800)

# Receive one frame
frame = receiver.receive_frame()
if frame:
    print(f"Frame sequence: {frame.sequence}, Points: {len(frame.points)}")
    
    # Get valid points (quality>5)
    good_points = frame.get_valid_points(min_quality=5)
    
    for pt in good_points[:10]:
        print(f"Angle={pt.angle_deg:.1f}°, Distance={pt.distance_mm:.0f}mm")

receiver.close()
```

#### Bluetooth BLE Version:
```python
from lidar_raw_receiver_ble import LidarRawReceiverBLE
import config

# Create BLE receiver
receiver = LidarRawReceiverBLE(config.BLE_ADDRESS)

# Connect
if receiver.connect():
    # Wait for one frame
    frame = receiver.wait_for_frame(timeout=2.0)
    
    if frame:
        print(f"Frame sequence: {frame.sequence}, Points: {len(frame.points)}")
        
        # Get valid points
        good_points = frame.get_valid_points(min_quality=5)
        
        for pt in good_points[:10]:
            print(f"Angle={pt.angle_deg:.1f}°, Distance={pt.distance_mm:.0f}mm")
    
    receiver.disconnect()
```

---

## 🎯 Common Applications

### Application 1: Obstacle Detection

```python
# Check if there are obstacles within 1 meter in front
def check_front_obstacle(frame):
    for pt in frame.get_valid_points():
        # Front ±10° range
        if -10 <= pt.angle_deg <= 10 or 350 <= pt.angle_deg <= 360:
            if pt.distance_mm < 1000:  # Within 1 meter
                print(f"⚠️  Obstacle {pt.distance_mm:.0f}mm ahead!")
                return True
    return False
```

### Application 2: Map Construction

```python
# Export as CSV for other tools
receiver = LidarRawReceiver('COM3', 460800)
with open('map.csv', 'w') as f:
    f.write('angle_deg,distance_mm,quality\n')
    
    for _ in range(100):  # Collect 100 frames
        frame = receiver.receive_frame()
        if frame:
            for pt in frame.get_valid_points():
                f.write(f'{pt.angle_deg},{pt.distance_mm},{pt.quality}\n')
```

### Application 3: SLAM Integration

```python
# Convert to ROS PointCloud2 format
import numpy as np

def to_pointcloud(frame):
    points = []
    for pt in frame.get_valid_points():
        x, y = pt.to_cartesian()
        points.append([x/1000, y/1000, 0])  # Convert to meters
    return np.array(points, dtype=np.float32)
```

---

## 🐛 Common Issues

### Q1: Cannot find header

**Problem**: Upper-level keeps showing "searching for header..."

**USB Serial Solution**:
1. Confirm STM32 is powered on and running
2. Send `'A'` command to start radar:
   ```python
   import serial
   ser = serial.Serial('COM3', 460800)
   ser.write(b'A\n')
   ser.close()
   ```
3. Check if serial port is correct (Windows Device Manager / Linux `ls /dev/ttyUSB*`)

**Bluetooth BLE Solution**:
1. Confirm BLE address is correct (configure in config.py)
2. Confirm STM32 Bluetooth module is connected
3. Use `-v` parameter to view detailed logs:
   ```bash
   python lidar_raw_receiver_ble.py --frames 5 -v
   ```

### Q2: Frequent CRC errors

**Problem**: `[ERROR] CRC verification failed`

**Solution**:
- Reduce baud rate for testing: `--baudrate 230400`
- Use USB-to-serial cable (avoid Bluetooth module)
- Check if wiring is loose

### Q3: Very few or zero points

**Problem**: `points=0` or `points=15` (normal should be 200-700)

**Solution**:
- Confirm radar motor is rotating
- Reduce quality threshold:
  ```python
  valid_points = frame.get_valid_points(min_quality=0)
  ```
- Check if radar is blocked

---

## 📚 Advanced Reading

- **Complete Documentation**: [LiDAR_Raw_Data_Passthrough_Solution.md](docs/LiDAR_Raw_Data_Passthrough_Solution.md)
- **RPLIDAR SDK**: https://github.com/Slamtec/rplidar_sdk
- **CRC Algorithm**: Standard MODBUS protocol

---

## 📞 Technical Support

- **Author**: Ye Jin (Group 12)
- **Date**: 2025-10-21
- **Email**: (Internal project)

If you have questions, please check:
1. "Common Issues" section of this document
2. "Troubleshooting" section of the complete documentation
3. Code comments

---

## ✅ Checklist

Before use, please confirm:

- [x] STM32 code has been compiled and flashed
- [x] Serial connection is normal (can receive other commands like `'F'`, `'S'`)
- [x] Python dependencies installed: `pip install pyserial numpy matplotlib`
- [x] Radar is powered on and motor is rotating
- [x] Send `'A'` command to start radar scanning
- [x] Serial port is correct (Windows: COM3, Linux: /dev/ttyUSB0)
- [x] Baud rate is set to 460800

After all confirmations, run `python lidar_visualizer.py --port COM3`!

---

**Wishing you a pleasant experience!** 🎉

