# XXQ Smart Car Control System

> Intelligent mobile robot platform based on STM32F446RET6  
> Triple-loop PID control + Gyroscope attitude + LiDAR + Upper computer communication

---

## 🎯 System Features

- ✅ **Triple-loop PID Control**: Position loop + Angle loop + Speed loop cascade control
- ✅ **Dual PWM Drive**: Complete L298N H-bridge control (4-channel PWM independent control)
- ✅ **Encoder Feedback**: 1560 PPR high-precision speed closed loop
- ✅ **Gyroscope Attitude**: UART5 real-time angle feedback (supports automatic calibration)
- ✅ **LiDAR**: Environment perception and scanning
- ✅ **Upper Computer Communication**: Python non-blocking command control

---

## 🚀 Quick Start

### 1️⃣ View Documentation
👉 **[docs/系统使用总览.md](docs/系统使用总览.md)** - Complete feature introduction

### 2️⃣ Hardware Connection
👉 **[docs/硬件连接方案.md](docs/硬件连接方案.md)** - Wiring guide

### 3️⃣ Flash Firmware
```bash
Compile and flash to STM32F446RET6 using STM32CubeIDE
```

### 4️⃣ Start Control
```c
// System includes complete initialization code
// Supports Python upper computer control and serial command control
```

---

## 📖 Core Documentation

| Document | Description |
|----------|-------------|
| [docs/系统使用总览.md](docs/系统使用总览.md) | System functionality overview (essential reading) |
| [docs/硬件连接方案.md](docs/硬件连接方案.md) | Hardware wiring diagram |
| [docs/双路PWM电机控制接线说明.md](docs/双路PWM电机控制接线说明.md) | Motor drive detailed explanation |
| [docs/激光雷达使用方案.md](docs/激光雷达使用方案.md) | LiDAR usage instructions |
| [docs/上位机开发完整指南.md](docs/上位机开发完整指南.md) | Python development guide |

---

## 🔌 Hardware Pins (Quick Reference)

### Motor Drive (TIM1 PWM)
```
PA8  → AIN1 (Left motor forward)
PA9  → AIN2 (Left motor reverse)
PA10 → BIN1 (Right motor forward)
PA11 → BIN2 (Right motor reverse)
```

### Encoders (TIM2/TIM3)
```
PC6/PC7   → Left wheel encoder
PA15/PB3  → Right wheel encoder
```

### Communication Interfaces
```
PC12/PD2  → UART5 (Gyroscope, 9600)
PC10/PC5  → UART3 (LiDAR, 115200)
PA0/PA1   → UART4 (Upper computer, 9600)
```

---

## 🎮 Usage Examples

### Python Control
```python
import serial

ser = serial.Serial('COM3', 9600)

# Move forward 60cm
ser.write(b'MOVE,60\n')
response = ser.readline()  # Wait for 'Z' (arrived)

# Turn left 90 degrees
ser.write(b'TURN,90\n')
response = ser.readline()  # Wait for 'Z' (arrived)
```

### C Code Control
```c
// Initialization
Motor_Init(&htim1);
Motor_PID_Init();

// Main loop
while(1) {
    Motor_PID_Control(&htim1, &htim3, &htim2);
    
    // Move forward 60cm
    Motor_Command_MoveForward(60.0f, 0);
}
```

---

## 📊 System Parameters

| Parameter | Value |
|-----------|-------|
| MCU | STM32F446RET6 @ 168MHz |
| Control frequency | 50Hz (20ms period) |
| Tire diameter | 7.5cm |
| Encoder PPR | 1560 (left) / 780 (right) |
| Gyroscope calibration factor | 0.99557522 |
| Position calibration factor | 0.88888889 |

---

## 🎓 Developer

- **Author:** Ye Jin
- **Group:** Group 12  
- **Version:** v1.0.0
- **Date:** 2025

---

**Enjoy using it!** 🚗💨

**For detailed documentation, please check the [docs/](docs/) directory**
