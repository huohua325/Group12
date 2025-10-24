# Installation Guide

## Dependency Installation

### 1. Basic Dependencies

```bash
cd maze_explorer
pip install bleak numpy matplotlib Pillow
```

### 2. BreezySLAM Installation (Important! ⭐)

**BreezySLAM is not on PyPI**, needs to be installed from GitHub:

```bash
pip install git+https://github.com/simondlevy/BreezySLAM.git#subdirectory=python
```

### 3. Verify Installation

```bash
# Verify BreezySLAM
python -c "from breezyslam.algorithms import RMHC_SLAM; print('BreezySLAM OK')"

# Verify matplotlib
python -c "import matplotlib.pyplot as plt; print('matplotlib OK')"

# Verify bleak
python -c "from bleak import BleakClient; print('bleak OK')"
```

## Complete Installation Process (Recommended)

```bash
# 1. Activate virtual environment (if using)
# venv\Scripts\activate  (Windows)
# source venv/bin/activate  (Linux/Mac)

# 2. Install basic dependencies
pip install bleak numpy matplotlib Pillow

# 3. Install BreezySLAM
pip install git+https://github.com/simondlevy/BreezySLAM.git#subdirectory=python

# 4. Verify installation
python test_system.py
```

## Possible Issues

### Issue 1: BreezySLAM Installation Failed

**Reason**: Requires C++ compiler

**Solution**:

**Windows**:
- Install [Visual Studio Build Tools](https://visualstudio.microsoft.com/downloads/)
- Or install [Microsoft C++ Build Tools](https://visualstudio.microsoft.com/visual-cpp-build-tools/)

**Linux**:
```bash
sudo apt-get install build-essential python3-dev
```

**macOS**:
```bash
xcode-select --install
```

### Issue 2: matplotlib Display Problems

**Symptoms**: Window not showing or black screen

**Solution**:
```bash
# Install backend
pip install PyQt5
# or
pip install tkinter
```

### Issue 3: BLE Connection Failed

**Symptoms**: Unable to connect to device

**Solution**:
1. Confirm Bluetooth is enabled
2. Check device address is correct
3. Ensure device is not occupied by other programs
4. Windows: May require administrator privileges

## System Requirements

- Python 3.7+
- Windows 10+ / Linux / macOS
- Bluetooth adapter (for BLE communication)

## Verify Complete System

```bash
# Test BLE communication and radar data
python test_system.py

# Test SLAM map construction
python test_slam.py --time 30
```

## Uninstall

```bash
pip uninstall bleak numpy matplotlib Pillow BreezySLAM
```

---

**After successful installation, please run test scripts to verify the system!** 🚀

