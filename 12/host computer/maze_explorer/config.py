"""
Configuration parameters

Reference:
- xxq/docs/Complete guide for upper computer development.md (radar data format)
- maze/ble_robot_control.py (BLE communication)
"""

# =============================================================================
# BLE Connection Configuration
# =============================================================================

BLE_ADDRESS = 'C4:25:01:20:02:8E'  # BLE device address
BLE_CONNECT_TIMEOUT = 15.0  # Connection timeout in seconds

# =============================================================================
# Lidar Parameters (from development guide lines 97-147)
# =============================================================================

LIDAR_FRAME_SIZE = 1012                      # Frame size in bytes
LIDAR_FRAME_HEADER = b'\xA5\x5A\x4C\x44'    # Frame header (A5 5A 4C 44)
LIDAR_POINT_COUNT = 500                      # Points per frame
LIDAR_ANGLE_STEP = 0.72                      # Angle step in degrees
LIDAR_FREQUENCY = 10                         # Transmission frequency in Hz

# Angle mapping
# index × 0.72 = angle(degrees)
# 0° → index 0
# 90° → index 125
# 180° → index 250
# 270° → index 375

# =============================================================================
# Maze Parameters
# =============================================================================

CELL_SIZE = 600   # Cell side length in mm
MAZE_ROWS = 4     # Number of maze rows
MAZE_COLS = 4     # Number of maze columns

# Total maze dimensions in mm
MAZE_WIDTH = MAZE_COLS * CELL_SIZE   # 2400 mm
MAZE_HEIGHT = MAZE_ROWS * CELL_SIZE  # 2400 mm

# =============================================================================
# Motion Parameters (requires actual measurement adjustment)
# =============================================================================

FORWARD_TIME = 2.0   # Time to move forward one cell (600mm) in seconds
TURN_TIME = 1.5      # Time to turn 90 degrees in seconds

# =============================================================================
# MODE Command Definitions
# =============================================================================

MODE_STOP = 0        # Stop
MODE_FORWARD = 1     # Move forward
MODE_BACKWARD = 2    # Move backward
MODE_TURN_LEFT = 3   # Turn left
MODE_TURN_RIGHT = 4  # Turn right

# =============================================================================
# SLAM Parameters (used in Stage 2)
# =============================================================================

SLAM_MAP_SIZE = 500           # Map size in pixels
SLAM_RESOLUTION = 10          # Map resolution in mm/pixel
SLAM_MAP_SIZE_METERS = 5.0    # Map actual size in meters, sufficient for 2.4m maze
SLAM_MAX_DISTANCE = 12000     # Maximum detection distance in mm

# Visualization parameters
VIS_UPDATE_RATE = 10          # Update rate in Hz
VIS_SHOW_ROBOT = True         # Show robot position
VIS_SHOW_LIDAR = False        # Show lidar points

# =============================================================================
# Debug Parameters
# =============================================================================

DEBUG_VERBOSE = False   # Whether to show detailed logs

