#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Wall Detection Test Script
Test radar data reception and wall recognition
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from communication.ble_comm import BLERobotComm
from perception.wall_detector import WallDetector
import config


def test_wall_detection(duration=30):
    """
    Test wall detection functionality
    
    Args:
        duration: Test duration (seconds)
    """
    print("=" * 70)
    print("Wall Detection Test")
    print("=" * 70)
    print("Please place the robot in the center of a maze or cardboard box")
    print(f"Will run for {duration} seconds, displaying wall detection results every second")
    print()
    
    # Initialize
    comm = BLERobotComm(config.BLE_ADDRESS, verbose=True)
    detector = WallDetector(cell_size=600, wall_threshold=400)
    
    # Connect
    print("Connecting...")
    if not comm.connect():
        print("[ERROR] Connection failed")
        return
    
    print("[OK] Connection successful\n")
    
    # Statistics
    frame_count = 0
    last_display_time = time.time()
    
    def on_lidar_frame(data):
        """Lidar data callback"""
        nonlocal frame_count, last_display_time
        
        frame_count += 1
        distances = data['distances_mm']
        
        # Display every 1 second
        now = time.time()
        if now - last_display_time >= 1.0:
            # Detect walls
            walls = detector.detect(distances)
            
            # Display results
            print(f"\n[Frame #{frame_count}] Wall detection results:")
            print(f"  {detector.format_detection(walls)}")
            
            # Simple visualization
            print("\n  Visualization (facing: front):")
            print("       " + ("███" if walls['front'] else "   "))
            print(f"    {('███' if walls['left'] else '   ')} 🚗 {('███' if walls['right'] else '   ')}")
            print("       " + ("███" if walls['back'] else "   "))
            
            last_display_time = now
    
    comm.on_lidar_frame = on_lidar_frame
    
    try:
        # Run for specified duration
        print("=" * 70)
        print("Starting to listen for lidar data...")
        print("=" * 70)
        time.sleep(duration)
        
        # Statistics
        print(f"\n\n{'=' * 70}")
        print("Test completed")
        print(f"{'=' * 70}")
        print(f"Total frames: {frame_count}")
        print(f"Average frame rate: {frame_count / duration:.2f} Hz")
        
    except KeyboardInterrupt:
        print("\n\nUser interrupted")
    finally:
        comm.disconnect()


if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='Wall Detection Test')
    parser.add_argument('-t', '--time', type=int, default=30,
                       help='Test duration (seconds), default 30 seconds')
    
    args = parser.parse_args()
    
    test_wall_detection(duration=args.time)

