#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Wall Following Main Program
Use right-hand rule or left-hand rule for maze exploration
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from communication.ble_comm import BLERobotComm
from perception.wall_detector import WallDetector
from control.motion_controller import MotionController, WallFollower
import config


def main_wall_following(method='right', max_steps=50):
    """
    Wall following main program
    
    Args:
        method: 'right' (right-hand rule) or 'left' (left-hand rule)
        max_steps: Maximum steps
    """
    print("=" * 70)
    print("Wall Following Exploration")
    print("=" * 70)
    print(f"Strategy: {'Right-hand rule' if method == 'right' else 'Left-hand rule'}")
    print(f"Maximum steps: {max_steps}")
    print()
    print("Please ensure:")
    print("  1. Robot is placed at maze starting point")
    print("  2. Robot orientation is known")
    print("  3. Sufficient space around")
    print()
    input("Press Enter to start...")
    print()
    
    # Initialize
    comm = BLERobotComm(config.BLE_ADDRESS, verbose=True)
    detector = WallDetector(cell_size=600, wall_threshold=400)
    motion = MotionController(comm, forward_time=2.0, turn_time=1.5)
    follower = WallFollower(motion)
    
    # Connect
    print("Connecting...")
    if not comm.connect():
        print("[ERROR] Connection failed")
        return
    
    print("[OK] Connection successful\n")
    
    # Wait for lidar data to stabilize
    print("Waiting for lidar data to stabilize...")
    time.sleep(2)
    
    # Main loop
    step_count = 0
    current_walls = None
    data_ready = False
    
    def on_lidar_frame(data):
        """Lidar data callback"""
        nonlocal current_walls, data_ready
        
        distances = data['distances_mm']
        current_walls = detector.detect(distances)
        data_ready = True
    
    comm.on_lidar_frame = on_lidar_frame
    
    try:
        print("=" * 70)
        print("Starting Wall Following Exploration")
        print("=" * 70)
        
        while step_count < max_steps:
            # Wait for lidar data
            data_ready = False
            wait_start = time.time()
            while not data_ready:
                time.sleep(0.1)
                if time.time() - wait_start > 5:
                    print("[WARN] Timeout waiting for lidar data")
                    break
            
            if not data_ready:
                print("[ERROR] No lidar data received, stopping")
                break
            
            # Display current state
            print(f"\n{'=' * 70}")
            print(f"Step #{step_count + 1}")
            print(f"{'=' * 70}")
            print(f"Wall detection: {detector.format_detection(current_walls)}")
            
            # Visualization
            print("\n  Current environment (facing: front):")
            print("       " + ("███" if current_walls['front'] else "   "))
            print(f"    {('███' if current_walls['left'] else '   ')}  Car {('███' if current_walls['right'] else '   ')}")
            print("       " + ("███" if current_walls['back'] else "   "))
            print()
            
            # Execute wall-following strategy
            if method == 'right':
                action = follower.follow_right_wall(current_walls)
            else:
                action = follower.follow_left_wall(current_walls)
            
            print(f"Action: {action}")
            
            step_count += 1
            
            # Brief delay
            time.sleep(1)
        
        print(f"\n\n{'=' * 70}")
        print("Exploration Complete")
        print(f"{'=' * 70}")
        print(f"Total steps: {step_count}")
        
    except KeyboardInterrupt:
        print("\n\nUser interrupted")
        motion.stop()
    except Exception as e:
        print(f"\n[ERROR] Exception: {e}")
        motion.stop()
    finally:
        comm.disconnect()


if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='Wall Following Exploration')
    parser.add_argument('-m', '--method', type=str, default='right',
                       choices=['right', 'left'],
                       help='Wall-following method: right (right-hand rule) or left (left-hand rule)')
    parser.add_argument('-s', '--steps', type=int, default=50,
                       help='Maximum steps, default 50')
    
    args = parser.parse_args()
    
    main_wall_following(method=args.method, max_steps=args.steps)

