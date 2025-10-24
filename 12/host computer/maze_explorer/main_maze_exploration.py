#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Complete Maze Exploration Main Program
Use wall-following strategy to explore maze, detect exit, then return to starting point
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from communication.ble_comm import BLERobotComm
from perception.wall_detector import WallDetector
from perception.exit_detector import ExitDetector
from control.motion_controller import MotionController, WallFollower
from strategy.path_tracker import PathTracker
import config


def wait_for_lidar_data(comm, detector, timeout=5.0):
    """Wait for and get lidar data
    
    Args:
        comm: BLERobotComm instance
        detector: WallDetector instance
        timeout: Timeout duration (seconds)
    
    Returns:
        dict: Wall detection result, returns None on timeout
    """
    walls_data = {'ready': False}
    
    def on_lidar(data):
        distances = data['distances_mm']
        walls = detector.detect(distances)
        walls_data['walls'] = walls
        walls_data['ready'] = True
    
    # Temporarily set callback
    original_callback = comm.on_lidar_frame
    comm.on_lidar_frame = on_lidar
    
    # Wait for data
    start_time = time.time()
    while not walls_data['ready']:
        time.sleep(0.1)
        if time.time() - start_time > timeout:
            print("[WARN] Timeout waiting for lidar data")
            comm.on_lidar_frame = original_callback
            return None
    
    # Restore original callback
    comm.on_lidar_frame = original_callback
    
    return walls_data['walls']


def main_maze_exploration(method='right', max_steps=100):
    """Complete maze exploration main program
    
    Args:
        method: str, 'right' (right-hand rule) or 'left' (left-hand rule)
        max_steps: int, maximum exploration steps
    """
    print("=" * 70)
    print("Maze Exploration System")
    print("=" * 70)
    print(f"Strategy: {'Right-hand rule' if method == 'right' else 'Left-hand rule'}")
    print(f"Maximum steps: {max_steps}")
    print()
    print("Features:")
    print("  1. Start from origin, explore using wall-following strategy")
    print("  2. Stop when exit is detected (distance >1500mm)")
    print("  3. Use A* algorithm to calculate shortest path back to origin")
    print()
    input("Press Enter to start...")
    print()
    
    # ========== Initialize Modules ==========
    print("[INIT] Initializing modules...")
    comm = BLERobotComm(config.BLE_ADDRESS, verbose=True)
    detector = WallDetector(cell_size=600, wall_threshold=400)
    exit_detector = ExitDetector(wall_threshold=400, exit_threshold=1500)
    motion = MotionController(comm, forward_time=2.5, turn_time=1.8, timeout=10.0)
    follower = WallFollower(motion)
    tracker = PathTracker()
    
    # ========== Connect to Robot ==========
    print("\n[CONNECT] Connecting to robot...")
    if not comm.connect():
        print("[ERROR] Connection failed")
        return
    
    print("[OK] Connection successful\n")
    
    # ========== Wait for Data Stabilization ==========
    print("[WAIT] Waiting for lidar data to stabilize...")
    time.sleep(2)
    print("[OK] System ready\n")
    
    # ========== Exploration Phase ==========
    print("=" * 70)
    print("Starting Exploration Phase")
    print("=" * 70)
    
    step_count = 0
    found_exit = False
    exit_info = None
    
    try:
        while step_count < max_steps and not found_exit:
            print(f"\n{'=' * 70}")
            print(f"Step #{step_count + 1}")
            print(f"{'=' * 70}")
            
            # Get lidar data
            print("[SENSE] Getting lidar data...")
            walls = wait_for_lidar_data(comm, detector)
            
            if walls is None:
                print("[ERROR] Unable to get lidar data, stopping exploration")
                break
            
            # Display wall status
            print(f"[SENSE] {detector.format_detection(walls)}")
            print("\n[SENSE] Environment visualization (facing: front):")
            print("       " + ("███" if walls['front'] else "   "))
            print(f"    {('███' if walls['left'] else '   ')}  Car {('███' if walls['right'] else '   ')}")
            print("       " + ("███" if walls['back'] else "   "))
            print()
            
            # Detect exit
            has_exit, exit_dir = exit_detector.detect_exit(walls)
            if has_exit:
                print(f"\n[SUCCESS] Exit found! Direction: {exit_dir}")
                print(f"[SUCCESS] Distance: {walls[f'{exit_dir}_dist']:.0f}mm")
                found_exit = True
                exit_info = exit_detector.get_exit_info(walls)
                break
            
            # Execute wall-following strategy
            if method == 'right':
                action = follower.follow_right_wall(walls)
            else:
                action = follower.follow_left_wall(walls)
            
            if action is None:
                print("[ERROR] Action execution failed, stopping exploration")
                break
            
            # Record path
            tracker.record_action(action)
            
            # Display current state
            state = tracker.get_current_state()
            print(f"\n[STATE] Position: {state['position']}, Facing: {state['facing']}, Total steps: {state['steps']}")
            
            step_count += 1
            
            # Brief delay
            time.sleep(0.5)
        
        # ========== Exploration Summary ==========
        print(f"\n\n{'=' * 70}")
        print("Exploration Phase Complete")
        print(f"{'=' * 70}")
        print(f"Total steps: {step_count}")
        print(f"Final position: {tracker.get_current_state()['position']}")
        print(f"Final facing: {tracker.get_current_state()['facing']}")
        
        if found_exit:
            print(f"\n[SUCCESS] Exit found!")
            print(f"  Primary exit direction: {exit_info['primary_exit']}")
            print(f"  Maximum distance: {exit_info['max_distance']:.0f}mm")
            print(f"  All exits: {', '.join(exit_info['exit_directions'])}")
        else:
            print(f"\n[INFO] No exit found (reached maximum steps or interrupted)")
        
        # ========== Return to Origin Phase ==========
        if found_exit:
            print(f"\n{'=' * 70}")
            print("Starting Return to Origin")
            print(f"{'=' * 70}")
            
            # Calculate return path
            print("\n[PLAN] Calculating return path...")
            return_actions = tracker.get_return_path()
            print(f"[PLAN] Return path: {len(return_actions)} steps")
            print(f"[PLAN] Action sequence: {', '.join(return_actions[:10])}{'...' if len(return_actions) > 10 else ''}")
            
            # Execute return path
            print("\n[EXECUTE] Executing return path...")
            for i, action in enumerate(return_actions):
                print(f"\n[RETURN {i+1}/{len(return_actions)}] Action: {action}")
                
                success = False
                if action == 'forward':
                    success = motion.move_forward()
                elif action == 'turn_left':
                    success = motion.turn_left_90()
                elif action == 'turn_right':
                    success = motion.turn_right_90()
                elif action == 'turn_around':
                    success = motion.turn_180()
                
                if not success:
                    print(f"[ERROR] Return action failed, stopping return")
                    break
            
            print(f"\n{'=' * 70}")
            print("[SUCCESS] Returned to origin!")
            print(f"{'=' * 70}")
        
    except KeyboardInterrupt:
        print("\n\n[INFO] User interrupted")
        motion.stop()
    
    except Exception as e:
        print(f"\n[ERROR] Exception: {e}")
        import traceback
        traceback.print_exc()
        motion.stop()
    
    finally:
        # ========== Cleanup ==========
        print("\n[CLEANUP] Disconnecting...")
        comm.disconnect()
        print("[OK] Program ended")


if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='Complete Maze Exploration')
    parser.add_argument('-m', '--method', type=str, default='right',
                       choices=['right', 'left'],
                       help='Wall-following method: right (right-hand rule) or left (left-hand rule)')
    parser.add_argument('-s', '--steps', type=int, default=100,
                       help='Maximum exploration steps, default 100')
    
    args = parser.parse_args()
    
    main_maze_exploration(method=args.method, max_steps=args.steps)

