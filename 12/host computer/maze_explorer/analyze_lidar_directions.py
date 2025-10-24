#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Lidar direction distance analysis script
Real-time display of distance data for each direction
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from communication.ble_comm import BLERobotComm
import config


def analyze_lidar_directions(duration=60, update_interval=1.0):
    """
    Analyze lidar distance for each direction
    
    Args:
        duration: Runtime duration in seconds
        update_interval: Display update interval in seconds
    """
    print("=" * 80)
    print("Lidar Direction Distance Analysis")
    print("=" * 80)
    print("Lidar data: 500 points, each point about 0.72 degrees")
    print()
    print("Direction definitions:")
    print("  Front(0°):     index 0-13, 486-499  (~28 points)")
    print("  Front-Right(45°):    index 55-68          (~14 points)")
    print("  Right(90°):    index 111-138        (~28 points)")
    print("  Back-Right(135°):   index 180-193        (~14 points)")
    print("  Back(180°):   index 236-263        (~28 points)")
    print("  Back-Left(225°):   index 305-318        (~14 points)")
    print("  Left(270°):   index 361-388        (~28 points)")
    print("  Front-Left(315°):   index 430-443        (~14 points)")
    print()
    print(f"Runtime duration: {duration} seconds")
    print(f"Update interval: {update_interval} seconds")
    print()
    input("Press Enter to start...")
    print()
    
    # Initialize
    print("[INIT] Initializing BLE communication...")
    comm = BLERobotComm(config.BLE_ADDRESS, verbose=False)
    
    # Connect
    print("[CONNECT] Connecting...")
    if not comm.connect():
        print("[ERROR] Connection failed")
        return
    
    print("[OK] Connection successful\n")
    time.sleep(1)
    
    # Statistics data
    frame_count = 0
    last_update_time = time.time()
    
    # Distance statistics for each direction
    direction_stats = {
        'Front(0°)': [],
        'Front-Right(45°)': [],
        'Right(90°)': [],
        'Back-Right(135°)': [],
        'Back(180°)': [],
        'Back-Left(225°)': [],
        'Left(270°)': [],
        'Front-Left(315°)': []
    }
    
    def get_direction_distances(distances):
        """Extract distance data for each direction
        
        Args:
            distances: 500 distance values in mm
        
        Returns:
            dict: Min, average, max distance for each direction
        """
        # Define index ranges for each direction
        directions = {
            'Front(0°)': list(range(0, 14)) + list(range(486, 500)),
            'Front-Right(45°)': list(range(55, 69)),
            'Right(90°)': list(range(111, 139)),
            'Back-Right(135°)': list(range(180, 194)),
            'Back(180°)': list(range(236, 264)),
            'Back-Left(225°)': list(range(305, 319)),
            'Left(270°)': list(range(361, 389)),
            'Front-Left(315°)': list(range(430, 444))
        }
        
        result = {}
        for dir_name, indices in directions.items():
            # Extract valid distances
            valid_dists = [distances[i] for i in indices if distances[i] > 0]
            
            if valid_dists:
                result[dir_name] = {
                    'min': min(valid_dists),
                    'avg': sum(valid_dists) / len(valid_dists),
                    'max': max(valid_dists),
                    'count': len(valid_dists),
                    'total': len(indices)
                }
            else:
                result[dir_name] = {
                    'min': 0,
                    'avg': 0,
                    'max': 0,
                    'count': 0,
                    'total': len(indices)
                }
        
        return result
    
    def on_lidar_frame(data):
        """Lidar data callback"""
        nonlocal frame_count, last_update_time
        
        frame_count += 1
        distances = data['distances_mm']
        
        # Get distance for each direction
        dir_dists = get_direction_distances(distances)
        
        # Record to statistics
        for dir_name in direction_stats.keys():
            direction_stats[dir_name].append(dir_dists[dir_name]['avg'])
        
        # Regular display update
        now = time.time()
        if now - last_update_time >= update_interval:
            print(f"\n{'=' * 80}")
            print(f"[Frame #{frame_count}] Timestamp: {data['timestamp']}ms")
            print(f"{'=' * 80}")
            
            # Display distance for each direction (current frame)
            print(f"\n{'Direction':<12} {'Min':>8} {'Avg':>8} {'Max':>8} {'Valid':>10}")
            print("-" * 80)
            
            for dir_name in ['Front(0°)', 'Front-Right(45°)', 'Right(90°)', 'Back-Right(135°)',
                           'Back(180°)', 'Back-Left(225°)', 'Left(270°)', 'Front-Left(315°)']:
                d = dir_dists[dir_name]
                if d['count'] > 0:
                    print(f"{dir_name:<12} {d['min']:>7.0f}mm {d['avg']:>7.0f}mm "
                          f"{d['max']:>7.0f}mm {d['count']:>3}/{d['total']:<3}pts")
                else:
                    print(f"{dir_name:<12} {'--':>7}   {'--':>7}   {'--':>7}   "
                          f"{d['count']:>3}/{d['total']:<3}pts")
            
            # Display total valid points
            total_valid = sum(1 for d in distances if d > 0)
            print(f"\nTotal valid points: {total_valid}/500")
            
            # Simple visualization (based on average distance)
            print(f"\n{'Visualization (based on average distance)':^80}")
            print("-" * 80)
            
            # Determine walls (threshold 400mm)
            wall_threshold = 400
            front = dir_dists['Front(0°)']['avg']
            right = dir_dists['Right(90°)']['avg']
            back = dir_dists['Back(180°)']['avg']
            left = dir_dists['Left(270°)']['avg']
            
            front_status = "Wall" if (front > 0 and front < wall_threshold) else f"{front:.0f}"
            right_status = "Wall" if (right > 0 and right < wall_threshold) else f"{right:.0f}"
            back_status = "Wall" if (back > 0 and back < wall_threshold) else f"{back:.0f}"
            left_status = "Wall" if (left > 0 and left < wall_threshold) else f"{left:.0f}"
            
            print(f"{'':>35}{front_status:^10}")
            print(f"{left_status:^10}{'Robot':^50}{right_status:^10}")
            print(f"{'':>35}{back_status:^10}")
            
            last_update_time = now
    
    comm.on_lidar_frame = on_lidar_frame
    
    try:
        # Run for specified duration
        print("=" * 80)
        print("Starting to listen to lidar data...")
        print("=" * 80)
        time.sleep(duration)
        
        # Final statistics
        print(f"\n\n{'=' * 80}")
        print("Final Statistics (average of all frames)")
        print(f"{'=' * 80}")
        print(f"Total frames: {frame_count}")
        print(f"Average frame rate: {frame_count / duration:.2f} Hz")
        print()
        
        print(f"{'Direction':<12} {'Avg Distance':>10} {'Std Dev':>10} {'Min':>8} {'Max':>8}")
        print("-" * 80)
        
        for dir_name in ['Front(0°)', 'Front-Right(45°)', 'Right(90°)', 'Back-Right(135°)',
                       'Back(180°)', 'Back-Left(225°)', 'Left(270°)', 'Front-Left(315°)']:
            data = direction_stats[dir_name]
            if data:
                avg = sum(data) / len(data)
                min_val = min(data)
                max_val = max(data)
                
                # Calculate standard deviation
                if len(data) > 1:
                    variance = sum((x - avg) ** 2 for x in data) / len(data)
                    std_dev = variance ** 0.5
                else:
                    std_dev = 0
                
                print(f"{dir_name:<12} {avg:>9.0f}mm {std_dev:>9.0f}mm "
                      f"{min_val:>7.0f}mm {max_val:>7.0f}mm")
        
        print(f"\n{'=' * 80}")
        print("Data Quality Assessment")
        print(f"{'=' * 80}")
        
        # Assess data stability
        for dir_name in ['Front(0°)', 'Right(90°)', 'Back(180°)', 'Left(270°)']:
            data = direction_stats[dir_name]
            if data and len(data) > 1:
                avg = sum(data) / len(data)
                variance = sum((x - avg) ** 2 for x in data) / len(data)
                std_dev = variance ** 0.5
                
                # Determine stability (std dev < 50mm is stable)
                if std_dev < 50:
                    status = "[Stable]"
                elif std_dev < 100:
                    status = "[Moderately Stable]"
                else:
                    status = "[Unstable]"
                
                print(f"{dir_name:<12}: Avg={avg:>6.0f}mm StdDev={std_dev:>5.0f}mm {status}")
        
    except KeyboardInterrupt:
        print("\n\n[INFO] User interrupted")
    finally:
        comm.disconnect()


if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='Lidar direction distance analysis')
    parser.add_argument('-t', '--time', type=int, default=60,
                       help='Runtime duration in seconds, default 60 seconds')
    parser.add_argument('-i', '--interval', type=float, default=1.0,
                       help='Display update interval in seconds, default 1.0 seconds')
    
    args = parser.parse_args()
    
    analyze_lidar_directions(duration=args.time, update_interval=args.interval)

