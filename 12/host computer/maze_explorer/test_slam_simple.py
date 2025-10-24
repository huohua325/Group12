#!/usr/bin/env python3
"""
SLAM Simplified Test (without matplotlib, only save map)
Suitable for cases where matplotlib display has issues
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from communication.ble_comm import BLERobotComm
from slam.breezy_slam import MazeSLAM
import config
import numpy as np


def test_slam_simple(duration=30):
    """
    Simplified SLAM test (no visualization)
    
    Args:
        duration: Test duration (seconds)
    """
    print("=" * 60)
    print("🤖 Maze Exploration System - SLAM Simplified Test (No Visualization)")
    print("=" * 60)
    print(f"Device address: {config.BLE_ADDRESS}")
    print(f"Test duration: {duration} seconds\n")
    
    # Initialize SLAM
    print("📐 Initializing SLAM system...")
    slam = MazeSLAM(
        map_size_pixels=config.SLAM_MAP_SIZE,
        map_size_meters=config.SLAM_MAP_SIZE * config.SLAM_RESOLUTION / 1000
    )
    
    # Initialize BLE communication
    print("🔵 Initializing BLE communication...")
    comm = BLERobotComm(config.BLE_ADDRESS)
    
    # Statistics
    frame_count = 0
    start_time = None
    
    def on_lidar_frame(data):
        """Lidar data callback"""
        nonlocal frame_count, start_time
        
        if start_time is None:
            start_time = time.time()
            print("✅ Started receiving lidar data!\n")
        
        frame_count += 1
        distances_mm = data['distances_mm']
        
        # Update SLAM
        slam.update(distances_mm)
        
        # Get pose
        x, y, theta = slam.get_position()
        
        # Display progress
        if frame_count % 10 == 0:
            elapsed = time.time() - start_time
            fps = frame_count / elapsed if elapsed > 0 else 0
            
            # Get map statistics
            map_array = slam.get_map_array()
            occupied = np.sum(map_array < 128)  # Black pixels (obstacles)
            free = np.sum(map_array >= 128)     # White pixels (empty)
            
            print(f"📡 Frame: {frame_count:4d} | "
                  f"Position: ({x:6.0f},{y:6.0f})mm | "
                  f"Angle: {theta:5.1f}° | "
                  f"FPS: {fps:4.1f} | "
                  f"Map: obstacles={occupied} empty={free}")
    
    comm.on_lidar_frame = on_lidar_frame
    
    # Connect
    print("Connecting...")
    if not comm.connect():
        print("❌ Connection failed")
        return
    
    print("✅ Connection successful\n")
    
    try:
        # Robot will automatically upload lidar data
        print("📡 Waiting for lidar data (robot auto-upload)...\n")
        print(f"⏳ Running for {duration} seconds...\n")
        
        # Run for specified duration
        for i in range(duration):
            time.sleep(1)
            if (i + 1) % 5 == 0:
                print(f"   ⏱️  Run for {i + 1}/{duration} seconds")
        
        # Display final statistics
        stats = slam.get_stats()
        print(f"\n{'='*60}")
        print("✅ SLAM test completed")
        print(f"{'='*60}")
        print(f"📊 Statistics:")
        print(f"   Lidar frames: {frame_count}")
        print(f"   SLAM updates: {stats['update_count']}")
        print(f"   Final position: ({stats['position_x_mm']:.0f}, {stats['position_y_mm']:.0f})mm")
        print(f"   Final angle: {stats['position_theta_deg']:.1f}°")
        
        if start_time:
            elapsed = time.time() - start_time
            print(f"   Average FPS: {frame_count / elapsed:.1f}")
        
        print(f"{'='*60}\n")
        
        # Save map
        if frame_count > 0:
            print("💾 Saving map...")
            try:
                slam.save_map('slam_map.dat')
                
                # Save as PNG image
                from PIL import Image
                map_array = slam.get_map_array()
                img = Image.fromarray(map_array, mode='L')
                img.save('slam_map.png')
                
                print("✅ Map saved:")
                print(f"   - slam_map.dat (raw data)")
                print(f"   - slam_map.png (image)")
            except Exception as e:
                print(f"❌ Save failed: {e}")
        
    except KeyboardInterrupt:
        print("\n\n⚠️ User interrupted")
    finally:
        print("\n🔵 Cleaning up resources...")
        comm.disconnect()
        print("✅ Test completed")


if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='SLAM Simplified Test (No Visualization)')
    parser.add_argument('-t', '--time', type=int, default=30,
                       help='Test duration (seconds), default 30 seconds')
    
    args = parser.parse_args()
    
    test_slam_simple(duration=args.time)

