#!/usr/bin/env python3
"""
BreezySLAM wrapper module
Supports SLAM mapping with 500-point radar data

Reference: breezyslam library documentation
"""

import numpy as np
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import Laser


class STM32Laser(Laser):
    """STM32 laser radar model (500 points, 0.72 degree interval)"""
    
    def __init__(self):
        """
        Initialize radar parameters
        
        Parameter description:
        - scan_size: 500 points (keep original data)
        - scan_rate_hz: 10Hz
        - detection_angle_degrees: 360 degrees
        - distance_no_detection_mm: 12000mm (12 meters)
        - detection_margin: 0
        - offset_mm: 0 (radar center offset)
        """
        Laser.__init__(
            self,
            scan_size=500,                      # 500 points/scan
            scan_rate_hz=10.0,                  # 10Hz update rate
            detection_angle_degrees=360.0,      # 360 degree coverage
            distance_no_detection_mm=12000,     # Maximum detection distance 12m
            detection_margin=0,                 # Detection boundary
            offset_mm=0                         # Radar offset
        )


class MazeSLAM:
    """Maze SLAM system
    
    Features:
    - Real-time reception of 500-point radar data
    - Build occupancy grid map
    - Track robot pose
    - Save and load maps
    """
    
    def __init__(self, map_size_pixels=500, map_size_meters=5.0):
        """
        Initialize SLAM system
        
        Args:
            map_size_pixels: Map size in pixels, default 500
            map_size_meters: Map actual size in meters, default 5 meters
                            For 2.4m×2.4m maze, 5 meters is sufficient
        """
        self.map_size_pixels = map_size_pixels
        self.map_size_meters = map_size_meters
        
        # Create radar model
        self.laser = STM32Laser()
        
        # Create SLAM instance
        self.slam = RMHC_SLAM(
            laser=self.laser,
            map_size_pixels=map_size_pixels,
            map_size_meters=map_size_meters
        )
        
        # Statistics
        self.update_count = 0
        
        print(f"[OK] SLAM initialization completed")
        print(f"   Map size: {map_size_pixels}x{map_size_pixels} pixels")
        print(f"   Actual size: {map_size_meters}x{map_size_meters} meters")
        print(f"   Resolution: {map_size_meters / map_size_pixels * 1000:.1f} mm/pixel")
    
    def update(self, distances_mm):
        """
        Update SLAM map
        
        Args:
            distances_mm: tuple or list, 500 distance values (unit: millimeters)
        
        Note:
        - BreezySLAM requires list type
        - Distance unit must be millimeters
        """
        # Convert to list (BreezySLAM requirement)
        if not isinstance(distances_mm, list):
            distances_mm = list(distances_mm)
        
        # Ensure 500 points
        if len(distances_mm) != 500:
            print(f"⚠️ Warning: Radar points not 500 ({len(distances_mm)})")
            return
        
        # Update SLAM
        self.slam.update(distances_mm)
        self.update_count += 1
    
    def get_position(self):
        """
        Get robot current pose
        
        Returns:
            tuple: (x_mm, y_mm, theta_deg)
            - x_mm: X coordinate (millimeters)
            - y_mm: Y coordinate (millimeters)
            - theta_deg: Orientation angle (degrees)
        """
        x_mm, y_mm, theta_deg = self.slam.getpos()
        return (x_mm, y_mm, theta_deg)
    
    def get_map(self):
        """
        Get map data (byte array)
        
        Returns:
            bytearray: Map data, size is map_size_pixels × map_size_pixels
        """
        # BreezySLAM's getmap requires passing a bytearray as buffer
        mapbytes = bytearray(self.map_size_pixels * self.map_size_pixels)
        self.slam.getmap(mapbytes)
        return mapbytes
    
    def get_map_array(self):
        """
        Get map array (numpy format, convenient for visualization)
        
        Returns:
            np.ndarray: Array with shape (map_size_pixels, map_size_pixels)
                        Value range 0-255 (0=obstacle, 255=empty)
        """
        # Create buffer and get map data
        mapbytes = bytearray(self.map_size_pixels * self.map_size_pixels)
        self.slam.getmap(mapbytes)
        
        # Convert to numpy array
        map_array = np.frombuffer(mapbytes, dtype=np.uint8)
        return map_array.reshape((self.map_size_pixels, self.map_size_pixels))
    
    def save_map(self, filename='slam_map.dat'):
        """
        Save map to file
        
        Args:
            filename: File name, default 'slam_map.dat'
        """
        map_bytes = self.get_map()
        with open(filename, 'wb') as f:
            f.write(map_bytes)
        print(f"[SAVED] Map saved: {filename}")
    
    def get_stats(self):
        """
        Get statistics
        
        Returns:
            dict: Statistics data
        """
        x, y, theta = self.get_position()
        return {
            'update_count': self.update_count,
            'position_x_mm': x,
            'position_y_mm': y,
            'position_theta_deg': theta,
            'map_size_pixels': self.map_size_pixels,
            'map_size_meters': self.map_size_meters
        }
    
    def reset_position(self, x_mm=0, y_mm=0, theta_deg=0):
        """
        Reset robot pose (if BreezySLAM supports)
        
        Args:
            x_mm: X coordinate (millimeters)
            y_mm: Y coordinate (millimeters)
            theta_deg: Orientation angle (degrees)
        """
        # Note: BreezySLAM may not support direct pose reset
        # If needed, may need to recreate SLAM instance
        print(f"⚠️ BreezySLAM does not support direct pose reset")
        print(f"   If reset is needed, please recreate MazeSLAM instance")


if __name__ == '__main__':
    # Simple test
    print("=" * 60)
    print("BreezySLAM module test")
    print("=" * 60)
    
    # Create SLAM instance
    slam = MazeSLAM(map_size_pixels=500, map_size_meters=5.0)
    
    # Simulate radar data (500 points, distance 1000mm)
    fake_distances = [1000] * 500
    
    # Update several times
    for i in range(5):
        slam.update(fake_distances)
        pos = slam.get_position()
        print(f"Update #{i+1}: Position ({pos[0]:.0f}, {pos[1]:.0f})mm, Angle {pos[2]:.1f}°")
    
    # Test map retrieval
    print("\nTesting map retrieval...")
    map_array = slam.get_map_array()
    print(f"  Map array shape: {map_array.shape}")
    print(f"  Map data range: {map_array.min()}-{map_array.max()}")
    
    # Display statistics
    stats = slam.get_stats()
    print(f"\nStatistics:")
    print(f"  Update count: {stats['update_count']}")
    print(f"  Map size: {stats['map_size_pixels']}×{stats['map_size_pixels']} pixels")
    
    print("\n✅ Test completed")

