#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Wall detection module
Detects surrounding walls based on 500-point radar data
"""


class WallDetector:
    """Wall detector"""
    
    def __init__(self, cell_size=600, wall_threshold=400):
        """
        Initialize wall detector
        
        Args:
            cell_size: Cell size in mm, default 600mm
            wall_threshold: Wall detection threshold in mm, default 400mm
        """
        self.cell_size = cell_size
        self.threshold = wall_threshold
    
    def detect(self, distances_mm):
        """
        Detect surrounding walls
        
        Principle:
        - Radar has 500 points, each point is about 0.72 degrees
        - Front (0°):   around index 0
        - Right (90°):  around index 125  
        - Back (180°):  around index 250
        - Left (270°):  around index 375
        
        Args:
            distances_mm: 500 distance values in mm, tuple or list
        
        Returns:
            dict: {
                'front': bool,       # Wall in front
                'right': bool,       # Wall on right
                'back': bool,        # Wall behind
                'left': bool,        # Wall on left
                'front_dist': float, # Minimum distance in front (mm)
                'right_dist': float, # Minimum distance on right (mm)
                'back_dist': float,  # Minimum distance behind (mm)
                'left_dist': float   # Minimum distance on left (mm)
            }
        """
        # Front: around index 0 (-10 to +10 degrees, about 28 points)
        front_indices = list(range(0, 14)) + list(range(486, 500))
        front_dists = [distances_mm[i] for i in front_indices if distances_mm[i] > 0]
        front_min_dist = min(front_dists) if front_dists else 9999.0
        has_front_wall = front_min_dist < self.threshold
        
        # Right: around index 125 (90 degrees ±10 degrees)
        right_indices = range(111, 139)  # 80-100 degrees
        right_dists = [distances_mm[i] for i in right_indices if distances_mm[i] > 0]
        right_min_dist = min(right_dists) if right_dists else 9999.0
        has_right_wall = right_min_dist < self.threshold
        
        # Back: around index 250 (180 degrees ±10 degrees)
        back_indices = range(236, 264)  # 170-190 degrees
        back_dists = [distances_mm[i] for i in back_indices if distances_mm[i] > 0]
        back_min_dist = min(back_dists) if back_dists else 9999.0
        has_back_wall = back_min_dist < self.threshold
        
        # Left: around index 375 (270 degrees ±10 degrees)
        left_indices = range(361, 389)  # 260-280 degrees
        left_dists = [distances_mm[i] for i in left_indices if distances_mm[i] > 0]
        left_min_dist = min(left_dists) if left_dists else 9999.0
        has_left_wall = left_min_dist < self.threshold
        
        return {
            'front': has_front_wall,
            'right': has_right_wall,
            'back': has_back_wall,
            'left': has_left_wall,
            'front_dist': front_min_dist,
            'right_dist': right_min_dist,
            'back_dist': back_min_dist,
            'left_dist': left_min_dist
        }
    
    def format_detection(self, walls):
        """
        Format wall detection results into readable string
        
        Args:
            walls: Return value from detect()
        
        Returns:
            str: Formatted string
        """
        symbols = []
        symbols.append(f"Front: {'Wall' if walls['front'] else 'Open'} ({walls['front_dist']:.0f}mm)")
        symbols.append(f"Right: {'Wall' if walls['right'] else 'Open'} ({walls['right_dist']:.0f}mm)")
        symbols.append(f"Back: {'Wall' if walls['back'] else 'Open'} ({walls['back_dist']:.0f}mm)")
        symbols.append(f"Left: {'Wall' if walls['left'] else 'Open'} ({walls['left_dist']:.0f}mm)")
        return " | ".join(symbols)

