#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Exit detection module
Detects maze exits (wall openings) based on radar data
"""


class ExitDetector:
    """Maze exit detector"""
    
    def __init__(self, wall_threshold=400, exit_threshold=1500):
        """
        Initialize exit detector
        
        Args:
            wall_threshold: Wall detection threshold in mm, default 400mm
            exit_threshold: Exit detection threshold in mm, default 1500mm
                           When distance in a direction > this threshold, it's considered an exit
        """
        self.wall_threshold = wall_threshold
        self.exit_threshold = exit_threshold
    
    def detect_exit(self, walls):
        """Detect if reached an exit
        
        Exit characteristics: sudden increase in distance in a direction (>1500mm), indicating a wall opening
        
        Args:
            walls: Return value from WallDetector.detect(), containing:
                {
                    'front': bool,
                    'right': bool,
                    'back': bool,
                    'left': bool,
                    'front_dist': float,
                    'right_dist': float,
                    'back_dist': float,
                    'left_dist': float
                }
        
        Returns:
            tuple: (has_exit, direction)
                has_exit: bool, whether an exit is detected
                direction: str, exit direction ('front', 'right', 'back', 'left')
                           If no exit, direction is None
        """
        # Check all directions, looking for sudden increase in distance
        for direction in ['front', 'right', 'back', 'left']:
            dist = walls[f'{direction}_dist']
            
            # Exit detection: distance > exit threshold
            if dist > self.exit_threshold:
                return True, direction
        
        return False, None
    
    def get_exit_info(self, walls):
        """Get detailed exit information
        
        Args:
            walls: Return value from WallDetector.detect()
        
        Returns:
            dict: {
                'has_exit': bool,
                'exit_directions': list,  # All exit directions
                'primary_exit': str,      # Primary exit direction (largest distance)
                'max_distance': float     # Maximum distance in mm
            }
        """
        exit_dirs = []
        max_dist = 0
        primary_dir = None
        
        for direction in ['front', 'right', 'back', 'left']:
            dist = walls[f'{direction}_dist']
            
            if dist > self.exit_threshold:
                exit_dirs.append(direction)
                
                if dist > max_dist:
                    max_dist = dist
                    primary_dir = direction
        
        return {
            'has_exit': len(exit_dirs) > 0,
            'exit_directions': exit_dirs,
            'primary_exit': primary_dir,
            'max_distance': max_dist
        }

