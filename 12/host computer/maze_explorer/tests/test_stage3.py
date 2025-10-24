#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Stage 3 functionality unit tests
Test wall detection and motion control modules
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from perception.wall_detector import WallDetector
from control.motion_controller import MotionController, WallFollower


def test_wall_detector_basic():
    """Test wall detector basic functionality"""
    detector = WallDetector(cell_size=600, wall_threshold=400)
    
    # Simulate radar data (walls on all sides, distance 300mm)
    distances = [300.0] * 500
    walls = detector.detect(distances)
    
    # Verify results
    assert walls['front'] == True, "Should detect wall in front"
    assert walls['right'] == True, "Should detect wall on right"
    assert walls['back'] == True, "Should detect wall behind"
    assert walls['left'] == True, "Should detect wall on left"
    
    print("[OK] test_wall_detector_basic passed")


def test_wall_detector_no_walls():
    """Test wall detector (no walls case)"""
    detector = WallDetector(cell_size=600, wall_threshold=400)
    
    # Simulate radar data (no walls on all sides, distance 800mm)
    distances = [800.0] * 500
    walls = detector.detect(distances)
    
    # Verify results
    assert walls['front'] == False, "Should not detect wall in front"
    assert walls['right'] == False, "Should not detect wall on right"
    assert walls['back'] == False, "Should not detect wall behind"
    assert walls['left'] == False, "Should not detect wall on left"
    
    print("[OK] test_wall_detector_no_walls passed")


def test_wall_detector_partial():
    """Test wall detector (partial walls)"""
    detector = WallDetector(cell_size=600, wall_threshold=400)
    
    # Simulate radar data (wall in front, no walls in other directions)
    distances = [800.0] * 500
    # Set front (around index 0) to close distance
    for i in range(0, 14):
        distances[i] = 300.0
    for i in range(486, 500):
        distances[i] = 300.0
    
    walls = detector.detect(distances)
    
    # Verify results
    assert walls['front'] == True, "Should detect wall in front"
    assert walls['right'] == False, "Should not detect wall on right"
    assert walls['back'] == False, "Should not detect wall behind"
    assert walls['left'] == False, "Should not detect wall on left"
    
    print("[OK] test_wall_detector_partial passed")


def test_wall_detector_format():
    """Test wall detection result formatting"""
    detector = WallDetector(cell_size=600, wall_threshold=400)
    
    distances = [500.0] * 500
    walls = detector.detect(distances)
    
    formatted = detector.format_detection(walls)
    
    # Verify formatted string contains key information
    assert 'Front' in formatted
    assert 'Right' in formatted
    assert 'Back' in formatted
    assert 'Left' in formatted
    assert 'mm' in formatted
    
    print("[OK] test_wall_detector_format passed")


def test_wall_follower_strategy():
    """Test wall following strategy logic"""
    # Create mock motion controller
    class MockMotionController:
        def __init__(self):
            self.actions = []
        
        def move_forward(self, cells=1):
            self.actions.append('forward')
        
        def turn_left_90(self):
            self.actions.append('turn_left')
        
        def turn_right_90(self):
            self.actions.append('turn_right')
        
        def turn_180(self):
            self.actions.append('turn_180')
    
    mock_motion = MockMotionController()
    follower = WallFollower(mock_motion)
    
    # Test right-hand rule: no wall on right
    walls = {'front': True, 'right': False, 'back': True, 'left': True}
    action = follower.follow_right_wall(walls)
    assert action == 'turn_right'
    assert 'turn_right' in mock_motion.actions
    
    # Test right-hand rule: no wall in front
    mock_motion.actions = []
    walls = {'front': False, 'right': True, 'back': True, 'left': True}
    action = follower.follow_right_wall(walls)
    assert action == 'forward'
    assert 'forward' in mock_motion.actions
    
    # Test right-hand rule: no wall on left
    mock_motion.actions = []
    walls = {'front': True, 'right': True, 'back': True, 'left': False}
    action = follower.follow_right_wall(walls)
    assert action == 'turn_left'
    assert 'turn_left' in mock_motion.actions
    
    # Test right-hand rule: dead end
    mock_motion.actions = []
    walls = {'front': True, 'right': True, 'back': True, 'left': True}
    action = follower.follow_right_wall(walls)
    assert action == 'turn_around'
    assert 'turn_180' in mock_motion.actions
    
    print("[OK] test_wall_follower_strategy passed")


def run_all_tests():
    """Run all tests"""
    print("=" * 70)
    print("Stage 3 unit tests")
    print("=" * 70)
    print()
    
    test_wall_detector_basic()
    test_wall_detector_no_walls()
    test_wall_detector_partial()
    test_wall_detector_format()
    test_wall_follower_strategy()
    
    print()
    print("=" * 70)
    print("[OK] All tests passed!")
    print("=" * 70)


if __name__ == '__main__':
    run_all_tests()

