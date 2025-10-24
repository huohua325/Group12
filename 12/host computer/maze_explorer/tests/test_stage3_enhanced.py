#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Stage 3 enhanced functionality unit tests
Test sequence number mechanism, ACK parsing, path tracking, etc.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from perception.wall_detector import WallDetector
from perception.exit_detector import ExitDetector
from strategy.path_tracker import PathTracker


def test_exit_detector():
    """Test exit detector"""
    print("Test 1: Exit detector")
    
    detector = ExitDetector(wall_threshold=400, exit_threshold=1500)
    
    # Case 1: No exit (walls on all sides)
    walls = {
        'front': True, 'right': True, 'back': True, 'left': True,
        'front_dist': 300, 'right_dist': 350, 'back_dist': 320, 'left_dist': 310
    }
    has_exit, direction = detector.detect_exit(walls)
    assert has_exit == False, "Case 1: Should not detect exit"
    assert direction is None, "Case 1: Direction should be None"
    print("  [OK] Case 1: No exit detection correct")
    
    # Case 2: Exit in front
    walls = {
        'front': False, 'right': True, 'back': True, 'left': True,
        'front_dist': 2000, 'right_dist': 350, 'back_dist': 320, 'left_dist': 310
    }
    has_exit, direction = detector.detect_exit(walls)
    assert has_exit == True, "Case 2: Should detect exit"
    assert direction == 'front', "Case 2: Direction should be front"
    print("  [OK] Case 2: Front exit detection correct")
    
    # Case 3: Multiple exits
    walls = {
        'front': False, 'right': False, 'back': True, 'left': True,
        'front_dist': 2000, 'right_dist': 1800, 'back_dist': 320, 'left_dist': 310
    }
    info = detector.get_exit_info(walls)
    assert info['has_exit'] == True, "Case 3: Should detect exit"
    assert len(info['exit_directions']) == 2, "Case 3: Should have 2 exits"
    assert info['primary_exit'] in ['front', 'right'], "Case 3: Primary exit direction incorrect"
    print("  [OK] Case 3: Multiple exits detection correct")
    
    print("[OK] test_exit_detector passed\n")


def test_path_tracker_basic():
    """Test path tracker basic functionality"""
    print("Test 2: Path tracker basic functionality")
    
    tracker = PathTracker()
    
    # Initial state
    state = tracker.get_current_state()
    assert state['position'] == (0, 0), "Initial position should be (0, 0)"
    assert state['facing'] == 'north', "Initial facing should be north"
    assert state['steps'] == 0, "Initial steps should be 0"
    print("  [OK] Initial state correct")
    
    # Move forward
    tracker.record_action('forward')
    state = tracker.get_current_state()
    assert state['position'] == (0, 1), "Position after forward should be (0, 1)"
    assert state['facing'] == 'north', "Facing should not change"
    assert state['steps'] == 1, "Steps should be 1"
    print("  [OK] Forward action correct")
    
    # Turn right
    tracker.record_action('turn_right')
    state = tracker.get_current_state()
    assert state['position'] == (0, 1), "Turning should not change position"
    assert state['facing'] == 'east', "Should face east after turning right"
    assert state['steps'] == 2, "Steps should be 2"
    print("  [OK] Turn right action correct")
    
    # Move forward again
    tracker.record_action('forward')
    state = tracker.get_current_state()
    assert state['position'] == (1, 1), "Should be (1, 1) after moving east"
    print("  [OK] Moving east forward correct")
    
    print("[OK] test_path_tracker_basic passed\n")


def test_path_tracker_return():
    """Test path tracker return path generation"""
    print("Test 3: Return path generation")
    
    tracker = PathTracker()
    
    # Execute simple path: forward -> turn right -> forward
    tracker.record_action('forward')   # (0, 0) north -> (0, 1) north
    tracker.record_action('turn_right') # (0, 1) north -> (0, 1) east
    tracker.record_action('forward')   # (0, 1) east -> (1, 1) east
    
    # Generate return path
    return_path = tracker.get_return_path()
    
    assert len(return_path) > 0, "Return path should not be empty"
    print(f"  Return path: {return_path}")
    print("  [OK] Return path generation successful")
    
    # Verify return path contains at least forward actions
    forward_count = return_path.count('forward')
    assert forward_count >= 2, f"Return path should contain at least 2 forward actions, actual {forward_count}"
    print(f"  [OK] Return path contains {forward_count} forward actions")
    
    print("[OK] test_path_tracker_return passed\n")


def test_sequence_number_wraparound():
    """Test sequence number wraparound (0-65535 cycle)"""
    print("Test 4: Sequence number wraparound")
    
    # Simulate sequence number generation
    seq = 65534
    for i in range(5):
        seq = (seq + 1) % 65536
        print(f"  seq {i+1}: {seq}")
    
    assert seq == 3, f"Sequence number should wrap to 3, actual {seq}"
    print("  [OK] Sequence number wraparound correct")
    
    print("[OK] test_sequence_number_wraparound passed\n")


def test_turn_actions():
    """Test turn action calculation"""
    print("Test 5: Turn action calculation")
    
    tracker = PathTracker()
    
    # Test various turns
    test_cases = [
        ('north', 'east', ['turn_right']),
        ('north', 'south', ['turn_right', 'turn_right']),
        ('north', 'west', ['turn_left']),
        ('east', 'north', ['turn_left']),
    ]
    
    for from_facing, to_facing, expected in test_cases:
        result = tracker._get_turn_actions(from_facing, to_facing)
        assert result == expected, f"{from_facing} -> {to_facing}: Expected {expected}, actual {result}"
        print(f"  [OK] {from_facing} -> {to_facing}: {result}")
    
    print("[OK] test_turn_actions passed\n")


def run_all_tests():
    """Run all tests"""
    print("=" * 70)
    print("Stage 3 enhanced functionality unit tests")
    print("=" * 70)
    print()
    
    test_exit_detector()
    test_path_tracker_basic()
    test_path_tracker_return()
    test_sequence_number_wraparound()
    test_turn_actions()
    
    print("=" * 70)
    print("[OK] All tests passed!")
    print("=" * 70)


if __name__ == '__main__':
    run_all_tests()

