#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Motion control module (Enhanced version)
Encapsulates precise motion control for the robot, supports sequence number tracking and action completion confirmation
"""

import time
import threading


class MotionController:
    """Motion controller (with sequence number tracking and timeout protection)"""
    
    def __init__(self, comm, forward_time=2.5, turn_time=1.8, timeout=40.0):
        """
        Initialize motion controller
        
        Args:
            comm: BLERobotComm instance
            forward_time: Expected time for moving one cell (600mm) in seconds, for display only
            turn_time: Expected time for 90-degree turn in seconds, for display only
            timeout: Action timeout in seconds, default 10 seconds
        """
        self.comm = comm
        self.forward_time = forward_time
        self.turn_time = turn_time
        self.timeout = timeout
        
        # Command tracking
        self.pending_seq = None
        self.completion_event = threading.Event()
        self.last_status = None
        
        # Register ACK callback
        self.comm.on_action_complete = self._on_action_complete
    
    def _on_action_complete(self, seq, status):
        """Action completion callback
        
        Args:
            seq: Command sequence number
            status: Completion status ('OK' or 'TIMEOUT')
        """
        if seq == self.pending_seq:
            self.last_status = status
            self.completion_event.set()
    
    def _execute_action(self, mode, expected_time, action_name):
        """Execute action and wait for completion
        
        Args:
            mode: Mode ID (0=stop, 1=forward, 3=turn left, 4=turn right)
            expected_time: Expected completion time in seconds
            action_name: Action name (for logging)
        
        Returns:
            bool: Returns True if action completed successfully
        
        Raises:
            RuntimeError: Command send failed
        """
        # Send command
        self.pending_seq = self.comm.send_command_with_seq(mode)
        if self.pending_seq < 0:
            raise RuntimeError(f"{action_name} command send failed")
        
        # Clear completion event
        self.completion_event.clear()
        self.last_status = None
        
        # Wait for completion (timeout protection)
        if not self.completion_event.wait(timeout=self.timeout):
            print(f"[WARN] {action_name} timeout ({self.timeout}s), sequence={self.pending_seq}")
            return False
        
        # Check status
        if self.last_status == 'TIMEOUT':
            print(f"[WARN] {action_name} lower computer reported timeout")
            return False
        elif self.last_status == 'OK':
            return True
        else:
            print(f"[WARN] {action_name} unknown status: {self.last_status}")
            return False
    
    def move_forward(self, cells=1):
        """
        Move forward specified number of cells
        
        Args:
            cells: Number of cells to move forward, default 1 cell (600mm)
        
        Returns:
            bool: Returns True if action completed successfully
        """
        print(f"[MOTION] Moving forward {cells} cells...")
        
        for i in range(cells):
            if cells > 1:
                print(f"[MOTION]   In progress: {i+1}/{cells}")
            
            if self._execute_action(1, self.forward_time, "Forward"):
                if i == cells - 1:
                    print("[MOTION] Forward movement completed")
            else:
                print(f"[MOTION] Forward movement failed (cell {i+1})")
                return False
        
        return True
    
    def turn_left_90(self):
        """
        Turn left 90 degrees
        
        Returns:
            bool: Returns True if action completed successfully
        """
        print("[MOTION] Turning left 90 degrees...")
        if self._execute_action(3, self.turn_time, "Turn left"):
            print("[MOTION] Left turn completed")
            return True
        else:
            print("[MOTION] Left turn failed")
            return False
    
    def turn_right_90(self):
        """
        Turn right 90 degrees
        
        Returns:
            bool: Returns True if action completed successfully
        """
        print("[MOTION] Turning right 90 degrees...")
        if self._execute_action(4, self.turn_time, "Turn right"):
            print("[MOTION] Right turn completed")
            return True
        else:
            print("[MOTION] Right turn failed")
            return False
    
    def turn_180(self):
        """
        Turn around (180 degrees)
        
        Returns:
            bool: Returns True if action completed successfully
        """
        print("[MOTION] Turning around (180 degrees)...")
        if not self.turn_left_90():
            return False
        if not self.turn_left_90():
            return False
        print("[MOTION] Turn around completed")
        return True
    
    def stop(self):
        """
        Emergency stop
        
        Returns:
            bool: Returns True if action completed successfully
        """
        print("[MOTION] Stopping")
        return self._execute_action(0, 0.5, "Stop")


class WallFollower:
    """Wall following controller"""
    
    def __init__(self, motion_ctrl):
        """
        Initialize wall following controller
        
        Args:
            motion_ctrl: MotionController instance
        """
        self.motion = motion_ctrl
    
    def follow_right_wall(self, walls):
        """
        Right-hand rule wall following
        
        Strategy:
        1. Priority: turn right (no wall on right)
        2. Second: go straight (no wall ahead)
        3. Third: turn left (no wall on left)
        4. Last: turn around (walls on all sides)
        
        Args:
            walls: Return value from WallDetector.detect()
        
        Returns:
            str: Executed action ('turn_right', 'forward', 'turn_left', 'turn_around')
            None: Action execution failed
        """
        if not walls['right']:
            # No wall on right → turn right and move forward
            print("[STRATEGY] Right-hand rule: no wall on right, turning right")
            if self.motion.turn_right_90() and self.motion.move_forward():
                return 'turn_right'
            return None
        
        elif not walls['front']:
            # No wall ahead → go straight
            print("[STRATEGY] Right-hand rule: no wall ahead, going straight")
            if self.motion.move_forward():
                return 'forward'
            return None
        
        elif not walls['left']:
            # No wall on left → turn left and move forward
            print("[STRATEGY] Right-hand rule: no wall on left, turning left")
            if self.motion.turn_left_90() and self.motion.move_forward():
                return 'turn_left'
            return None
        
        else:
            # Dead end → turn around
            print("[STRATEGY] Right-hand rule: dead end, turning around")
            if self.motion.turn_180():
                return 'turn_around'
            return None
    
    def follow_left_wall(self, walls):
        """
        Left-hand rule wall following
        
        Strategy:
        1. Priority: turn left (no wall on left)
        2. Second: go straight (no wall ahead)
        3. Third: turn right (no wall on right)
        4. Last: turn around (walls on all sides)
        
        Args:
            walls: Return value from WallDetector.detect()
        
        Returns:
            str: Executed action
            None: Action execution failed
        """
        if not walls['left']:
            # No wall on left → turn left and move forward
            print("[STRATEGY] Left-hand rule: no wall on left, turning left")
            if self.motion.turn_left_90() and self.motion.move_forward():
                return 'turn_left'
            return None
        
        elif not walls['front']:
            # No wall ahead → go straight
            print("[STRATEGY] Left-hand rule: no wall ahead, going straight")
            if self.motion.move_forward():
                return 'forward'
            return None
        
        elif not walls['right']:
            # No wall on right → turn right and move forward
            print("[STRATEGY] Left-hand rule: no wall on right, turning right")
            if self.motion.turn_right_90() and self.motion.move_forward():
                return 'turn_right'
            return None
        
        else:
            # Dead end → turn around
            print("[STRATEGY] Left-hand rule: dead end, turning around")
            if self.motion.turn_180():
                return 'turn_around'
            return None
