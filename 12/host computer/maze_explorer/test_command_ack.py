#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Command ACK Test Script
Test CMD command sending, ACK receiving, sequence number mechanism
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from communication.ble_comm import BLERobotComm
import config


def test_command_ack():
    """Test command ACK system"""
    print("=" * 70)
    print("Command ACK Test")
    print("=" * 70)
    print("Test objectives:")
    print("  1. Verify CMD command sending")
    print("  2. Verify ACK reply receiving")
    print("  3. Verify sequence number mechanism")
    print("  4. Verify timeout protection")
    print()
    
    # Initialize communication
    print("[INIT] Initializing BLE communication...")
    comm = BLERobotComm(config.BLE_ADDRESS, verbose=True)
    
    # Connect
    print("\n[CONNECT] Connecting to robot...")
    if not comm.connect():
        print("[ERROR] Connection failed")
        return
    
    print("[OK] Connection successful\n")
    
    # Wait for stability
    print("[WAIT] Waiting for communication stability...")
    time.sleep(2)
    print("[OK] Communication ready\n")
    
    # Record ACKs
    ack_received = []
    
    def on_ack(seq, status):
        """ACK callback"""
        ack_received.append({'seq': seq, 'status': status, 'time': time.time()})
        print(f"  [ACK] seq={seq} status={status}")
    
    comm.on_action_complete = on_ack
    
    # ========== Test 1: Stop command (immediate completion) ==========
    print("=" * 70)
    print("Test 1: Stop command (should return ACK immediately)")
    print("=" * 70)
    
    ack_received.clear()
    print("[SEND] Sending CMD seq=? mode=0 (STOP)")
    seq1 = comm.send_command_with_seq(mode=0)
    print(f"[INFO] Command sequence number: {seq1}")
    
    print("[WAIT] Waiting for ACK (max 3 seconds)...")
    start_time = time.time()
    while len(ack_received) == 0 and (time.time() - start_time) < 3:
        time.sleep(0.1)
    
    if len(ack_received) > 0:
        elapsed = time.time() - start_time
        ack = ack_received[0]
        print(f"[OK] Received ACK: seq={ack['seq']} status={ack['status']} elapsed={elapsed:.2f}s")
        
        if ack['seq'] == seq1:
            print("[OK] Sequence number matches")
        else:
            print(f"[ERROR] Sequence number mismatch: expected {seq1}, actual {ack['seq']}")
    else:
        print("[ERROR] No ACK received")
    
    time.sleep(2)
    
    # ========== Test 2: Forward command ==========
    print("\n" + "=" * 70)
    print("Test 2: Forward command (600mm, should complete in 2-5 seconds)")
    print("=" * 70)
    print("[WARN] Ensure robot has enough space in front!")
    input("Press Enter to continue...")
    
    ack_received.clear()
    print("\n[SEND] Sending CMD seq=? mode=1 (FORWARD 600mm)")
    seq2 = comm.send_command_with_seq(mode=1)
    print(f"[INFO] Command sequence number: {seq2}")
    
    print("[WAIT] Waiting for ACK (max 20 seconds)...")
    print("       Observe if robot moves forward ~60mm and stops")
    start_time = time.time()
    
    while len(ack_received) == 0 and (time.time() - start_time) < 20:
        elapsed = time.time() - start_time
        if int(elapsed) % 3 == 0 and elapsed > 0.5:
            print(f"  [INFO] Waited {int(elapsed)} seconds...")
        time.sleep(0.3)
    
    if len(ack_received) > 0:
        elapsed = time.time() - start_time
        ack = ack_received[0]
        print(f"\n[OK] Received ACK: seq={ack['seq']} status={ack['status']} elapsed={elapsed:.2f}s")
        
        if ack['seq'] == seq2:
            print("[OK] Sequence number matches")
        else:
            print(f"[ERROR] Sequence number mismatch: expected {seq2}, actual {ack['seq']}")
        
        if ack['status'] == 'OK':
            print("[OK] Action completed normally")
        elif ack['status'] == 'TIMEOUT':
            print("[WARN] Action timed out")
    else:
        print("[ERROR] No ACK received (over 20 seconds)")
    
    time.sleep(3)
    
    # ========== Test 3: Left turn command ==========
    print("\n" + "=" * 70)
    print("Test 3: Left turn command (90 degrees, should complete in 2-5 seconds)")
    print("=" * 70)
    print("[WARN] Ensure robot has enough turning space!")
    input("Press Enter to continue...")
    
    ack_received.clear()
    print("\n[SEND] Sending CMD seq=? mode=3 (TURN_LEFT 90deg)")
    seq3 = comm.send_command_with_seq(mode=3)
    print(f"[INFO] Command sequence number: {seq3}")
    
    print("[WAIT] Waiting for ACK (max 10 seconds)...")
    print("       Observe if robot turns left ~90 degrees and stops")
    start_time = time.time()
    
    while len(ack_received) == 0 and (time.time() - start_time) < 10:
        elapsed = time.time() - start_time
        if int(elapsed) % 2 == 0 and elapsed > 0.5:
            print(f"  [INFO] Waited {int(elapsed)} seconds...")
        time.sleep(0.3)
    
    if len(ack_received) > 0:
        elapsed = time.time() - start_time
        ack = ack_received[0]
        print(f"\n[OK] Received ACK: seq={ack['seq']} status={ack['status']} elapsed={elapsed:.2f}s")
        
        if ack['seq'] == seq3:
            print("[OK] Sequence number matches")
        else:
            print(f"[ERROR] Sequence number mismatch: expected {seq3}, actual {ack['seq']}")
        
        if ack['status'] == 'OK':
            print("[OK] Action completed normally")
        elif ack['status'] == 'TIMEOUT':
            print("[WARN] Action timed out")
    else:
        print("[ERROR] No ACK received (over 10 seconds)")
    
    time.sleep(2)
    
    # ========== Test 4: Right turn command ==========
    print("\n" + "=" * 70)
    print("Test 4: Right turn command (90 degrees, should complete in 2-5 seconds)")
    print("=" * 70)
    input("Press Enter to continue...")
    
    ack_received.clear()
    print("\n[SEND] Sending CMD seq=? mode=4 (TURN_RIGHT 90deg)")
    seq4 = comm.send_command_with_seq(mode=4)
    print(f"[INFO] Command sequence number: {seq4}")
    
    print("[WAIT] Waiting for ACK (max 10 seconds)...")
    print("       Observe if robot turns right ~90 degrees and stops")
    start_time = time.time()
    
    while len(ack_received) == 0 and (time.time() - start_time) < 10:
        elapsed = time.time() - start_time
        if int(elapsed) % 2 == 0 and elapsed > 0.5:
            print(f"  [INFO] Waited {int(elapsed)} seconds...")
        time.sleep(0.3)
    
    if len(ack_received) > 0:
        elapsed = time.time() - start_time
        ack = ack_received[0]
        print(f"\n[OK] Received ACK: seq={ack['seq']} status={ack['status']} elapsed={elapsed:.2f}s")
        
        if ack['seq'] == seq4:
            print("[OK] Sequence number matches")
        else:
            print(f"[ERROR] Sequence number mismatch: expected {seq4}, actual {ack['seq']}")
        
        if ack['status'] == 'OK':
            print("[OK] Action completed normally")
        elif ack['status'] == 'TIMEOUT':
            print("[WARN] Action timed out")
    else:
        print("[ERROR] No ACK received (over 10 seconds)")
    
    # ========== Test Summary ==========
    print("\n\n" + "=" * 70)
    print("Test Summary")
    print("=" * 70)
    print(f"Sequence number range: {seq1} - {seq4}")
    print("Note: Sequence numbers should be consecutive and incrementing (0-65535 cycle)")
    print()
    print("Check items:")
    print("  [  ] Stop command returns ACK immediately")
    print("  [  ] Forward command executes and returns ACK")
    print("  [  ] Left turn command executes and returns ACK")
    print("  [  ] Right turn command executes and returns ACK")
    print("  [  ] Sequence numbers are consecutive and match")
    print("  [  ] Robot actions are accurate (distance/angle)")
    print()
    print("If all above pass, the command ACK system is working properly!")
    print()
    
    # Cleanup
    print("[CLEANUP] Disconnecting...")
    comm.disconnect()
    print("[OK] Test completed")


if __name__ == '__main__':
    test_command_ack()

