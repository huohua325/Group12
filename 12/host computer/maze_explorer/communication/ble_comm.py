#!/usr/bin/env python3
"""
BLE robot communication module
Support the transmission and reception of binary radar data and text commands

Based on: maze/ble_robot_control. py (verified BLE communication)
Data format: xxq/docs/Complete guide for upper computer development. md (lines 97-147)
"""

import asyncio
import struct
import time
import threading
from typing import Optional, Callable
import logging

try:
    from bleak import BleakClient
except ImportError:
    print("❌ BLEAK library not installed")
    print("Please run: pip install bleak")
    raise


class BLERobotComm:
    """BLE robot communication class (supports binary radar data)
    
Function:
-Receive binary radar frames (1012 bytes, frame header A5 5A 4C 44)
-Receive text data (POSE, MPU, ODO)
-Send control commands (MODE, SPD, etc.)
    
Example:
>>> comm = BLERobotComm('C4:25:01:20:02:8E')
>>>Comm.on_lidar frame=lambda data: print (f "Radar: {data ['point_comnt ']} points")
>>> comm.connect()
>>> comm.request_lidar_scan()
>>> time.sleep(5)
>>> comm.disconnect()
    """
    
    # BLE UART Service UUID (HC-04BLE Standard)
    UART_SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
    UART_CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"
    
    def __init__(self, address: str, verbose: bool = False):
        """Initialize BLE communication
        
     Args:
Address: BLE device address (e.g. 'C4:25:01:02:8E')
Verbose: whether to display detailed logs
        """
        self.address = address
        self.verbose = verbose
        self.client: Optional[BleakClient] = None
        self.running = False
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.thread: Optional[threading.Thread] = None
        self._connect_event = threading.Event()
        self._stop_requested = False
        
        # Binary buffer (handling BLE packets)
        self.buffer = bytearray()
        
        # Latest data cache
        self.latest_lidar_frame = None
        self.latest_pose = None
        
        # callback function
        self.on_lidar_frame: Optional[Callable[[dict], None]] = None
        self.on_pose_update: Optional[Callable[[dict], None]] = None
        self.on_message: Optional[Callable[[str], None]] = None
        self.on_action_complete: Optional[Callable[[int, str], None]] = None  # (seq, status)
        
        # statistics
        self.stats = {
            'lidar_frames': 0,
            'pose_updates': 0,
            'messages': 0
        }
        
        # Command Serial Number (Phase 3: Maze Exploration)
        self.command_seq = 0  # 0-65535 cycle
        
        # log
        self.logger = logging.getLogger(__name__)
        if verbose:
            logging.basicConfig(level=logging.DEBUG)
    
    def connect(self) -> bool:
        """Connect BLE devices
        
Returns:
Connect successfully and return True
        """
        if self.verbose:
            print(f"🔵 Connect BLE devices: {self.address}")
        
        self.running = True
        self._stop_requested = False
        self._connect_event.clear()
        
        # Running BLE event loop in independent thread
        self.thread = threading.Thread(
            target=self._run_ble_thread,
            daemon=True,
            name="BLERobotComm"
        )
        self.thread.start()
        
       #Wait for connection completion (up to 15 seconds)
        if self._connect_event.wait(timeout=15.0):
            if self.client and self.client.is_connected:
                if self.verbose:
                    print("✅ BLE connected")
                return True
        
        if self.verbose:
            print(" ❌  BLE connection timeout ")
        self.disconnect()
        return False
    
    def _run_ble_thread(self):
        """Run BLE event loop in independent thread"""
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        
        try:
            self.loop.run_until_complete(self._ble_main_loop())
        except Exception as e:
            if self.verbose:
                print(f"❌ BLE thread exception: {e}")
        finally:
            #Clean up event loop

            try:
                pending = asyncio.all_tasks(self.loop)
                for task in pending:
                    task.cancel()
                if pending:
                    self.loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
            except:
                pass
            finally:
                self.loop.close()
    
    async def _ble_main_loop(self):
        """BLE main loop"""
        try:
            self.client = BleakClient(self.address, timeout=20.0)
            await self.client.connect()
            
            # Subscription Notification
            await self.client.start_notify(
                self.UART_CHAR_UUID,
                self._notification_handler
            )
            
            # Notification of successful connection
            self._connect_event.set()
            
            # Stay connected
            while not self._stop_requested:
                if not self.client.is_connected:
                    if self.verbose:
                        print("⚠️ BLE connection disconnected")
                    break
                await asyncio.sleep(0.1)
            
        except Exception as e:
            if self.verbose:
                print(f"❌ BLE connection failed: {e}")
            self._connect_event.set()
        finally:
            if self.client and self.client.is_connected:
                try:
                    await self.client.stop_notify(self.UART_CHAR_UUID)
                    await self.client.disconnect()
                except:
                    pass
    
    def _notification_handler(self, sender, data: bytearray):
        """BLE data receiving callback"""
        if self._stop_requested or not self.running:
            return
            
        try:
            # Add to buffer
            self.buffer.extend(data)
            # Process buffer
            self._process_buffer()
                
        except Exception as e:
            if self.verbose:
                print(f"❌ Data processing exception: {e}")
    
    def _process_buffer(self):
        """Process buffer: separate binary frames and text data
        
        Data stream contains:
        1. Binary radar frames (1012 bytes, frame header A5 5A 4C 44)
        2. Text data (POSE, MPU, ODO, etc., ending with \n)
        """
        while len(self.buffer) > 0:
            # 1. Check radar frame (frame header A5 5A 4C 44)
            if len(self.buffer) >= 4 and self.buffer[0:4] == b'\xA5\x5A\x4C\x44':
                if len(self.buffer) >= 1012:
                    # Complete radar frame
                    frame = bytes(self.buffer[:1012])
                    self._parse_lidar_frame(frame)
                    self.buffer = self.buffer[1012:]
                else:
                    # Wait for complete frame
                    break
            
            # 2. Text data (starts with letter)
            elif self.buffer[0:1].isalpha() or self.buffer[0:1] == b'[':
                idx = self.buffer.find(b'\n')
                if idx >= 0:
                    line = self.buffer[:idx].decode('utf-8', errors='ignore')
                    self._parse_text_line(line.strip())
                    self.buffer = self.buffer[idx+1:]
                else:
                    # Wait for complete line
                    break
            
            else:
                # Discard invalid bytes
                self.buffer.pop(0)
    
    def _parse_lidar_frame(self, frame: bytes):
        """Parse binary radar frame (1012 bytes)
        
        Frame structure (from development guide lines 97-147):
        Offset     Field         Type           Description
        0-3       Frame header  4 bytes        A5 5A 4C 44 (fixed)
        4-7       Timestamp     uint32_t       HAL_GetTick(), unit ms
        8-9       Point count   uint16_t       Fixed 500
        10-1009   Distance array uint16_t[500]  Unit mm, little-endian
        1010-1011 CRC16         uint16_t       CRC-16/MODBUS
        
        Angle mapping:
        Index 0   → 0.0°
        Index 1   → 0.72°
        Index 250 → 180.0°
        Index 499 → 359.28°
        angle(degrees) = index × 0.72
        """
        # Verify frame header
        if frame[0:4] != b'\xA5\x5A\x4C\x44':
            if self.verbose:
                print("⚠️ Invalid radar frame header")
            return
        
        try:
            # Parse fields (little-endian)
            timestamp = struct.unpack('<I', frame[4:8])[0]
            point_count = struct.unpack('<H', frame[8:10])[0]
            distances = struct.unpack('<500H', frame[10:1010])  # 500 uint16s
            crc = struct.unpack('<H', frame[1010:1012])[0]
            
            # Build data dictionary
            data = {
                'timestamp': timestamp,
                'point_count': point_count,
                'distances_mm': distances,  # Unit: millimeters
                'crc': crc
            }
            
            # Cache
            self.latest_lidar_frame = data
            self.stats['lidar_frames'] += 1
            
            # Callback
            if self.on_lidar_frame:
                self.on_lidar_frame(data)
            
        except Exception as e:
            if self.verbose:
                print(f"❌ Radar frame parsing failed: {e}")
    
    def _parse_text_line(self, line: str):
        """Parse text line (POSE, MPU, etc.)
        
        Format:
        - POSE,timestamp,x,y,theta
        - MPU,timestamp,roll,pitch,ax,ay,az,gx,gy,gz
        - ODO,timestamp,left_rps,right_rps,left_count,right_count
        - [LOG:...] - STM32 debug info
        - [DEBUG] - STM32 debug info
        - [WARN] - STM32 warning info
        """
        if not line:
            return
        
        try:
            if line.startswith('POSE,'):
                parts = line.split(',')
                if len(parts) >= 5:
                    pose = {
                        'timestamp': int(parts[1]),
                        'x': float(parts[2]),
                        'y': float(parts[3]),
                        'theta': float(parts[4])
                    }
                    self.latest_pose = pose
                    self.stats['pose_updates'] += 1
                    
                    if self.on_pose_update:
                        self.on_pose_update(pose)
            
            # Handle ACK reply (Stage 3: Maze exploration)
            elif line.startswith('ACK,'):
                parts = line.strip().split(',')
                if len(parts) >= 2:
                    try:
                        seq = int(parts[1])
                        status = parts[2] if len(parts) > 2 else 'OK'
                        time_str = parts[3] if len(parts) > 3 else ''
                        
                        if self.on_action_complete:
                            self.on_action_complete(seq, status)
                        
                        if self.verbose:
                            if status == 'TIMEOUT':
                                print(f"[WARN] ACK: seq={seq} TIMEOUT {time_str}")
                            else:
                                print(f"[OK] ACK: seq={seq} status={status} {time_str}")
                    except (ValueError, IndexError) as e:
                        if self.verbose:
                            print(f"[WARN] ACK parsing error: {line} - {e}")
            else:
                # Display STM32 debug info directly (help with debugging)
                # Pay special attention to [LOG:FRAME] and [LOG:RANGE]
                if line.startswith('[LOG:FRAME]') or line.startswith('[LOG:RANGE]'):
                    print(f"\n>>> {line}")  # Highlight display
                elif line.startswith('[DEBUG]') or line.startswith('[WARN]') or line.startswith('[LOG:'):
                    print(f"  {line}")  # Normal STM32 output
                
                # Other messages
                self.stats['messages'] += 1
                if self.on_message:
                    self.on_message(line)
                    
        except Exception as e:
            if self.verbose:
                print(f"⚠️ Text line parsing failed: {line[:50]}... Error: {e}")
    
    def send_command(self, cmd: str) -> bool:
        """Send command to STM32
        
        Args:
            cmd: Command string (must include \n)
            
        Returns:
            Returns True if sent successfully
        """
        if not self.running or self._stop_requested:
            if self.verbose:
                print("⚠️ BLE not running, cannot send command")
            return False
            
        if not self.client or not self.client.is_connected:
            if self.verbose:
                print("⚠️ BLE not connected, cannot send command")
            return False
        
        try:
            # Send asynchronously in BLE thread event loop
            if self.loop and not self.loop.is_closed():
                future = asyncio.run_coroutine_threadsafe(
                    self._async_send(cmd.encode('utf-8')),
                    self.loop
                )
                # Wait for send completion (max 1 second)
                future.result(timeout=1.0)
                
                if self.verbose:
                    print(f"📤 Send command: {cmd.strip()}")
                return True
            else:
                if self.verbose:
                    print("⚠️ Event loop not available")
                return False
                
        except Exception as e:
            if self.verbose:
                print(f"❌ Command send failed: {e}")
            return False
    
    async def _async_send(self, data: bytes):
        """Send data asynchronously"""
        try:
            if self.client and self.client.is_connected:
                await self.client.write_gatt_char(self.UART_CHAR_UUID, data)
        except Exception as e:
            if self.verbose:
                print(f"❌ Async send failed: {e}")
    
    def send_command_with_seq(self, mode: int) -> int:
        """Send command with sequence number (Stage 3: Maze exploration)
        
        Args:
            mode: Mode ID (0=stop, 1=forward, 2=backward, 3=left turn, 4=right turn)
        
        Returns:
            int: Command sequence number, returns -1 on failure
        """
        if not self.running or self._stop_requested:
            if self.verbose:
                print("⚠️ BLE not running, cannot send command")
            return -1
            
        if not self.client or not self.client.is_connected:
            if self.verbose:
                print("⚠️ BLE not connected, cannot send command")
            return -1
        
        try:
            # Generate sequence number (0-65535 cycle)
            self.command_seq = (self.command_seq + 1) % 65536
            cmd = f"CMD,{self.command_seq},{mode}\n"
            
            # Debug: show actual command being sent
            if self.verbose:
                cmd_bytes = cmd.encode('utf-8')
                cmd_hex = ' '.join(f'{b:02X}' for b in cmd_bytes)
                print(f"[DEBUG] Preparing to send: '{cmd.strip()}' (hex: {cmd_hex})")
            
            # Send asynchronously in BLE thread event loop
            if self.loop and not self.loop.is_closed():
                future = asyncio.run_coroutine_threadsafe(
                    self._async_send(cmd.encode('utf-8')),
                    self.loop
                )
                # Wait for send completion (max 1 second)
                future.result(timeout=1.0)
                
                if self.verbose:
                    print(f"📤 Send CMD: seq={self.command_seq} mode={mode}")
                return self.command_seq
            else:
                if self.verbose:
                    print("⚠️ Event loop not available")
                return -1
                
        except Exception as e:
            if self.verbose:
                print(f"❌ Command send failed: {e}")
            return -1
    
    # ========== Convenient command methods ==========
    
    def stop(self) -> bool:
        """Stop robot"""
        return self.send_command("MODE,0\n")
    
    def forward(self) -> bool:
        """Move forward"""
        return self.send_command("MODE,1\n")
    
    def backward(self) -> bool:
        """Move backward"""
        return self.send_command("MODE,2\n")
    
    def turn_left(self) -> bool:
        """Turn left"""
        return self.send_command("MODE,3\n")
    
    def turn_right(self) -> bool:
        """Turn right"""
        return self.send_command("MODE,4\n")
    
    def request_lidar_scan(self) -> bool:
        """Request radar scan"""
        return self.send_command("A\n")
    
    # ========== Status queries ==========
    
    def is_connected(self) -> bool:
        """Check if connected"""
        return self.client is not None and self.client.is_connected
    
    def get_stats(self) -> dict:
        """Get statistics"""
        return self.stats.copy()
    
    def disconnect(self):
        """Disconnect"""
        if not self.running:
            return
        
        self.running = False
        self._stop_requested = True
        
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=3.0)
        
        if self.verbose:
            print("🔵 BLE disconnected")
        
        # Wait for BLE to fully release (avoid reconnection failure)
        time.sleep(1.0)
    
    def __del__(self):
        """Destructor"""
        self.disconnect()

