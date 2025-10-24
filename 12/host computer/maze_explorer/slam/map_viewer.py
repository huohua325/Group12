#!/usr/bin/env python3
"""
Map visualization module
Real-time SLAM map display using matplotlib
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrow
import matplotlib.animation as animation


class MapViewer:
    """SLAM map visualizer
    
    Features:
    - Real-time display of occupancy grid map
    - Show robot position and orientation
    - Display radar scan points (optional)
    - Auto-update display
    """
    
    def __init__(self, map_size_pixels=500, map_size_meters=5.0, 
                 show_robot=True, show_lidar=False):
        """
        Initialize map display
        
        Args:
            map_size_pixels: Map size in pixels
            map_size_meters: Map actual size in meters
            show_robot: Whether to show robot position
            show_lidar: Whether to show radar points
        """
        self.map_size_pixels = map_size_pixels
        self.map_size_meters = map_size_meters
        self.show_robot = show_robot
        self.show_lidar = show_lidar
        
        # Calculate resolution (mm/pixel)
        self.resolution = map_size_meters * 1000 / map_size_pixels
        
        # Create figure
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.fig.canvas.manager.set_window_title('Maze SLAM Map')
        
        # Initialize map display
        self.im = None
        self.robot_marker = None
        self.robot_arrow = None
        
        # Set coordinate axes
        self.ax.set_xlim(0, map_size_pixels)
        self.ax.set_ylim(0, map_size_pixels)
        self.ax.set_aspect('equal')
        self.ax.set_xlabel('X (pixels)')
        self.ax.set_ylabel('Y (pixels)')
        self.ax.set_title('SLAM Map (Black=Obstacle, White=Empty)')
        
        # Enable interactive mode
        plt.ion()
        
        # Show window immediately
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.show(block=False)
        
        print("[OK] Map visualizer started")
    
    def update(self, map_array, robot_pos=None):
        """
        Update map display
        
        Args:
            map_array: np.ndarray, shape (map_size_pixels, map_size_pixels)
                       Value range 0-255 (0=obstacle, 255=empty)
            robot_pos: tuple, robot pose (x_mm, y_mm, theta_deg), optional
        """
        # Display map
        if self.im is None:
            # First display
            self.im = self.ax.imshow(
                map_array, 
                cmap='gray',
                origin='lower',  # Origin at bottom-left
                vmin=0, 
                vmax=255,
                interpolation='nearest'
            )
            self.fig.colorbar(self.im, ax=self.ax, label='Occupancy Probability')
        else:
            # Update data
            self.im.set_data(map_array)
        
        # Display robot position
        if self.show_robot and robot_pos is not None:
            self._update_robot(robot_pos)
        
        # Refresh display
        try:
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            plt.pause(0.001)  # Reduce pause time for better responsiveness
        except Exception as e:
            # If update fails, continue running
            pass
    
    def _update_robot(self, robot_pos):
        """
        Update robot position display
        
        Args:
            robot_pos: tuple (x_mm, y_mm, theta_deg)
        """
        x_mm, y_mm, theta_deg = robot_pos
        
        # Convert to pixel coordinates
        x_px = x_mm / self.resolution
        y_px = y_mm / self.resolution
        
        # Convert angle (degrees to radians)
        theta_rad = np.deg2rad(theta_deg)
        
        # Remove old markers
        if self.robot_marker is not None:
            self.robot_marker.remove()
        if self.robot_arrow is not None:
            self.robot_arrow.remove()
        
        # Draw robot position (red circle)
        self.robot_marker = Circle(
            (x_px, y_px), 
            radius=5,  # 5 pixel radius
            color='red', 
            fill=True,
            zorder=10
        )
        self.ax.add_patch(self.robot_marker)
        
        # Draw orientation arrow
        arrow_length = 15  # pixels
        dx = arrow_length * np.cos(theta_rad)
        dy = arrow_length * np.sin(theta_rad)
        
        self.robot_arrow = FancyArrow(
            x_px, y_px, dx, dy,
            width=3,
            head_width=8,
            head_length=6,
            fc='blue',
            ec='blue',
            zorder=11
        )
        self.ax.add_patch(self.robot_arrow)
        
        # Update title with position information
        self.ax.set_title(
            f'SLAM Map | Position: ({x_mm:.0f}, {y_mm:.0f})mm | Angle: {theta_deg:.1f}°'
        )
    
    def save_figure(self, filename='slam_map.png'):
        """
        Save current map as image
        
        Args:
            filename: File name, default 'slam_map.png'
        """
        self.fig.savefig(filename, dpi=150, bbox_inches='tight')
        print(f"[SAVED] Map image saved: {filename}")
    
    def close(self):
        """Close display window"""
        plt.close(self.fig)
        print("[CLOSED] Map display closed")


class RealTimeViewer:
    """Real-time map display (using matplotlib animation)"""
    
    def __init__(self, slam_system, update_interval=100):
        """
        Initialize real-time display
        
        Args:
            slam_system: MazeSLAM instance
            update_interval: Update interval in milliseconds, default 100ms (10Hz)
        """
        self.slam = slam_system
        self.update_interval = update_interval
        
        # Create figure
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.fig.canvas.manager.set_window_title('Real-time SLAM Map')
        
        # Initialize display
        map_array = self.slam.get_map_array()
        self.im = self.ax.imshow(
            map_array,
            cmap='gray',
            origin='lower',
            vmin=0,
            vmax=255
        )
        
        self.ax.set_title('Real-time SLAM Map')
        self.fig.colorbar(self.im, ax=self.ax)
        
        # Create animation
        self.anim = animation.FuncAnimation(
            self.fig,
            self._update_frame,
            interval=update_interval,
            blit=False
        )
    
    def _update_frame(self, frame):
        """Animation update function"""
        # Get latest map
        map_array = self.slam.get_map_array()
        self.im.set_data(map_array)
        
        # Get position information
        x, y, theta = self.slam.get_position()
        self.ax.set_title(
            f'Real-time SLAM | Position: ({x:.0f}, {y:.0f})mm | Angle: {theta:.1f}° | '
            f'Updates: {self.slam.update_count}'
        )
        
        return [self.im]
    
    def show(self):
        """Show window"""
        plt.show()
    
    def close(self):
        """Close window"""
        self.anim.event_source.stop()
        plt.close(self.fig)


if __name__ == '__main__':
    # Simple test
    print("=" * 60)
    print("Map visualization module test")
    print("=" * 60)
    
    # Create test map (black and white checkerboard)
    map_size = 500
    test_map = np.zeros((map_size, map_size), dtype=np.uint8)
    
    # Create checkerboard pattern
    for i in range(0, map_size, 50):
        for j in range(0, map_size, 50):
            if (i // 50 + j // 50) % 2 == 0:
                test_map[i:i+50, j:j+50] = 255
    
    # Create visualizer
    viewer = MapViewer(map_size_pixels=map_size, map_size_meters=5.0)
    
    # Display map
    viewer.update(test_map)
    
    # Simulate robot movement
    print("\nSimulating robot movement (5 seconds)...")
    import time
    for t in range(50):
        x = 2500 + 500 * np.cos(t * 0.2)
        y = 2500 + 500 * np.sin(t * 0.2)
        theta = t * 10
        viewer.update(test_map, (x, y, theta))
        time.sleep(0.1)
    
    print("\n✅ Test completed (close window to continue)")
    viewer.close()

