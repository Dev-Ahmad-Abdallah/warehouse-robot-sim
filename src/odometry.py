"""
Odometry module for warehouse robot simulation.
Simulates wheel encoders with noise for SLAM localization.
"""

import math
import random

DEBUG = True  # Enable/disable debugging output

def debug_print(message):
    """Print debug message if debugging is enabled."""
    if DEBUG:
        print(f"[ODOMETRY DEBUG] {message}")


class Odometry:
    """
    Odometry class that simulates wheel encoders with noise.
    Tracks robot movement with uncertainty for SLAM.
    """
    
    def __init__(self, initial_x=0, initial_y=0, initial_theta=0,
                 translation_noise=0.05, rotation_noise=0.02):
        """
        Initialize odometry.
        
        Args:
            initial_x: Initial x position
            initial_y: Initial y position
            initial_theta: Initial orientation in degrees
            translation_noise: Standard deviation of translation noise (as fraction of movement)
            rotation_noise: Standard deviation of rotation noise in degrees
        """
        self.x = initial_x
        self.y = initial_y
        self.theta = initial_theta  # Orientation in degrees
        
        self.translation_noise = translation_noise
        self.rotation_noise = rotation_noise
        
        # Store initial pose for reset
        self.initial_x = initial_x
        self.initial_y = initial_y
        self.initial_theta = initial_theta
        
        debug_print(f"Odometry initialized at ({initial_x}, {initial_y}), theta={initial_theta}°")
        debug_print(f"Noise parameters: translation={translation_noise}, rotation={rotation_noise}°")
    
    def update_odometry(self, dx, dy, dtheta):
        """
        Update odometry with movement deltas, adding noise.
        
        Args:
            dx: Change in x (ground truth)
            dy: Change in y (ground truth)
            dtheta: Change in orientation in degrees (ground truth)
        
        Returns:
            tuple: (noisy_dx, noisy_dy, noisy_dtheta) - measured movement with noise
        """
        # Calculate distance moved
        distance = math.sqrt(dx * dx + dy * dy)
        
        # Add Gaussian noise to translation
        # Noise is proportional to distance moved
        if distance > 0:
            noise_magnitude = random.gauss(0, self.translation_noise * distance)
            # Apply noise in the direction of movement
            noisy_dx = dx + noise_magnitude * (dx / distance) if distance > 0 else dx
            noisy_dy = dy + noise_magnitude * (dy / distance) if distance > 0 else dy
        else:
            noisy_dx = dx
            noisy_dy = dy
        
        # Add Gaussian noise to rotation
        noisy_dtheta = dtheta + random.gauss(0, self.rotation_noise)
        
        # Update estimated pose
        # Convert current theta to radians for calculation
        theta_rad = math.radians(self.theta)
        
        # Rotate the movement vector by current orientation
        cos_theta = math.cos(theta_rad)
        sin_theta = math.sin(theta_rad)
        
        # Apply rotation to movement vector
        rotated_dx = noisy_dx * cos_theta - noisy_dy * sin_theta
        rotated_dy = noisy_dx * sin_theta + noisy_dy * cos_theta
        
        # Update position
        self.x += rotated_dx
        self.y += rotated_dy
        self.theta = (self.theta + noisy_dtheta) % 360
        
        debug_print(f"Odometry update: dx={dx:.3f}->{noisy_dx:.3f}, dy={dy:.3f}->{noisy_dy:.3f}, "
                   f"dtheta={dtheta:.3f}->{noisy_dtheta:.3f}")
        debug_print(f"Estimated pose: ({self.x:.3f}, {self.y:.3f}), theta={self.theta:.3f}°")
        
        return (noisy_dx, noisy_dy, noisy_dtheta)
    
    def get_estimated_pose(self):
        """
        Get current estimated pose from odometry.
        
        Returns:
            tuple: (x, y, theta) - estimated position and orientation
        """
        return (self.x, self.y, self.theta)
    
    def reset_odometry(self):
        """Reset odometry to initial position."""
        self.x = self.initial_x
        self.y = self.initial_y
        self.theta = self.initial_theta
        debug_print(f"Odometry reset to initial pose: ({self.x}, {self.y}), theta={self.theta}°")
    
    def set_pose(self, x, y, theta):
        """
        Set the odometry pose directly (used for pose correction).
        
        Args:
            x: X position
            y: Y position
            theta: Orientation in degrees
        """
        self.x = x
        self.y = y
        self.theta = theta % 360
        debug_print(f"Odometry pose set to: ({x}, {y}), theta={theta}°")

