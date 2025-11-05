"""
LIDAR sensor module for warehouse robot simulation.
Implements LIDAR-like ray casting to detect obstacles and features.
"""

DEBUG = True  # Enable/disable debugging output

def debug_print(message):
    """Print debug message if debugging is enabled."""
    if DEBUG:
        print(f"[LIDAR DEBUG] {message}")


class LidarSensor:
    """
    LIDAR sensor that casts rays in multiple directions to detect obstacles,
    goals, and other features in the environment.
    """
    
    def __init__(self, max_range=10, num_rays=8):
        """
        Initialize the LIDAR sensor.
        
        Args:
            max_range: Maximum range of LIDAR rays (in grid cells)
            num_rays: Number of rays to cast (distributed around 360 degrees)
        """
        self.max_range = max_range
        self.num_rays = num_rays
        self.ray_angles = [i * (360.0 / num_rays) for i in range(num_rays)]
        
        debug_print(f"LIDAR sensor initialized: max_range={max_range}, num_rays={num_rays}")
    
    def cast_ray(self, start_x, start_y, angle_deg, warehouse):
        """
        Cast a single ray and return the first obstacle hit and distance.
        
        Args:
            start_x: Starting x position
            start_y: Starting y position
            angle_deg: Angle in degrees (0 = right, 90 = up, 180 = left, 270 = down)
            warehouse: Warehouse object to check obstacles against
            
        Returns:
            tuple: (hit_x, hit_y, distance, hit_type) or None if no hit
        """
        import math
        
        # Convert angle to radians
        angle_rad = math.radians(angle_deg)
        
        # Direction vector
        dx = math.cos(angle_rad)
        dy = -math.sin(angle_rad)  # Negative because y increases downward
        
        # Cast ray step by step
        for step in range(1, self.max_range + 1):
            check_x = int(start_x + dx * step)
            check_y = int(start_y + dy * step)
            
            # Check bounds
            if check_x < 0 or check_x >= warehouse.width or check_y < 0 or check_y >= warehouse.height:
                # Hit boundary
                return (check_x, check_y, step, "boundary")
            
            # Check for obstacle
            if warehouse.is_blocked(check_x, check_y):
                return (check_x, check_y, step, "obstacle")
        
        # No hit within max range
        return None
    
    def scan(self, robot_x, robot_y, warehouse):
        """
        Perform a full LIDAR scan from the robot's position.
        
        Args:
            robot_x: Robot's x position
            robot_y: Robot's y position
            warehouse: Warehouse object to scan
            
        Returns:
            dict: Scan results with detected obstacles, goals, and docks
        """
        scan_results = {
            'obstacles': [],
            'goals': [],
            'loading_dock': None,
            'discharge_dock': None,
            'free_cells': [],
            'ray_hits': []
        }
        
        # Cast rays in all directions
        for angle in self.ray_angles:
            hit = self.cast_ray(robot_x, robot_y, angle, warehouse)
            if hit:
                hit_x, hit_y, distance, hit_type = hit
                scan_results['ray_hits'].append({
                    'x': hit_x,
                    'y': hit_y,
                    'distance': distance,
                    'type': hit_type,
                    'angle': angle
                })
                
                if hit_type == "obstacle":
                    if (hit_x, hit_y) not in scan_results['obstacles']:
                        scan_results['obstacles'].append((hit_x, hit_y))
        
        # Scan immediate surroundings (4-directional) for goals and docks
        directions = [(0, 0), (0, -1), (0, 1), (-1, 0), (1, 0)]  # Current, up, down, left, right
        
        for dx, dy in directions:
            check_x = int(robot_x) + dx
            check_y = int(robot_y) + dy
            
            if 0 <= check_x < warehouse.width and 0 <= check_y < warehouse.height:
                # Check for goal
                for goal in warehouse.goals:
                    if goal[0] == check_x and goal[1] == check_y:
                        if (check_x, check_y) not in scan_results['goals']:
                            scan_results['goals'].append((check_x, check_y))
                            debug_print(f"LIDAR detected goal at ({check_x}, {check_y})")
                
                # Check for loading dock
                if warehouse.loading_dock and warehouse.loading_dock[0] == check_x and warehouse.loading_dock[1] == check_y:
                    scan_results['loading_dock'] = (check_x, check_y)
                    debug_print(f"LIDAR detected loading dock at ({check_x}, {check_y})")
                
                # Check for discharge dock
                if warehouse.discharge_dock and warehouse.discharge_dock[0] == check_x and warehouse.discharge_dock[1] == check_y:
                    scan_results['discharge_dock'] = (check_x, check_y)
                    debug_print(f"LIDAR detected discharge dock at ({check_x}, {check_y})")
                
                # Mark as free if not obstacle
                if not warehouse.is_blocked(check_x, check_y):
                    scan_results['free_cells'].append((check_x, check_y))
        
        # Also check cells along ray paths for obstacles
        for ray_hit in scan_results['ray_hits']:
            if ray_hit['type'] == 'obstacle':
                hit_x, hit_y = ray_hit['x'], ray_hit['y']
                if (hit_x, hit_y) not in scan_results['obstacles']:
                    scan_results['obstacles'].append((hit_x, hit_y))
        
        debug_print(f"LIDAR scan complete: {len(scan_results['obstacles'])} obstacles, {len(scan_results['goals'])} goals detected")
        
        return scan_results
    
    def check_path_clear(self, start_x, start_y, target_x, target_y, warehouse):
        """
        Check if a path from start to target is clear of obstacles.
        Uses ray casting to determine if the path is blocked.
        
        Args:
            start_x: Starting x position
            start_y: Starting y position
            target_x: Target x position
            target_y: Target y position
            warehouse: Warehouse object to check
            
        Returns:
            bool: True if path is clear, False if blocked
        """
        import math
        
        # Calculate angle to target
        dx = target_x - start_x
        dy = target_y - start_y
        
        if dx == 0 and dy == 0:
            return True
        
        # Calculate distance
        distance = math.sqrt(dx * dx + dy * dy)
        
        # Normalize direction
        dx_norm = dx / distance
        dy_norm = dy / distance
        
        # Check each cell along the path
        steps = int(distance) + 1
        for step in range(1, steps + 1):
            check_x = int(start_x + dx_norm * step)
            check_y = int(start_y + dy_norm * step)
            
            # Check bounds
            if check_x < 0 or check_x >= warehouse.width or check_y < 0 or check_y >= warehouse.height:
                return False
            
            # Check for obstacle
            if warehouse.is_blocked(check_x, check_y):
                debug_print(f"LIDAR: Path blocked by obstacle at ({check_x}, {check_y})")
                return False
            
            # If we've reached the target, stop
            if check_x == target_x and check_y == target_y:
                break
        
        return True
    
    def get_safe_directions(self, robot_x, robot_y, warehouse):
        """
        Get directions that are safe to move (no obstacles detected).
        
        Args:
            robot_x: Robot's x position
            robot_y: Robot's y position
            warehouse: Warehouse object to check
            
        Returns:
            list: List of safe (dx, dy) directions to move
        """
        safe_directions = []
        directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]  # up, down, left, right
        
        for dx, dy in directions:
            next_x = int(robot_x) + dx
            next_y = int(robot_y) + dy
            
            # Check bounds
            if 0 <= next_x < warehouse.width and 0 <= next_y < warehouse.height:
                # Check if path is clear
                if self.check_path_clear(robot_x, robot_y, next_x, next_y, warehouse):
                    safe_directions.append((dx, dy))
        
        return safe_directions

