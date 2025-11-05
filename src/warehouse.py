"""
Warehouse module for the warehouse robot simulation.
"""

import pygame
from constants import (
    WAREHOUSE_WIDTH, WAREHOUSE_HEIGHT, GRID_SIZE, 
    GRAY, LOADING_DOCK, DISCHARGE_DOCK, GOAL_COLOR, 
    YELLOW, DARK_GRAY, BLACK, WHITE, debug_log
)

DEBUG = True

def debug_print(message):
    """Print debug message if debugging is enabled."""
    if DEBUG:
        print(f"[WAREHOUSE DEBUG] {message}")


class Warehouse:
    """Warehouse environment with obstacles, goals, and docks."""
    
    def __init__(self):
        self.obstacles = set()
        self.goals = []
        self.loading_dock = None
        self.discharge_dock = None
        self.width = WAREHOUSE_WIDTH
        self.height = WAREHOUSE_HEIGHT
        debug_log("Creating warehouse layout...")
        self.create_maze_layout()
        self.create_docks_and_goals()
        debug_log(f"Warehouse created with {len(self.obstacles)} obstacles and {len(self.goals)} goals")
    
    def create_maze_layout(self):
        """Create the maze layout with obstacles."""
        # Create walls around the perimeter
        for x in range(WAREHOUSE_WIDTH):
            self.obstacles.add((x, 0))
            self.obstacles.add((x, WAREHOUSE_HEIGHT - 1))
        for y in range(WAREHOUSE_HEIGHT):
            self.obstacles.add((0, y))
            self.obstacles.add((WAREHOUSE_WIDTH - 1, y))
        
        # Create internal maze structure with wider paths
        # Horizontal corridors (skip every 5th row for wider paths)
        for y in [4, 8, 12]:
            if y < WAREHOUSE_HEIGHT:
                for x in range(2, WAREHOUSE_WIDTH - 2):
                    if x % 8 not in [0, 1]:  # Wider gaps for vertical passages
                        self.obstacles.add((x, y))
        
        # Vertical corridors (wider spacing)
        for x in [7, 15]:
            if x < WAREHOUSE_WIDTH:
                for y in range(2, WAREHOUSE_HEIGHT - 2):
                    if y % 6 not in [0, 1]:  # Wider gaps for horizontal passages
                        self.obstacles.add((x, y))
        
        # Add strategic obstacles
        maze_obstacles = [
            (4, 6), (10, 6), (18, 6),
            (4, 10), (12, 10), (20, 10),
            (6, 14), (14, 14), (22, 14),
        ]
        for x, y in maze_obstacles:
            if x < WAREHOUSE_WIDTH and y < WAREHOUSE_HEIGHT:
                self.obstacles.add((x, y))
    
    def create_docks_and_goals(self):
        """Create loading dock, discharge dock, and goal locations."""
        # Loading dock at top left area
        self.loading_dock = (2, 2)
        
        # Discharge dock at robot starting position
        self.discharge_dock = (1, 1)
        
        # Create goal points (packages to collect)
        self.goals = [
            (5, 5), (9, 3), (13, 6),
            (17, 5), (19, 9), (11, 11),
            (7, 13), (21, 13)
        ]
    
    def is_blocked(self, x, y):
        """Check if a position is blocked by an obstacle."""
        return (int(x), int(y)) in self.obstacles
    
    def get_all_cells(self):
        """Get all cells in the warehouse (for complete exploration)."""
        cells = []
        for y in range(self.height):
            for x in range(self.width):
                cells.append((x, y))
        return cells
    
    def get_free_cells(self):
        """Get all free (non-obstacle) cells in the warehouse."""
        free_cells = []
        for y in range(self.height):
            for x in range(self.width):
                if not self.is_blocked(x, y):
                    free_cells.append((x, y))
        return free_cells
    
    def draw(self, surface, robot):
        """Draw the warehouse grid, obstacles, goals, and docks."""
        from constants import SCREEN_WIDTH, SCREEN_HEIGHT, WHITE, DARK_GRAY
        
        # Draw background - dark for unexplored, light for explored
        # First draw all cells as dark (unexplored)
        for y in range(WAREHOUSE_HEIGHT):
            for x in range(WAREHOUSE_WIDTH):
                cell_x = x * GRID_SIZE
                cell_y = y * GRID_SIZE
                # Dark gray for unexplored cells
                if robot.ogm and not robot.ogm.is_explored(x, y):
                    pygame.draw.rect(surface, DARK_GRAY, (cell_x, cell_y, GRID_SIZE, GRID_SIZE))
                else:
                    # Light/white for explored cells
                    pygame.draw.rect(surface, WHITE, (cell_x, cell_y, GRID_SIZE, GRID_SIZE))
        
        # Draw grid lines
        for x in range(0, SCREEN_WIDTH, GRID_SIZE):
            pygame.draw.line(surface, GRAY, (x, 0), (x, SCREEN_HEIGHT), 1)
        for y in range(0, SCREEN_HEIGHT, GRID_SIZE):
            pygame.draw.line(surface, GRAY, (0, y), (SCREEN_WIDTH, y), 1)
        
        # Draw loading dock
        if self.loading_dock:
            x = self.loading_dock[0] * GRID_SIZE
            y = self.loading_dock[1] * GRID_SIZE
            pygame.draw.rect(surface, LOADING_DOCK, (x + 1, y + 1, GRID_SIZE - 2, GRID_SIZE - 2))
        
        # Draw discharge dock (highlight if robot has cargo)
        if self.discharge_dock:
            x = self.discharge_dock[0] * GRID_SIZE
            y = self.discharge_dock[1] * GRID_SIZE
            from constants import RED
            color = (255, 100, 100) if robot.has_cargo else DISCHARGE_DOCK
            pygame.draw.rect(surface, color, (x + 1, y + 1, GRID_SIZE - 2, GRID_SIZE - 2))
            if robot.has_cargo:
                pygame.draw.rect(surface, RED, (x + 1, y + 1, GRID_SIZE - 2, GRID_SIZE - 2), 3)
        
        # Draw goal points (packages) with priority numbers
        # Only draw goals that have been discovered (in OGM)
        font_small = pygame.font.Font(None, 18)
        discovered_goals = set()
        if robot.ogm:
            # Get discovered goals from OGM (convert set to sorted list for consistent ordering)
            discovered_goals = robot.ogm.goals
        
        for idx, (col, row) in enumerate(self.goals):
            # Only draw if goal has been discovered
            if (col, row) in discovered_goals:
                x = col * GRID_SIZE
                y = row * GRID_SIZE
                pygame.draw.rect(surface, GOAL_COLOR, (x + 4, y + 4, GRID_SIZE - 8, GRID_SIZE - 8))
                # Draw priority number
                priority_text = font_small.render(str(idx + 1), True, BLACK)
                text_rect = priority_text.get_rect(center=(x + GRID_SIZE // 2, y + GRID_SIZE // 2))
                surface.blit(priority_text, text_rect)
        
        # Draw obstacles - only draw discovered obstacles (in OGM)
        discovered_obstacles = set()
        if robot.ogm:
            discovered_obstacles = robot.ogm.obstacles
        
        for col, row in self.obstacles:
            # Only draw if obstacle has been discovered
            if (col, row) in discovered_obstacles:
                x = col * GRID_SIZE
                y = row * GRID_SIZE
                pygame.draw.rect(surface, YELLOW, (x + 1, y + 1, GRID_SIZE - 2, GRID_SIZE - 2))
                pygame.draw.rect(surface, DARK_GRAY, (x + 1, y + 1, GRID_SIZE - 2, GRID_SIZE - 2), 1)
