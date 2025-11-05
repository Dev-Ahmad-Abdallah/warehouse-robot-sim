"""
Main entry point for the warehouse robot simulation.
"""

import pygame
import sys
from constants import (
    SCREEN_WIDTH, SCREEN_HEIGHT, FPS, GRID_SIZE,
    WHITE, BLACK, GREEN, RED,
    debug_log
)
from robot import Robot
from warehouse import Warehouse

# Initialize pygame
pygame.init()

# Create the display
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Warehouse Robot Simulation - SLAM")
clock = pygame.time.Clock()

# Initialize warehouse and robot
debug_log("=" * 50)
debug_log("INITIALIZING WAREHOUSE ROBOT SIMULATION WITH SLAM")
debug_log("=" * 50)
warehouse = Warehouse()
robot = Robot(1, 1, warehouse=warehouse)  # Pass warehouse for position validation

# Start autonomous mapping phase
debug_log("")
robot.start_mapping(warehouse)
debug_log("")

# Main game loop
running = True
while running:
    clock.tick(FPS)
    current_time = pygame.time.get_ticks()
    
    # Event handling
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                running = False
    
    # Update robot rotation
    robot.update_rotation(current_time)
    
    # Handle mapping phase or normal gameplay
    if robot.is_mapping:
        # Autonomous mapping phase - robot explores automatically
        robot.explore_next_cell(current_time)
    else:
        # Normal gameplay - only allow manual control after mapping is complete
        if robot.mapping_complete:
            # Handle continuous input
            keys = pygame.key.get_pressed()
            robot.handle_input(keys, warehouse, current_time)
            
            # Check for pickup (space bar)
            if keys[pygame.K_SPACE]:
                if not robot.has_cargo:
                    robot.try_pickup(warehouse, current_time)
            
            # Check for drop (V key)
            if keys[pygame.K_v]:
                if robot.has_cargo:
                    robot.try_drop(warehouse, current_time)
            
            # Auto-select closest goal if none selected
            if robot.current_goal is None and warehouse.goals:
                robot.current_goal = warehouse.goals[0]
    
    # Clear screen
    screen.fill(WHITE)
    
    # Draw warehouse
    warehouse.draw(screen, robot)
    
    # Draw robot
    robot.draw(screen)
    
    # Draw current goal indicator
    if robot.current_goal:
        font_small = pygame.font.Font(None, 20)
        goal_x = int(robot.current_goal[0] * GRID_SIZE + GRID_SIZE // 2)
        goal_y = int(robot.current_goal[1] * GRID_SIZE - 10)
        pygame.draw.line(
            screen, RED,
            (int(robot.x * GRID_SIZE + GRID_SIZE // 2), int(robot.y * GRID_SIZE + GRID_SIZE // 2)),
            (goal_x, goal_y + 10),
            2
        )
    
    # Draw instructions and status
    font = pygame.font.Font(None, 24)
    if robot.is_mapping:
        text = font.render("MAPPING IN PROGRESS... Robot exploring autonomously", True, RED)
        screen.blit(text, (10, 10))
        mapping_status = font.render(
            f"Cells explored: {len(robot.visited)} | Queue: {len(robot.exploration_queue)}",
            True, BLACK
        )
        screen.blit(mapping_status, (10, 35))
        
        # SLAM pose information
        estimated_pose = robot.slam.get_estimated_pose()
        uncertainty = robot.slam.get_uncertainty()
        robot_pos = font.render(
            f"Actual: ({int(robot.x)}, {int(robot.y)}) | Est: ({estimated_pose[0]:.1f}, {estimated_pose[1]:.1f}) | Angle: {int(robot.rotation_angle)}°",
            True, BLACK
        )
        screen.blit(robot_pos, (10, 60))
        
        slam_status = font.render(
            f"Uncertainty: {uncertainty[0]:.3f} | Loop Closure: {'DETECTED' if robot.loop_closure_detected else 'None'}",
            True, BLACK
        )
        screen.blit(slam_status, (10, 85))
        
        # Show mapping progress
        total_free = len(warehouse.get_free_cells())
        progress = len(robot.visited) / total_free * 100 if total_free > 0 else 0
        progress_text = font.render(
            f"Mapping Progress: {progress:.1f}% ({len(robot.visited)}/{total_free} cells)",
            True, BLACK
        )
        screen.blit(progress_text, (10, 110))
    else:
        text = font.render("Arrow Keys: Move | Space: Pickup | V: Drop | ESC: Quit", True, BLACK)
        screen.blit(text, (10, 10))
        
        # SLAM pose information
        estimated_pose = robot.slam.get_estimated_pose()
        uncertainty = robot.slam.get_uncertainty()
        robot_pos = font.render(
            f"Actual: ({int(robot.x)}, {int(robot.y)}) | Est: ({estimated_pose[0]:.1f}, {estimated_pose[1]:.1f}) | Cargo: {'Yes' if robot.has_cargo else 'No'} | Angle: {int(robot.rotation_angle)}°",
            True, BLACK
        )
        screen.blit(robot_pos, (10, 35))
        
        status = font.render(
            f"Goals Remaining: {len(warehouse.goals)} | Score: {robot.score} | Uncertainty: {uncertainty[0]:.3f}",
            True, BLACK
        )
        screen.blit(status, (10, 60))
        
        if robot.loop_closure_detected:
            loop_text = font.render("LOOP CLOSURE DETECTED!", True, (255, 0, 255))
            screen.blit(loop_text, (10, 85))
        
        if robot.mapping_complete:
            mapping_done = font.render("MAPPING COMPLETE - All obstacles and goals identified!", True, GREEN)
            screen.blit(mapping_done, (10, 110))
    
    # Legend
    legend_y = SCREEN_HEIGHT - 100
    legend_font = pygame.font.Font(None, 20)
    legend1 = legend_font.render("Blue circle = Actual robot position", True, BLACK)
    screen.blit(legend1, (10, legend_y))
    legend2 = legend_font.render("Green circle = Estimated pose (SLAM)", True, BLACK)
    screen.blit(legend2, (10, legend_y + 20))
    legend3 = legend_font.render("Orange line = Pose error", True, BLACK)
    screen.blit(legend3, (10, legend_y + 40))
    legend4 = legend_font.render("Magenta circle = Loop closure detected", True, BLACK)
    screen.blit(legend4, (10, legend_y + 60))
    
    # Show completion message
    if len(warehouse.goals) == 0:
        victory_text = font.render("MISSION COMPLETE! All packages delivered!", True, GREEN)
        screen.blit(victory_text, (300, 300))
    
    # Update display
    pygame.display.flip()

# Cleanup
pygame.quit()
sys.exit()
