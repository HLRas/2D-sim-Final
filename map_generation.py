import pygame
import numpy as np

from config import *
from cubes import Cube
from parking_space import ParkingSpace
from astar_pathfind import AStarPathfinder

class Map:
    def __init__(self, layout_type=0):
        """Initialize the map with optimized performance
        
        Args:
            layout_type (int): 0=Default with parking, 1=Empty, 2=Minimal
        """
        self.layout_type = layout_type
        self.rows = SCREEN_HEIGHT // CUBE_SIZE
        self.cols = SCREEN_WIDTH // CUBE_SIZE
        
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        self.screen.fill(BLACK)
        
        # Initialize dirty cubes set first
        self.dirty_cubes = set()  # Track which cubes need redrawing
        
        # Grid management
        self.cubes = self._generate_blank_cubes()
        self.grid_surface = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT))
        self._draw_initial_grid()
        
        # Pathfinding
        self.start = None
        self.end = None
        self.pathfinder = AStarPathfinder(SCREEN_WIDTH, SCREEN_HEIGHT, CUBE_SIZE)
        
        # Parking spaces
        self.parking_spaces = []
        self._create_layout()
        
        # Neighbors cache for performance
        self.neighbors_updated = False
        
        # Mark all cubes as dirty for initial draw
        for row in self.cubes:
            for cube in row:
                self.dirty_cubes.add(cube)

    def _generate_blank_cubes(self):
        """Generate initial grid of cubes"""
        cubes = np.empty((self.rows, self.cols), dtype=object)
        for i in range(self.rows):
            for j in range(self.cols):
                cubes[i][j] = Cube(i, j)
        return cubes

    def _draw_initial_grid(self):
        """Draw the initial grid lines"""
        self.grid_surface.fill(WHITE)
        for x in range(0, SCREEN_WIDTH, CUBE_SIZE):
            pygame.draw.line(self.grid_surface, BLACK, (x, 0), (x, SCREEN_HEIGHT))
        for y in range(0, SCREEN_HEIGHT, CUBE_SIZE):
            pygame.draw.line(self.grid_surface, BLACK, (0, y), (SCREEN_WIDTH, y))

    def _create_layout(self):
        """Create the appropriate layout based on layout_type"""
        if self.layout_type == 0:
            self._create_default_parking_spaces()
        elif self.layout_type == 1:
            self._create_empty_layout()
        elif self.layout_type == 2:
            self._create_minimal_layout()

    def _create_default_parking_spaces(self):
        """Create default layout with parking spaces (Layout 0)"""
        # Generate 5 horizontal parking spaces on the right side, stacked vertically
        self.parking_spaces = []
        space_height = 5
        num_spaces = 5
        
        for i in range(num_spaces):
            y_position = i * space_height  # Stack them vertically with no gap
            if i == 2:
                space = ParkingSpace(self.cols - 7, y_position, 7, space_height, 'horizontal', occupied=True, permanently_occupied=True)
            else:
                space = ParkingSpace(self.cols - 7, y_position, 7, space_height, 'horizontal')
            self.parking_spaces.append(space)

        # Apply to grid
        for space in self.parking_spaces:
            space.apply_to_grid(self.cubes)
    
    def _create_empty_layout(self):
        """Create completely empty layout (Layout 1)"""
        # No parking spaces or obstacles - completely empty
        self.parking_spaces = []
    
    def _create_minimal_layout(self):
        """Create minimal layout with some basic obstacles (Layout 2)"""
        self.parking_spaces = []
        
        # Add some simple barriers/obstacles for variety
        # Create a few scattered obstacles
        obstacles = [
            (10, 10, 3, 3),  # x, y, width, height
            (25, 8, 2, 5),
            (15, 20, 4, 2),
            (30, 15, 2, 2)
        ]
        
        for obs_x, obs_y, obs_width, obs_height in obstacles:
            for x in range(obs_x, min(obs_x + obs_width, self.cols)):
                for y in range(obs_y, min(obs_y + obs_height, self.rows)):
                    if 0 <= x < self.cols and 0 <= y < self.rows:
                        self.cubes[y][x].make_barrier(self.cubes)

        # Create parking space
        self.add_parking_space(30, 10, 7, 5, 'horizontal')

    def add_parking_space(self, grid_x, grid_y, width=7, height=5, orientation='horizontal'):
        """Add a new parking space"""
        space = ParkingSpace(grid_x, grid_y, width, height, orientation)
        self.parking_spaces.append(space)
        space.apply_to_grid(self.cubes)
        self.neighbors_updated = False
        return space

    def get_cube(self, pos):
        """Get cube at world position"""
        grid_x = pos[0] // CUBE_SIZE
        grid_y = pos[1] // CUBE_SIZE
        
        if 0 <= grid_x < self.cols and 0 <= grid_y < self.rows:
            return self.cubes[grid_y][grid_x]
        return None

    def mark_dirty(self, cube):
        """Mark a cube as needing redraw"""
        self.dirty_cubes.add(cube)

    def _update_neighbors_if_needed(self):
        """Update neighbors only when needed"""
        if not self.neighbors_updated:
            for row in self.cubes:
                for cube in row:
                    cube.update_neighbours(self.cubes)
            self.neighbors_updated = True

    def draw(self):
        """Optimized drawing - only redraw dirty cubes"""
        # Draw base grid
        self.screen.blit(self.grid_surface, (0, 0))
        
        # Draw all cubes (barriers will show as black)
        for row in self.cubes:
            for cube in row:
                if cube.type != 0:  # Only draw non-white cubes (barriers, paths, etc.)
                    cube.draw()
        
        # Draw path if exists
        if self.pathfinder.has_path():
            path_surface = self.pathfinder.get_path_surface()
            self.screen.blit(path_surface, (0, 0))
        
        # Draw parking space info
        font = pygame.font.Font(None, 24)
        for space in self.parking_spaces:
            space.draw_info(self.screen, font)

    def check_inputs(self, event, car):
        """Handle input events"""
        if event.key == pygame.K_q:
            # Set start at car position
            car_center = car.get_rect().center
            cube = self.get_cube(car_center)
            if cube:
                if self.start:
                    self.start.make_clear()
                    self.mark_dirty(self.start)
                
                cube.make_start()
                self.start = cube
                self.mark_dirty(cube)
        
        elif event.key == pygame.K_a and self.start:
            # Find nearest parking space and pathfind to it
            nearest_space = self._find_nearest_parking_space(car)
            if nearest_space and nearest_space.target_cube:
                self.end = nearest_space.target_cube
                self._update_neighbors_if_needed()
                self.pathfinder.clear_path(self.cubes, self.mark_dirty)
                path_found = self.pathfinder.pathfind(self.cubes, self.start, self.end, self.mark_dirty)
                
                # If path was found, start automatic path following
                if path_found and self.pathfinder.has_path():
                    path_points = self.pathfinder.get_smooth_points()
                    if path_points:
                        car.carrot_start_following(path_points)
                        print("Starting automatic path following to parking space!")

        elif event.key == pygame.K_s and self.start:
            # Find nearest parking space and pathfind to it using cross-track error method
            nearest_space = self._find_nearest_parking_space(car)
            if nearest_space and nearest_space.target_cube:
                self.end = nearest_space.target_cube
                self._update_neighbors_if_needed()
                self.pathfinder.clear_path(self.cubes, self.mark_dirty)
                path_found = self.pathfinder.pathfind(self.cubes, self.start, self.end, self.mark_dirty)
                
                # If path was found, start cross-track error path following
                if path_found and self.pathfinder.has_path():
                    path_points = self.pathfinder.get_smooth_points()
                    if path_points:
                        car.cross_start_following(path_points)
                        print("Starting cross-track error path following to parking space!")

        elif event.key == pygame.K_c:
            # Clear path and stop path following
            self.pathfinder.clear_path(self.cubes, self.mark_dirty)
            car.stop_path_following()

    def _find_nearest_parking_space(self, car):
        """Find the nearest available parking space"""
        car_pos = car.get_rect().center
        nearest_space = None
        min_distance = float('inf')
        
        available_count = 0
        occupied_count = 0
        
        for i, space in enumerate(self.parking_spaces):
            if not space.occupied:
                available_count += 1
                target_pos = space.get_target_position()
                if target_pos:
                    distance = ((car_pos[0] - target_pos[0]) ** 2 + 
                               (car_pos[1] - target_pos[1]) ** 2) ** 0.5
                    if distance < min_distance:
                        min_distance = distance
                        nearest_space = space
            else:
                occupied_count += 1
        
        return nearest_space

    def get_obstacles(self):
        """Return the obstacle grid for collision detection"""
        return self.cubes



                
            
    