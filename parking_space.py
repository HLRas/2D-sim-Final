import pygame
from config import *

class ParkingSpace:
    def __init__(self, grid_x, grid_y, width=7, height=5, orientation='horizontal', occupied=False, permanently_occupied=False):
        """
        Initialize a parking space
        
        Args:
            grid_x, grid_y: Top-left corner in grid coordinates
            width, height: Size in grid units
            orientation: 'horizontal' or 'vertical'
            occupied: Whether currently occupied by a car
            permanently_occupied: Whether permanently blocked (won't be freed when car leaves)
        """
        self.grid_x = grid_x
        self.grid_y = grid_y
        self.width = width
        self.height = height
        self.orientation = orientation
        self.occupied = occupied
        self.permanently_occupied = permanently_occupied
        self.target_cube = None  # The cube cars should navigate to
        
        # Calculate world coordinates
        self.world_x = grid_x * CUBE_SIZE
        self.world_y = grid_y * CUBE_SIZE
        
        # Create parking space boundaries and target
        self.barrier_positions = []
        self.entry_position = None
        self._create_layout()

    def _create_layout(self):
        """Create the parking space layout with barriers and entry point"""
        if self.orientation == 'horizontal':
            # Create horizontal parking space
            for i in range(self.height):
                for j in range(self.width):
                    if i in [1,2,3]:  # Middle row is the gap (entry)
                        if j == 5 and i == 2:  # Target position
                            self.entry_position = (self.grid_x + j, self.grid_y + i)
                        continue
                    self.barrier_positions.append((self.grid_x + j, self.grid_y + i))
        
        elif self.orientation == 'vertical':
            # Create vertical parking space
            for i in range(self.height):
                for j in range(self.width):
                    if j in [1,2,3]:  # Middle column is the gap (entry)
                        if i == self.height - 2:  # Target position
                            self.entry_position = (self.grid_x + j, self.grid_y + i)
                        continue
                    self.barrier_positions.append((self.grid_x + j, self.grid_y + i))

    def apply_to_grid(self, cubes):
        """Apply parking space layout to the grid"""
        rows, cols = len(cubes), len(cubes[0])
        
        # Add barriers
        for x, y in self.barrier_positions:
            if 0 <= x < cols and 0 <= y < rows:
                cubes[y][x].make_barrier(cubes)
        
        # Set target position
        if self.entry_position:
            x, y = self.entry_position
            if 0 <= x < cols and 0 <= y < rows:
                if self.occupied:
                    cubes[y][x].make_occupied()  # Red for occupied
                else:
                    cubes[y][x].make_end()  # Target color for available
                self.target_cube = cubes[y][x]
        
        # If initially occupied, apply interior barriers
        if self.occupied:
            self._make_interior_barriers(cubes)

    def set_occupied(self, occupied, cubes):
        """Set the occupation status and update the grid accordingly"""
        if self.occupied == occupied:
            return  # No change needed
            
        self.occupied = occupied
        rows, cols = len(cubes), len(cubes[0])
        
        if occupied:
            # Make all positions barriers except entry
            for x, y in self.barrier_positions:
                if 0 <= x < cols and 0 <= y < rows:
                    cubes[y][x].make_barrier(cubes)
            
            # Make entry position red (occupied indicator)
            if self.entry_position:
                x, y = self.entry_position
                if 0 <= x < cols and 0 <= y < rows:
                    cubes[y][x].make_occupied()
                    
            # Add interior positions as barriers too
            self._make_interior_barriers(cubes)
            
        else:
            # Restore original layout - barriers stay, but entry becomes available again
            for x, y in self.barrier_positions:
                if 0 <= x < cols and 0 <= y < rows:
                    cubes[y][x].make_barrier(cubes)
            
            # Make entry position available (target color)
            if self.entry_position:
                x, y = self.entry_position
                if 0 <= x < cols and 0 <= y < rows:
                    cubes[y][x].make_end()
                    
            # Clear interior barriers
            self._clear_interior_barriers(cubes)

    def _make_interior_barriers(self, cubes):
        """Make all interior positions of the parking space into barriers when occupied"""
        rows, cols = len(cubes), len(cubes[0])
        
        if self.orientation == 'horizontal':
            # Fill the middle area (where cars would park) with barriers
            for i in range(self.height):
                for j in range(self.width):
                    x, y = self.grid_x + j, self.grid_y + i
                    if (x, y) != self.entry_position and (x, y) not in self.barrier_positions:
                        if 0 <= x < cols and 0 <= y < rows:
                            cubes[y][x].make_barrier(cubes)
                            
        elif self.orientation == 'vertical':
            # Fill the middle area (where cars would park) with barriers
            for i in range(self.height):
                for j in range(self.width):
                    x, y = self.grid_x + j, self.grid_y + i
                    if (x, y) != self.entry_position and (x, y) not in self.barrier_positions:
                        if 0 <= x < cols and 0 <= y < rows:
                            cubes[y][x].make_barrier(cubes)

    def _clear_interior_barriers(self, cubes):
        """Regenerate original parking space layout when becoming unoccupied"""
        rows, cols = len(cubes), len(cubes[0])
        
        # First, clear the entire parking space area
        for i in range(self.height):
            for j in range(-1, self.width): # also clear the soft barriers at the entrance
                x, y = self.grid_x + j, self.grid_y + i
                if 0 <= x < cols and 0 <= y < rows:
                    cubes[y][x].make_clear()
        
        # Regenerate the original parking space layout
        # Add the original barriers
        for x, y in self.barrier_positions:
            if 0 <= x < cols and 0 <= y < rows:
                cubes[y][x].make_barrier(cubes)
        
        # Set the entry position as available (target)
        if self.entry_position:
            x, y = self.entry_position
            if 0 <= x < cols and 0 <= y < rows:
                cubes[y][x].make_end()
                self.target_cube = cubes[y][x]

    def is_car_in_space(self, car):
        """Check if a car is properly parked in this space"""
        car_grid_pos = car.get_grid_position()
        
        if self.entry_position:
            target_x, target_y = self.entry_position
            # Allow some tolerance for parking
            return (abs(car_grid_pos[0] - target_x) <= 1 and 
                    abs(car_grid_pos[1] - target_y) <= 1)
        return False

    def get_target_position(self):
        """Get the world coordinates of the target position"""
        if self.entry_position:
            x, y = self.entry_position
            return (x * CUBE_SIZE + CUBE_SIZE // 2, 
                    y * CUBE_SIZE + CUBE_SIZE // 2)
        return None

    def get_target_cube(self):
        """Get the target cube for pathfinding"""
        return self.target_cube

    def draw_info(self, screen, font):
        """Draw parking space information"""
        if self.entry_position:
            pos = self.get_target_position()
            if pos:
                color = GREEN if not self.occupied else RED
                pygame.draw.circle(screen, color, (int(pos[0]), int(pos[1])), 8)
                
                # Draw ID
                text = font.render(f"P{id(self) % 1000}", True, BLACK)
                screen.blit(text, (pos[0] - 15, pos[1] - 25))
