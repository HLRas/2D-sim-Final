from queue import PriorityQueue
import pygame
import numpy as np
from scipy.interpolate import splprep, splev
from config import *

class AStarPathfinder:
    def __init__(self, screen_width, screen_height, cube_size):
        """Initialize the A* pathfinder"""
        self.screen_width = screen_width
        self.screen_height = screen_height
        self.cube_size = cube_size

        # Path visualization
        self.path_points = None
        self.path_surface = pygame.Surface((screen_width, screen_height), pygame.SRCALPHA)

    def heuristic(self,p1,p2):
        """Octile distance heuristic for A*"""
        x1, y1 = p1
        x2, y2 = p2
        dx = abs(x1 - x2)
        dy = abs(y1 - y2)
        return STRAIGHT_COST * (dx + dy) + (DIAGONAL_COST - 2 * STRAIGHT_COST) * min(dx, dy)
    
    def pathfind(self, cubes, start, end, mark_dirty_callback = None):
        """
        A* pathfinding algorithm with optimizations
        
        Args:
            cubes: 2D array of cube objects
            start: Starting cube
            end: Target cube
            mark_dirty_callback: Function to call when marking cube as dirty for redraw
            
        Returns:
            bool: True if path found, False otherwise
            """
        
        if not start or not end:
            return False
        
        count = 0
        open_set = PriorityQueue()
        open_set.put((0, count, start))
        came_from = {}

        g_score = {cube: float("inf") for row in cubes for cube in row}
        g_score[start] = 0

        f_score = {cube: float("inf") for row in cubes for cube in row}
        f_score[start] = self.heuristic(start.get_pos(), end.get_pos())

        open_set_hash = {start}

        while not open_set.empty():
            current = open_set.get()[2]
            open_set_hash.remove(current)
            
            if current == end:
                self.reconstruct_path(came_from, end, start, mark_dirty_callback)
                return True

            for neighbor in current.neighbours:
                # Calculate movement cost (diag vs straight)
                if (abs(neighbor.row - current.row) == 1 and 
                    abs(neighbor.col - current.col) == 1):
                    move_cost = DIAGONAL_COST
                else:
                    move_cost = STRAIGHT_COST

                temp_g_score = g_score[current] + move_cost

                if temp_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = temp_g_score
                    f_score[neighbor] = temp_g_score + self.heuristic(neighbor.get_pos(), end.get_pos())

                    if neighbor not in open_set_hash:
                        count += 1
                        open_set.put((f_score[neighbor], count, neighbor))
                        open_set_hash.add(neighbor)
                        neighbor.make_open()
                        if mark_dirty_callback:
                            mark_dirty_callback(neighbor)
            if current != start:
                current.make_closed()
                if mark_dirty_callback:
                    mark_dirty_callback(current)
                    
        return False
    
    def reconstruct_path(self, came_from, current, start, mark_dirty_callback=None):
        """Reconstruct and visualize the path"""
        path_cubes = []
        while current in came_from:
            current = came_from[current]
            if current != start:
                current.make_path()
                if mark_dirty_callback:
                    mark_dirty_callback(current)
                path_cubes.append(current)
        
        # Create smooth path
        if len(path_cubes) >= 2:
            points = []
            for cube in reversed(path_cubes):
                pos = cube.get_pos()
                points.append((pos[0] + self.cube_size // 2, pos[1] + self.cube_size//2))

                self.path_points = points
                self.draw_smooth_path(points)

    def draw_smooth_path(self, points):
        """Draw smooth spline path"""
        if len(points) < 2:
            return
        
        self.path_surface.fill((0,0,0,0))

        try:
            x, y = zip(*points)
            x, y = np.array(x), np.array(y)

            k = min(3, len(x) -1)

            if k<1:
                return
            
            tck, u = splprep([x,y], s=SPLINE_SMOOTHNESS, k=k)
            u_fine = np.linspace(0,1,400)
            x_fine, y_fine = splev(u_fine, tck)

            coords = [(int(x_fine[i]), int(y_fine[i])) for i in range(len(x_fine))]

            for i in range(len(coords) -1):
                pygame.draw.line(self.path_surface, RED, coords[i], coords[i+1], PATH_WIDTH)

        except Exception as e:
            # Fallback to simeple line drawing
            for i in range(len(points)-1):
                pygame.draw.line(self.path_surface, RED, points[i], points[i+1], PATH_WIDTH)

    def clear_path(self, cubes, mark_dirty_callback=None):
        """Clear the current path visualization"""
        for row in cubes:
            for cube in row:
                if cube.type in [4,5,6]:
                    cube.make_clear()
                    if mark_dirty_callback:
                        mark_dirty_callback(cube)
        
        self.path_surface.fill((0,0,0,0))
        self.path_points = None

    def get_path_surface(self):
        """Get the path surface for drawing"""
        return self.path_surface
    
    def has_path(self):
        """Check if there is a current path"""
        return self.path_points is not None
    
    def get_path_points(self):
        """Get the current path points"""
        return self.path_points