import pygame
from config import *

class Cube:
    def __init__(self, row:int, col:int, type:int=0, squareSize:int=CUBE_SIZE) -> None:
        """Initialize the Cube object\n
        row : y (row) position (topleft) of cube
        col : x (col) position (topleft) of cube
        type : what type of cube (0=free, 1=barrier, 2=start, 3=end)
        """
        self.neighbours = []
        self.row = row
        self.col = col
        self.x = col * squareSize
        self.y = row * squareSize
        self.type = type if type in range(7) else 0
        self.squareSize = squareSize
        self.totRow = SCREEN_HEIGHT // self.squareSize
        self.totCol = SCREEN_WIDTH // self.squareSize
        self.set_color()

        self.rect = pygame.Rect(self.x, self.y, squareSize, squareSize)
        # Don't draw here - let the map handle drawing

    def set_color(self) -> None:
        """Set the color of the cube based on its type\n
        0 = Free\n
        1 = Barrier\n
        2 = Start\n
        3 = End\n
        4 = Closed\n
        5 = Open\n
        6 = Path\n
        7 = Soft Barrier"""
        if self.type == 0:
            self.color = WHITE
        elif self.type == 1:
            self.color = BLACK
        elif self.type == 2:
            self.color = ORANGE
        elif self.type == 3:
            self.color = TURQUOISE
        elif self.type == 4:
            self.color = RED
        elif self.type == 5:
            self.color = GREEN
        elif self.type == 6:
            self.color = PURPLE
        elif self.type == 7:
            self.color = GREY
        else:
            self.color = WHITE
    
    def get_pos(self):
        return self.x,self.y

    def make_clear(self):
        self.change_type(0)

    def make_start(self):
        self.change_type(2)

    def make_closed(self):
        self.change_type(4)

    def make_open(self):
        self.change_type(5)

    def make_barrier(self, cubes=None):
        """Make this cube a barrier and create soft barriers around this barrier"""
        self.change_type(1)
        
        # If cubes grid is provided, create soft barriers around this barrier
        if cubes is not None:
            rows, cols = len(cubes), len(cubes[0])
            # Check all 8 surrounding positions
            for dy in range(-1, 2):
                for dx in range(-1, 2):
                    if dx == 0 and dy == 0:
                        continue
                    new_row, new_col = self.row + dy, self.col + dx
                    if (0 <= new_row < rows and 0 <= new_col < cols):
                        neighbor = cubes[new_row][new_col]
                        # Only make it a soft barrier if it's currently clear
                        if neighbor.type == 0:  # Clear/free space
                            neighbor.make_softbarrier()

    def make_softbarrier(self):
        self.change_type(7)

    def make_end(self):
        self.change_type(3)

    def make_path(self):
        self.change_type(6)

    def make_occupied(self):
        self.change_type(4)

    def is_barrier(self):
        return self.type == 1 or self.type == 7
    
    def is_softbarrier(self):
        return self.type == 7
    
    def is_hardbarrier(self):
        return self.type == 1

    def change_type(self, type:int) -> None:
        """Set the type of the cube and update its color\n
        type: The new type of the cube (0=free, 1=barrier, 2=start, 3=end, 4=closed, 5=open, 6=path, 7=softbarrier)"""
        self.type = type if type in range(8) else 0
        self.set_color()

    def update_neighbours(self, cubes):
        self.neighbours = []

        # DOWN
        if self.row < self.totRow - 1 and not cubes[self.row + 1][self.col].is_barrier():
            self.neighbours.append(cubes[self.row + 1][self.col])
        # UP
        if self.row > 0 and not cubes[self.row - 1][self.col].is_barrier():
            self.neighbours.append(cubes[self.row - 1][self.col])
        # RIGHT
        if self.col < self.totCol - 1 and not cubes[self.row][self.col + 1].is_barrier():
            self.neighbours.append(cubes[self.row][self.col + 1])
        # LEFT
        if self.col > 0 and not cubes[self.row][self.col - 1].is_barrier():
            self.neighbours.append(cubes[self.row][self.col - 1])

        # DOWN-RIGHT
        if (self.row < self.totRow - 1 and self.col < self.totCol - 1
            and not cubes[self.row + 1][self.col + 1].is_barrier()):
            self.neighbours.append(cubes[self.row + 1][self.col + 1])

        # DOWN-LEFT
        if (self.row < self.totRow - 1 and self.col > 0
            and not cubes[self.row + 1][self.col - 1].is_barrier()):
            self.neighbours.append(cubes[self.row + 1][self.col - 1])

        # UP-RIGHT
        if (self.row > 0 and self.col < self.totCol - 1
            and not cubes[self.row - 1][self.col + 1].is_barrier()):
            self.neighbours.append(cubes[self.row - 1][self.col + 1])

        # UP-LEFT
        if (self.row > 0 and self.col > 0
            and not cubes[self.row - 1][self.col - 1].is_barrier()):
            self.neighbours.append(cubes[self.row - 1][self.col - 1])

        
    def draw(self) -> None:
        """Draw the cube on the screen"""
        pygame.draw.rect(pygame.display.get_surface(), self.color, self.rect)

        