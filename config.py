"""Configuration constants for the car simulation"""

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165, 0)
GREY = (128, 128, 128)
TURQUOISE = (64, 224, 208)

# Screen settings
SCREEN_WIDTH = 900
SCREEN_HEIGHT = 750
CUBE_SIZE = 25
FPS = 120

# Car physics
CAR_ACCELERATION = 400.0 #pixels/s^2 (=0.8m/s^2)
CAR_FRICTION = 0.0  # Base friction value
STATIC_FRICTION_RATIO = 0.8  # Fraction of CAR_FRICTION used for static friction
ROLLING_FRICTION_RATIO = 0.2  # Fraction of CAR_FRICTION used for rolling friction
STATIC_THRESHOLD = 50.0  # Speed threshold for static vs rolling friction (pixels/s)
MAX_WHEEL_SPEED = 300 # pixels/s (=0.6m/s)
TURN_RATE = 1
CAR_WIDTH = 65
CAR_LENGTH = 100
WHEEL_RADIUS = 10 # consider changing to 16

# Pathfinding
STRAIGHT_COST = 10
DIAGONAL_COST = 14
SPLINE_SMOOTHNESS = 1000
PATH_WIDTH = 2
