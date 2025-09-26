import pygame
import math
from config import *

class Car:
    def __init__(self, x=50, y=50):
        """Initialize the car with its properties"""
        # Position
        self.x = float(x)
        self.y = float(y)
        self.prev_x = self.x
        self.prev_y = self.y

        # Movement properties
        self.wheel_acceleration = CAR_ACCELERATION
        self.static_friction = CAR_FRICTION * STATIC_FRICTION_RATIO  # Static friction coefficient
        self.rolling_friction = CAR_FRICTION * ROLLING_FRICTION_RATIO  # Rolling friction coefficient
        self.static_threshold = STATIC_THRESHOLD  # Speed threshold below which static friction applies
        self.max_wheel_speed = MAX_WHEEL_SPEED  # Direct linear speed limit (pixels/s)

        # Wheel speeds (linear speeds in pixels/s)
        self.wheel_L_speed = 0.0 # pixels/s
        self.prev_wheel_L = 0.0
        self.wheel_R_speed = 0.0 # pixels/s
        self.prev_wheel_R = 0.0

        # Vehicle properties
        self.speed = 0.0 # total speed pixels/s
        self.angle = 0.0 # radians
        self.width = CAR_WIDTH
        self.length = CAR_LENGTH

        # Visual representation
        self._create_surface()
        self.screen = pygame.display.get_surface()
        self.max_wheel_speed = MAX_WHEEL_SPEED  # Direct linear speed limit (pixels/s)

        # Wheel speeds (linear speeds in pixels/s)
        self.wheel_L_speed = 0.0 # pixels/s
        self.wheel_R_speed = 0.0 # pixels/s

        # Vehicle properties
        self.speed = 0.0 # total speed pixels/s
        self.angle = 0.0 # radians
        self.width = CAR_WIDTH
        self.length = CAR_LENGTH

        # Visual representation
        self._create_surface()
        self.screen = pygame.display.get_surface()

        # Carrot path following
        self.carrot_following = False
        self.carrot_path_points = []
        self.carrot_index = 0
        self.carrot_base_distance = 25 # Minimum lookahead distance
        self.carrot_arrival_threshold = 30
        self.carrot_slowdown_coeff = 0.6 # Slowdown near destination
        self.carrot_base_speed = 1.7 # This was 0.5
        self.carrot_max_velo_bonus = 0.2 # Maximum velocity maintainence bonus
        self.carrot_min_target_speed = 0.2
        self.carrot_velo_bonus_coeff = 0.1 # Velocity maintainence coeff
        self.carrot_max_turn_rate = 2.5 # Max turn rate in rad
        self.carrot_command_coeff = 0.4 # Scale wheel commands by this
        self.carrot_turn_pen_coeff = 0.05 # Turn penalty coefficient

        # Cross-track error following
        self.cross_following = False
        self.cross_path_points = []
        self.cross_index = 0
        self.cross_base_distance = 100 # Minimum lookahead distance
        self.cross_arrival_threshold = 30
        self.cross_max_turn_rate = 250 # Max turn rate in rad
        self.cross_heading_scale = 100
        self.cross_base_speed = 200 # p/s
        self.cross_turn_pen_coeff = 0.0 # Turn penalty coefficient
        self.cross_min_target_speed = 150
        self.cross_command_coeff = 1 # Scale wheel commands by this

    
    """===========================GENERAL FUNCTIONS==============================="""
    def _create_surface(self):
        """Create the car's visual surface"""
        self.car_surface = pygame.Surface((self.length, self.width), pygame.SRCALPHA)
        self.car_surface.fill(YELLOW)

        # Draw direction indicator
        front_center = (self.length - 1, self.width //2)
        mid_center = (self.length // 2, self.width //2)
        pygame.draw.line(self.car_surface, BLACK, mid_center, front_center, 4)
        pygame.draw.rect(self.car_surface, BLACK, self.car_surface.get_rect(), 2)

        self.car_rect = self.car_surface.get_rect()

    def draw(self):
        """Draw the car on screen"""
        self.car_rect.center = (self.x, self.y)
        rotated_surf = pygame.transform.rotate(self.car_surface, math.degrees(self.angle))
        rotated_rect = rotated_surf.get_rect(center=self.car_rect.center)
        self.screen.blit(rotated_surf, rotated_rect.topleft)

    def get_grid_position(self):
        """Get car's position in grid coordinates"""
        return (int(self.x // CUBE_SIZE), int(self.y // CUBE_SIZE))
    
    def get_pos(self):
        """Get the position of the car"""
        return (self.x, self.y)

    def apply_friction(self, dt): # unused!!!
        """Apply static and rolling friction to the wheels"""
        
        # Apply friction to left wheel
        self.wheel_L_speed = self._apply_wheel_friction(self.wheel_L_speed, self.prev_wheel_L, dt)
        
        # Apply friction to right wheel
        self.wheel_R_speed = self._apply_wheel_friction(self.wheel_R_speed, self.prev_wheel_R, dt)

    def _apply_wheel_friction(self, wheel_speed, wheel_prevSpeed, dt): # unused!!!
        if (abs(wheel_speed) == abs(wheel_prevSpeed)) and (abs(wheel_speed) > 1e-3):
            speed = wheel_speed - self.rolling_friction*dt
            if math.copysign(1, speed) == math.copysign(1, wheel_speed):
                return speed
            else:
                return 0
        else:
            return wheel_speed
        
            

    def find_next_pos(self, dt):
        """Calculate and apply next position"""
        # Store previous position
        self.prev_x, self.prev_y = self.x, self.y
        prev_angle = self.angle

        # Apply friction
        self.prev_wheel_L = self.wheel_L_speed
        self.prev_wheel_R = self.wheel_R_speed
        # self.apply_friction(dt)

        # Calculate movement - wheel speeds are directly in pixels/s
        self.speed = (self.wheel_L_speed + self.wheel_R_speed) / 2
        self.angle += (self.wheel_R_speed - self.wheel_L_speed) / self.width * dt * TURN_RATE
        
        # Calculate new position using direct wheel speeds
        new_x = self.x + self.speed * math.cos(self.angle) * dt
        new_y = self.y - self.speed * math.sin(self.angle) * dt

        self.x, self.y = new_x, new_y

        # Update position
        self.x = max(0, min(self.x, SCREEN_WIDTH))
        self.y = max(0, min(self.y, SCREEN_HEIGHT))

    def change_wheel_speed(self, wheel, dt, accelerate):
        """Change speed of specified wheel"""
        delta = self.wheel_acceleration * dt * (1 if accelerate else -1)

        if wheel == 'left':
            self.wheel_L_speed = max(-self.max_wheel_speed, 
                                   min(self.wheel_L_speed + delta, self.max_wheel_speed))
        elif wheel == 'right':
            self.wheel_R_speed = max(-self.max_wheel_speed, 
                                   min(self.wheel_R_speed + delta, self.max_wheel_speed))
            
    def get_rect(self):
        """Get the rectangle of the car"""
        return self.car_rect
    
    def check_inputs(self, dt, keys):
        """Process keyboard inputs"""
        if not self.carrot_following and not self.cross_following:
            if keys[pygame.K_z]:
                self.change_wheel_speed('left', dt, True)
            if keys[pygame.K_x]:
                self.change_wheel_speed('left', dt, False)
            if keys[pygame.K_COMMA]:
                self.change_wheel_speed('right', dt, True)
            if keys[pygame.K_PERIOD]:
                self.change_wheel_speed('right', dt, False)

    def set_position(self, pos: tuple):
        """Set position of car"""
        self.x = pos[0]
        self.y = pos[1]

        # Update the car rectangle to reflect the new position
        """SURELY I DONT HAVE TO UPDATE IT HERE? I DO IT IN DRAW...----------------"""
        self.car_rect.center = (self.x, self.y)

        return
    
    def set_orientation(self, orient: float):
        """Set orientation of car
        
            Args:
                orient: Orientation in radians
        """
        self.angle = orient

        return
    
    def get_speeds(self):
        """Get wheel speeds in m/s for Arduino communication"""
        # Convert from pixels/s to m/s 
        # Simple conversion factor - adjust as needed for your scale
        pixels_to_meters = 1.0/500  # 1p = 2mm, 1000p = 2m (adjust as needed)
        return [self.wheel_L_speed * pixels_to_meters, 
                self.wheel_R_speed * pixels_to_meters]
    
    def get_friction_info(self):
        """Get current friction state information for debugging"""
        left_speed_mag = abs(self.wheel_L_speed)
        right_speed_mag = abs(self.wheel_R_speed)
        
        left_friction_type = "static" if left_speed_mag <= self.static_threshold else "rolling"
        right_friction_type = "static" if right_speed_mag <= self.static_threshold else "rolling"
        
        return {
            "left_friction_type": left_friction_type,
            "right_friction_type": right_friction_type,
            "left_speed": self.wheel_L_speed,
            "right_speed": self.wheel_R_speed,
            "static_threshold": self.static_threshold
        }
    """==========================================================================="""

    """==================CARROT FOLLOWING FUNCTIONS==============================="""
    def carrot_start_following(self, path_points):
        """Start following a path using carrot-stick method"""
        # If there arent enough points, quit.
        if len(path_points) < 2:
            print("[Carrot] Not enough points to follow")
            return False
        
        self.carrot_path_points = path_points[:]
        self.carrot_target_index = 0
        self.carrot_following = True
        self.cross_following = False # Disable cross-track following

        print(f"[Carrot] Started following path with {len(path_points)} points")
        return True

    def _carrot_find_point(self):
        """Find the carrot point ahead of the path"""
        if not self.carrot_path_points or self.carrot_index >= len(self.carrot_path_points):
            return None
        
        car_pos = (self.x, self.y)

        # Dynamic carrot distance based on current speed for smoother high-speed driving
        current_speed = self.speed
        dynamic_carrot_distance = self.carrot_base_distance + current_speed * 10 # Further distance if quicker

        # Start from current target and look ahead
        for i in range(self.carrot_target_index, len(self.carrot_path_points)):
            point = self.carrot_path_points[i]
            distance = math.sqrt((point[0] - car_pos[0])**2 + (point[1] - car_pos[1])**2)

            # If this point is at the desired carrot distance or beyond
            if distance >= dynamic_carrot_distance:
                return point
            
        # If no point is far enough, return the last point
        return self.carrot_path_points[-1]
    
    def _carrot_update_index(self):
        """Update the current target index based on car position"""
        if not self.carrot_path_points:
            print("[Carrot] No path points to update index with")
            return
        
        car_pos = (self.x, self.y)

        # Check if we have reached the current target
        if self.carrot_index < len(self.carrot_path_points):
            target = self.carrot_path_points[self.carrot_index]
            distance = math.sqrt((target[0] - car_pos[0])**2 + (target[1] - car_pos[1])**2)

            if distance < self.carrot_arrival_threshold:
                self.carrot_index += 1
                print(self.carrot_index)
                # Only show progress every 5 waypoints
                if (self.carrot_index -1) % 5 == 0 or self.carrot_index >= len(self.carrot_path_points):
                    print(f"[Carrot] Reached waypoint {self.carrot_index-1}, moving to next ({self.carrot_index}/{len(self.carrot_path_points)})")
                if self.carrot_index >= len(self.carrot_path_points):
                    # Reached the end of the path
                    print("[Carrot] Reached destination! Path following complete")
                    self.stop_path_following()
                    return
                
    def _carrot_calc_steering_command(self, target_point):
        """Calculate wheel commands to steer towards target point"""
        if not target_point:
            print("[Carrot] No target point, sending (0, 0) speeds")
            return 0, 0
        
        # Calculate angle to target
        dx = target_point[0] - self.x
        dy = target_point[1] - self.y

        # Fix coordinate system - pygame Y increases downward, but car's physics
        # expects negative Y for forward movement, so we need to flip dy
        target_angle = math.atan2(-dy,dx)

        # Calculate angle difference
        angle_diff = target_angle - self.angle
        
        # Normalise angle to -pi to pi
        while angle_diff > math.pi:
            angle_diff -= 2*math.pi
        while angle_diff < math.pi:
            angle_diff += 2*math.pi

        # Calculate distance to target for speed control
        distance = math.sqrt(dx**2 + dy**2)# might not work, change to sqrt

        # PID-like control for steering
        max_turn_rate = self.carrot_max_turn_rate

        """----------------WHERE DOES THIS 1.5 COME FROM?? ---------------------------------------"""
        turn_command = max(-max_turn_rate, min(max_turn_rate, angle_diff)) 

        # Reduce turn penalty to maintain speed through curves
        turn_penalty = abs(turn_command) * self.carrot_turn_pen_coeff

        # Velocity maintenance bonus - less aggresive slowdown
        current_velocity = self.speed
        
        velocity_bonus = min(self.carrot_max_velo_bonus, current_velocity * self.carrot_velo_bonus_coeff)
        target_speed = max(self.carrot_min_target_speed, self.carrot_base_speed + velocity_bonus - turn_penalty)

        # Only slow down significantly when very close to the FINAL destination
        if (distance < self.carrot_arrival_threshold):
            target_speed *= self.carrot_slowdown_coeff

        # Calculate differential drive commands
        command_coeff = self.carrot_command_coeff
        left_command = target_speed - turn_command *command_coeff
        right_command = target_speed + turn_command *command_coeff

        return left_command, right_command

    def _carrot_update_following(self, dt):
        """Update carrot-stick path following"""
        # Update current target based on position
        self._carrot_update_index()

        if not self.carrot_following:
            print("[Carrot] Finsished following")
            return

        # Find carrot point
        carrot_point = self._carrot_find_point()

        if carrot_point:
            # Calculate sterring commands
            left_cmd, right_cmd = self._carrot_calc_steering_command(carrot_point)

            # Apply commands
            self._apply_wheel_commands(left_cmd, right_cmd, dt)
        else:
            # No valid carrot point, stop
            self.stop_path_following()
    """==========================================================================="""

    """=====================CROSS-TRACK FOLLOWING FUNCTIONS======================="""
    def cross_start_following(self, path_points):
        """Start following a path using cross-track error method"""
        if len(path_points) < 2:
            print("[Cross] Not enough points to follow")
            return False
        
        self.cross_path_points = path_points[:]
        self.cross_index = 0
        self.cross_following = True
        self.carrot_following = False # Disable carrot following
        
        print(f"[Cross] Started cross-track following with {len(path_points)} points")
        return True

    def _cross_closest_point(self):
        """Find the closest point on the path to the car"""
        if not self.cross_path_points:
            print("[Cross] No points")
            return None, 0
        
        car_pos = self.get_pos()
        min_dist = float('inf')
        closest_point = None
        closest_index = 0

        # Start searching from current index to avoid going backwards
        """WHY MINUS 2???---------------------------------------------------------"""
        start_index = max(0, self.cross_index -2)

        for i in range(start_index, len(self.cross_path_points)):
            point = self.cross_path_points[i]
            distance = math.sqrt((point[0] - car_pos[0])**2 + (point[1] - car_pos[1])**2)
            
            if distance < min_dist:
                min_dist = distance
                closest_point = point
                closest_index = i

        return closest_point, closest_index

    def _cross_closest_lookahead(self):
        """Find a point ahead on the path for steering"""
        if not self.cross_path_points:
            print("[Cross] No points")
            return None
        
        car_pos = self.get_pos()

        # Start from current index and look ahead
        for i in range(self.cross_index, len(self.cross_path_points)):
            point = self.cross_path_points[i]
            dist = math.sqrt((point[0] - car_pos[0])**2 + (point[1] - car_pos[1])**2)

            # If this point is at the desired lookahead distance or beyond
            if dist >= self.cross_base_distance:
                return point
            
        # If no point is far enough, return the last point
        return self.cross_path_points[-1]

    def _cross_calc_error(self, closest_point):
        """Calculate the cross-track error (perpendicular distance from path)"""
        if not closest_point or self.cross_index >= len(self.cross_path_points)-1:
            print("[Cross] Error could not be calculated")
            return 0
        
        # Get the path segment
        index = self.cross_index
        current_point = self.cross_path_points[index]
        next_point = self.cross_path_points[min(index+1, len(self.cross_path_points)-1)]

        # Calculate cte using perpendicular distance to the line segment
        # vector from current to next point on path
        path_dx = next_point[0] - current_point[0]
        path_dy = next_point[1] - current_point[1]

        # Vector from current path point to car
        car_dx = self.x - current_point[0]
        car_dy = self.y - current_point[1]

        # Calculate cross product to get signed distance
        if abs(path_dx) < 1e-6 and abs(path_dy) < 1e-6:
            return 0 # Path segment too short
        
        # Normalize path vector
        path_length = math.sqrt(path_dx**2 + path_dy**2)
        path_dx /= path_length
        path_dy /= path_length

        # Cross-track error is the perpendicular component
        cte = car_dx*(-path_dy) + car_dy * path_dx

        return cte

    def _cross_calc_steering(self, lookahead_point, cte):
        """Calculate steering command using cte and lookaheadpoint"""
        if not lookahead_point:
            print("[Cross] No lookahead point for steering command")
            return 0,0
        
        # PID gains for cte correction
        kp = 0.0

        # Calculate angle to lookahead point
        dx = lookahead_point[0] - self.x
        dy = lookahead_point[1] - self.y
        target_angle = math.atan2(-dy, dx)

        # Calculate angle difference
        angle_diff = target_angle - self.angle
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # Combine heading error and cte
        
        max_turn_rate = self.cross_max_turn_rate
        heading_command = max(-max_turn_rate, min(max_turn_rate, angle_diff * self.cross_heading_scale))
        cross_track_command = -cte * kp # Negative to correct towards path
        
        
        total_turn_command = heading_command + cross_track_command
        total_turn_command = max(-max_turn_rate, min(max_turn_rate, total_turn_command))
        
        # Speed control
        base_speed = self.cross_base_speed
        turn_penalty = abs(total_turn_command) * self.cross_turn_pen_coeff
        target_speed = max(self.cross_min_target_speed, base_speed - turn_penalty)

        # Calculate differential drive commands
        left_command = target_speed - total_turn_command * self.cross_command_coeff
        right_command = target_speed + total_turn_command * self.cross_command_coeff

        return left_command, right_command

    def _cross_update_following(self, dt):
            """Update cross-track path following"""
            if not self.cross_path_points:
                print("[Cross] No path points")
                return
            
            # Find closest point on path
            closest_point, closest_index = self._cross_closest_point()

            if closest_point is None:
                self.stop_path_following()
                print("[Cross] No closest point found")
                return
            
            # Update current index
            self.cross_index = max(self.cross_index, closest_index)

            # Check if reached end
            if self.cross_index >= len(self.cross_path_points) -1 :
                distance_to_end = math.sqrt(
                (self.x - self.cross_path_points[-1][0])**2 + 
                (self.y - self.cross_path_points[-1][1])**2
            )
                if distance_to_end < self.cross_arrival_threshold:
                    print("[Cross] Reached destination! Cross-track error following complete")
                    self.stop_path_following()
                    return
            
            # Find lookahead point 
            lookahead_point = self._cross_closest_lookahead()

            # Draw lookahead point
            pygame.draw.circle(pygame.display.get_surface(), RED, lookahead_point,5)
            pygame.display.flip()

            if lookahead_point:
                # Calculate cross-track error
                cte = self._cross_calc_error(closest_point)

                # Calculate steering comands using cte
                left_cmd, right_cmd = self._cross_calc_steering(lookahead_point, cte)

                # Apply commands
                self._apply_wheel_commands(left_cmd, right_cmd, dt)
    """==========================================================================="""
    
    """=====================GENERAL PATH FOLLOWING FUNCTIONS======================"""
    def _apply_wheel_commands(self, left_cmd, right_cmd, dt):
        """Apply wheel speed commands smoothly for any path following method"""
        # Commands are already in the correct units (pixels/s equivalent)
        # Just scale them to reasonable speeds and apply limits
        max_cmd = self.max_wheel_speed
        left_target = left_cmd  # Scale factor to convert from path command to pixels/s
        right_target = right_cmd  # Scale factor to convert from path command to pixels/s
        
        # Limit target values
        left_target = max(-max_cmd, min(max_cmd, left_target))
        right_target = max(-max_cmd, min(max_cmd, right_target))

        # Smooth transition to target speeds
        speed_change_rate = self.wheel_acceleration * dt

        # Left wheel
        if abs(left_target - self.wheel_L_speed) <= speed_change_rate:
            self.wheel_L_speed = left_target
        elif left_target > self.wheel_L_speed:
            self.wheel_L_speed += speed_change_rate
        else:
            self.wheel_L_speed -= speed_change_rate

        # Right wheel
        if abs(right_target - self.wheel_R_speed) <= speed_change_rate:
            self.wheel_R_speed = right_target
        elif right_target > self.wheel_R_speed:
            self.wheel_R_speed += speed_change_rate
        else:
            self.wheel_R_speed -= speed_change_rate

    def update_path_following(self, dt):
        if self.carrot_following:
            self._carrot_update_following(dt)
        elif self.cross_following:
            self._cross_update_following(dt)

    def stop_path_following(self):
        """Stop path following (any methods)"""
        if self.carrot_following:
            print("[Carrot] Stopped following")
        elif self.cross_following:
            print("[Cross] Stopped following")

        self.carrot_following = False
        self.cross_following = False
        self.carrot_path_points = []
        self.cross_path_points = []
        self.carrot_index = 0
        self.cross_index = 0
    """==========================================================================="""