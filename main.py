import pygame
import sys
import math
import os
import socket
import threading
import time
import serial
from car import Car
from map_generation import Map
from config import *
import bisect

# Default command-line arguments
HEADLESS_MODE = 0
AUTOPATH_FOLLOW = 0
PATHFOLLOW_METHOD = 0
ENABLE_ARDUINO = 0

# --- TCP Client for receiving coordinates from Jetson (Headless only) ---
received_coords = None
last_coord_time = 0
coord_lock = threading.Lock()
receiver_thread = None

# --- Arduino Serial Communication for wheel speeds ---
arduino_serial = None
wheel_speed_queue = []
arduino_lock = threading.Lock()
arduino_comm_thread = None

# Starting time of auto-follow
start_time_follow = time.time()
restarted = True

def arduino_thread():
    """Thread to handle Arduino communication"""
    global arduino_serial, wheel_speed_queue, restarted

    # Initialize Arduino connection to specific port
    try:
        arduino_serial = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
        time.sleep(2) # Wait for the Arduino to reset after connection
        print("[Arduino] Connected to /dev/ttyACM0")
    except Exception as e:
        print(f"[Arduino] Connection error: {e}")
        print("[Arduino] Running in simulation-only mode (no Arduino communication)")
        arduino_serial = None

    while True:
        if restarted:
            restarted = False
            start_time = time.time()

        try:
            if arduino_serial:
                try:
                    if arduino_serial.in_waiting > 0:
                        incoming_data = arduino_serial.readline().decode('utf-8').strip()
                        if incoming_data:
                            print(f"[Arduino] Received: {incoming_data}")
                except Exception as e:
                    print(f"[Python] Read error: {e}")
                
                new_t = time.time()
                dt = new_t - start_time
                
                
                left, right, timestamp = find_closest(wheel_speed_queue, dt)
                # Always remove data from queue (either send or discard)
                with arduino_lock:
                    if left:
                            try:
                                msg = f"{left},{right}\n"
                                arduino_serial.write(msg.encode('utf-8'))
                                arduino_serial.flush()
                                print(f"[Python] Sent: {msg} of relative time {timestamp}")
                            except Exception as e:
                                print(f"[Python] Write error: {e}")

            time.sleep(0.01)
            
        except Exception as e:
            #print(f"[Python] Failed: {e}")
            pass
            

def tcp_receiver_thread():
    global received_coords
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.connect((socket.gethostname(), 1234))
        print("[Jetson] Connected, waiting 5 seconds")
        time.sleep(5)
        print("[Jetson] Now receiving first coordinate")

        msg = s.recv(21) # Receive exactly one message limited to 21 characters
        if msg:
            try:
                decoded = msg.decode("utf-8").strip()
                parts = decoded.split(",")
                x_, y_, or_ = map(float, parts)
                with coord_lock:
                    received_coords = (x_, y_, or_)
                print(f"[Jetson] Received coordinate: {x_}, {y_}, Orientation: {or_}")
            except Exception as e:
                print(f"[Jetson] Error parsing message: {msg} ({e})")
        else:
            print("[Jetson] No message received")

        print("[Jetson] Closing connection after receiving first coordinate")

    except Exception as e:
        print(f"[Jetson] Socket error: {e}")
    finally:
        s.close()
    
def get_args():
    """Get arguments from command-line"""
    global HEADLESS_MODE, AUTOPATH_FOLLOW, PATHFOLLOW_METHOD, ENABLE_ARDUINO

    numargs = len(sys.argv)
    arguments = sys.argv
    
    if numargs != 5 and numargs != 1:
        print("[ERROR] Please enter the desired configuration as follows [main.py <HEADLESS=0/1> <AUTO_PATHFINDING=0/1> <PATHFOLLOW_METHOD=0/1 (cross track/carrot-stick)> <ENABLE_ARDUINO=0/1>]")
        sys.exit(1)
    elif numargs == 5:
        try:
            arguments_ = map(int, arguments[1:5])
            for argument in arguments_:
                if argument != 0 and argument != 1:
                    print(argument)
                    raise ValueError
                
            HEADLESS_MODE, AUTOPATH_FOLLOW, PATHFOLLOW_METHOD, ENABLE_ARDUINO = map(int, arguments[1:5])

        except ValueError as e:
            print(f"[Value Error]: {e}\n[ERROR] Please enter the desired configuration as follows\n[main.py <HEADLESS=0/1> <AUTO_PATHFINDING=0/1> <PATHFOLLOW_METHOD=0/1 (cross track/carrot-stick)> <ENABLE_ARDUINO=0/1>]")
            sys.exit(1)
    else:
        print("[DEBUG] Running in default mode\n" \
        "[DEBUG] Please provide arguments if needed as follows:\n" \
        "[DEBUG] main.py <HEADLESS=0/1> <AUTO_PATHFINDING=0/1> <PATHFOLLOW_METHOD=0/1 (cross track/carrot-stick)> <ENABLE_ARDUINO=0/1>]")
        return

def headless_handling(headless):
    """Change to a dummy driver if headless"""
    if headless:
        os.environ['SDL_VIDEODRIVER'] = 'dummy'

def queue_wheel_speeds(left_speed, right_speed, time_since_pathfollow):
    """Queue wheel speeds for sending to Arduino"""
    wheel_speed_queue.append((left_speed, right_speed, time_since_pathfollow))

def handle_automated_pathfinding(frame_count, game_map : Map, car : Car):
    """Handle automated pathfinding setup for headless mode"""
    global start_time_follow
    if not AUTOPATH_FOLLOW:
        return False
    
    auto_pathfinding_started = getattr(handle_automated_pathfinding, 'started', False)

    if not auto_pathfinding_started:
        if frame_count == 500: # Set start position after a few frames
            car_center = car.get_rect().center
            cube = game_map.get_cube(car_center)
            if cube:
                if game_map.start:
                    game_map.start.make_clear()
                    game_map.mark_dirty(game_map.start)

                cube.make_start()
                game_map.start = cube
                game_map.mark_dirty(cube)
                print(f"[DEBUG] Frame {frame_count}: Auto-set start position at ({car.get_pos()})")
        
        elif frame_count == 550: # Start pathfinding after start is set
            start_time_follow = time.time()
            if game_map.start:
                nearest_space = game_map._find_nearest_parking_space(car)
                if nearest_space and nearest_space.target_cube:
                    game_map.end = nearest_space.target_cube
                    game_map._update_neighbors_if_needed()
                    game_map.pathfinder.clear_path(game_map.cubes,game_map.mark_dirty)
                    path_found = game_map.pathfinder.pathfind(game_map.cubes, game_map.start, game_map.end, game_map.mark_dirty)

                    if path_found:
                        if PATHFOLLOW_METHOD == 0: # Cross track
                            car.cross_start_following(game_map.pathfinder.path_points)
                            print(f"[DEBUG] {frame_count}: Auto-started cross-track pathfinding to parking space")
                        else: # Default to carrot
                            car.carrot_start_following(game_map.pathfinder.path_points)
                            print(f"[DEBUG] {frame_count}: Auto-started carrot pathfinding to parking space")
                    else:
                        print(f"[DEBUG] Frame {frame_count}: Pathfinding failed!")
    
    return auto_pathfinding_started

def run_simulation(layout_type):
    """Run the simulation with the speified layout type"""
    game_map = Map(layout_type=layout_type)
    car = Car(50,50)

    #Set up display
    layout_names = ["Default Layout", "Empty Layout", "Minimal Layout"]
    caption = f"2D Car Simulation - {layout_names[layout_type]}"
    pygame.display.set_caption(caption)
    clock = pygame.time.Clock()

    run(clock, car, game_map, caption)

def run(clock, car, game_map, caption):
    global received_coords, last_coord_time, receiver_thread, arduino_comm_thread

    # Performance tracking
    frame_count = 0

    # Start TCP receiver thread in headless mode
    if HEADLESS_MODE and receiver_thread is None:

        receiver_thread = threading.Thread(target=tcp_receiver_thread, daemon=True)
        receiver_thread.start()

        if ENABLE_ARDUINO:
            if arduino_comm_thread is None:
                arduino_comm_thread = threading.Thread(target=arduino_thread, daemon=True)
                arduino_comm_thread.start()
        else:
            print("[Arduino] Communication disabled - running simulation only")
        print("[DEBUG] Starting headless simulation...")
        print("[DEBUG] Waiting for TCP coordinate before starting pathfinding...")
    elif AUTOPATH_FOLLOW:
        print("[DEBUG] Will automatically set start position and begin pathfinding...")

    # For coordinate update timing
    coordinate_processed = False
    
    while True:
        dt = clock.tick(FPS) / 1000.0 # Delta time in seconds
        frame_count += 1

        # Immediately get the wheel speeds and queue them
        speeds = car.get_speeds()
        queue_wheel_speeds(speeds[0], speeds[1], time.time())
        
        # --- Check for received coordinates (headless only) ---
        if HEADLESS_MODE and not coordinate_processed:
            with coord_lock:
                coords = received_coords
            if coords:
                x, y, orien = coords
                print(f"[Jetson] Setting car position to ({x:.1f}, {y:.1f}) with orientation {math.degrees(orien)}° at frame {frame_count}")
                car.set_position((x,y))
                car.set_orientation(orien)

                coordinate_processed = True

                # Execute pathfinding once after receiving coordinates
                # Set start position
                car_center = car.get_rect().center
                print("[DEBUG] Car center after position update: {car_center}")
                cube = game_map.get_cube(car_center)
                if cube:
                    if game_map.start:
                        game_map.start.make_clear()
                        game_map.mark_dirty(game_map.start)
                    cube.make_start()
                    game_map.start = cube
                    game_map.mark_dirty(cube)
                    print(f"[Jetson] Auto-set start position at ({car.x:.1f}, {car.y:.1f})")
                
                # Find nearest parking space and pathfind
                print(f"[DEBUG] Finding nearest parking space from car position")
                nearest_space = game_map._find_nearest_parking_space(car)
                if nearest_space and nearest_space.target_cube:
                    target_pos = nearest_space.get_target_position()
                    print(f"[DEBUG] Selected parking space target position: {target_pos}")
                    game_map.end = nearest_space.target_cube
                    game_map._update_neighbors_if_needed()
                    game_map.pathfinder.clear_path(game_map.cubes, game_map.mark_dirty)
                    path_found = game_map.pathfinder.pathfind(game_map.cubes, game_map.start, game_map.end, game_map.mark_dirty)
                    if path_found:
                        if PATHFOLLOW_METHOD == 0: # Cross Track
                            car.cross_start_following(game_map.pathfinder.path_points)
                            print(f"[Cross] Auto-started cross-track pathfinding to parking space")
                        else: # Default to carrot
                            car.carrot_start_following(game_map.pathfinder.path_points)
                            print(f"[Carrot] Auto started carrot pathfinding to parking space")
                    else:
                        print("[DEBUG] Pathfinding failed!")
                else:
                    print("[DEBUG] No available parking space found!")

        # Handle automated pathfinding
        if AUTOPATH_FOLLOW and not HEADLESS_MODE:
            # Normal auto-pathfinding for GUI mode
            auto_pathfinding_started = handle_automated_pathfinding(frame_count, game_map, car)
        elif AUTOPATH_FOLLOW and HEADLESS_MODE and not coordinate_processed:
            # Auto-pathfinding for headless mode (without TCP coords)
            auto_pathfinding_started = handle_automated_pathfinding(frame_count, game_map, car)

        # Print periodic status updates in headless mode
        if HEADLESS_MODE and frame_count % 500 == 0:
            status = "Carrot" if car.carrot_following else "Cross-Track" if car.cross_following else "Manual"
            print(f"[DEBUG] Frame {frame_count}: Car at ({car.get_pos()}), Mode: {status}")

        # Handle events (only in GUI mode)
        if not HEADLESS_MODE:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
                
                elif event.type == pygame.KEYDOWN:
                    # Return to main menu with ESC key
                    if event.key == pygame.K_ESCAPE:
                        return main() # Restart from menu
                    
                    game_map.check_inputs(event, car)

                    # Stop autopilot with SPACE key
                    if event.key == pygame.K_SPACE and (car.cross_following or car.carrot_following):
                        car.stop_path_following()
                        print("[DEBUG] Autopilot manually stopped")
                    
                    # Display debugging info
                    if event.key == pygame.K_F1:
                        print(f"Car position: ({car.x:.1f}, {car.y:.1f})")
                        print(f"Car angle: {math.degrees(car.angle):.1f}°")
                        print(f"Wheel speeds: L={car.wheel_L:.2f}, R={car.wheel_R:.2f}")
                        print(f"Carrot following: {car.carrot_following}")
                        print(f"Cross-track following: {car.cross_following}")

            # Handle continuous inputs
            keys = pygame.key.get_pressed()
            if any(keys):
                car.check_inputs(dt, keys)
        else:
            # In headless mode, stil need to process pygame event to prevent hanging
            pygame.event.pump()

        # Update path followig if active
        car.update_path_following(dt)

        # Update car physics
        car.find_next_pos(dt)
        
        # Check parking status
        for space in game_map.parking_spaces:
            if space.is_car_in_space(car):
                if not space.occupied:
                    space.set_occupied(True, game_map.cubes)
                    print(f"[DEBUG] SUCCESS! Car parked in space at frame {frame_count}!")
                    print(f"[DEBUG] Final car position: ({car.get_pos()})")

                    # Send zero speeds to Arduino
                    # ...

                    print("[DEBUG] Simulation completed successfully!")
                    pygame.quit()
                    return
            else:
                # Only set to unoccupied if it's not permanently occupied
                if space.occupied and not space.permanently_occupied:
                    space.set_occupied(False, game_map.cubes)
        
        if not HEADLESS_MODE:
            game_map.draw()
            car.draw()

            # Show FPS and cotrols info
            fps = clock.get_fps()
            if car.carrot_following:
                status = " - CARROT FOLLOWING"
            elif car.cross_following:
                status = " - CROSS-TRACK FOLLOWING"
            else:
                status = ""
            pygame.display.set_caption(f"{caption} - FPS: {fps:.1f}{status} - ESC: Menu")

            pygame.display.flip()               

def find_closest(data, timestamp, index=2):
    """Finds the closest data value to the specified timestamp"""
    timestamps = [tp[index] for tp in data]
    return data[bisect.bisect_left(timestamps, timestamp)]


def main():
    """Main application entry"""
    # Initialize pygame
    pygame.init()
    pygame.font.init()

    get_args()
    headless_handling(HEADLESS_MODE)

    if HEADLESS_MODE:
        # Skip the main menu and head straight to layout 0
        print("[DEBUG] Running in headless mode - skipping menu")
        print("[DEBUG] Using default layout (layout 0)")
        selected_layout = 0
    else:
        # Show the main menu and get selected layout
        from menu import MainMenu
        menu = MainMenu()
        selected_layout = menu.run()

    run_simulation(selected_layout)

if __name__ == '__main__': main()