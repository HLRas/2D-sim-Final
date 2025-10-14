import pygame
import sys
import math
import os
import socket
import threading
import time
import serial
import csv
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
jetbot_tcp = None
last_coord_time = 0
coord_lock = threading.Lock()
receiver_thread = None
request_pos = True
closedLoop = True
closedLoop_delay = 1

# --- Arduino Serial Communication for wheel speeds ---
arduino_serial = None
wheel_speed_queue = []
arduino_lock = threading.Lock()
arduino_comm_thread = None

# Starting time of auto-follow
start_time_follow = 0

# Set to true each time a new coord is sent
restarted = True
gotFirstCoord = False

# Set to true if vehicle should stop
stop = False

# --- Position tracking for CSV output ---
car_positions = []  # Array to store positions during path following
path_following_started = False

def save_positions_to_csv():
    """Save car positions to CSV file"""
    if not car_positions:
        print("[CSV] No positions recorded")
        return
    
    # Create output directory if it doesn't exist
    output_dir = "output"
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # Find the next available output number
    x = 1
    while os.path.exists(os.path.join(output_dir, f"output{x}.csv")):
        x += 1
    
    filename = os.path.join(output_dir, f"output{x}.csv")
    
    try:
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            # Write header
            writer.writerow(['x', 'y', 'orientation'])
            # Write data (convert orientation from radians to degrees)
            for position in car_positions:
                x_pos, y_pos, orientation_rad = position
                orientation_deg = math.degrees(orientation_rad)
                writer.writerow([x_pos, y_pos, orientation_deg])
        
        print(f"[CSV] Saved {len(car_positions)} positions to {filename}")
    except Exception as e:
        print(f"[CSV] Error saving positions: {e}")

def connect_arduino():
    """Connect arduino to jetbot via serial"""
    global arduino_serial
    try:
        arduino_serial = serial.Serial('/dev/ttyACM0', 115200, timeout=0)
        time.sleep(2) # Wait for the Arduino to reset after connection
        print("[Arduino] Connected to /dev/ttyACM0")
        return True
    except Exception as e:
        print(f"[Arduino] Connection error: {e}, retrying...")
        return False

def arduino_thread():
    """Thread to handle Arduino communication"""
    global wheel_speed_queue, restarted
    sendLeft = True
    lastSentLeft = 0
    sendRight = True
    lastSentRight = 0
    changeThres = 0.001 # By how much should a speed change for a new one to be sent
    while True:
        if restarted:
            restarted = False
            wheel_speed_queue.clear()
            # remember to add new time for closed loop

        if start_time_follow != 0:
            try:
                if arduino_serial:
                    try:
                        if arduino_serial.in_waiting > 0:
                            incoming_data = arduino_serial.readline().decode('utf-8').strip()
                            if incoming_data:
                                print(f"[Python] Received: {incoming_data} from Arduino")
                    except Exception as e:
                        print(f"[Python] Read error: {e}")
                    
                    new_t = time.time()
                    dt = new_t - start_time_follow
                    
                    time.sleep(0.001)
                    if not stop:
                        left, right, timestamp = find_closest(wheel_speed_queue, dt)

                        # Check if reached threshold
                        if abs(lastSentLeft-left) > changeThres:
                            sendLeft = True
                        else: sendLeft = False
                        if abs(lastSentRight-right) > changeThres: 
                            sendRight = True
                        else: sendRight = False

                    else:
                        sendLeft = True
                        sendRight = True
                        left, right = (0.0, 0.0)
                    
                    # Only send if speed changed by enough, otherwise keep speed
                    if sendLeft or sendRight:
                        with arduino_lock:
                            try:
                                msg = f"{left:.3f},{right:.3f}\n"
                                arduino_serial.write(msg.encode('utf-8'))
                                arduino_serial.flush()
                                print(f"[Python] Sent: {msg} of relative time {timestamp} to Arduino")
                                lastSentLeft, lastSentRight = left, right
                            except Exception as e:
                                print(f"[Python] Write error: {e}")
                    else:
                        print(f"[Python] Not sending ({left},{right})")

                time.sleep(0.01)
                
            except Exception as e:
                #print(f"[Python] Failed: {e}")
                pass
            
def connect_tcp():
    global jetbot_tcp
    jetbot_tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        jetbot_tcp.connect((socket.gethostname(), 1234))
        print("[Jetson] Connected, waiting 5 seconds")
        time.sleep(5)
        print("[Jetson] Now receiving first coordinate...")
        return True
    except Exception as e:
        print("[Jetson] Failed to connect, retrying...")
        return False

def tcp_receiver_thread():
    global received_coords, request_pos, gotFirstCoord
    message_buffer = ""
    
    while True:
        if request_pos: # If a new coordinate has been requested
            try:
                # Receive data in chunks and build complete messages
                data = jetbot_tcp.recv(1024).decode("utf-8")
                if data:
                    message_buffer += data
                    
                    # Process all complete messages in buffer
                    while '\n' in message_buffer:
                        # Extract one complete message (up to newline)
                        message, message_buffer = message_buffer.split('\n', 1)
                        message = message.strip()
                        
                        if message:  # Process non-empty messages
                            try:
                                print(f"[Jetson] Raw message: '{message}'")
                                parts = message.split(",")
                                if len(parts) == 3:
                                    x_, y_, or_ = map(float, parts)
                                    with coord_lock:
                                        received_coords = (x_, y_, or_)
                                    print(f"[Jetson] Received coordinate: {x_:.3f}, {y_:.3f}, Orientation: {or_:.6f}")
                                    if not gotFirstCoord:
                                        gotFirstCoord = True # if the first coordinate had been found
                                    request_pos = False # drop flag for requesting position
                                    break  # Process only one message per request
                                else:
                                    print(f"[Jetson] Invalid message format: expected 3 parts, got {len(parts)}")
                            except Exception as e:
                                print(f"[Jetson] Error parsing message: '{message}' ({e})")
                else:
                    print("[Jetson] No data received")
                    time.sleep(0.1)  # Brief pause if no data
            except Exception as e:
                print(f"[Jetson] Socket error: {e}")
                time.sleep(1)  # Pause on socket error

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
    #print(f"[Debug] Attemping to add {left_speed}, {right_speed} at {time_since_pathfollow}")
    if start_time_follow != 0:
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
            #start_time_follow = time.time()
            if game_map.start:
                nearest_space = game_map._find_nearest_parking_space(car)
                if nearest_space and nearest_space.target_cube:
                    game_map.end = nearest_space.target_cube
                    game_map._update_neighbors_if_needed()
                    game_map.pathfinder.clear_path(game_map.cubes,game_map.mark_dirty)
                    path_found = game_map.pathfinder.pathfind(game_map.cubes, game_map.start, game_map.end, game_map.mark_dirty)

                    if path_found:
                        #start_time_follow = time.time()
                        if PATHFOLLOW_METHOD == 0: # Cross track
                            car.cross_start_following(game_map.pathfinder.get_smooth_points())
                            print(f"[DEBUG] {frame_count}: Auto-started cross-track pathfinding to parking space")
                        else: # Default to carrot
                            car.carrot_start_following(game_map.pathfinder.get_smooth_points())
                            print(f"[DEBUG] {frame_count}: Auto-started carrot pathfinding to parking space")
                    else:
                        print(f"[DEBUG] Frame {frame_count}: Pathfinding failed!")
    
    return auto_pathfinding_started

def run_simulation(layout_type):
    """Run the simulation with the speified layout type"""
    global arduino_comm_thread, receiver_thread, start_time_follow

    game_map = Map(layout_type=layout_type)
    car = Car(78,303)

    #Set up display
    layout_names = ["Default Layout", "Empty Layout", "Minimal Layout"]
    caption = f"2D Car Simulation - {layout_names[layout_type]}"
    pygame.display.set_caption(caption)
    clock = pygame.time.Clock()

    # Setup up connections
    if HEADLESS_MODE:
        # keep on trying until it connects
        while not connect_tcp():
            pass
        receiver_thread = threading.Thread(target=tcp_receiver_thread, daemon=True)
        if ENABLE_ARDUINO:
            # keep on trying until it connects
            while not connect_arduino():
                pass
            arduino_comm_thread = threading.Thread(target=arduino_thread, daemon=True)
        else:
            print("[Arduino] Communication disabled - running simulation only")
        print("[DEBUG] Starting headless simulation...")
        print("[DEBUG] Waiting for TCP coordinate before starting pathfinding...")
    elif AUTOPATH_FOLLOW:
        print("[DEBUG] Will automatically set start position and begin pathfinding...")

    if HEADLESS_MODE: # Start the chosen threads
        receiver_thread.start()
        if ENABLE_ARDUINO:
            arduino_comm_thread.start()

        while not gotFirstCoord: print("[DEBUG] Still waiting for first coord") # wait for the first coord

    #start_time_follow = time.time()

    if HEADLESS_MODE and gotFirstCoord:  # only start if the first coordinate has been found in headless mode
        run(clock, car, game_map, caption)
    elif not HEADLESS_MODE: # Start instantly if not in headless mode
        run(clock, car, game_map, caption)

def run(clock, car, game_map, caption):
    global received_coords, last_coord_time, receiver_thread, arduino_comm_thread, stop, start_time_follow, request_pos

    # Performance tracking
    frame_count = 0

    # For coordinate update timing
    coordinate_processed = False

    now = time.time()
    prev = 0
    closedLoop_now = time.time()
    closedLoop_prev = 0
    path_following_started = False
    while True:
        now = time.time()

        if HEADLESS_MODE:
            dt = 1/FPS
            pygame.event.pump()
        else:
            dt = clock.tick(FPS) / 1000.0 # Delta time in seconds

        if HEADLESS_MODE and now - prev < dt:
            continue
        else:
            prev = now

        #Closed loop handling
        closedLoop_now = time.time()
        if HEADLESS_MODE and closedLoop and not request_pos and closedLoop_now - closedLoop_prev > closedLoop_delay:
            request_pos = True
            closedLoop_prev = closedLoop_now
            print(f"[DEBUG] Requesting new pos at {closedLoop_now-start_time_follow}")
        elif closedLoop and frame_count % 120:
            print(f"[DEBUG] Waiting for closed loop {request_pos} {closedLoop_now - closedLoop_prev}")
            
        # ---
        frame_count += 1
        # Immediately get the wheel speeds and queue them
        if start_time_follow != 0:
            speeds = car.get_speeds()
            queue_wheel_speeds(speeds[0], speeds[1], time.time()-start_time_follow)
        
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
                print(f"[DEBUG] Car center after position update: {car_center}")
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
                        start_time_follow = time.time()
                        path_following_started = True  # Start position tracking
                        car_positions.clear()  # Clear any previous positions
                        if PATHFOLLOW_METHOD == 0: # Cross Track
                            car.cross_start_following(game_map.pathfinder.get_smooth_points())
                            print(f"[Cross] Auto-started cross-track pathfinding to parking space")
                        else: # Default to carrot
                            car.carrot_start_following(game_map.pathfinder.get_smooth_points())
                            print(f"[Carrot] Auto started carrot pathfinding to parking space")
                        print(f"[CSV] Started position tracking")
                    else:
                        print("[DEBUG] Pathfinding failed!")
                else:
                    print("[DEBUG] No available parking space found!")

# hallo????
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
        
        # Record position during path following (headless mode only)
        if HEADLESS_MODE and path_following_started and (car.carrot_following or car.cross_following):
            car_positions.append([car.x, car.y, car.angle])
            if len(car_positions) % 60 == 0:  # Debug every 60 frames
                print(f"[CSV] Recorded {len(car_positions)} positions")
        
        # Check if path following just stopped and save positions
        """elif HEADLESS_MODE and path_following_started and not (car.carrot_following or car.cross_following):
            print("[CSV] Path following stopped, saving positions...")
            save_positions_to_csv()
            path_following_started = False  # Reset flag
        """
        # Check parking status
        for space in game_map.parking_spaces:
            if space.is_car_in_space(car):
                if not space.occupied:
                    space.set_occupied(True, game_map.cubes)
                    stop = True # stop the car!
                    print(f"[DEBUG] SUCCESS! Car parked in space at frame {frame_count}!")
                    print(f"[DEBUG] Final car position: ({car.get_pos()})")

                    # Send zero speeds to Arduino
                    # ...

                    print("[DEBUG] Simulation completed successfully!")
                    print(wheel_speed_queue)
                    
                    # Save position data to CSV
                    if HEADLESS_MODE and car_positions:
                        save_positions_to_csv()
                    
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
    """Finds the closest data values to the specified timestamp"""
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