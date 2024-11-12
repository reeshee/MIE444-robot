import heapq
#from PFLocalization import
from test_pf import eposition, eorientation

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# A* Search Algorithm
def a_star_search(maze, start, goal):
    rows, cols = len(maze), len(maze[0]) # iterate through all possible direction nodes
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Define the directions of the nodes to consider
    open_set = [] # set empty array open_set to store location of these nodes
    heapq.heappush(open_set, (0, start)) # defines the first node as the start position form the localization
    path_history = {} # stores the previous steps
    # Next g_score and f_score are defined which are crucial for the A* search algorithm to work successfully
    # algorithm uses g_score to see if a node is lower cost compared to the saved, if it is, g_score is reupdated
    # f_score is used to determine the priority of the nodes in the open set, the lower
    # the f_score, the higher the priority of the node in the open set
    g_score = {start: 0} # defines the initial point 0 as the start point 
    f_score = {start: heuristic(start, goal)} # represents estimated total cost if a specific node is chosen
    #max_iterations = 1000
    iteration_count = 0

    while open_set: # and iteration_count < max_iterations:
        iteration_count += 1
        current_cost, current = heapq.heappop(open_set)

        #print(f"Iteration: {iteration_count}, Current node: {current}, Cost: {current_cost}")

        if current == goal:
            path = []
            while current in path_history:
                path.append(current)
                current = path_history[current]
            path.reverse()
            #print(f"Path found: {path}")
            return (path)

        for dx, dy in directions:
            neighbor = (current[0] + dx, current[1] + dy)
            #print(f"Evaluating neighbor: {neighbor}")

            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and maze[neighbor[0]][neighbor[1]] != 0:
                if not is_valid_position_for_rover(maze, neighbor): 
                    #print(f"Neighbor {neighbor} is not valid due to clearance.")
                    continue

                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    path_history[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
                    #print(f"Neighbor {neighbor} added to open set with f_score: {f_score[neighbor]}, heuristic: {heuristic(neighbor, goal)}")
    print (iteration_count)
    print("No path found.")
    return float('inf')


# Helper function to determine if the position is valid for the rover given its size and required clearance
def is_valid_position_for_rover(maze, position):
    rows, cols = len(maze), len(maze[0])
    cx, cy = position

    # Iterate over a square area around the position to ensure no walls are within 5 inches
    for dx in range(-5, 5):
        for dy in range(-5, 5):
            nx, ny = cx + dx, cy + dy
            # Check if the position is out of bounds
            if not (0 <= nx < rows and 0 <= ny < cols):
                continue  # Out of bounds cells are ignored
            # Check if the position is too close to a wall
            if maze[nx][ny] == 0:
                return False  # Position is too close to a wall or obstacle
    
    return True

def navigate_to(maze, target_loading_zone, ser, max_retries=3, current_retry=0, orientation=0):
    if current_retry >= max_retries:
        print("Maximum retries reached. Navigation aborted.")
        return current_position  # Abort navigation after too many retries
    current_position, lidar_angle = eposition, eorientation ### localization function
    adjust_rover_orientation(lidar_angle,ser) # sets orientation to 0 with respect to East
    # Use A* to find the best path to the target loading zone
    path_points = a_star_search(maze, current_position, target_loading_zone)

    if target_loading_zone is None:
        print("No valid loading zone to navigate to.")
        return current_position  # No valid path found, return current position

    print(f"Navigating from {current_position} to Target Zone {target_loading_zone} with path points: {path_points}")

    # Iterate through each path point using the helper function
    for point in path_points:
        current_position, orientation = move_to_waypoint_with_localization(current_position, point, maze, ser, orientation)

        # Check if the current position is False, indicating an out-of-tolerance situation
        if current_position is False:
            # Out of tolerance boundary, recalculating path
            print("Out of tolerance boundary, recalculating path.")
            adjust_rover_orientation(orientation,ser)
            orientation = 0
            path_points = a_star_search(maze, current_position, target_loading_zone)

    return current_position



def move_to_waypoint_with_localization(current_position, waypoint, maze, ser, orientation):
    import time
    import math
    ############
    tolerance = 1  # Tolerance in units (e.g., inches)

    # Calculate the direction vector to the waypoint
    dx = waypoint[0] - current_position[0]
    dy = waypoint[1] - current_position[1]

    # Calculate the angle to the waypoint from the current position (in degrees)
    target_angle = math.degrees(math.atan2(dy, dx)) % 360

    # Calculate the minimal angle to turn
    angle_difference = (target_angle - orientation + 360) % 360
    if angle_difference > 180:
        angle_difference -= 360  # Choose the shortest rotation

    # Determine turning direction and execute turn
    if angle_difference > 0:
        ser.write(b'rotate_right\n')
        print(f"Command: rotate_right by {angle_difference} degrees")
        # Update orientation
        orientation = (orientation + angle_difference) % 360
    elif angle_difference < 0:
        ser.write(b'rotate_left\n')
        print(f"Command: rotate_left by {-angle_difference} degrees")
        # Update orientation
        orientation = (orientation + angle_difference) % 360
    else:
        print("No rotation needed.")

    # Wait for rotation to complete (adjust time as needed)
    time.sleep(0.4)

    # Move forward towards the waypoint
    ser.write(b'move_forward\n')
    print("Command: move_forward")

    # Wait for movement to complete (adjust time as needed)
    time.sleep(0.2)

    # Simulate the rover moving to the waypoint
    localized_position, orientation = eposition, eorientation   ######## 

    # Check if within tolerance
    error_x = abs(localized_position[0] - waypoint[0])
    error_y = abs(localized_position[1] - waypoint[1])

    if error_x <= tolerance and error_y <= tolerance:
        print(f"Rover reached the waypoint {waypoint} within tolerance.")
        current_position = localized_position  # Update position
    else:
        print(f"Rover did not reach the waypoint {waypoint} within tolerance.")
        current_position = False

    return current_position, orientation

def adjust_rover_orientation(lidar_angle, ser):
    import time

    # Round the LIDAR angle to the nearest integer
    lidar_angle_int = int(round(lidar_angle))

    # Convert LIDAR angle (north=0°) to rover coordinate system (east=0°)
    # Mapping: code_angle = (450 - lidar_angle) % 360
    code_angle = (lidar_angle_int) % 360

    # Desired orientation is 0 degrees (east)
    desired_orientation = 0

    # Calculate minimal angle difference to get to desired orientation
    angle_to_rotate = code_angle


    # Determine rotation direction and prepare command
    if angle_to_rotate > desired_orientation:
        command = f'adjust{abs(int(angle_to_rotate))}\n'
        ser.write(command.encode())
        print(f"Sending command: {command.strip()}")
    else:
        print("No rotation needed.")
        # Optionally send a command or do nothing

    # Wait for rotation to complete (adjust time as needed based on your rover's rotation speed)
    time.sleep(1.2)

    return angle_to_rotate  # Return the angle for confirmation or further processing

# Actual maze configuration as provided earlier
walls = [
    [3, 3, 1, 1, 0, 2, 0, 2],
    [3, 3, 0, 1, 1, 1, 1, 1],
    [1, 0, 2, 0, 0, 1, 0, 1],
    [1, 1, 1, 1, 1, 1, 0, 2]
]

# Adjusting wall matrix to represent cells of 1-inch each
# Each 12-inch cell becomes a 12x12 grid of 1-inch cells
expanded_maze = []
for row in walls:
    expanded_row = []
    for cell in row:
        expanded_row.extend([cell] * 12)
    for _ in range(12):
        expanded_maze.append(expanded_row)

# Convert expanded wall matrix to a traversable maze representation (1: traversable, 0: wall)
maze = [[1 if cell != 0 else 0 for cell in row] for row in expanded_maze]

loading_zones = [
    (8, 66),  # Loading bay 1 is at the 5th square, row 1
    (8,90),  # Loading bay 2 is at 7th square, row 1
    (32, 30), # Loading bay 3 is at 3rd square, row 3
    (40, 90)  # Loading bay 4 is at 7th square row 4
]
target_loading_zone = 2
position = loading_zones [target_loading_zone]
#print(path_points)x

import serial
import time

# Initialize serial communication
ser = serial.Serial('COM7', 9600, timeout=1)  # Replace 'COM3' with your actual port
time.sleep(2)  # Wait for the connection to initialize

loading_zones = [
    (8, 66),  # Loading bay 1 is at the 5th square, row 1
    (8,90),  # Loading bay 2 is at 7th square, row 1
    (32, 30), # Loading bay 3 is at 3rd square, row 3
    (40, 90)  # Loading bay 4 is at 7th square row 4
]
final_position = navigate_to(loading_zones, maze, loading_zones[target_loading_zone], ser, orientation=0)
ser.close()

print(f"Final Position: {final_position}")