from machine import Pin, UART
import heapq
import time
#-------------------------------------------------------
# Initialize serial communication

uart = UART(1, 115200, tx=Pin(1), rx=Pin(3))  # Adjust pins if needed

#-------------------------------------------------------
# Define grid and costs

# Add startup delay before main loop
print("Starting in 5 seconds...")
time.sleep(5)  # 5 second delay
uart.write("DEBUG:ESP32 Started\n")

SENSOR_THRESHOLD = 700  # Adjustable threshold for line detection
LINE_FOLLOW_DELAY = 50  # Control loop delay in milliseconds
TURN_DELAY = 750       # Delay at intersections in milliseconds

def create_grid():
    grid = ([
        [0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
        [0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0],
        [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0]
        ])
    return grid

grid = create_grid()  # Create the grid for the robot's environment

def create_costs():
    costs = [
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,],
        [2, 1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 2,],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,],
        [2, 1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 2,],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,]
    ]
    return costs



def dijkstra(grid, costs, start, goal):
    rows, cols = len(grid), len(grid[0])
    visited = set()
    distances = {start: 0}  # Distance to the start node is 0
    parents = {start: None}  # A parent is the node that preceeds the current one: used to reconstruct the path

    priority_queue = []  # Priority queue ensures that nodes are processed in order of increasing distance
    heapq.heappush(priority_queue, (0, start))  # Initializes queue with the start node and distance to start

    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # Right, Down, Left, Up

    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)

        #  If a node has already been visited, it means the shortest distance to that node has been determined.
        if current_node in visited:
            continue 
        visited.add(current_node)

        # If the current node is the goal node, then the path has been found.
        if current_node == goal:
            break

        # Gets the coordinates of each neighboring cell
        for direction in directions:
            neighbor = (current_node[0] + direction[0], current_node[1] + direction[1])

            # Can only move within the boundaries of the world, and if there's no obstacle
            if (0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and grid[neighbor[0]][neighbor[1]] == 0):
                distance = current_distance + costs[neighbor[0]][neighbor[1]]  # Use cost from the `costs` array

                # Updates the shortest known distance to a neighboring node and prepares it for further exploration
                if neighbor not in distances or distance < distances[neighbor]:
                    distances[neighbor] = distance  # Updates the shortest known distance to the neighbor
                    parents[neighbor] = current_node # Makes the current node be the parent of the neighbor to reconstruct the shortest path
                    heapq.heappush(priority_queue, (distance, neighbor))  # Adds the neighbor to the priority queue with its updated distance as the priority
    
    path = []
    node = goal
    while node is not None:
        if node not in parents:
            print(f"Error: Node {node} has no parent. Path reconstruction failed.")
            return []
        path.append(node)
        node = parents[node]  # Gets the parent of the node
    path.reverse()  # Reverse the path to make it from start to the goal node
    return path
    # After path calculation

def at_intersection(line_left, line_center, line_right):
    # Detect intersection types based on sensor combinations
    if line_left and line_right:
        uart.write("DEBUG:Full intersection detected\n")
        return 'full'
    elif line_left and line_center:
        uart.write("DEBUG:Left turn detected\n")
        return 'left'
    elif line_right and line_center:
        uart.write("DEBUG:Right turn detected\n")
        return 'right'
    return None

def decide_direction(current, next):
    dr = next[0] - current[0]
    dc = next[1] - current[1]
    
    # More explicit direction mapping
    if dr == 0 and dc > 0:
        return 'forward'
    elif dr > 0:
        return 'turn_right'
    elif dr < 0:
        return 'turn_left'
    elif dc < 0:
        return 'turn_left'  # Turn around - may need two left turns
    
    uart.write(f"DEBUG:Direction decision dr={dr}, dc={dc}\n")
    return 'forward'  # Default case

def send_command(cmd):
    global last_command
    if cmd != last_command:
        uart.write(f"DEBUG:Changing direction from {last_command} to {cmd}\n")
        uart.write(cmd + '\n')
        last_command = cmd
    else:
        uart.write(cmd + '\n')

def process_sensors(msg_str):
    """Process sensor data and apply thresholds"""
    if len(msg_str) >= 3:
        # Apply threshold when converting to boolean
        line_left = int(msg_str[0]) > 700
        line_center = int(msg_str[1]) > 700
        line_right = int(msg_str[2]) > 700
        uart.write("DEBUG:Sensors L:{} C:{} R:{}\n".format(
            line_left, line_center, line_right))
        return line_left, line_center, line_right
    return False, False, False

current_state = 'forward'
last_command = 'forward'
start = (0, 0)
goal = (0, 2)
robot_pos = start  # Starting position of the robot
grid = create_grid()  # Create the grid for the robot's environment
costs = create_costs()  # Create the costs for the grid
path = dijkstra(grid, costs, start, goal)  # Find the shortest path from start to goal
current_path_index = 0  # Index to track the current position in the path
print("initial path calculated:", path)

while True:
    ##################   See   ###################
    # Receive message from Webots
    if uart.any():
        try:
            msg_bytes = uart.read()
            msg_str = str(msg_bytes, 'UTF-8').strip()
            line_left, line_center, line_right = process_sensors(msg_str)
                
        except Exception as e:
            uart.write(f"DEBUG:Error processing message: {str(e)}\n")
            continue
    ##################   Think   ###################
    intersection_type = at_intersection(line_left, line_center, line_right)
    if intersection_type:
        if current_path_index < len(path) - 1:
            current = path[current_path_index]
            next_pos = path[current_path_index + 1]
            direction = decide_direction(current, next_pos)
            uart.write(f"DEBUG:At {intersection_type} intersection, taking {direction}\n")
            send_command(direction)
            current_path_index += 1
            time.sleep_ms(750)  # Brief pause at intersections
        else:
            uart.write("DEBUG:Reached end of path!\n")
            send_command('stop')
    else:
        # Enhanced line following
        if line_center:
            send_command('forward')
        elif line_right:
            send_command('turn_right')
        elif line_left:
            send_command('turn_left')
        else:
            # If no line detected, continue last command
            send_command(last_command)

    ##################   Act   ###################
    # (Webots will handle motor commands based on the state sent above)

    # Small delay
    time.sleep_ms(50)