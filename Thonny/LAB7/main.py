from machine import Pin, UART
from time import sleep
import heapq
import math

#startup sequence
led_board = Pin(2, Pin.OUT)  # onboard LED
print("Starting in 5 seconds... Close Thonny and start Webots.")
for i in range(5):
    led_board.value(1)  # LED ON
    sleep(0.5)
    led_board.value(0)  # LED OFF
    sleep(0.5)
led_board.value(0)  # Ensure LED is OFF at the end

obstacle_detected = False

# Now switch to UART1 for communication with Webots
uart = UART(1, 115200, tx=1, rx=3)



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
    """
    Implements Dijkstra's Algorithm for a grid-based environment.
    Args:
    - grid (2D array): The environment grid.
    - costs (2D array): The cost of moving through each cell.
    - start (tuple): The start node (row, col).
    - goal (tuple): The goal node (row, col).

    Returns:
    - path (list of tuples): The shortest path from start to goal.
    """
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

    # Reconstruct path from the goal to the start
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

costs = create_costs()
grid = create_grid()
start = (0, 0)
goal = (0, 2)
path = dijkstra(grid, costs, start, goal)
obstacle_pos = None # This should be set to the position of the obstacle when detected

def pathfinder(obstacle_pos, costs, start, goal):
    if obstacle_pos is not None:
        y, x = obstacle_pos
        if 0 <= y < len(costs) and 0 <= x < len(costs[0]):
            costs[y][x] = 999
    return dijkstra(grid, costs, start, goal)

# Initial status of the line sensor: updated by Webots via serial
line_left = False
line_center = False
line_right = False

# Variables to implement the line-following state machine
current_state = 'forward'  # Initial state
counter = 0
COUNTER_MAX = 5
COUNTER_STOP = 50
state_updated = True

x = 0.0  # Robot's x position
y = 0.0  # Robot's y position
phi = 0.0  # Robot's orientation in radians

R = 0.020 # Wheel radius
D = 0.057 # Wheel separation distance
A = 0.05 # Distance from the center of the wheels to the point of interest

delta_t = 0.02  # Time step for odometry calculations

encoder = []
encoderNames = ['left wheel sensor', 'right wheel sensor']
for i in range(2):
    encoder.append(Pin(i))
oldEncoderValues = [0, 0]  # Initialize old encoder values

# Odometry functions (From Lab3ctrl)
def get_wheels_speed(encoderValues, oldEncoderValues, delta_t):
    """Computes speed of the wheels based on encoder readings"""
    wl = (encoderValues[0] - oldEncoderValues[0]) / delta_t  # left wheel speed [rad/s]
    wr = (encoderValues[1] - oldEncoderValues[1]) / delta_t  # right wheel speed [rad/s]
    return [wl, wr]

def get_robot_speeds(wl, wr, r, d):
    """Computes robot linear and angular speeds"""
    u = (r / 2) * (wl + wr)  # linear speed [m/s]
    w = (r / d) * (wr - wl)  # angular speed [rad/s]
    return [u, w]

def get_robot_pose(u, w, x_old, y_old, phi_old, delta_t):
    """Updates robot pose based on heading and linear and angular speeds"""
    x = x_old + u * math.cos(phi_old) * delta_t
    y = y_old + u * math.sin(phi_old) * delta_t
    phi = phi_old + w * delta_t
    return [x, y, phi]

#path following variables
current_path_index = 0 # Index of the current point in the path
path = pathfinder(obstacle_pos, costs, start, goal)  # Initial path calculation
turning = False

while True:
    # Check if anything was received via serial to update sensor status
    if uart.any():
        try:
            msg_line = uart.readline()  # safer: read one line
            msg_str = msg_line.decode('utf-8').strip()

            if len(msg_str) == 4 and all(c in '01' for c in msg_str[:3]) and msg_str[3] in 'ON':
                line_left   = msg_str[0] == '1'
                line_center = msg_str[1] == '1'
                line_right  = msg_str[2] == '1'
                obstacle_detected = msg_str[3] == 'O'
        except Exception as e:
            # Blink LED rapidly 3 times to indicate UART error
            for _ in range(3):
                led_board.value(1)
                sleep(0.1)
                led_board.value(0)
                sleep(0.1)
    ##################   Think   ###################
    encoderValues = [enc.getValue() for enc in encoder]
    [wl, wr] = get_wheels_speed(encoderValues, oldEncoderValues, delta_t)
    [u, w] = get_robot_speeds(wl, wr, R, D)
    [x, y, phi] = get_robot_pose(u, w, x, y, phi, delta_t)

    oldEncoderValues = encoderValues.copy()

    # Convert robot's position to grid coordinates
    current_grid_x = int(x / 0.1)  # Assuming each grid cell is 0.1 meters wide
    current_grid_y = int(y / 0.1)  # Assuming each grid cell is 0.1 meters high
    current_grid_pos = (current_grid_y, current_grid_x)
    if obstacle_detected:
        path = pathfinder(obstacle_pos, costs, start, goal)
    
    # Line Following Logic (Only active when not turning)
    if current_state == 'forward':
        if line_right and not line_left:
            current_state = 'turn_right'
            state_updated = True
        elif line_left and not line_right:
            current_state = 'turn_left'
            state_updated = True
        elif line_left and line_right and line_center:
            current_state = 'turn_left'
            state_updated = True

    elif current_state in ['turn_right', 'turn_left']:
        if counter >= COUNTER_MAX:
            current_state = 'forward'
            state_updated = True
            current_path_index += 1  # Move to the next point in the path

    elif current_state == 'stop':
        if counter >= COUNTER_STOP:
            current_state = 'forward'
            state_updated = True

    # Send the new state when updated
    if state_updated:
        uart.write(current_state + '\n')
        state_updated = False

    counter += 1    # increment counter
    sleep(0.09)     # wait 
   

