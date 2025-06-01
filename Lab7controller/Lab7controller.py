"""line_following_with_HIL controller."""
# This program implements Hardware-in-the-Loop simulation of the e-puck robot.
# Ground sensor data is pre-processed and transfered to an external board 
# that must decide the next state of the robot. Communication between Webots
# and the external board is implemented via Serial port.

# Tested on Webots R2023a, on Windows 11 running Python 3.10.5 64-bit
# communicating with MicroPython v1.25.0 on generic ESP32 module with ESP32


from controller import Robot
import math 
import heapq
import matplotlib.pyplot as plt

#-------------------------------------------------------
# Open serial port to communicate with the microcontroller

import serial
try:
    # Change the port parameter according to your system
    ser =  serial.Serial(port='COM7', baudrate=115200, timeout=5) 
except:
    print("Communication failed. Check the cable connections and serial settings 'port' and 'baudrate'.")
    raise
    
#-------------------------------------------------------
# Initialize variables

MAX_SPEED = 6.28
speed = 0.4 * MAX_SPEED

leftSpeed = 0.0
rightSpeed = 0.0

# create the Robot instance for the simulation.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())   # [ms]

# states
states = ['forward', 'turn_right', 'turn_left', 'stop']
current_state = 'forward'

#-------------------------------------------------------
# Initialize devices

# proximity sensors
# Ref.: https://cyberbotics.com/doc/guide/tutorial-4-more-about-controllers?tab-language=python#understand-the-e-puck-model
ps = []
psNames = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(timestep)

# ground sensors
gs = []
gsNames = ['gs0', 'gs1', 'gs2']
for i in range(3):
    gs.append(robot.getDevice(gsNames[i]))
    gs[i].enable(timestep)

# motors    
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)



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

grid = create_grid()

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

def plot_grid(grid, costs, path=None):
    plt.figure(figsize=(5, 5))
    plt.imshow(grid, cmap="Greys", origin="upper")
    rows = len(grid)
    cols = len(grid[0])
    for i in range(rows):
        for j in range(cols):
            if grid[i][j] == 0:
                plt.text(j, i, f'{costs[i][j]}', ha='center', va='center', color='lightgray', fontsize=10)
    if path:
        i = []
        j = []
        for (x, y) in path:
            plt.scatter(y, x, color="red")
            i.append(x)
            j.append(y)
        plt.plot(j, i, 'r')
    plt.title("Map of the Environment")
    plt.show()
grid = create_grid()



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
#-------------------------------------------------------
# Main loop:
# perform simulation steps until Webots is stopping the controller
# Implements the see-think-act cycle
# Example usage (outside the main loop)


costs = create_costs()
start = (0, 0)
goal = (0, 2)
path = dijkstra(grid, costs, start, goal)
plot_grid(grid, costs, path)
print("Shortest Path:", path)

while robot.step(timestep) != -1:

    ############################################
    #                  See                     #
    ############################################

    # Update sensor readings
    gsValues = []
    for i in range(3):
        gsValues.append(gs[i].getValue())

    # Process sensor data
    line_right = gsValues[0] > 600
    line_center = gsValues[1] > 600
    line_left = gsValues[2] > 600
    
    # Build the message to be sent to the ESP32 with the ground
    # sensor data: 0 = line detected; 1 = line not detected
    message = ''
    if line_left:
        message += '1'
    else:
        message += '0'
    if line_center:
        message += '1'
    else:
        message += '0'
    if line_right:
        message += '1'
    else:
        message += '0'
    msg_bytes = bytes(message + '\n', 'UTF-8')
    

    ############################################
    #                 Think                    #
    ############################################

    # Serial communication: if something is received, then update the current state
    if ser.in_waiting:
        value = str(ser.readline(), 'UTF-8')[:-1]  # ignore the last character
        current_state = value

    # Update speed according to the current state
    if current_state == 'forward':
        leftSpeed = speed
        rightSpeed = speed
            
    elif current_state == 'turn_right':
        leftSpeed = 0.5 * speed
        rightSpeed = 0 * speed

    elif current_state == 'turn_left':
        leftSpeed = 0 * speed
        rightSpeed = 0.5 * speed
        
    elif current_state == 'stop':
        leftSpeed = 0.0
        rightSpeed = 0.0
 

    ############################################
    #                  Act                     #
    ############################################

    # Update velocity commands for the motors
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
   
    # Print sensor message and current state for debugging
    print(f'Sensor message: {msg_bytes} - Current state: {current_state}')

    # Send message to the microcontroller 
    ser.write(msg_bytes)  

ser.close()
