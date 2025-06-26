#MAGGIE is a robot that has a stepper motor that moves a belt with a magnet on it so it can grab boxes with a magnetic strip
#this code needs to control said robot
#it will move on a predeterimed grid that we can measure 
#it has line sensors in the form of 5 ground sensors and 2 proximity sensons for obstacles
#it has 2 wheels and a swivel wheel that isnt powered just for balance
#it has to go the course pick up 4 boxes return through the course and unload the 4 boxes in 4 separate coordinates

def create_grid(): 
    """
    Creates and returns a 2D grid representing the environment.
    A value of 0 indicates a traversable path, and 1 indicates an obstacle.

    Returns:
        list[list[int]]: A 2D list representing the grid.
    """
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

def create_costs(): #costs example used from Lab examples its arbitrary
    """
    Creates and returns a 2D array representing the cost of traversing each cell.
    These costs are arbitrary and can be adjusted based on environmental factors.

    Returns:
        list[list[int]]: A 2D list representing the traversal costs.
    """
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
    rows, cols = len(grid), len(grid[0]) #Example code from the labs
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

def pathfinder(obstacle_pos, costs, start, goal): #calculates new cost, runs dijkstra, generates new checkpoints
    """
    Calculates a new path based on potential obstacles and generates odometry goals.

    Args:
        obstacle_pos (tuple[int, int] or None): The grid coordinates of a detected obstacle.
            If None, no obstacle is considered.
        costs (list[list[int]]): The 2D array representing the cost of traversing each cell.
        start (tuple[int, int]): The current starting coordinates (row, col) for pathfinding.
        goal (tuple[int, int]): The final goal coordinates (row, col).

    Returns:
        tuple[list[tuple[int, int]], list[tuple[float, float]]]: A tuple containing:
            - path (list[tuple[int, int]]): The calculated path in grid coordinates.
            - odom_goals (list[tuple[float, float]]): The calculated path in odometry coordinates.
    Side effects:
        Modifies the 'costs' array by setting the cost of an 'obstacle_pos' to a very high value (999).
    """
    if obstacle_pos is not None:
        x, y = obstacle_pos
        if 1 <= x < len(costs) and 1 <= y < len(costs[0]):
            costs[x][y] = 999
    path = dijkstra(grid, costs, start, goal)
    odom_goals = generate_path_goals(path) 
    return path, odom_goals

# World to grid calibration, uses the known (or measured) path and maps it to the grid points these are the relative values
# the following 2 are how many meters correspond to a grid point
CELL_WIDTH = 0.06245      # meters
CELL_HEIGHT = 0.0587    # meters           

costs = create_costs()
grid = create_grid()
start = (0, 0)
Box_1 = (16, 10)
Box_2 = (16, 12)
Box_3 = (16, 14)
Box_4 = (16, 16)

def odom_to_grid(x, y):
    """
    Translates odometry coordinates (x, y) into grid coordinates (row, col).

    Args:
        x (float): The x-coordinate from odometry.
        y (float): The y-coordinate from odometry.

    Returns:
        tuple[int, int]: The corresponding grid coordinates (row, col).
    """
    col = round((x - ORIGIN_X) / -CELL_WIDTH)
    row = round((y - ORIGIN_Y) / CELL_HEIGHT)
    return (row, col)

def grid_to_odom(row, col): 
    """
    Translates grid coordinates (row, col) into odometry coordinates (x, y).

    Args:
        row (int): The row index in the grid.
        col (int): The column index in the grid.

    Returns:
        tuple[float, float]: The corresponding odometry coordinates (x, y).
    """
    x = ORIGIN_X - (col * CELL_WIDTH)
    y = ORIGIN_Y + (row * CELL_HEIGHT)
    return (x, y)

def generate_path_goals(path):
    """
    Generates a list of odometry goals from a given path of grid coordinates.

    Args:
        path (list[tuple[int, int]]): A list of (row, col) tuples representing the grid path.

    Returns:
        list[tuple[float, float]]: A list of (x, y) tuples representing the odometry goals.
    """
    odom_goals = []
    for row, col in path:
        x, y = grid_to_odom(row, col)
        odom_goals.append((x, y))
    return odom_goals

obstacle_pos = None # This should be set to the position of the obstacle when detected
current_pos = (0, 0)  # Current position in the grid (row, col)
obstacle_detected = False
start = current_pos
path, odom_goals = [], []  

def read_proximity_sensors(ps):
    """
    Reads the current values from all enabled proximity sensors.

    Args:
        ps (list): A list of Webots ProximitySensor objects.

    Returns:
        list[float]: A list of sensor values.
    """
    return [sensor.getValue() for sensor in ps]

def check_for_obstacle(psValues, threshold):
    """
    Checks if any proximity sensor detects an obstacle above a given threshold.

    Args:
        psValues (list[float]): A list of proximity sensor values.
        threshold (float): The threshold value above which an obstacle is considered detected.

    Returns:
        bool: True if an obstacle is detected, False otherwise.
    """
    return any(val > threshold for val in psValues)

def calculate_goal_info(x, y, phi, x_goal, y_goal):
    """
    Calculates the distance and angle error to the current goal.

    Args:
        x (float): Current x-coordinate of the robot.
        y (float): Current y-coordinate of the robot.
        phi (float): Current orientation (heading) of the robot in radians.
        x_goal (float): X-coordinate of the target goal.
        y_goal (float): Y-coordinate of the target goal.

    Returns:
        tuple[float, float]: A tuple containing:
            - distance (float): Euclidean distance to the goal.
            - angle_error (float): Angular difference between robot's heading and direction to goal.
    """
    dx = x_goal - x
    dy = y_goal - y
    distance = math.hypot(dx, dy)

    angle_to_goal = math.atan2(dy, dx)
    angle_error = angle_to_goal - phi
    angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

    return distance, angle_error

def send_status(ser, obstacle_reported, x, y, phi):
    """
    Sends the robot's current status (obstacle detected, x, y, phi) over serial.

    Args:
        ser (serial.Serial): The serial port object.
        obstacle_reported (bool): True if an obstacle was reported in the current cycle, False otherwise.
        x (float): Current x-coordinate of the robot.
        y (float): Current y-coordinate of the robot.
        phi (float): Current orientation (heading) of the robot in radians.
    Side effects:
        Writes data to the serial port.
    """
    status = 'O' if obstacle_reported else 'N'
    message = f"{status},{x:.3f},{y:.3f},{phi:.3f}\n"
    print(message)
    ser.write(message.encode())
    ser.flush()

def wait_for_new_goal(robot, ser, motors, timestep):
    """
    Halts the robot and waits for a new goal message from the serial port.

    Args:
        robot (controller.Robot): The Webots Robot object.
        ser (serial.Serial): The serial port object.
        motors (dict): A dictionary containing 'left' and 'right' motor objects.
        timestep (int): The simulation timestep in milliseconds.

    Returns:
        tuple[float or None, float or None, bool]: A tuple containing:
            - x_goal (float or None): New x-coordinate of the goal, or None if data is incomplete.
            - y_goal (float or None): New y-coordinate of the goal, or None if data is incomplete.
            - failed (bool): True if data is incomplete, False otherwise.
    Side effects:
        Stops the robot motors.
        Reads data from the serial port.
    """
    stop_motors(motors)
    while ser.in_waiting == 0:
        robot.step(timestep)
        stop_motors(motors)

    data = ser.read(8)  # Expecting 2 floats
    if len(data) == 8:
        x_goal, y_goal = struct.unpack('<ff', data)
        print(f"New goal: x={x_goal:.3f}, y={y_goal:.3f}")
        return x_goal, y_goal, False  # Reset obstacle_reported
    else:
        print("Incomplete data, Stopping.")
        stop_motors(motors)
        return None, None, True  # Signal failure

def stop_motors(motors):
    """
    Sets the velocity of both left and right motors to zero, effectively stopping the robot.

    Args:
        motors (dict): A dictionary containing 'left' and 'right' motor objects.
    Side effects:
        Sets motor velocities.
    """
    motors['left'].setVelocity(0)
    motors['right'].setVelocity(0)

def read_encoders(encoders):
    """
    Reads the current values from all enabled wheel encoders.

    Args:
        encoders (list): A list of Webots PositionSensor objects for the wheels.

    Returns:
        list[float]: A list of encoder values.
    """
    return [enc.getValue() for enc in encoders]

def get_wheels_speed(encoderValues, oldEncoderValues, delta_t):
    """
    Calculates the angular speed of the left and right wheels.

    Args:
        encoderValues (list[float]): Current encoder readings for left and right wheels.
        oldEncoderValues (list[float]): Previous encoder readings for left and right wheels.
        delta_t (float): The time step (difference between current and previous reading) in seconds.

    Returns:
        list[float]: A list containing the angular speed of the left wheel (wl) and right wheel (wr) in rad/s.
    """
    wl = (encoderValues[0] - oldEncoderValues[0]) / delta_t  # left wheel speed [rad/s]
    wr = (encoderValues[1] - oldEncoderValues[1]) / delta_t    # right wheel speed [rad/s]
    return [wl, wr]

def get_robot_speeds(wl, wr, r, d):
    """
    Calculates the linear and angular speeds of the robot based on wheel speeds.

    Args:
        wl (float): Angular speed of the left wheel in rad/s.
        wr (float): Angular speed of the right wheel in rad/s.
        r (float): Wheel radius in meters.
        d (float): Distance between the wheels in meters.

    Returns:
        list[float]: A list containing the linear speed (u) in m/s and angular speed (w) in rad/s.
    """
    u = (r / 2) * (wl + wr)  # linear speed [m/s]
    w = (r / d) * (wr - wl)  # angular speed [rad/s]
    return [u, w]

def get_robot_pose(u, w, x_old, y_old, phi_old, delta_t):
    """
    Updates the robot's pose (x, y, phi) using odometry calculations.

    Args:
        u (float): Linear speed of the robot in m/s.
        w (float): Angular speed of the robot in rad/s.
        x_old (float): Previous x-coordinate of the robot.
        y_old (float): Previous y-coordinate of the robot.
        phi_old (float): Previous orientation (heading) of the robot in radians.
        delta_t (float): The time step in seconds.

    Returns:
        list[float]: A list containing the updated x, y, and phi coordinates.
    """
    x = x_old + u * math.cos(phi_old) * delta_t
    y = y_old + u * math.sin(phi_old) * delta_t
    phi = phi_old + w * delta_t
    phi = math.atan2(math.sin(phi), math.cos(phi))
    return [x, y, phi]

def build_message(Obstacle, x, y, phi):
    """
    Constructs a status message string for serial communication.

    Args:
        Obstacle (bool): True if an obstacle is detected, False otherwise.
        x (float): Current x-coordinate of the robot.
        y (float): Current y-coordinate of the robot.
        phi (float): Current orientation (heading) of the robot in radians.

    Returns:
        bytes: The encoded status message.
    """
    message = ''
    if Obstacle:
        message += 'O'
    else:
        message += 'N'
    
    message += f',{x:.3f},{y:.3f},{phi:.3f}'
    return bytes(message + '\n', 'UTF-8')

def go_to_goal(x, y, phi, x_goal, y_goal, R, D, MAX_SPEED):
    """
    Calculates the required wheel velocities to move the robot towards a goal,
    considering obstacle detection and turning behavior.

    Args:
        x (float): Current x-coordinate of the robot.
        y (float): Current y-coordinate of the robot.
        phi (float): Current orientation (heading) of the robot in radians.
        x_goal (float): X-coordinate of the target goal.
        y_goal (float): Y-coordinate of the target goal.
        D (float): Distance between the wheels in meters.
        R (float): Wheel radius in meters.
        MAX_SPEED (float): Maximum allowable wheel speed.

    Returns:
        tuple[float, float, float, bool]: A tuple containing:
            - wl (float): Calculated left wheel velocity.
            - wr (float): Calculated right wheel velocity.
            - distance (float): Current distance to the goal.
            - is_obstacle_currently_detected (bool): True if an obstacle is currently detected.
    Side effects:
        Reads proximity sensor values.
    """
    psValues = read_proximity_sensors(ps)
    is_obstacle_currently_detected = check_for_obstacle(psValues, psThreshold)
    distance, angle_error = calculate_goal_info(x, y, phi, x_goal, y_goal)

    K_v = 3.0
    K_w = 7.0

    TURN_IN_PLACE_ANGLE_THRESHOLD = math.radians(2) # in degrees

    if distance > 0.020: # Still far from goal
        if abs(angle_error) > TURN_IN_PLACE_ANGLE_THRESHOLD:
            # If angle error is large, turn in place (v=0)
            v = 0.0
            w = K_w * angle_error 
        else:
            # Angle error is small, move towards goal and fine-tune angle
            v = K_v * distance
            w = K_w * angle_error 
    else: # Close to goal
        v = 0.0
        w = 0.0

    wl = (v - w * D / 2) / R
    wr = (v + w * D / 2) / R

    # Saturate wheel speeds
    wl = max(-MAX_SPEED, min(MAX_SPEED, wl))
    wr = max(-MAX_SPEED, min(MAX_SPEED, wr))

    return wl, wr, distance, is_obstacle_currently_detected

on = #pin no.##
#init
#grid Intersection as a 2d array
#costs same array as grid but the values will differ and be updated when an obstalce gets detected
#start goal and end goals 
#run initial dijkstra get initial route
#if on = True
#loop start
    #read sensors
    #encoder start
    #store grid coordinates
    #set goal to first point on path
    #go to next goal
    #linefollowing
    #if obstacle is detected and not near box goal
        # Obstacle Sequence:
            # 1. Stop motors immediately.
            # 2. Using encoder data that was counted from the last intersection return to said intersection .
            # 3. Turn 90 degrees to the right (or left, depending on strategy) using encoder data.
            # 4. Pathfind
    #if line lost
        #new goal calculated through odometry data, goes back to nearest line
    #If at intersection
        #new odometry counter activated from estimated 
        #if near box goal 
            #if touchsensor active:
                #activate magnet
                #sleep 0.3s
                #Boxstoring sequence
                #box counter incremented
        #if box counter < 3
            #next goal is next box
        #else
            #return original to starting coordinates
            #if at original coordinates initiate unloading sequence

