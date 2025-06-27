# === MAGGIE Full Robot Controller ===
# MAGGIE is a robot that has a stepper motor that moves a belt with a magnet on it so it can grab boxes with a magnetic strip
# This code controls MAGGIE on a predetermined grid using 3 (simulated) ground sensors, 2 proximity sensors, 2 wheels and 1 swivel wheel.
# It must go the course, pick up 4 boxes, return, and unload them at specific coordinates.

from controller import Robot
import math
import heapq
import time

# === Constants ===
CELL_WIDTH = 0.06245
CELL_HEIGHT = 0.0587
ORIGIN_X = 0.0 # Assuming (0,0) of the grid maps to (0,0) in odometry based on J node. Adjust if your grid origin is different.
ORIGIN_Y = 0.0 # Assuming (0,0) of the grid maps to (0,0) in odometry based on J node. Adjust if your grid origin is different.
LINE_THRESHOLD = 600
MAX_SPEED = 6.28
BASE_SPEED = 1.2
Kp = 0.006
Kd = 0.008
OBSTACLE_THRESHOLD = 100.0
NODE_DEBOUNCE_TIME = 40
NODE_MATCH_TOLERANCE = 0.01 # Tolerance for matching odometry coordinates to named nodes (in meters)

# Robot physical parameters for odometry
WHEEL_RADIUS = 0.0205  # Approximate wheel radius in meters (you might need to adjust this)
WHEEL_DISTANCE = 0.052  # Distance between the centers of the two wheels in meters (you might need to adjust this)

# Named node positions corresponding to odometry data
node_positions = {
    'X': (-0.5, 0.36), 'Y': (-0.4, 0.36), 'Z': (-0.3, 0.36), 'ZZ': (-0.2, 0.36),
    'A': (-0.5, 0.25), 'B': (-0.4, 0.25), 'C': (-0.3, 0.25), 'D': (-0.2, 0.25), 'E': (0.0, 0.25), 'F': (0.5, 0.25),
    'G': (0.0, 0.1), 'H': (0.5, 0.1),
    'I': (-0.5, 0.0), 'J': (0.0, 0.0), 'K': (0.5, 0.0),
    'L': (-0.5, -0.1), 'M': (0.0, -0.1),
    'N': (-0.5, -0.25), 'O': (0.0, -0.25), 'P': (0.2, -0.25), 'Q': (0.3, -0.25), 'R': (0.4, -0.25), 'S': (0.5, -0.25),
    'T': (0.2, -0.36), 'U': (0.3, -0.36), 'V': (0.4, -0.36), 'W': (0.5, -0.36)
}

# === Setup Robot ===
robot = Robot()
timestep = int(robot.getBasicTimeStep())
delta_t = timestep / 1000.0  # Convert timestep to seconds

# Motors
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Sensors
gs = [robot.getDevice(f"gs{i}") for i in range(3)]
for g in gs:
    if g is not None:
        g.enable(timestep)

ps = [robot.getDevice("ps0"), robot.getDevice("ps1")]
for p in ps:
    if p is not None:
        p.enable(timestep)

# Encoders (PositionSensors for wheels)
left_encoder = robot.getDevice("left wheel sensor")
right_encoder = robot.getDevice("right wheel sensor")
if left_encoder is not None:
    left_encoder.enable(timestep)
if right_encoder is not None:
    right_encoder.enable(timestep)
encoders = [left_encoder, right_encoder]

# === Odometry Functions ===
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
    wr = (encoderValues[1] - oldEncoderValues[1]) / delta_t  # right wheel speed [rad/s]
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
    # If angular speed is very small, robot is moving in a straight line
    if abs(w) < 1e-6: # Using a small threshold to avoid division by zero or large errors from floating point inaccuracies
        x = x_old + u * math.cos(phi_old) * delta_t
        y = y_old + u * math.sin(phi_old) * delta_t
        phi = phi_old
    else:
        # Calculate radius of curvature
        R_curve = u / w
        # Calculate change in angle
        d_phi = w * delta_t
        
        # Calculate new x and y using arc movement equations
        x = x_old - R_curve * math.sin(phi_old) + R_curve * math.sin(phi_old + d_phi)
        y = y_old + R_curve * math.cos(phi_old) - R_curve * math.cos(phi_old + d_phi)
        phi = phi_old + d_phi
    
    # Normalize phi to be within -pi to pi
    phi = math.atan2(math.sin(phi), math.cos(phi))
    return [x, y, phi]

# === States ===
class State:
    INIT = 0
    LINE_FOLLOW = 1
    NODE_DETECTED = 2
    OBSTACLE_AVOIDANCE = 3
    GOAL_REACHED = 4
    RETURN_TO_BASE = 5
    DROP_OFF = 6
    IDLE = 7

current_state = State.INIT

# === Pathfinding Functions ===
def create_grid():
    # Example grid size, adjust to match your map
    return [[0]*17 for _ in range(13)]

def create_costs():
    return [[1]*17 for _ in range(13)]

def dijkstra(grid, costs, start, goal):
    rows, cols = len(grid), len(grid[0])
    visited = set()
    distances = {start: 0}
    parents = {start: None}
    queue = [(0, start)]
    directions = [(0,1), (1,0), (0,-1), (-1,0)]

    while queue:
        dist, node = heapq.heappop(queue)
        if node in visited:
            continue
        visited.add(node)
        if node == goal:
            break

        for dx, dy in directions:
            nx, ny = node[0] + dx, node[1] + dy
            if 0 <= nx < rows and 0 <= ny < cols and grid[nx][ny] == 0:
                cost = dist + costs[nx][ny]
                if (nx, ny) not in distances or cost < distances[(nx, ny)]:
                    distances[(nx, ny)] = cost
                    parents[(nx, ny)] = node
                    heapq.heappush(queue, (cost, (nx, ny)))

    path = []
    node = goal
    while node:
        path.append(node)
        node = parents[node]
    path.reverse()
    return path

def grid_to_odom(row, col):
    """Converts grid (row, col) to odometry (x, y) coordinates."""
    x = ORIGIN_X - col * CELL_WIDTH
    y = ORIGIN_Y + row * CELL_HEIGHT
    return (x, y)

def get_node_name_from_odom(current_x, current_y, tolerance=NODE_MATCH_TOLERANCE):
    """
    Checks if the current odometry coordinates match any known named node positions.

    Args:
        current_x (float): Robot's current x-coordinate.
        current_y (float): Robot's current y-coordinate.
        tolerance (float): Maximum distance for a match.

    Returns:
        str or None: The name of the matching node (e.g., 'A', 'J') or None if no match.
    """
    for name, (node_x, node_y) in node_positions.items():
        distance = math.sqrt((current_x - node_x)**2 + (current_y - node_y)**2)
        if distance <= tolerance:
            return name
    return None

def angle_between(n1, n2):
    """
    Calculates the angle (in radians) between two grid nodes.
    Uses grid_to_odom for coordinate conversion.
    """
    x1, y1 = grid_to_odom(*n1)
    x2, y2 = grid_to_odom(*n2)
    return math.atan2(y2 - y1, x2 - x1)

def angle_diff(a1, a2):
    """Calculates the shortest angular difference between two angles in degrees."""
    diff = a2 - a1
    while diff > math.pi:
        diff -= 2 * math.pi
    while diff < -math.pi:
        diff += 2 * math.pi
    return math.degrees(diff)

def determine_turn(prev, curr, nxt):
    """
    Determines the type of turn needed (forward, turn_left, turn_right)
    based on three consecutive grid nodes.
    """
    a1 = angle_between(prev, curr)
    a2 = angle_between(curr, nxt)
    diff = angle_diff(a1, a2)
    if abs(diff) < 30: # Within 30 degrees, consider it mostly straight
        return "forward"
    elif diff > 0: # Positive difference means turn left
        return "turn_left"
    else: # Negative difference means turn right
        return "turn_right"

def execute_turn_left():
    left_motor.setVelocity(-1.5)
    right_motor.setVelocity(2.5)
    robot.step(13 * timestep) # This step duration might need tuning based on actual turn

def execute_turn_right():
    left_motor.setVelocity(2.5)
    right_motor.setVelocity(-1.5)
    robot.step(13 * timestep) # This step duration might need tuning based on actual turn

def is_node_detected():
    return all(sensor.getValue() < LINE_THRESHOLD for sensor in gs)

def line_follow():
    global previous_error
    left = gs[0].getValue()
    center = gs[1].getValue()
    right = gs[2].getValue()

    if center < LINE_THRESHOLD:
        error = right - left
    else:
        if left < LINE_THRESHOLD:
            error = -1000
        elif right < LINE_THRESHOLD:
            error = 1000
        else:
            error = 0

    derivative = error - previous_error
    previous_error = error
    correction = Kp * error + Kd * derivative

    left_speed = max(min(BASE_SPEED - correction, MAX_SPEED), 0.7)
    right_speed = max(min(BASE_SPEED + correction, MAX_SPEED), 0.7)

    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

def is_obstacle_detected():
    return any(p.getValue() > OBSTACLE_THRESHOLD for p in ps)

# === Runtime Variables ===
previous_error = 0
node_detected_cooldown = 0
curr_index = 1
prev_node = None

# Odometry variables initialization
x_robot, y_robot, phi_robot = 0.0, 0.0, 0.0 # Initial pose, will be reset at first intersection
old_encoder_values = [0.0, 0.0] # Will be set at first intersection

last_known_good_x, last_known_good_y, last_known_good_phi = 0.0, 0.0, 0.0
last_known_good_encoders = [0.0, 0.0]
odometry_active = False # Flag to indicate when odometry is actively tracking relative movement

# === Main Control Loop ===
grid = create_grid()
costs = create_costs()
start = (0, 0) # Your initial grid start point
goals = [(12, 10), (12, 12), (12, 14), (12, 16)] # Example goals
box_counter = 0
path = []


while robot.step(timestep) != -1: # This is your main simulation loop step
    # --- Odometry Update (Only calculate if odometry_active is True) ---
    if odometry_active and encoders[0] is not None and encoders[1] is not None:
        current_encoder_values = read_encoders(encoders)
        wl, wr = get_wheels_speed(current_encoder_values, old_encoder_values, delta_t)
        u, w = get_robot_speeds(wl, wr, WHEEL_RADIUS, WHEEL_DISTANCE)
        x_robot, y_robot, phi_robot = get_robot_pose(u, w, x_robot, y_robot, phi_robot, delta_t)
        old_encoder_values = current_encoder_values
    # --- End Odometry Update ---

    if current_state == State.INIT:
        current_goal = goals[box_counter]
        path = dijkstra(grid, costs, start, current_goal)
        print(f"Initial path to box {box_counter+1}: {path}")
        prev_node = path[0]
        curr_index = 1
        current_state = State.LINE_FOLLOW

    elif current_state == State.LINE_FOLLOW:
        if is_obstacle_detected():
            print("Obstacle detected!")
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            current_state = State.OBSTACLE_AVOIDANCE
            continue

        if node_detected_cooldown > 0:
            node_detected_cooldown -= 1

        if node_detected_cooldown == 0 and is_node_detected():
            print("Node detected!")
            node_detected_cooldown = NODE_DEBOUNCE_TIME
            current_state = State.NODE_DETECTED
        else:
            line_follow()

    elif current_state == State.NODE_DETECTED:
        # Re-calibrate odometry to the known grid coordinates of the detected node
        current_grid_node = path[curr_index - 1]
        expected_node_x, expected_node_y = grid_to_odom(*current_grid_node)
        
        # Determine the expected orientation at this node, assuming it's facing the next path segment
        expected_phi = phi_robot # Default to current robot phi, will be corrected if there's a next path segment
        if curr_index < len(path): # If there's a next node in the path
            next_grid_node = path[curr_index]
            expected_phi = angle_between(current_grid_node, next_grid_node)
        
        # Set robot's pose to the known grid coordinates and orientation
        x_robot, y_robot, phi_robot = expected_node_x, expected_node_y, expected_phi
        
        # Save this as the last known good pose for recovery
        last_known_good_x, last_known_good_y, last_known_good_phi = x_robot, y_robot, phi_robot
        
        # Also re-initialize old_encoder_values to current readings for accurate odometry from this point
        if encoders[0] is not None and encoders[1] is not None:
            old_encoder_values = read_encoders(encoders)
            last_known_good_encoders = list(old_encoder_values) # Store a copy

        odometry_active = True # Odometry tracking is now active and reliable

        # Check if the detected node corresponds to a named node
        named_node = get_node_name_from_odom(x_robot, y_robot)
        if named_node:
            print(f"--> Robot is at NAMED NODE: {named_node} (Grid: {current_grid_node})")
            # You can add specific logic here for each named node if needed

        print(f"Odometry at Node: x={x_robot:.3f}, y={y_robot:.3f}, phi={math.degrees(phi_robot):.2f} degrees")


        if curr_index < len(path):
            curr_node_path = path[curr_index - 1]
            next_node_path = path[curr_index]
            direction = determine_turn(prev_node, curr_node_path, next_node_path)
            print(f"Turning from {curr_node_path} to {next_node_path}: {direction}")
            if direction == "turn_left":
                execute_turn_left()
            elif direction == "turn_right":
                execute_turn_right()
            prev_node = curr_node_path
            curr_index += 1
        else:
            print("Reached final node in path.")
            current_state = State.GOAL_REACHED
        current_state = State.LINE_FOLLOW

    elif current_state == State.OBSTACLE_AVOIDANCE:
        print("[OBSTACLE_AVOIDANCE] Obstacle detected! Reverting to last known good pose.")
        # Revert robot's internal pose to the last known good intersection
        x_robot, y_robot, phi_robot = last_known_good_x, last_known_good_y, last_known_good_phi
        if encoders[0] is not None and encoders[1] is not None:
            old_encoder_values = list(last_known_good_encoders) # Reset encoders too
        
        # Stop motors to prevent further collision/movement
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        
        # if not odometry reverse then rotate back:
        # left_motor.setVelocity(BASE_SPEED * 0)
        # right_motor.setVelocity(-BASE_SPEED / 2)
        # robot.step(int(0.5 / delta_t)) # Move back for 0.5 seconds
        # left_motor.setVelocity(0)
        # right_motor.setVelocity(0)


        # Transition back to line following to try and re-engage
        current_state = State.LINE_FOLLOW

    elif current_state == State.GOAL_REACHED:
        print("Goal reached! Placeholder for pickup.")
        current_state = State.IDLE

    elif current_state == State.IDLE:
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        break