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

# If 'J' is at (0.0, 0.0) odometry and its grid position is grid[6][8]
# Then:
# x_odom = ORIGIN_X - col * CELL_WIDTH => 0.0 = ORIGIN_X - 8 * CELL_WIDTH => ORIGIN_X = 8 * CELL_WIDTH
# y_odom = ORIGIN_Y + row * CELL_HEIGHT => 0.0 = ORIGIN_Y + 6 * CELL_HEIGHT => ORIGIN_Y = -6 * CELL_HEIGHT
ORIGIN_X = 8 * CELL_WIDTH
ORIGIN_Y = -6 * CELL_HEIGHT

LINE_THRESHOLD = 600
MAX_SPEED = 6.28
BASE_SPEED = 1.2
Kp = 0.06
Kd = 0.005
OBSTACLE_THRESHOLD = 100.0
NODE_DEBOUNCE_TIME = 25

# Robot physical parameters for odometry
WHEEL_RADIUS = 0.0205  # Approximate wheel radius in meters (you might need to adjust this)
WHEEL_DISTANCE = 0.052  # Distance between the centers of the two wheels in meters (you might need to adjust this)

# Removed: node_positions dictionary as requested by the user.

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

# Global variable to store the mapping of node names to their grid coordinates
node_name_to_grid_coords = {}

# === Pathfinding Functions ===
def create_grid():
    global node_name_to_grid_coords # Declare intent to modify global variable
    # The new grid provided by the user, containing named nodes as strings
    explicit_named_grid_data = [
        ['X',  1,  'Y',  1,  'Z',  1, 'ZZ',  1,   1,   1,   1,   1,   1,   1,   1,   1,   1], # Row 0
        [  0,  1,    0,  1,    0,  1,    0,  1,   1,   1,   1,   1,   1,   1,   1,   1,   1], # Row 1
        ['A',  0,  'B',  0,  'C',  0,  'D',  0, 'E',   0,   0,   0,   0,   0,   0,   0, 'F'], # Row 2
        [  0,  1,    1,  1,    1,  1,    1,  1,   0,   1,   1,   1,   1,   1,   1,   1,   0], # Row 3
        [  0,  1,    1,  1,    1,  1,    1,  1,   'G',   0,   0,   0,   0,   0,   0,   0, 'H'], # Row 4
        [  0,  1,    1,  1,    1,  1,    1,  1,   0,   1,   1,   1,   1,   1,   1,   1,   0], # Row 5
        ['I',  0,    0,  0,    0,  0,    0,  0, 'J',   0,   0,   0,   0,   0,   0,   0, 'K'], # Row 6
        [  0,  1,    1,  1,    1,  1,    1,  1,   0,   1,   1,   1,   1,   1,   1,   1,   0], # Row 7
        ['L',  0,    0,  0,    0,  0,    0,  0, 'M',   1,   1,   1,   1,   1,   1,   1,   0], # Row 8
        [  0,  1,    1,  1,    1,  1,    1,  1,   0,   1,   1,   1,   1,   1,   1,   1,   0], # Row 9
        ['N',  0,    0,  0,    0,  0,    0,  0, 'O',   0, 'P',   0, 'Q',   0, 'R',   0, 'S'], # Row 10
        [  1,  1,    1,  1,    1,  1,    1,  1,   1,   1,   0,   1,   0,   1,   0,   1,   0], # Row 11
        [  1,  1,    1,  1,    1,  1,    1,  1,   1,   1, 'T',   1, 'U',   1, 'V',   1, 'W']  # Row 12
    ]

    converted_grid = []
    node_name_to_grid_coords_local = {} # Use a local dictionary first

    for r_idx, row_data in enumerate(explicit_named_grid_data):
        converted_row = []
        for c_idx, cell_value in enumerate(row_data):
            if isinstance(cell_value, str): # It's a named node
                node_name_to_grid_coords_local[cell_value] = (r_idx, c_idx)
                converted_row.append(0) # Named nodes are traversable in pathfinding grid
            else:
                converted_row.append(cell_value) # 0 or 1
        converted_grid.append(converted_row)

    # Assign the locally populated dictionary to the global variable
    globals()['node_name_to_grid_coords'] = node_name_to_grid_coords_local
    return converted_grid

def create_costs():
    # Initialize costs with 1 for all traversable cells, and higher for obstacles
    initial_grid = create_grid() # Call create_grid to ensure node_name_to_grid_coords is populated
    costs = []
    for r in range(len(initial_grid)):
        row_costs = []
        for c in range(len(initial_grid[0])):
            if initial_grid[r][c] == 1: # Represents a wall/blocked path
                row_costs.append(1000000) # Very high cost for impassable areas
            else:
                row_costs.append(1) # Default cost for traversable areas
        costs.append(row_costs)
    return costs

def dijkstra(grid, costs, start, goal):
    rows, cols = len(grid), len(grid[0])
    visited = set()
    distances = {start: 0}
    parents = {start: None}
    queue = [(0, start)]
    directions = [(0,1), (1,0), (0,-1), (-1,0)] # Right, Down, Left, Up

    while queue:
        dist, node = heapq.heappop(queue)
        if node in visited:
            continue
        visited.add(node)
        if node == goal:
            break

        for dx, dy in directions:
            nx, ny = node[0] + dx, node[1] + dy
            # Check if neighbor is within bounds and traversable (grid value 0)
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
        node = parents.get(node) # Use .get to handle the start node which has None parent
    path.reverse()
    # Ensure the path actually starts at 'start' and ends at 'goal'
    return path if path and path[0] == start and path[-1] == goal else []

def grid_to_odom(row, col):
    """Converts grid (row, col) to odometry (x, y) coordinates."""
    # The X-axis decreases as column index increases from ORIGIN_X.
    # The Y-axis increases as row index increases from ORIGIN_Y.
    x = ORIGIN_X - col * CELL_WIDTH
    y = ORIGIN_Y + row * CELL_HEIGHT
    return (x, y)

# Removed: get_node_name_from_odom function as requested by the user.

def angle_between(n1, n2):
    """
    Calculates the angle (in radians) between two grid nodes.
    Uses grid_to_odom for coordinate conversion.
    """
    x1, y1 = grid_to_odom(*n1)
    x2, y2 = grid_to_odom(*n2)
    return math.atan2(y2 - y1, x2 - x1)

def angle_diff(a1, a2):
    """Calculates the shortest angular difference between two angles in radians."""
    diff = a2 - a1
    while diff > math.pi:
        diff -= 2 * math.pi
    while diff < -math.pi:
        diff += 2 * math.pi
    return diff # Return in radians

def determine_turn(prev, curr, nxt):
    """
    Determines the type of turn needed (forward, turn_left, turn_right)
    based on three consecutive grid nodes.
    """
    # Calculate the angle of the segment from prev to curr
    angle_in = angle_between(prev, curr)
    # Calculate the angle of the segment from curr to nxt
    angle_out = angle_between(curr, nxt)

    # Calculate the difference between the two angles
    diff = angle_diff(angle_in, angle_out) # Result is in radians

    # Convert to degrees for easier interpretation for thresholding
    diff_degrees = math.degrees(diff)

    # A small tolerance for straight movement (e.g., +/- 10-15 degrees)
    if abs(diff_degrees) < 15: # Within +/- 15 degrees, consider it mostly straight
        return "forward"
    else: # Positive difference means turn left, negative means turn right
        if diff_degrees > 0:
            return "turn_left"
        else:
            return "turn_right"

def execute_turn_left():
    left_motor.setVelocity(-1.5)
    right_motor.setVelocity(2.5)
    robot.step(int(13 * timestep)) # This step duration might need tuning based on actual turn

def execute_turn_right():
    left_motor.setVelocity(2.5)
    right_motor.setVelocity(-1.5)
    robot.step(int(13 * timestep)) # This step duration might need tuning based on actual turn

def is_node_detected():
    # Check if all ground sensors detect the line (indicating an intersection/node)
    # This logic assumes the robot is on a dark line over a light background, or vice-versa
    # and "LINE_THRESHOLD" is the cutoff. If all are below, it's on a dark intersection.
    return all(sensor.getValue() < LINE_THRESHOLD for sensor in gs)

def line_follow():
    global previous_error
    left = gs[0].getValue()
    center = gs[1].getValue()
    right = gs[2].getValue()

    # Basic PID-like line following
    if center < LINE_THRESHOLD: # Robot is on the line
        error = right - left
    else: # Robot is off the line
        if left < LINE_THRESHOLD: # Off to the right
            error = -1000
        elif right < LINE_THRESHOLD: # Off to the left
            error = 1000
        else: # Completely off line, try to go straight or turn back
            error = 0 # May need more sophisticated logic here if it gets lost
            # If completely off and no clear direction, stopping or a searching pattern might be better
            # For now, let's assume it finds the line quickly

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
path = [] # Stores the full Dijkstra path (cell by cell)
current_path_node_idx = 0 # Tracks the index of the node in 'path' the robot is currently at.

# Initialize grid and costs at the beginning to populate node_name_to_grid_coords
grid = create_grid() # This call also populates the global node_name_to_grid_coords
costs = create_costs()

# Initial start and goal setup
start_node_name = 'X' # Robot now starts at 'X' as per user's request
start_grid_coords = node_name_to_grid_coords[start_node_name] # Use the new mapping

# These are the actual grid coordinates for the boxes to pick up, derived from the new grid structure
goals_grid_coords_boxes = [
    node_name_to_grid_coords['T'],
    node_name_to_grid_coords['U'],
    node_name_to_grid_coords['V'],
    node_name_to_grid_coords['W']
]

# The return to base goal will be 'J'
return_to_base_grid_coords = node_name_to_grid_coords['J']

# Odometry variables initialization. Robot starts at 'X' grid coordinate, convert to odometry.
x_robot, y_robot = grid_to_odom(*start_grid_coords)
phi_robot = 0.0 # Assuming robot starts aligned with X-axis

# Step robot once to get initial encoder readings
robot.step(timestep)
old_encoder_values = [0.0, 0.0]
if left_encoder is not None and right_encoder is not None:
    old_encoder_values = read_encoders(encoders) # Read initial encoder values

last_known_good_x, last_known_good_y, last_known_good_phi = x_robot, y_robot, phi_robot
last_known_good_encoders = list(old_encoder_values)
odometry_active = True # Odometry should be active from the start

box_counter = 0 # Tracks which box we are currently going to pick up
current_target_goal = None # This will hold the current grid goal (either a box or base)


# === Main Control Loop ===
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
        # Determine the next goal based on current mission phase
        if box_counter < len(goals_grid_coords_boxes): # Still need to pick up boxes
            current_target_goal = goals_grid_coords_boxes[box_counter]
            print(f"Planning path to box {box_counter+1} at grid: {current_target_goal}")
        else: # All boxes picked up, return to base
            current_target_goal = return_to_base_grid_coords
            print(f"Planning path to return to base at grid: {current_target_goal}")

        path = dijkstra(grid, costs, start_grid_coords, current_target_goal)
        
        if not path:
            print(f"No path found from {start_grid_coords} to {current_target_goal}. Check grid and costs.")
            current_state = State.IDLE # Stop if no path
            continue

        print(f"Path calculated: {path}")
        current_path_node_idx = 0 # Robot starts at the very first node in the path.
        current_state = State.LINE_FOLLOW

    elif current_state == State.LINE_FOLLOW:
        if is_obstacle_detected():
            #print("Obstacle detected!")
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            current_state = State.OBSTACLE_AVOIDANCE
            continue

        if node_detected_cooldown > 0:
            node_detected_cooldown -= 1

        # Node detection for intersections
        if node_detected_cooldown == 0 and is_node_detected():
            #print("Node detected!")
            node_detected_cooldown = NODE_DEBOUNCE_TIME
            current_state = State.NODE_DETECTED
        else:
            line_follow()

    elif current_state == State.NODE_DETECTED:
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        robot.step(timestep) # Ensure motors stop for calibration

        current_grid_node_arrived_at = None

        # Only advance if the next node in the path is a named node
        # Find the next named node in the path
        next_named_node_idx = None
        for idx in range(current_path_node_idx + 1, len(path)):
            node = path[idx]
            if node in node_name_to_grid_coords.values():
                next_named_node_idx = idx
                break

        if next_named_node_idx is not None:
            current_path_node_idx = next_named_node_idx
            current_grid_node_arrived_at = path[current_path_node_idx]
            print(f"--> Node detected. Arrived at named node: {current_grid_node_arrived_at}.")
        else:
            print("Warning: No more named nodes in path or at end of path.")

        # Only recalibrate and check goal if at a named node
        if current_grid_node_arrived_at:
            expected_node_x, expected_node_y = grid_to_odom(*current_grid_node_arrived_at)
            expected_phi = phi_robot
            if current_path_node_idx + 1 < len(path):
                next_grid_node_for_phi = path[current_path_node_idx + 1]
                expected_phi = angle_between(current_grid_node_arrived_at, next_grid_node_for_phi)
            elif current_path_node_idx > 0:
                prev_grid_node_for_phi = path[current_path_node_idx - 1]
                expected_phi = angle_between(prev_grid_node_for_phi, current_grid_node_arrived_at)

            x_robot, y_robot, phi_robot = expected_node_x, expected_node_y, expected_phi
            last_known_good_x, last_known_good_y, last_known_good_phi = x_robot, y_robot, phi_robot
            if encoders[0] is not None and encoders[1] is not None:
                old_encoder_values = read_encoders(encoders)
                last_known_good_encoders = list(old_encoder_values)
            odometry_active = True
            #print(f"Odometry updated at Node: x={x_robot:.3f}, y={y_robot:.3f}, phi={math.degrees(phi_robot):.2f} degrees")

            # Check if at goal
            if current_grid_node_arrived_at == current_target_goal:
                print(f"Reached final target node: {current_grid_node_arrived_at}.")
                if current_target_goal in goals_grid_coords_boxes:
                    print(f"Picked up box {box_counter+1} at {current_grid_node_arrived_at}!")
                    box_counter += 1
                    if box_counter < len(goals_grid_coords_boxes):
                        start_grid_coords = current_grid_node_arrived_at
                        current_state = State.INIT
                    else:
                        print("All boxes collected! Initiating return to base.")
                        start_grid_coords = current_grid_node_arrived_at
                        current_state = State.INIT
                elif current_target_goal == return_to_base_grid_coords:
                    print("Returned to base (J node)! Initiating drop-off sequence.")
                    current_state = State.IDLE
            else:
                # Not the final goal, determine turn for the next segment and continue line following.
                if current_grid_node_arrived_at and current_path_node_idx + 1 < len(path):
                    turn_prev = path[current_path_node_idx - 1] if current_path_node_idx > 0 else path[0]
                    turn_curr = current_grid_node_arrived_at
                    turn_next = path[current_path_node_idx + 1]

                    direction = determine_turn(turn_prev, turn_curr, turn_next)
                    #print(f"Turn from {turn_prev} -> {turn_curr} -> {turn_next}: {direction}")
                    if direction == "turn_left":
                        execute_turn_left()
                    elif direction == "turn_right":
                        execute_turn_right()
                    current_state = State.LINE_FOLLOW
                else:
                    print("Error: Path ended unexpectedly before reaching target goal (in NODE_DETECTED else block).")
                    current_state = State.IDLE
        else:
            # If not at a named node, just keep following the line
            print("Intersection detected, but not at a named node. Continuing line following.")
            current_state = State.LINE_FOLLOW

    elif current_state == State.OBSTACLE_AVOIDANCE:
        print("[OBSTACLE_AVOIDANCE] Attempting to revert to last known good intersection...")

        # Stop motors
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        robot.step(timestep) # Ensure motors stop

        # Calculate distance to revert
        target_distance = 0.0
        if encoders[0] is not None and encoders[1] is not None:
            current_encoder_values = read_encoders(encoders)
            # Distance from last_known_good_encoders to current_encoder_values
            dl = (current_encoder_values[0] - last_known_good_encoders[0]) * WHEEL_RADIUS
            dr = (current_encoder_values[1] - last_known_good_encoders[1]) * WHEEL_RADIUS
            target_distance = (dl + dr) / 2.0  # Average linear distance traveled since last known good
            print(f"Reversing distance: {target_distance:.3f} meters")

            reverse_distance_traveled = 0.0
            initial_reverse_encoders = read_encoders(encoders)

            # Reverse until distance is covered. Use a small threshold to avoid infinite loop.
            while reverse_distance_traveled < abs(target_distance) - 0.005: # Subtract a small value for tolerance
                left_motor.setVelocity(-BASE_SPEED)
                right_motor.setVelocity(-BASE_SPEED)
                robot.step(timestep)

                current_reverse_encoders = read_encoders(encoders)
                dl_rev = (current_reverse_encoders[0] - initial_reverse_encoders[0]) * WHEEL_RADIUS
                dr_rev = (current_reverse_encoders[1] - initial_reverse_encoders[1]) * WHEEL_RADIUS
                reverse_distance_traveled = abs((dl_rev + dr_rev) / 2.0)

        # Stop motors after reversing
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        print("[OBSTACLE_AVOIDANCE] Returned to last intersection area (approx).")

        # Recalibrate pose to the last known good intersection's pose
        x_robot, y_robot, phi_robot = last_known_good_x, last_known_good_y, last_known_good_phi
        # Re-initialize encoders to correspond to this known pose
        if encoders[0] is not None and encoders[1] is not None:
            old_encoder_values = list(last_known_good_encoders)

        # Identify the segment that had the obstacle.
        # The robot was traversing the segment from `path[current_path_node_idx]` (current node)
        # to `path[current_path_node_idx + 1]` (next target, where obstacle was found).
        problem_segment_start_node = path[current_path_node_idx]
        problem_segment_end_node = path[current_path_node_idx + 1] # The next node in the path

        print(f"Marking segment from {problem_segment_start_node} to {problem_segment_end_node} as high cost.")

        # Increase cost for every grid cell along this segment
        dx = problem_segment_end_node[0] - problem_segment_start_node[0]
        dy = problem_segment_end_node[1] - problem_segment_start_node[1]

        steps = max(abs(dx), abs(dy))
        if steps == 0: # If start and end nodes are the same (shouldn't happen for a segment in a valid path)
            steps = 1

        for i in range(steps + 1): # Include start and end nodes
            # Interpolate grid coordinates
            row_to_mark = problem_segment_start_node[0] + (dx * i // steps)
            col_to_mark = problem_segment_start_node[1] + (dy * i // steps)

            if 0 <= row_to_mark < len(costs) and 0 <= col_to_mark < len(costs[0]):
                # Make it very undesirable, potentially impassable for future paths
                costs[row_to_mark][col_to_mark] += 100000 
                print(f"[COST UPDATE] Increased cost at ({row_to_mark}, {col_to_mark})")

        # Recalculate path from the node where the robot reverted to.
        # The robot is now effectively at `problem_segment_start_node` (which is `path[current_path_node_idx]`).
        start_for_new_path = path[current_path_node_idx]
        path = dijkstra(grid, costs, start_for_new_path, current_target_goal)

        if not path:
            print(f"No new path found from {start_for_new_path} to {current_target_goal}. Robot stuck.")
            current_state = State.IDLE # No path found, robot is stuck
            continue

        print(f"[OBSTACLE_AVOIDANCE] New path calculated: {path}")
        current_path_node_idx = 0 # Reset index for the new path, as robot is at path[0] of this new path.

        # Resume line following
        current_state = State.LINE_FOLLOW


    elif current_state == State.GOAL_REACHED:
        print("Goal reached (internal logic, actual transitions happen in NODE_DETECTED).")
        # This state is more of a placeholder as actual goal-reaching logic
        # is handled within NODE_DETECTED based on `current_path_node_idx` reaching the goal.
        current_state = State.IDLE # For now, transition to IDLE after goal processing in NODE_DETECTED

    elif current_state == State.IDLE:
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        break # Exit the main loop if in IDLE state