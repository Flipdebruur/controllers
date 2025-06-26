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
ORIGIN_X = 0.0
ORIGIN_Y = 0.0
LINE_THRESHOLD = 600
MAX_SPEED = 6.28
BASE_SPEED = 1.2
Kp = 0.006
Kd = 0.008
OBSTACLE_THRESHOLD = 80.0
NODE_DEBOUNCE_TIME = 40

# === Setup Robot ===
robot = Robot()
timestep = int(robot.getBasicTimeStep())

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
    x = ORIGIN_X - col * CELL_WIDTH
    y = ORIGIN_Y + row * CELL_HEIGHT
    return (x, y)

def angle_between(n1, n2):
    x1, y1 = grid_to_odom(*n1)
    x2, y2 = grid_to_odom(*n2)
    return math.atan2(y2 - y1, x2 - x1)

def angle_diff(a1, a2):
    diff = a2 - a1
    while diff > math.pi:
        diff -= 2 * math.pi
    while diff < -math.pi:
        diff += 2 * math.pi
    return math.degrees(diff)

def determine_turn(prev, curr, nxt):
    a1 = angle_between(prev, curr)
    a2 = angle_between(curr, nxt)
    diff = angle_diff(a1, a2)
    if abs(diff) < 30:
        return "forward"
    elif diff > 0:
        return "turn_left"
    else:
        return "turn_right"

def execute_turn_left():
    left_motor.setVelocity(-1.5)
    right_motor.setVelocity(2.5)
    robot.step(13 * timestep)

def execute_turn_right():
    left_motor.setVelocity(2.5)
    right_motor.setVelocity(-1.5)
    robot.step(13 * timestep)

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

# === Main Control Loop ===
grid = create_grid()
costs = create_costs()
start = (0, 0)
goals = [(12, 10), (12, 12), (12, 14), (12, 16)]
box_counter = 0
path = []

while robot.step(timestep) != -1:
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
        if curr_index < len(path):
            curr_node = path[curr_index - 1]
            next_node = path[curr_index]
            direction = determine_turn(prev_node, curr_node, next_node)
            print(f"Turning from {curr_node} to {next_node}: {direction}")
            if direction == "turn_left":
                execute_turn_left()
            elif direction == "turn_right":
                execute_turn_right()
            prev_node = curr_node
            curr_index += 1
        else:
            print("Reached final node in path.")
            current_state = State.GOAL_REACHED
        current_state = State.LINE_FOLLOW

    elif current_state == State.OBSTACLE_AVOIDANCE:
        print("[OBSTACLE_AVOIDANCE] Stopping. Replanning... (TODO)")
        current_state = State.IDLE

    elif current_state == State.GOAL_REACHED:
        print("Goal reached! Placeholder for pickup.")
        current_state = State.IDLE

    elif current_state == State.IDLE:
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        break
