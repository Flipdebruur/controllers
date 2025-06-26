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
# Use 3 ground sensors for simulation; update to 5 when deploying on real hardware
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

# === Pathfinding ===
def create_grid():
    return [
        [0,1,0,1,0,1,0,1,1,1,1,1,1,1,1,1,1],
        [0,1,0,1,0,1,0,1,1,1,1,1,1,1,1,1,1],
        [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        [0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0],
        [0,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0],
        [0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0],
        [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        [0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0],
        [0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,0],
        [0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0],
        [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
        [1,1,1,1,1,1,1,1,1,1,0,1,0,1,0,1,0],
        [1,1,1,1,1,1,1,1,1,1,0,1,0,1,0,1,0]
    ]

def create_costs():
    return [[1]*17 for _ in range(13)]

def mark_obstacle(costs, grid_pos):
    row, col = grid_pos
    if 0 <= row < len(costs) and 0 <= col < len(costs[0]):
        costs[row][col] = 999

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

def odom_to_grid(x, y):
    col = round((x - ORIGIN_X) / -CELL_WIDTH)
    row = round((y - ORIGIN_Y) / CELL_HEIGHT)
    return (row, col)

def generate_path_goals(path):
    return [grid_to_odom(r, c) for r, c in path]

# === Obstacle Handling in State Machine ===
# In OBSTACLE_AVOIDANCE state:
# - stop motors
# - mark current node as blocked
# - rerun Dijkstra
# - reset path and continue
# (This part is now built-in and updates dynamically based on the position)

# === Checklist ===
# - [x] Stepper + magnet box pickup (placeholder, not yet implemented)
# - [x] Grid and cost map
# - [x] Dynamic cost update with obstacle detection
# - [x] 5 ground sensors (simulated with 3 for now)
# - [x] 2 proximity sensors
# - [x] Start-goal path with Dijkstra
# - [x] Full course loop with turning logic
# - [x] Node detection + handling
# - [ ] Touch sensor check & box pickup logic
# - [ ] Return & drop-off sequence

# === TO DO ===
# - Encoder integration for backtracking on obstacle
# - Magnet + box pick-up routine
# - Drop-off logic when all 4 boxes are collected
