from machine import Pin, PWM
import time
import heapq

# === IR Sensors ===
pid_ir_pins = [2, 4, 39]            # left, center, right
pid_weights = [-1, 0, 1]
irs_pid = [Pin(pin, Pin.IN) for pin in pid_ir_pins]

all_ir_pins = [15, 2, 4, 39, 36]    # far left, left, center, right, far right
irs_all = [Pin(pin, Pin.IN) for pin in all_ir_pins]

# === Motor PWM ===
LEFT_FWD = PWM(Pin(33), freq=1000)
LEFT_BWD = PWM(Pin(32), freq=1000)
RIGHT_FWD = PWM(Pin(26), freq=1000)
RIGHT_BWD = PWM(Pin(25), freq=1000)

def set_motor(left, right):
    def apply(fwd, bwd, speed):
        duty = min(max(int(abs(speed) * 1023), 0), 1023)
        if speed > 0:
            fwd.duty(duty)
            bwd.duty(0)
        else:
            fwd.duty(0)
            bwd.duty(duty)
    apply(LEFT_FWD, LEFT_BWD, left)
    apply(RIGHT_FWD, RIGHT_BWD, right)

def stop():
    set_motor(0, 0)

def turn_90(direction, turn_time=0.5, speed=0.5):
    """
    Turn the robot 90 degrees using timed motor control.
    direction: 'left' or 'right'
    turn_time: duration in seconds (calibrate for your robot!)
    speed: motor speed (0 to 1)
    """
    if direction == 'left':
        set_motor(-speed, speed)
    elif direction == 'right':
        set_motor(speed, -speed)
    else:
        return
    time.sleep(turn_time)
    stop()

def get_direction(from_node, to_node):
    fx, fy = from_node
    tx, ty = to_node
    if fx == tx and ty > fy:
        return 'E'
    if fx == tx and ty < fy:
        return 'W'
    if fy == ty and tx > fx:
        return 'S'
    if fy == ty and tx < fx:
        return 'N'
    return None

def get_turn(current_dir, next_dir):
    dirs = ['N', 'E', 'S', 'W']
    idx = dirs.index(current_dir)
    next_idx = dirs.index(next_dir)
    diff = (next_idx - idx) % 4
    if diff == 0:
        return 'straight'
    elif diff == 1:
        return 'right'
    elif diff == 3:
        return 'left'
    else:
        return 'u-turn'

# === Box and Stepper Logic ===
def setup_pins(Pin):
    IN1 = Pin(14, Pin.OUT)
    IN2 = Pin(27, Pin.OUT)
    IN3 = Pin(26, Pin.OUT)
    IN4 = Pin(25, Pin.OUT)
    switch_pin = Pin(12, Pin.IN, Pin.PULL_DOWN)
    magnet_pin = Pin(13, Pin.OUT)
    magnet_pin.value(0)
    release_switch_pin = Pin(33, Pin.IN, Pin.PULL_DOWN)
    reverse_trigger_pin = Pin(32, Pin.IN, Pin.PULL_DOWN)
    return IN1, IN2, IN3, IN4, switch_pin, magnet_pin, release_switch_pin, reverse_trigger_pin

fullstep_seq = [
    [1, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 1],
    [1, 0, 0, 1]
]
STEPS_PER_REV = 2048

def step_motor(IN1, IN2, IN3, IN4, steps, delay_ms=2, direction=1):
    pins = [IN1, IN2, IN3, IN4]
    seq_len = len(fullstep_seq)
    seq_index = 0
    for _ in range(steps):
        for pin, val in zip(pins, fullstep_seq[seq_index]):
            pin.value(val)
        seq_index = (seq_index + direction) % seq_len
        time.sleep_ms(delay_ms)
    for pin in pins:
        pin.value(0)

def rotate(IN1, IN2, IN3, IN4, rotations, delay_ms=2, direction=1):
    total_steps = int(rotations * STEPS_PER_REV)
    step_motor(IN1, IN2, IN3, IN4, total_steps, delay_ms, direction)

def forward_process(IN1, IN2, IN3, IN4, magnet_pin, release_switch_pin, rotation_sequence, press_count):
    rotations = rotation_sequence[press_count % len(rotation_sequence)]
    print("Rotations this cycle:", rotations)
    if (press_count % len(rotation_sequence)) == 3:
        print("4th press → magnet ON until released")
        magnet_pin.value(1)
        print("Magnet ON. Waiting for release signal...")
        while release_switch_pin.value() == 0:
            time.sleep_ms(50)
        stable_count = 0
        while stable_count < 5:
            if release_switch_pin.value() == 1:
                stable_count += 1
            else:
                stable_count = 0
            time.sleep_ms(10)
        magnet_pin.value(0)
        print("Magnet OFF after release condition.")
        return True  # Enter reverse mode
    else:
        magnet_pin.value(1)
        print("Magnet ON.")
        time.sleep(1)
        rotate(IN1, IN2, IN3, IN4, rotations=rotations, delay_ms=5, direction=-1)
        print("Forward rotation done.")
        magnet_pin.value(0)
        print("Magnet OFF.")
        rotate(IN1, IN2, IN3, IN4, rotations=rotations, delay_ms=5, direction=1)
        print("Backward rotation done.")
        return False

# === Pathfinding Functions ===
def create_grid():
    global node_name_to_grid_coords
    explicit_named_grid_data = [
        ['X',  1,  'Y',  1,  'Z',  1, 'ZZ',  1,   1,   1,   1,   1,   1,   1,   1,   1,   1],
        [  0,  1,    0,  1,    0,  1,    0,  1,   1,   1,   1,   1,   1,   1,   1,   1,   1],
        ['A',  0,  'B',  0,  'C',  0,  'D',  0, 'E',   0,   0,   0,   0,   0,   0,   0, 'F'],
        [  0,  1,    1,  1,    1,  1,    1,  1,   0,   1,   1,   1,   1,   1,   1,   1,   0],
        [  0,  1,    1,  1,    1,  1,    1,  1, 'G',   0,   0,   0,   0,   0,   0,   0, 'H'],
        [  0,  1,    1,  1,    1,  1,    1,  1,   0,   1,   1,   1,   1,   1,   1,   1,   0],
        ['I',  0,    0,  0,    0,  0,    0,  0, 'J',   0,   0,   0,   0,   0,   0,   0, 'K'],
        [  0,  1,    1,  1,    1,  1,    1,  1,   0,   1,   1,   1,   1,   1,   1,   1,   0],
        ['L',  0,    0,  0,    0,  0,    0,  0, 'M',   1,   1,   1,   1,   1,   1,   1,   0],
        [  0,  1,    1,  1,    1,  1,    1,  1,   0,   1,   1,   1,   1,   1,   1,   1,   0],
        ['N',  0,    0,  0,    0,  0,    0,  0, 'O',   0, 'P',   0, 'Q',   0, 'R',   0, 'S'],
        [  1,  1,    1,  1,    1,  1,    1,  1,   1,   1,   0,   1,   0,   1,   0,   1,   0],
        [  1,  1,    1,  1,    1,  1,    1,  1,   1,   1, 'T',   1, 'U',   1, 'V',   1, 'W']
    ]
    converted_grid = []
    node_name_to_grid_coords_local = {}
    for r_idx, row_data in enumerate(explicit_named_grid_data):
        converted_row = []
        for c_idx, cell_value in enumerate(row_data):
            if isinstance(cell_value, str):
                node_name_to_grid_coords_local[cell_value] = (r_idx, c_idx)
                converted_row.append(0)
            else:
                converted_row.append(cell_value)
        converted_grid.append(converted_row)
    globals()['node_name_to_grid_coords'] = node_name_to_grid_coords_local
    return converted_grid

def create_costs():
    initial_grid = create_grid()
    costs = []
    for r in range(len(initial_grid)):
        row_costs = []
        for c in range(len(initial_grid[0])):
            if initial_grid[r][c] == 1:
                row_costs.append(1000000)
            else:
                row_costs.append(1)
        costs.append(row_costs)
    return costs

def dijkstra(grid, costs, start, goal):
    rows, cols = len(grid), len(grid[0])
    visited = [[False for _ in range(cols)] for _ in range(rows)]
    dist = [[float('inf') for _ in range(cols)] for _ in range(rows)]
    prev = [[None for _ in range(cols)] for _ in range(rows)]
    heap = []
    sx, sy = start
    gx, gy = goal
    dist[sx][sy] = 0
    heapq.heappush(heap, (0, (sx, sy)))
    while heap:
        current_dist, (x, y) = heapq.heappop(heap)
        if visited[x][y]:
            continue
        visited[x][y] = True
        if (x, y) == (gx, gy):
            break
        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < rows and 0 <= ny < cols and not visited[nx][ny]:
                if grid[nx][ny] == 1:
                    continue
                new_dist = dist[x][y] + costs[nx][ny]
                if new_dist < dist[nx][ny]:
                    dist[nx][ny] = new_dist
                    prev[nx][ny] = (x, y)
                    heapq.heappush(heap, (new_dist, (nx, ny)))
    path = []
    x, y = gx, gy
    if prev[x][y] or (x, y) == (sx, sy):
        while (x, y) != (sx, sy):
            path.append((x, y))
            x, y = prev[x][y]
        path.append((sx, sy))
        path.reverse()
    return path

# === State Machine ===
class State:
    INIT = 0
    LINE_FOLLOW = 1
    NODE_DETECTED = 2
    OBSTACLE_AVOIDANCE = 3
    BOX_PICKUP = 4
    RETURN_TO_BASE = 5
    DROP_OFF = 6
    IDLE = 7

current_state = State.INIT

# === Node Detection ===
from time import ticks_ms, ticks_diff
node_cooldown = 2000
last_node_time = ticks_ms() - node_cooldown

def detect_node():
    global last_node_time
    readings = [s.value() for s in irs_all]
    far_left = readings[0]
    far_right = readings[4]
    now = ticks_ms()
    if (far_left or far_right) and ticks_diff(now, last_node_time) > node_cooldown:
        last_node_time = now
        return True
    return False

# === Line Following ===
def line_follow():
    sensor_values = [s.value() for s in irs_pid]
    active = sum(sensor_values)
    if active == 0:
        set_motor(0.2, 0.2)
        return
    error = sum(w * v for w, v in zip(pid_weights, sensor_values)) / active
    base_speed = 0.6
    adjust = error * 0.2
    left_speed = max(min(base_speed - adjust, 1.0), 0.1)
    right_speed = max(min(base_speed + adjust, 1.0), 0.1)
    set_motor(left_speed, right_speed)

def handle_obstacle_avoidance():
    print("[SIM] Obstacle avoidance... (replace with your logic)")

def handle_drop_off():
    print("[SIM] Drop off... (replace with your logic)")

def handle_box_pickup(box_vars):
    IN1, IN2, IN3, IN4, magnet_pin, release_switch_pin = box_vars
    rotation_sequence = [4, 3, 2, 1]
    press_count = 0
    reverse_mode = forward_process(
        IN1, IN2, IN3, IN4, magnet_pin, release_switch_pin, rotation_sequence, press_count
    )
    print("Box pickup complete.")
    return True

# === Path Following Globals ===
planned_path = []
current_path_idx = 0
current_dir = 'E'  # Assume starting facing East (adjust as needed)

def plan_path():
    grid = create_grid()
    costs = create_costs()
    global node_name_to_grid_coords, planned_path, current_path_idx, current_dir
    start = node_name_to_grid_coords['A']
    goal = node_name_to_grid_coords['B']
    path = dijkstra(grid, costs, start, goal)
    if not path:
        print("No path found!")
        return False
    print(f"Planned path: {path}")
    planned_path = path
    current_path_idx = 0
    current_dir = 'E'
    return True

def main():
    global current_state, current_path_idx, current_dir, planned_path
    IN1, IN2, IN3, IN4, switch_pin, magnet_pin, release_switch_pin, reverse_trigger_pin = setup_pins(Pin)
    box_vars = (IN1, IN2, IN3, IN4, magnet_pin, release_switch_pin)
    while True:
        if current_state == State.INIT:
            print("State: INIT")
            if plan_path():
                current_state = State.LINE_FOLLOW
        elif current_state == State.LINE_FOLLOW:
            line_follow()
            if detect_node():
                current_state = State.NODE_DETECTED
        elif current_state == State.NODE_DETECTED:
            print("State: NODE_DETECTED")
            if current_path_idx < len(planned_path) - 1:
                from_node = planned_path[current_path_idx]
                to_node = planned_path[current_path_idx + 1]
                next_dir = get_direction(from_node, to_node)
                turn_type = get_turn(current_dir, next_dir)
                print(f"From {from_node} to {to_node}: {turn_type}")
                if turn_type == 'left':
                    turn_90('left', turn_time=0.5, speed=0.5)
                elif turn_type == 'right':
                    turn_90('right', turn_time=0.5, speed=0.5)
                elif turn_type == 'u-turn':
                    turn_90('left', turn_time=1.0, speed=0.5)
                # Update direction and path index
                current_dir = next_dir
                current_path_idx += 1
            current_state = State.LINE_FOLLOW
        elif current_state == State.OBSTACLE_AVOIDANCE:
            handle_obstacle_avoidance()
            current_state = State.LINE_FOLLOW
        elif current_state == State.BOX_PICKUP:
            print("State: BOX_PICKUP")
            if handle_box_pickup(box_vars):
                current_state = State.RETURN_TO_BASE
        elif current_state == State.RETURN_TO_BASE:
            print("State: RETURN_TO_BASE")
            current_state = State.DROP_OFF
        elif current_state == State.DROP_OFF:
            handle_drop_off()
            current_state = State.IDLE
        elif current_state == State.IDLE:
            print("State: IDLE - Robot is idle.")
            stop()
            break
        else:
            print(f"Unknown state: {current_state}")
            stop()
            break
        time.sleep(0.1)

if __name__ == "__main__":
    main()