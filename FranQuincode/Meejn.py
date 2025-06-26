ESP CODE WITH DIJKSTRA AND WEBOTS CODE BEFORE THIS: import ujson
from machine import UART
import time
import math
import heapq

# UART config
uart = UART(1, baudrate=115200, tx=17, rx=16)  # Adjust pins as needed

# --- Graph for Dijkstra ---
graph = {
    'X': [('A', 0.11)],
    'Y': [('B', 0.11)],
    'Z': [('C', 0.11)],
    'ZZ': [('D', 0.11)],
    'A': [('X', 0.11), ('B', 0.1), ('I', 0.25)],
    'B': [('A', 0.1), ('Y', 0.11), ('C', 0.1)],
    'C': [('B', 0.1), ('Z', 0.11), ('D', 0.1)],
    'D': [('C', 0.1), ('E', 0.2), ('ZZ', 0.11)],
    'E': [('D', 0.2), ('G', 0.15), ('F', 0.5)],
    'F': [('E', 0.5), ('H', 0.15)],
    'G': [('E', 0.15), ('H', 0.5), ('J', 0.1)],
    'H': [('F', 0.15), ('G', 0.5), ('K', 0.1)],
    'I': [('A', 0.25), ('L', 0.1), ('J', 0.5)],
    'J': [('I', 0.5), ('G', 0.1), ('K', 0.5), ('M', 0.1)],
    'K': [('H', 0.1), ('J', 0.5), ('S', 0.25)],
    'L': [('I', 0.1), ('M', 0.5), ('N', 0.15)],
    'M': [('L', 0.5), ('O', 0.15), ('J', 0.1)],
    'N': [('L', 0.15), ('O', 0.5)],
    'O': [('M', 0.15), ('N', 0.5), ('P', 0.2)],
    'P': [('O', 0.2), ('Q', 0.1), ('T', 0.11)],
    'Q': [('P', 0.1), ('R', 0.1), ('U', 0.11)],
    'R': [('Q', 0.1), ('S', 0.1), ('V', 0.11)],
    'S': [('K', 0.25), ('R', 0.1), ('W', 0.11)],
    'T': [('P', 0.11)],
    'U': [('Q', 0.11)],
    'V': [('R', 0.11)],
    'W': [('S', 0.11)]
}

node_positions = {
    'X': (-0.5, 0.36), 'Y': (-0.4, 0.36), 'Z': (-0.3, 0.36), 'ZZ': (-0.2, 0.36),
    'A': (-0.5, 0.25), 'B': (-0.4, 0.25), 'C': (-0.3, 0.25), 'D': (-0.2, 0.25), 'E': (0.0, 0.25), 'F': (0.5, 0.25),
    'G': (0.0, 0.1), 'H': (0.5, 0.1),
    'I': (-0.5, 0.0), 'J': (0.0, 0.0), 'K': (0.5, 0.0),
    'L': (-0.5, -0.1), 'M': (0.0, -0.1),
    'N': (-0.5, -0.25), 'O': (0.0, -0.25), 'P': (0.2, -0.25), 'Q': (0.3, -0.25), 'R': (0.4, -0.25), 'S': (0.5, -0.25),
    'T': (0.2, -0.36), 'U': (0.3, -0.36), 'V': (0.4, -0.36), 'W': (0.5, -0.36)
}

def dijkstra(graph, start, goal):
    queue = [(0, start, [])]
    visited = set()
    while queue:
        cost, node, path = heapq.heappop(queue)
        if node in visited:
            continue
        path = path + [node]
        visited.add(node)
        if node == goal:
            return path, cost
        for neighbor, weight in graph.get(node, []):
            if neighbor not in visited:
                heapq.heappush(queue, (cost + weight, neighbor, path))
    return None, float('inf')

def angle_between_nodes(n1, n2):
    x1, z1 = node_positions[n1]
    x2, z2 = node_positions[n2]
    return math.atan2(z2 - z1, x2 - x1)

def angle_diff(a1, a2):
    diff = a2 - a1
    while diff > math.pi:
        diff -= 2 * math.pi
    while diff < -math.pi:
        diff += 2 * math.pi
    return diff * 180 / math.pi

def determine_turn_direction(prev_node, current_node, next_node):
    incoming_angle = angle_between_nodes(prev_node, current_node)
    outgoing_angle = angle_between_nodes(current_node, next_node)
    diff = angle_diff(incoming_angle, outgoing_angle)
    if abs(diff) < 30:
        return "forward"
    elif diff > 0:
        return "turn_left"
    else:
        return "turn_right"

# --- Variables for path following ---
start_node = 'A'
goal_node = 'W'
path, cost = dijkstra(graph, start_node, goal_node)
if not path:
    print("No path found!")
    raise SystemExit

current_index = 1  # next node index
previous_node = path[0]

def send_cmd(cmd):
    uart.write((cmd + '\n').encode('utf-8'))
    print("Sent to Webots:", cmd)

def read_line():
    line = b''
    timeout = time.ticks_ms() + 100
    while time.ticks_ms() < timeout:
        if uart.any():
            c = uart.read(1)
            if c == b'\n':
                break
            line += c
    return line.decode('utf-8').strip()

while True:
    line = read_line()
    if not line:
        continue

    print("Received from Webots:", line)

    if line == '1110':  # no line detected, need to decide turn or move
        # If at end of path, stop
        if current_index >= len(path):
            send_cmd('stop')
            continue

        current_node = path[current_index]
        # If we can advance safely
        if current_index + 1 < len(path):
            next_node = path[current_index + 1]
            turn = determine_turn_direction(previous_node, current_node, next_node)
            send_cmd(turn)
            if turn == "forward":
                previous_node = current_node
                current_index += 1
        else:
            # No more nodes, stop
            send_cmd('stop')

    else:
        # Normal line sensor bits: just move forward
        send_cmd('forward')