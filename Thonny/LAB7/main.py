from machine import Pin, UART
from time import sleep
import heapq
import struct
#startup sequence
led_board = Pin(2, Pin.OUT)  # onboard LED
print("Starting in 3 seconds... Close Thonny and start Webots.")
for i in range(2):
    led_board.value(1)  # LED ON
    sleep(0.5)
    led_board.value(0)  # LED OFF
    sleep(0.5)
led_board.value(0)  # Ensure LED is OFF at the end

safe_pin = Pin(0, Pin.IN, Pin.PULL_UP)

if safe_pin.value() == 0:  # pin not grounded
    print("Safe mode active, skipping loop.")
else:
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

    def pathfinder(obstacle_pos, costs, start, goal): #calculates new cost, runs dijkstra, generates new checkpoints
        if obstacle_pos is not None:
            x, y = obstacle_pos
            if 0 <= x < len(costs) and 0 <= y < len(costs[0]):
                costs[x][y] = 999
        path = dijkstra(grid, costs, start, goal)
        odom_goals = generate_path_goals(path) 
        return path, odom_goals
    # === World-to-Grid Calibration ===
    CELL_WIDTH = 0.06245      # meters
    CELL_HEIGHT = 0.0587    # meters 
    #starting position
    ORIGIN_X = 0.5           # x position  
    ORIGIN_Y = -0.34     # y position  
    HEADING = 0              # phi position

    costs = create_costs()
    grid = create_grid()
    start = (0, 0)
    goal = (12, 14)

    def odom_to_grid(x, y): #translate odometry data to grid position
        col = round((x - ORIGIN_X) / -CELL_WIDTH)
        row = round((y - ORIGIN_Y) / CELL_HEIGHT)
        return (row, col)
    
    def grid_to_odom(row, col): #translates grid position to odometry data (it's reversed)
        x = ORIGIN_X - (col * CELL_WIDTH)
        y = ORIGIN_Y + (row * CELL_HEIGHT)
        return (x, y)

    def generate_path_goals(path):
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

    uart = UART(1, 115200, tx=1, rx=3)
    while True:
        # Check if anything was received via serial to update sensor status
        if uart.any():
            led_board.value(1)  # LED ON
            try:
                msg = uart.readline().decode().strip()
                data = msg.split(',')
                if len(data) == 4:
                    status = data[0] # This will be 'O' or 'N'
                    x = float(data[1]) 
                    y = float(data[2])
                    phi = float(data[3])
                    
                    if status == 'O':
                        obstacle_detected = True

                    current_pos = odom_to_grid(x, y)
                    if obstacle_detected:
                        obstacle_pos = (current_pos[0], current_pos[1])
                        start = current_pos
                        # Assign both return values from pathfinder
                        path, odom_goals = pathfinder(obstacle_pos, costs, start, goal)
                        obstacle_detected = False 
                    start = current_pos #<<<<problem lies somewhere from here
                    path, odom_goals = pathfinder(obstacle_pos, costs, start, goal) 
                    x_goal, y_goal = odom_goals[1] #<<<<to here
                    message = struct.pack('<ff', x_goal, y_goal)  # 2 floats, its been tested this works
                    uart.write(message)

            except Exception as e:            
                # Blink LED rapidly 3 times to indicate UART error
                for _ in range(3):
                    led_board.value(1)
                    sleep(0.1)
                    led_board.value(0)
                    sleep(0.1)
                    
        sleep(0.1)     # wait 


