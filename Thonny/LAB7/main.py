from machine import Pin, UART
from time import sleep
import heapq
import struct
#startup sequence it either connects to webots or I can work with it in Thonny
led_board = Pin(2, Pin.OUT)
print("Starting in 2 seconds... Close Thonny and start Webots.")
for i in range(2):
    led_board.value(1) 
    sleep(0.5)
    led_board.value(0)  
    sleep(0.5)
led_board.value(0) 

safe_pin = Pin(0, Pin.IN, Pin.PULL_UP)

if safe_pin.value() == 0:  # If the safe pin is pressed I can bypass the code, this way I can update my code without resetting the esp
    print("Safe mode active, skipping loop.")
else:
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
    CELL_WIDTH = 0.06245      # meters
    CELL_HEIGHT = 0.0587    # meters 
    #starting position
    ORIGIN_X = 0.5   
    ORIGIN_Y = -0.34    
    HEADING = 0              

    costs = create_costs()
    grid = create_grid()
    start = (0, 0)
    goal = (11, 16)

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
                        if len(odom_goals) > 1:
                            obstacle_pos = odom_to_grid(*odom_goals[1])  # Take the next intended goal position
                        obstacle_detected = False 
                    start = current_pos
                    path, odom_goals = pathfinder(obstacle_pos, costs, start, goal) 
                    x_goal, y_goal = odom_goals[1]
                    message = struct.pack('<ff', x_goal, y_goal)
                    uart.write(message)

            except Exception as e:            
                # Blinks thrice to indicate UART error
                for _ in range(3):
                    led_board.value(1)
                    sleep(0.1)
                    led_board.value(0)
                    sleep(0.1)
                    
        sleep(0.1)     # wait 


