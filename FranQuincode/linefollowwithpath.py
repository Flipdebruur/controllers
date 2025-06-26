WEBOTS CODE THAT WORKS WITH THE ESP CODE: """
Webots Line-Following Robot Controller with UART Communication

This script controls a two-wheeled robot in Webots for line following and node detection.
It uses PID control to follow a black line on the floor and communicates with an external
ESP32 device via UART to receive movement instructions (turns, stops, etc.).

Features:
- PID-based line-following behavior using 3 ground sensors.
- Detects intersections (nodes) and notifies the ESP32 via serial.
- Receives navigation commands like TURN:LEFT, TURN:RIGHT, GO:STRAIGHT, and STOP.
- Executes turn commands using motor speed control with timed holding.

Usage:
- Run inside a Webots simulation environment.
- Ensure serial communication matches the ESP32 configuration (e.g., COM port and baud rate).
- Configure sensors and motor device names in the Webots robot.

Dependencies:
- Webots Python API (from controller import Robot)
- serial, time for communication and delay handling

Author: [Quinten de Theije]
"""
from controller import Robot
import serial
import time # import all the variables that we are going to use

robot = Robot()
timestep = int(robot.getBasicTimeStep()) # get the simulation timestep

MAX_SPEED = 6.28 # maximum motor speed
BASE_SPEED = 1.2 # base speed when line following
Kp = 0.006 # proportional from PID to follow the line 
Kd = 0.008 # derivative from PID for line following
LINE_THRESHOLD = 600 # line sensor value threshold

#initialize motors
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf')) # set to velocity control mode
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0) # initially stopped
right_motor.setVelocity(0.0)

#initialize line sensors
gs = [robot.getDevice(f"gs{i}") for i in range(3)] # get 3 ground sensors
for sensor in gs:
    sensor.enable(timestep) # enable sensors with simulation timestep

# Serial setup
try:
    ser = serial.Serial('COM6', 115200, timeout=0.1) # connect to ESP32 via serial
except Exception as e:
    print("Failed to open serial port:", e)
    exit(1)

previous_error = 0 # for PID derivative calculation

#line following function with PID for smoothing 
def line_follow():
    global previous_error
    left = gs[0].getValue()   # read left sensor
    center = gs[1].getValue() # read center sensor
    right = gs[2].getValue()  # read right sensor

    # decide where the line is and calculate error
    if center < LINE_THRESHOLD:
        error = right - left
    else:
        if left < LINE_THRESHOLD:
            error = -1000 # strong left bias
        elif right < LINE_THRESHOLD:
            error = 1000  # strong right bias
        else:
            error = 0 # assume centered

    derivative = error - previous_error # compute derivative
    previous_error = error # store error for next time

    # PID correction
    correction = Kp * error + Kd * derivative

    # Adjust motor speeds with correction
    left_speed = BASE_SPEED - correction
    right_speed = BASE_SPEED + correction

    # Clamp speeds to safe values
    left_speed = max(min(left_speed, MAX_SPEED), 0.7)
    right_speed = max(min(right_speed, MAX_SPEED), 0.7)

    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

# Detect a node (intersection) when all sensors read below threshold (line under all sensors)
def is_node_detected():
    return all(sensor.getValue() < LINE_THRESHOLD for sensor in gs)

#update the ESP on the node detection
def send_node_detected():
    msg = "NODE_DETECTED\n"
    ser.write(msg.encode()) # send message over serial
    print("Sent to ESP32:", msg.strip())

#debounce cooldown for node detection
NODE_DEBOUNCE_TIME = 40 # delay between node detections
node_detected_cooldown = 0  # counts down timesteps after sending NODE_DETECTED

#command hold for turning using a timer
command_hold_time = 0 # how long to hold a turning command
COMMAND_HOLD_DURATION = 13  # number of timesteps to hold a turn command
current_command = None # currently active command

# handle command sent from ESP32
def execute_command(cmd):
    global command_hold_time, current_command
    cmd = cmd.strip()
    if cmd in ["TURN:LEFT", "TURN:RIGHT"]:
        current_command = cmd
        command_hold_time = COMMAND_HOLD_DURATION # start holding turn
    elif cmd == "GO:STRAIGHT":
        current_command = "GO:STRAIGHT"
        command_hold_time = 0 # cancel any hold
    elif cmd == "STOP":
        current_command = "STOP"
        command_hold_time = 0 # cancel any hold
    else:
        # Unknown command fallback to line follow
        current_command = "GO:STRAIGHT"
        command_hold_time = 0

# perform command based on what is active (turning, stop, or follow line)
def perform_current_command():
    global command_hold_time, current_command
    if command_hold_time > 0:
        command_hold_time -= 1
        # Continue executing the current turn
        if current_command == "TURN:LEFT":
            left_motor.setVelocity(-1.5) # reverse left
            right_motor.setVelocity(2.5) # forward right = turn left
        elif current_command == "TURN:RIGHT":
            left_motor.setVelocity(2.5)  # forward left
            right_motor.setVelocity(-1.5) # reverse right = turn right
        else:
            # Should not happen but fallback function
            line_follow()
    else:
        # No hold, execute normal behavior
        if current_command == "STOP":
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
        else:
            #default -> line follow (straight)
            line_follow()

# main loop
while robot.step(timestep) != -1:
    # Node detection cooldown countdown
    if node_detected_cooldown > 0:
        node_detected_cooldown -= 1

    # Check for node detection only if cooldown expired
    if node_detected_cooldown == 0:
        if is_node_detected():
            send_node_detected() # tell ESP node found
            node_detected_cooldown = NODE_DEBOUNCE_TIME

    # Read commands from ESP32
    try:
        if ser.in_waiting > 0:
            line = ser.readline().decode().strip()
            if line:
                print("Received from ESP32:", line)
                execute_command(line) # handle received command
    except Exception as e:
        print("Serial read error:", e)

    # Perform whatever command is active (line follow, turning or stop)
    perform_current_command()