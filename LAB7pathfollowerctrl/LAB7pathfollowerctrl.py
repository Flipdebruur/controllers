from controller import Robot
#-------------------------------------------------------
# Open serial port to communicate with the microcontroller
import serial #3
import math
try:
    # Change the port parameter according to your system
    ser =  serial.Serial(port='COM7', baudrate=115200, timeout=5) 
except:
    print("Communication failed. Check the cable connections and serial settings 'port' and 'baudrate'.")
    raise
import time
from time import sleep
import struct

#-------------------------------------------------------
# Initialize variables
GOAL_TOLERANCE = 0.03  # meters
MAX_SPEED = 6.28
speed = 0.4 * MAX_SPEED
leftSpeed = 0.0
rightSpeed = 0.0
obstacle_reported = False

# create the Robot instance for the simulation.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())   # [ms]

#-------------------------------------------------------
# Initialize devices

# proximity sensors
ps = []
psNames = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(timestep)
psThreshold = 300
Obstacle = False
# motors    
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

encoder = []
encoderNames = ['left wheel sensor', 'right wheel sensor']
for name in encoderNames:
    enc = robot.getDevice(name)
    enc.enable(timestep)
    encoder.append(enc)

robot.step(timestep)

# Initial pose
x, y, phi = 0.5, -0.34, 1.5708  # or whatever your starting pose is
x_goal, y_goal = x, y  # Initialize with current position
distance = 0.0

# Odometry parameters (place at the top of your file)
R = 0.020    # wheel radius [m]
D = 0.057    # distance between wheels [m]
delta_t = timestep / 1000.0

# --- Modular Angle Tracking Configuration ---
# Set to True to enable angle tracking, False to disable it.
ENABLE_ANGLE_TRACKING = True 

# --- Start of angle tracking variables (conditional initialization) ---
if ENABLE_ANGLE_TRACKING:
    # Define the reference angle for 0 degrees (1.5708 radians)
    REFERENCE_ANGLE_RAD = 1.5708

    # Initialize the previous phi to the starting phi for the first iteration
    prev_phi = phi

    # Initialize the total degrees turned
    total_degrees_turned = 0.0
# --- End of angle tracking variables ---

# --- Obstacle Retreat Variables ---
retreat_active = False
retreat_start_time = 0.0
RETREAT_DURATION = 1.5 # seconds to retreat backwards
RETREAT_SPEED = 0.4 * MAX_SPEED # Speed for retreating
# --- End of Obstacle Retreat Variables ---
def read_proximity_sensors(ps):
    return [sensor.getValue() for sensor in ps]

def check_for_obstacle(psValues, threshold):
    return any(val > threshold for val in psValues)

def calculate_goal_info(x, y, phi, x_goal, y_goal):
    dx = x_goal - x
    dy = y_goal - y
    distance = math.hypot(dx, dy)

    angle_to_goal = math.atan2(dy, dx)
    angle_error = angle_to_goal - phi
    angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

    return distance, angle_error

def send_status(ser, obstacle_reported, x, y, phi):
    status = 'O' if obstacle_reported else 'N'
    message = f"{status},{x:.3f},{y:.3f},{phi:.3f}\n"
    print(message)
    ser.write(message.encode())
    ser.flush()

def wait_for_new_goal(robot, ser, motors, timestep):
    stop_motors(motors)
    while ser.in_waiting == 0:
        robot.step(timestep)
        stop_motors(motors)

    data = ser.read(8)  # Expecting 2 floats
    if len(data) == 8:
        x_goal, y_goal = struct.unpack('<ff', data)
        print(f"🎯 New goal: x={x_goal:.3f}, y={y_goal:.3f}")
        return x_goal, y_goal, False  # Reset obstacle_reported
    else:
        print("⚠️ Incomplete binary data. Stopping.")
        stop_motors(motors)
        return None, None, True  # Signal failure

def stop_motors(motors):
    motors['left'].setVelocity(0)
    motors['right'].setVelocity(0)

def read_encoders(encoders):
    return [enc.getValue() for enc in encoders]

def get_wheels_speed(encoderValues, oldEncoderValues, delta_t):
    wl = (encoderValues[0] - oldEncoderValues[0]) / delta_t  # left wheel speed [rad/s]
    wr = (encoderValues[1] - oldEncoderValues[1]) / delta_t    # right wheel speed [rad/s]
    return [wl, wr]

def get_robot_speeds(wl, wr, r, d):
    u = (r / 2) * (wl + wr)  # linear speed [m/s]
    w = (r / d) * (wr - wl)  # angular speed [rad/s]
    return [u, w]

def get_robot_pose(u, w, x_old, y_old, phi_old, delta_t):
    x = x_old + u * math.cos(phi_old) * delta_t
    y = y_old + u * math.sin(phi_old) * delta_t
    phi = phi_old + w * delta_t
    # Normalize phi to be within -pi to pi range if it goes beyond.
    # This normalization is crucial for accurate continuous angle tracking.
    phi = math.atan2(math.sin(phi), math.cos(phi))
    return [x, y, phi]

def build_message(Obstacle, x, y, phi):
    message = ''
    if Obstacle:
        message += 'O'
    else:
        message += 'N'
    
    message += f',{x:.3f},{y:.3f},{phi:.3f}'
    return bytes(message + '\n', 'UTF-8')

def go_to_goal(x, y, phi, x_goal, y_goal, R, D, MAX_SPEED):
    psValues = read_proximity_sensors(ps)
    is_obstacle_currently_detected = check_for_obstacle(psValues, psThreshold)
    distance, angle_error = calculate_goal_info(x, y, phi, x_goal, y_goal)

    K_v = 3.0
    K_w = 7.0

    TURN_IN_PLACE_ANGLE_THRESHOLD = math.radians(3) # in degrees, adjust as needed

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

def update_angle_tracking(phi, prev_phi, total_degrees_turned):
    delta_phi = phi - prev_phi
    if delta_phi > math.pi:
        delta_phi -= 2 * math.pi
    elif delta_phi < -math.pi:
        delta_phi += 2 * math.pi

    total_degrees_turned += math.degrees(delta_phi)
    prev_phi = phi

    return prev_phi, total_degrees_turned


robot.step(timestep)
encoderValues = [enc.getValue() for enc in encoder]
oldEncoderValues = encoderValues.copy()

while robot.step(timestep) != -1:
    robot.step(timestep)

    encoderValues = read_encoders(encoder)
    wl, wr = get_wheels_speed(encoderValues, oldEncoderValues, delta_t)
    u, w = get_robot_speeds(wl, wr, R, D)
    x, y, phi = get_robot_pose(u, w, x, y, phi, delta_t)
    oldEncoderValues = encoderValues.copy()

    # --- Start of angle tracking logic (conditional execution) ---
    if ENABLE_ANGLE_TRACKING:
        prev_phi, total_degrees_turned = update_angle_tracking(phi, prev_phi, total_degrees_turned)


    # Get current goal-following speeds and obstacle status from go_to_goal
    # Note: wl, wr here are for potential goal following, not necessarily what is applied
    current_wl, current_wr, distance, is_obstacle_currently_detected = go_to_goal(x, y, phi, x_goal, y_goal, D, R, MAX_SPEED)       

    # Obstacle Handling Logic
    if is_obstacle_currently_detected and not obstacle_reported and not retreat_active:
        # Obstacle detected for the first time, initiate retreat
        print("⚠️ Obstacle detected! Initiating retreat.")
        obstacle_reported = True  # Mark this obstacle as reported
        retreat_active = True     # Activate retreat mode
        retreat_start_time = robot.getTime() # Record start time for retreat
        
        # Set velocities for retreat (backing up)
        leftMotor.setVelocity(-RETREAT_SPEED)
        rightMotor.setVelocity(-RETREAT_SPEED)
    
    if retreat_active:
        if (robot.getTime() - retreat_start_time) < RETREAT_DURATION:
            # Continue retreating, velocities are already set above
            pass 
        else:
            # Retreat duration finished
            retreat_active = False
            print("Retreat complete. Robot is now stopped and waiting for a new goal.")
            # Stop motors after retreat, and now it can proceed to wait for new goal logic below
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)

    # Main Robot State Machine
    if retreat_active:
        # If retreating, continue this state, do not proceed to goal following or waiting logic
        pass # Velocities are already set for retreat
    elif (distance < GOAL_TOLERANCE) or (obstacle_reported and is_obstacle_currently_detected):
        # Goal reached, or obstacle was reported and robot is now stopped after retreat
        print("🎯 Goal reached or obstacle handled, requesting next")
        
        # Determine status for message to microcontroller
        send_status(ser, obstacle_reported, x, y, phi)


        # Print the total degrees turned when a goal is reached or obstacle is detected (conditional)
        if ENABLE_ANGLE_TRACKING:
            print(f"Total degrees turned: {total_degrees_turned:.2f} degrees")
            
        # Wait for response from microcontroller, keeping motors at 0
        x_new, y_new, failed = wait_for_new_goal(robot, ser, {'left': leftMotor, 'right': rightMotor}, timestep)
        if not failed:
            x_goal, y_goal = x_new, y_new
            obstacle_reported = False
        else:
            break  # or add recovery logic

            
    else:
        # Normal goal following if not at goal, no active obstacle, and not waiting for new goal
        leftMotor.setVelocity(current_wl)
        rightMotor.setVelocity(current_wr)

ser.close()
