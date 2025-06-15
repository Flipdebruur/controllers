from controller import Robot
#-------------------------------------------------------
# Open serial port to communicate with the microcontroller
import serial
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
    # Read proximity sensors values

    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())
    
    # Determine if an obstacle is currently detected
    is_obstacle_currently_detected = (
        psValues[0] > psThreshold or psValues[1] > psThreshold or psValues[2] > psThreshold or psValues[3] > psThreshold or
        psValues[4] > psThreshold or psValues[5] > psThreshold or psValues[6] > psThreshold or psValues[7] > psThreshold
    )
    
    # This function now only calculates speeds for goal following.
    # Obstacle avoidance/retreat logic is handled in the main loop.
    dx = x_goal - x
    dy = y_goal - y
    distance = math.hypot(dx, dy)

    angle_to_goal = math.atan2(dy, dx)
    angle_error = angle_to_goal - phi
    angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

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


robot.step(timestep)
encoderValues = [enc.getValue() for enc in encoder]
oldEncoderValues = encoderValues.copy()

while robot.step(timestep) != -1:
    robot.step(timestep)

    encoderValues = [enc.getValue() for enc in encoder]
    wl, wr = get_wheels_speed(encoderValues, oldEncoderValues, delta_t)
    u, w = get_robot_speeds(wl, wr, R, D)
    x, y, phi = get_robot_pose(u, w, x, y, phi, delta_t)
    oldEncoderValues = encoderValues.copy()
    #print(f"Odometry pose: x={x:.3f} m, y={y:.3f} m, phi={phi:.3f} rad")

    # --- Start of angle tracking logic (conditional execution) ---
    if ENABLE_ANGLE_TRACKING:
        # Calculate the change in phi
        delta_phi = phi - prev_phi

        # Handle angle wrapping: if the robot crosses the -pi/pi (or 0/2pi) boundary
        if delta_phi > math.pi:
            delta_phi -= 2 * math.pi
        elif delta_phi < -math.pi:
            delta_phi += 2 * math.pi

        # Add the change to total_degrees_turned (now includes direction)
        total_degrees_turned += math.degrees(delta_phi)
        
        # Update prev_phi for the next iteration
        prev_phi = phi
    # --- End of angle tracking logic ---

    # Get current goal-following speeds and obstacle status from go_to_goal
    # Note: wl, wr here are for potential goal following, not necessarily what is applied
    current_wl, current_wr, distance, is_obstacle_currently_detected = go_to_goal(x, y, phi, x_goal, y_goal, D, R, MAX_SPEED)       

    # --- Obstacle Handling Logic ---
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

    # --- Main Robot State Machine ---
    if retreat_active:
        # If retreating, just continue this state, do not proceed to goal following or waiting logic
        pass # Velocities are already set for retreat
    elif (distance < GOAL_TOLERANCE) or (obstacle_reported and is_obstacle_currently_detected):
        # Goal reached, or obstacle was reported and robot is now stopped after retreat
        print("🎯 Goal reached or obstacle handled, requesting next")
        
        # Determine status for message to microcontroller
        status = 'O' if obstacle_reported else 'N' # If obstacle_reported is True, send 'O'
        message = f"{status},{x:.3f},{y:.3f},{phi:.3f}\n"
        print(message)
        ser.write(message.encode())
        ser.flush()

        # Print the total degrees turned when a goal is reached or obstacle is detected (conditional)
        if ENABLE_ANGLE_TRACKING:
            print(f"Total degrees turned: {total_degrees_turned:.2f} degrees")
            
        # Wait for response from microcontroller, keeping motors at 0
        while ser.in_waiting == 0:
            robot.step(timestep)
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
        
        # Read response and process new goal
        data = ser.read(8)  # 2 floats = 8 bytes
        if len(data) == 8:
            x_goal, y_goal = struct.unpack('<ff', data)
            print(f"🎯 New goal: x={x_goal:.3f}, y={y_goal:.3f}")
            obstacle_reported = False # Reset obstacle_reported only after a new goal is successfully received
        else:
            print("⚠️ Incomplete binary data. Stopping.")
            # In case of incomplete data, stop the robot to prevent erratic behavior
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            # You might want to add more robust error handling here, e.g., re-requesting
            
    else:
        # Normal goal following if not at goal, no active obstacle, and not waiting for new goal
        leftMotor.setVelocity(current_wl)
        rightMotor.setVelocity(current_wr)

ser.close()
