from controller import Robot
import serial
import math
try:
    ser =  serial.Serial(port='COM7', baudrate=115200, timeout=5) 
except:
    print("Communication failed. Check the cable connections and serial settings 'port' and 'baudrate'.")
    raise
from time import sleep
import struct

# variables
GOAL_TOLERANCE = 0.03  # meters
MAX_SPEED = 6.28
speed = 1.0 * MAX_SPEED
leftSpeed = 0.0
rightSpeed = 0.0
obstacle_reported = False

#init Webots
robot = Robot()
timestep = int(robot.getBasicTimeStep())   # [ms]

#-------------------------------------------------------
#init devices

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
x, y, phi = 0.5, -0.34, 1.5708
x_goal, y_goal = x, y
distance = 0.0

# Odometry parameters
R = 0.020    # wheel radius [m]
D = 0.057    # distance between wheels [m]
delta_t = timestep / 1000.0

# Set to True to enable angle tracking, False to disable it. for debugging purposes
ENABLE_ANGLE_TRACKING = True 
if ENABLE_ANGLE_TRACKING:
    REFERENCE_ANGLE_RAD = 1.5708
    prev_phi = phi
    total_degrees_turned = 0.0


#Obstacle retreat variables
retreat_active = False
retreat_start_time = 0.0
RETREAT_DURATION = 0.2
RETREAT_SPEED = 0.7 * MAX_SPEED 

def read_proximity_sensors(ps):
    """
    Reads the current values from all enabled proximity sensors.

    Args:
        ps (list): A list of Webots ProximitySensor objects.

    Returns:
        list[float]: A list of sensor values.
    """
    return [sensor.getValue() for sensor in ps]

def check_for_obstacle(psValues, threshold):
    """
    Checks if any proximity sensor detects an obstacle above a given threshold.

    Args:
        psValues (list[float]): A list of proximity sensor values.
        threshold (float): The threshold value above which an obstacle is considered detected.

    Returns:
        bool: True if an obstacle is detected, False otherwise.
    """
    return any(val > threshold for val in psValues)

def calculate_goal_info(x, y, phi, x_goal, y_goal):
    """
    Calculates the distance and angle error to the current goal.

    Args:
        x (float): Current x-coordinate of the robot.
        y (float): Current y-coordinate of the robot.
        phi (float): Current orientation (heading) of the robot in radians.
        x_goal (float): X-coordinate of the target goal.
        y_goal (float): Y-coordinate of the target goal.

    Returns:
        tuple[float, float]: A tuple containing:
            - distance (float): Euclidean distance to the goal.
            - angle_error (float): Angular difference between robot's heading and direction to goal.
    """
    dx = x_goal - x
    dy = y_goal - y
    distance = math.hypot(dx, dy)

    angle_to_goal = math.atan2(dy, dx)
    angle_error = angle_to_goal - phi
    angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

    return distance, angle_error

def send_status(ser, obstacle_reported, x, y, phi):
    """
    Sends the robot's current status (obstacle detected, x, y, phi) over serial.

    Args:
        ser (serial.Serial): The serial port object.
        obstacle_reported (bool): True if an obstacle was reported in the current cycle, False otherwise.
        x (float): Current x-coordinate of the robot.
        y (float): Current y-coordinate of the robot.
        phi (float): Current orientation (heading) of the robot in radians.
    Side effects:
        Writes data to the serial port.
    """
    status = 'O' if obstacle_reported else 'N'
    message = f"{status},{x:.3f},{y:.3f},{phi:.3f}\n"
    print(message)
    ser.write(message.encode())
    ser.flush()

def wait_for_new_goal(robot, ser, motors, timestep):
    """
    Halts the robot and waits for a new goal message from the serial port.

    Args:
        robot (controller.Robot): The Webots Robot object.
        ser (serial.Serial): The serial port object.
        motors (dict): A dictionary containing 'left' and 'right' motor objects.
        timestep (int): The simulation timestep in milliseconds.

    Returns:
        tuple[float or None, float or None, bool]: A tuple containing:
            - x_goal (float or None): New x-coordinate of the goal, or None if data is incomplete.
            - y_goal (float or None): New y-coordinate of the goal, or None if data is incomplete.
            - failed (bool): True if data is incomplete, False otherwise.
    Side effects:
        Stops the robot motors.
        Reads data from the serial port.
    """
    stop_motors(motors)
    while ser.in_waiting == 0:
        robot.step(timestep)
        stop_motors(motors)

    data = ser.read(8)  # Expecting 2 floats
    if len(data) == 8:
        x_goal, y_goal = struct.unpack('<ff', data)
        print(f"New goal: x={x_goal:.3f}, y={y_goal:.3f}")
        return x_goal, y_goal, False  # Reset obstacle_reported
    else:
        print("Incomplete data, Stopping.")
        stop_motors(motors)
        return None, None, True  # Signal failure

def stop_motors(motors):
    """
    Sets the velocity of both left and right motors to zero, effectively stopping the robot.

    Args:
        motors (dict): A dictionary containing 'left' and 'right' motor objects.
    Side effects:
        Sets motor velocities.
    """
    motors['left'].setVelocity(0)
    motors['right'].setVelocity(0)

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
    wr = (encoderValues[1] - oldEncoderValues[1]) / delta_t    # right wheel speed [rad/s]
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
    x = x_old + u * math.cos(phi_old) * delta_t
    y = y_old + u * math.sin(phi_old) * delta_t
    phi = phi_old + w * delta_t
    phi = math.atan2(math.sin(phi), math.cos(phi))
    return [x, y, phi]

def build_message(Obstacle, x, y, phi):
    """
    Constructs a status message string for serial communication.

    Args:
        Obstacle (bool): True if an obstacle is detected, False otherwise.
        x (float): Current x-coordinate of the robot.
        y (float): Current y-coordinate of the robot.
        phi (float): Current orientation (heading) of the robot in radians.

    Returns:
        bytes: The encoded status message.
    """
    message = ''
    if Obstacle:
        message += 'O'
    else:
        message += 'N'
    
    message += f',{x:.3f},{y:.3f},{phi:.3f}'
    return bytes(message + '\n', 'UTF-8')

def go_to_goal(x, y, phi, x_goal, y_goal, R, D, MAX_SPEED):
    """
    Calculates the required wheel velocities to move the robot towards a goal,
    considering obstacle detection and turning behavior.

    Args:
        x (float): Current x-coordinate of the robot.
        y (float): Current y-coordinate of the robot.
        phi (float): Current orientation (heading) of the robot in radians.
        x_goal (float): X-coordinate of the target goal.
        y_goal (float): Y-coordinate of the target goal.
        D (float): Distance between the wheels in meters.
        R (float): Wheel radius in meters.
        MAX_SPEED (float): Maximum allowable wheel speed.

    Returns:
        tuple[float, float, float, bool]: A tuple containing:
            - wl (float): Calculated left wheel velocity.
            - wr (float): Calculated right wheel velocity.
            - distance (float): Current distance to the goal.
            - is_obstacle_currently_detected (bool): True if an obstacle is currently detected.
    Side effects:
        Reads proximity sensor values.
    """
    psValues = read_proximity_sensors(ps)
    is_obstacle_currently_detected = check_for_obstacle(psValues, psThreshold)
    distance, angle_error = calculate_goal_info(x, y, phi, x_goal, y_goal)

    K_v = 3.0
    K_w = 7.0

    TURN_IN_PLACE_ANGLE_THRESHOLD = math.radians(2) # in degrees

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
    """
    Updates the total degrees turned by the robot for debugging purposes.

    Args:
        phi (float): Current orientation (heading) of the robot in radians.
        prev_phi (float): Previous orientation (heading) of the robot in radians.
        total_degrees_turned (float): Accumulator for total degrees turned.

    Returns:
        tuple[float, float]: A tuple containing:
            - prev_phi (float): Updated previous orientation for the next iteration.
            - total_degrees_turned (float): Updated total degrees turned.
    """
    delta_phi = phi - prev_phi
    if delta_phi > math.pi:
        delta_phi -= 2 * math.pi
    elif delta_phi < -math.pi:
        delta_phi += 2 * math.pi

    total_degrees_turned += math.degrees(delta_phi)
    prev_phi = phi

    return prev_phi, total_degrees_turned

encoderValues = [enc.getValue() for enc in encoder]
oldEncoderValues = encoderValues.copy()

while robot.step(timestep) != -1:
    robot.step(timestep)

    encoderValues = read_encoders(encoder)
    wl, wr = get_wheels_speed(encoderValues, oldEncoderValues, delta_t)
    u, w = get_robot_speeds(wl, wr, R, D)
    x, y, phi = get_robot_pose(u, w, x, y, phi, delta_t)
    oldEncoderValues = encoderValues.copy()

    if ENABLE_ANGLE_TRACKING:
        prev_phi, total_degrees_turned = update_angle_tracking(phi, prev_phi, total_degrees_turned)

    current_wl, current_wr, distance, is_obstacle_currently_detected = go_to_goal(x, y, phi, x_goal, y_goal, D, R, MAX_SPEED)       


    if is_obstacle_currently_detected and not obstacle_reported and not retreat_active:
        print("Obstacle detected! Initiating retreat.")
        obstacle_reported = True
        retreat_active = True
        retreat_start_time = robot.getTime()
        
        leftMotor.setVelocity(-RETREAT_SPEED)
        rightMotor.setVelocity(-RETREAT_SPEED)
    
    if retreat_active:
        if (robot.getTime() - retreat_start_time) < RETREAT_DURATION:
            # Continue retreating
            pass 
        else:
            # Retreat duration finished
            retreat_active = False
            print("Retreat complete. Robot is now stopped and waiting for a new goal.")
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)

    # Main Robot State Machine
    if retreat_active:
        # If retreating, continue this state
        pass 
    elif (distance < GOAL_TOLERANCE) or (obstacle_reported and is_obstacle_currently_detected):
        print("Goal reached or obstacle handled, requesting next")
        
        send_status(ser, obstacle_reported, x, y, phi)

        if ENABLE_ANGLE_TRACKING:
            print(f"Total degrees turned: {total_degrees_turned:.2f} degrees")
            
        # Wait for response from microcontroller, keeping motors at 0
        x_new, y_new, failed = wait_for_new_goal(robot, ser, {'left': leftMotor, 'right': rightMotor}, timestep)
        if not failed:
            x_goal, y_goal = x_new, y_new
            obstacle_reported = False
        else:
            break

    else:
        # Normal goal following
        leftMotor.setVelocity(current_wl)
        rightMotor.setVelocity(current_wr)

ser.close()
