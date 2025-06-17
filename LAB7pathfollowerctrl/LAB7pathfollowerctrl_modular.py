"""line_following_with_HIL controller."""
# This program implements Hardware-in-the-Loop simulation of the e-puck robot.
# Ground sensor data is pre-processed and transfered to an external board 
# that must decide the next state of the robot. Communication between Webots
# and the external board is implemented via Serial port.

# Tested on Webots R2023a, on Windows 11 running Python 3.10.5 64-bit
# communicating with MicroPython v1.25.0 on generic ESP32 module with ESP32

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

#-------------------------------------------------------
# Initialize variables


GOAL_TOLERANCE = 0.02  # meters

MAX_SPEED = 6.28
speed = 0.4 * MAX_SPEED
leftSpeed = 0.0
rightSpeed = 0.0
# Initial pose
x, y, phi = 0.5, -0.34, 0.0
x_goal, y_goal = 0.5, -0.13

def initialize():
    from controller import Robot  # safe to include here
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    # Proximity sensors
    ps = []
    psNames = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
    for name in psNames:
        sensor = robot.getDevice(name)
        sensor.enable(timestep)
        ps.append(sensor)

    # Motors
    leftMotor = robot.getDevice('left wheel motor')
    rightMotor = robot.getDevice('right wheel motor')
    leftMotor.setPosition(float('inf'))
    rightMotor.setPosition(float('inf'))
    leftMotor.setVelocity(0.0)
    rightMotor.setVelocity(0.0)

    # Encoders
    encoder = []
    encoderNames = ['left wheel sensor', 'right wheel sensor']
    for name in encoderNames:
        enc = robot.getDevice(name)
        enc.enable(timestep)
        encoder.append(enc)

    # Control gains
    K_v = 4.0
    K_w = 6.0

    # Odometry
    R = 0.020
    D = 0.057
    delta_t = timestep / 1000.0



    return (robot, timestep, ps, leftMotor, rightMotor, encoder,
            K_v, K_w, R, D, delta_t,
            GOAL_TOLERANCE, MAX_SPEED)
(robot, timestep, ps, leftMotor, rightMotor, encoder,
 K_v, K_w, R, D, delta_t,
 GOAL_TOLERANCE, MAX_SPEED) = initialize()

def get_wheels_speed(encoderValues, oldEncoderValues, delta_t):
    wl = (encoderValues[0] - oldEncoderValues[0]) / delta_t  # left wheel speed [rad/s]
    wr = (encoderValues[1] - oldEncoderValues[1]) / delta_t  # right wheel speed [rad/s]
    return [wl, wr]
def get_robot_speeds(wl, wr, r, d):
    u = (r / 2) * (wl + wr)  # linear speed [m/s]
    w = (r / d) * (wr - wl)  # angular speed [rad/s]
    return [u, w]
def get_robot_pose(u, w, x_old, y_old, phi_old, delta_t):
    x = x_old + u * math.cos(phi_old) * delta_t
    y = y_old + u * math.sin(phi_old) * delta_t
    phi = phi_old + w * delta_t
    return [x, y, phi]
def go_to_goal_control(x, y, phi, x_goal, y_goal, GOAL_TOLERANCE, K_v, K_w, D, R, MAX_SPEED):
    # === Go-to-goal control ===
    dx = x_goal - x
    dy = y_goal - y
    distance = math.hypot(dx, dy)

    # Simple proportional controller
    angle_to_goal = math.atan2(dy, dx)
    angle_error = angle_to_goal - phi
    # Normalize angle error to [-pi, pi]
    angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

    if distance > GOAL_TOLERANCE:
        v = K_v * distance
        w = K_w * angle_error
    else:
        v = 0.0
        w = 0.0

    # Convert to wheel speeds
    wl = (v - w * D / 2) / R
    wr = (v + w * D / 2) / R

    # Limit speeds
    wl = max(-MAX_SPEED, min(MAX_SPEED, wl))
    wr = max(-MAX_SPEED, min(MAX_SPEED, wr))
    return wl, wr, distance
def read_Psensor(ps):
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())
    Obstacle = psValues[0] > 600 or psValues[1] > 600 or psValues[2] > 600 or psValues[3] > 600
    return psValues, Obstacle
def build_message(Obstacle, x, y, phi):
    message = ''
    if Obstacle:
        message += 'O'
    else:
        message += 'N'
    message += f',{x:.3f},{y:.3f},{phi:.3f}'
    return bytes(message + '\n', 'UTF-8')
def odometry(encoder, oldEncoderValues, delta_t, R, D, x, y, phi):
    encoderValues = [enc.getValue() for enc in encoder]
    wl, wr = get_wheels_speed(encoderValues, oldEncoderValues, delta_t)
    u, w = get_robot_speeds(wl, wr, R, D)
    x, y, phi = get_robot_pose(u, w, x, y, phi, delta_t)
    return encoderValues, wl, wr, x, y, phi

robot.step(timestep)
encoderValues = [enc.getValue() for enc in encoder]
oldEncoderValues = encoderValues.copy()

while robot.step(timestep) != -1:
    ############################################
    #                  See                     #
    ############################################
    # Read proximity sensors values
    psValues, Obstacle = read_Psensor(ps)

    #odometry
    encoderValues, wl, wr, x, y, phi = odometry(encoder, oldEncoderValues, delta_t, R, D, x, y, phi)
    oldEncoderValues = encoderValues.copy()

    #send message
    msg_bytes = build_message(Obstacle, x, y, phi)

    leftSpeed, rightSpeed, distance = go_to_goal_control(x, y, phi, x_goal, y_goal, GOAL_TOLERANCE, K_v, K_w, D, R, MAX_SPEED)
    
    # recieve the message from the microcontroller
    # Serial communication: if something is received, then update the current state
    if ser.in_waiting > 0:
        try:
            line = ser.readline().decode('UTF-8').strip()
            parts = line.split(',')
            if len(parts) == 2:
                x_goal = float(parts[0])
                y_goal = float(parts[1])
                print(f"Received new goal: x={x_goal:.3f}, y={y_goal:.3f}")
            else:
                print("Received non-goal message:", line)
        except Exception as e:
            print("Serial read failed:", e)

    ############################################
    #                  Act                     #
    ############################################

    # Update velocity commands for the motors
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
   
    # Print sensor message and current state for debugging
    print(f"Pose: x={x:.2f}, y={y:.2f}, phi={phi:.2f} | Goal: x={x_goal:.2f}, y={y_goal:.2f} | Distance: {distance:.3f}")

    # Send message to the microcontroller 
    ser.write(msg_bytes)  

ser.close()
