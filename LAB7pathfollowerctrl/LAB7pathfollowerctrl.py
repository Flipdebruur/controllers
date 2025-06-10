"""line_following_with_HIL controller."""
# This program implements Hardware-in-the-Loop simulation of the e-puck robot.
# Ground sensor data is pre-processed and transfered to an external board 
# that must decide the next state of the robot. Communication between Webots
# and the external board is implemented via Serial port.

# Tested on Webots R2023a, on Windows 11 running Python 3.10.5 64-bit
# communicating with MicroPython v1.25.0 on generic ESP32 module with ESP32

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
goal_recieved = False
x_goal, y_goal = x, y  # Initialize with current position
distance = 0.0

# Odometry parameters (place at the top of your file)
R = 0.020    # wheel radius [m]
D = 0.057    # distance between wheels [m]
delta_t = timestep / 1000.0
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
def build_message(Obstacle, x, y, phi):
    message = ''
    if Obstacle:
        message += 'O'
    else:
        message += 'N'
    
    message += f',{x:.3f},{y:.3f},{phi:.3f}'
    return bytes(message + '\n', 'UTF-8')
def go_to_goal(x, y, phi, x_goal, y_goal, R, D, MAX_SPEED):
    print(f"going to {x_goal}, {y_goal}")
    dx = x_goal - x
    dy = y_goal - y
    distance = math.hypot(dx, dy)

    angle_to_goal = math.atan2(dy, dx)
    angle_error = angle_to_goal - phi
    angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

    K_v = 3.0
    K_w = 8.0

    if distance > 0.03:  # TOLERANCE
        v = K_v * distance
        w = K_w * angle_error
    else:
        v = 0.0
        w = 0.0

    wl = (v - w * D / 2) / R
    wr = (v + w * D / 2) / R

    # Saturate wheel speeds
    wl = max(-MAX_SPEED, min(MAX_SPEED, wl))
    wr = max(-MAX_SPEED, min(MAX_SPEED, wr))
    #print(f"wl= {wl:.2f} wr= {wr:.2f} distance= {distance:.3f} angle_error= {angle_error:.4f} v= {v:.2f} w= {w:.2f} dx= {dx:.5f} dy= {dy:.5f}")
    return wl, wr, distance
robot.step(timestep)
encoderValues = [enc.getValue() for enc in encoder]
oldEncoderValues = encoderValues.copy()
while robot.step(timestep) != -1:
    robot.step(timestep)
    # Read proximity sensors values
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())
    Obstacle = psValues[0] > 600 or psValues[1] > 600 or psValues[2] > 600 or psValues[3] > 600

    encoderValues = [enc.getValue() for enc in encoder]
    wl, wr = get_wheels_speed(encoderValues, oldEncoderValues, delta_t)
    u, w = get_robot_speeds(wl, wr, R, D)
    x, y, phi = get_robot_pose(u, w, x, y, phi, delta_t)
    oldEncoderValues = encoderValues.copy()
    print(f"Odometry pose: x={x:.3f} m, y={y:.3f} m, phi={phi:.3f} rad")

    if distance < GOAL_TOLERANCE:
        print("🎯 Goal reached, requesting next")
        status = 'O' if Obstacle else 'N'
        message = f"{status},{x:.3f},{y:.3f},{phi:.3f}\n"
        ser.write(message.encode())
        ser.flush()
        # Wait for response
        print(f"Pose: x={x:.2f}, y={y:.2f}, phi={phi:.2f} | Goal: x={x_goal:.2f}, y={y_goal:.2f} | Distance: {distance:.3f}")
        print("wait for response")
        while ser.in_waiting == 0:
            robot.step(timestep)
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
        print("read response")
        data = ser.read(8)  # 2 floats = 8 bytes
        if len(data) == 8:
            x_goal, y_goal = struct.unpack('<ff', data)
            print(f"🎯 New goal: x={x_goal}, y={y_goal}")
        else:
            print("⚠️ Incomplete binary data")


    wl, wr, distance = go_to_goal(x, y, phi, x_goal, y_goal, D, R, MAX_SPEED)        

    leftSpeed = wl
    rightSpeed = wr

    # Update velocity commands for the motors
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)

ser.close()
