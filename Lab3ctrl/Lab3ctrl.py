"""Lab2ctrl controller."""

# Import the controller module from Webots.
from controller import Robot, Motor
from math import pi
import math 

TIME_STEP = 64
MAX_SPEED = 6.28
lost = False
LineFront = False
LineLeft = False
LineRight = False

delta_t = TIME_STEP/1000.0 

# Initialize pose and encoders
x, y, phi = 0.04, 0.44, 0.0

# e-puck Physical parameters for the kinematics model (constants)
R = 0.020    # radius of the wheels: 20.5mm [m]
D = 0.057    # distance between the wheels: 52mm [m]
# distance from the center of the wheels to the point of interest [m]
A = 0.05



def get_wheels_speed(encoderValues, oldEncoderValues, delta_t):
    """Computes speed of the wheels based on encoder readings"""
    wl = (encoderValues[0] - oldEncoderValues[0]) / delta_t  # left wheel speed [rad/s]
    wr = (encoderValues[1] - oldEncoderValues[1]) / delta_t  # right wheel speed [rad/s]
    return [wl, wr]


def get_robot_speeds(wl, wr, r, d):
    """Computes robot linear and angular speeds"""
    u = (r / 2) * (wl + wr)  # linear speed [m/s]
    w = (r / d) * (wr - wl)  # angular speed [rad/s]
    return [u, w]



def get_robot_pose(u, w, x_old, y_old, phi_old, delta_t):
    """Updates robot pose based on heading and linear and angular speeds"""
    x = x_old + u * math.cos(phi_old) * delta_t
    y = y_old + u * math.sin(phi_old) * delta_t
    phi = phi_old + w * delta_t
    return [x, y, phi]

class Lab2Bot(Robot):
    def __init__(self):
        super().__init__()
        self.left_motor = self.getDevice("left wheel motor")
        self.right_motor = self.getDevice("right wheel motor")
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        self.ps = [self.getDevice(f"ps{i}") for i in range(8)]
        self.psNames = [
        'ps0', 'ps1', 'ps2', 'ps3',
        'ps4', 'ps5', 'ps6', 'ps7'
        ]
        self.gs = [self.getDevice(f"gs{i}") for i in range(3)]
        self.gsNames = ['gs0', 'gs1', 'gs2']
        for sensor in self.ps:
            sensor.enable(TIME_STEP)  # <-- Enable proximity sensors
        self.gs = [self.getDevice(f"gs{i}") for i in range(3)]
        for sensor in self.gs:
            sensor.enable(TIME_STEP)  # <-- Enable ground sensors

  
    def read_psensors(self):
        self.psValues = [sensor.getValue() for sensor in self.ps]

    def read_gsensors(self):
        self.gsValues = [sensor.getValue() for sensor in self.gs]
        gsValues = self.gsValues
        self.line_right = gsValues[0]
        self.line_center = gsValues[1]
        self.line_left = gsValues[2]

    def Line_search(self):
        self.read_gsensors()
        self.LineFront = False
        self.LineLeft = False
        self.LineRight = False
        self.lost = False

        threshold = 500.0
        center_on = self.line_center < threshold
        left_on = self.line_left < threshold
        right_on = self.line_right < threshold

        # Only left sensor on
        if left_on and not center_on and not right_on:
            self.LineLeft = True
        # Only right sensor on
        elif right_on and not center_on and not left_on:
            self.LineRight = True
        # Only center sensor on
        elif center_on and not left_on and not right_on:
            self.LineFront = True
        # All sensors off
        elif not center_on and not left_on and not right_on:
            self.lost = True
        # If multiple sensors are on (e.g., left+center), prioritize forward
        elif center_on:
            self.LineFront = True

    def forward(self):
        duration = 0.3  # seconds
        steps = int(duration * 1000 / TIME_STEP)
        for _ in range(steps):
            self.left_motor.setVelocity(MAX_SPEED)
            self.right_motor.setVelocity(MAX_SPEED)
            if self.step(TIME_STEP) == -1:
                break

    def obstacle_avoidanceL(self):
        duration = 0.3  # seconds
        steps = int(duration * 1000 / TIME_STEP)
        for _ in range(steps):
            self.moveright()
            if self.step(TIME_STEP) == -1:
                break
        self.forward()

    def obstacle_avoidanceR(self):
        duration = 0.3  # seconds
        steps = int(duration * 1000 / TIME_STEP)
        for _ in range(steps):
            self.moveleft()
            if self.step(TIME_STEP) == -1:
                break     

    def moveleft(self):
        duration = 0.1  # seconds
        steps = int(duration * 1000 / TIME_STEP)
        for _ in range(steps):
            self.left_motor.setVelocity(0.1 * MAX_SPEED)
            self.right_motor.setVelocity(-0.1 * MAX_SPEED)
            if self.step(TIME_STEP) == -1:
                break
        
    def moveright(self):
        duration = 0.1  # seconds
        steps = int(duration * 1000 / TIME_STEP)
        for _ in range(steps):
            self.left_motor.setVelocity(-0.1 * MAX_SPEED)
            self.right_motor.setVelocity(0.1 * MAX_SPEED)
            if self.step(TIME_STEP) == -1:
                break
# Initialize robot and devices
bot = Lab2Bot()
encoder = []
encoderNames = ['left wheel sensor', 'right wheel sensor']
for i in range(2):
    encoder.append(bot.getDevice(encoderNames[i]))
    encoder[i].enable(TIME_STEP)

bot.step(TIME_STEP)
encoderValues = [enc.getValue() for enc in encoder]
oldEncoderValues = encoderValues.copy()

while bot.step(TIME_STEP) != -1:
    bot.read_psensors()
    bot.Line_search()
    psValues = bot.psValues
    right_obstacle = psValues[0] > 80.0 or psValues[1] > 80.0 or psValues[2] > 80.0
    left_obstacle = psValues[5] > 80.0 or psValues[6] > 80.0 or psValues[7] > 80.0

    if right_obstacle:
        bot.obstacle_avoidanceL()
    elif left_obstacle:
        bot.obstacle_avoidanceR()
    elif bot.lost:
        bot.moveleft()
    elif bot.LineLeft:
        bot.moveleft()
    elif bot.LineRight:
        bot.moveright()
    else:
        bot.forward()

    # Update encoder values
    encoderValues = [enc.getValue() for enc in encoder]
    print(f"Encoder values: {encoderValues}")
    print(f"Old encoder values: {oldEncoderValues}")
    print(f"delta_t: {delta_t}")
    # Compute speed of the wheels
    [wl, wr] = get_wheels_speed(encoderValues, oldEncoderValues, delta_t)

    # Compute robot linear and angular speeds
    [u, w] = get_robot_speeds(wl, wr, R, D)

    # Compute new robot pose
    [x, y, phi] = get_robot_pose(u, w, x, y, phi, delta_t)
    if any(math.isnan(val) for val in [x, y, phi, wl, wr, u, w]):
        print("NaN detected in pose or speeds!")
        break
    print(f"Odometry pose: x={x:.2f} m, y={y:.2f} m, phi={phi:.4f} rad.")

    # Update old encoder values for next cycle
    oldEncoderValues = encoderValues.copy()