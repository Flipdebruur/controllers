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

# e-puck Physical parameters for the kinematics model (constants)
R = 0.020    # radius of the wheels: 20.5mm [m]
D = 0.057    # distance between the wheels: 52mm [m]
# distance from the center of the wheels to the point of interest [m]
A = 0.05


#def get_robot_pose(self, u, w, x_old, y_old, phi_old, delta_t):
class Lab2Bot(Robot):
    def __init__(self):
        super().__init__()
        self.x = 0.0
        self.y = 0.0
        self.phi = 0.0
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

        self.encoder = []
        self.encoderNames = ['left wheel sensor', 'right wheel sensor']
        for i in range(2):
            self.encoder.append(self.getDevice(self.encoderNames[i]))
            self.encoder[i].enable(TIME_STEP)  # <-- Enable encoders
        self.oldEncoderValues = [enc.getValue() for enc in self.encoder]

        leftMotor = self.getDevice('left wheel motor')
        rightMotor = self.getDevice('right wheel motor')
        leftMotor.setPosition(float('inf'))
        rightMotor.setPosition(float('inf'))
        leftMotor.setVelocity(0.0)
        rightMotor.setVelocity(0.0)

    def update_encoders(self, delta_t):
        encoderValues = [enc.getValue() for enc in self.encoder]
        wl, wr = self.get_wheel_speeds(encoderValues, self.oldEncoderValues, delta_t)
        self.oldEncoderValues = encoderValues.copy()
        return wl, wr

    def get_wheel_speeds(self, encoderValues, oldEncoderValues, delta_t):
        delta_left = encoderValues[0] - oldEncoderValues[0]
        delta_right = encoderValues[1] - oldEncoderValues[1]
        wl = (delta_left / delta_t) * (2 * math.pi / 360)
        wr = (delta_right / delta_t) * (2 * math.pi / 360)
        return wl, wr

    def get_robot_speeds(self, wl, wr):
        u = (R * wl + R * wr) / 2.0
        w = (R * wl - R * wr) / D
        return u, w

    def update_pose(self, u, w, delta_t):
        self.x += u * delta_t * math.cos(self.phi)
        self.y += u * delta_t * math.sin(self.phi)
        self.phi += w * delta_t

    def get_pose(self):
        return self.x, self.y, self.phi

  
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

bot = Lab2Bot()
while bot.step(TIME_STEP) != -1:
    wl, wr = bot.update_encoders(delta_t)
    u, w = bot.get_robot_speeds(wl, wr)
    bot.update_pose(u, w, delta_t)
    x, y, phi = bot.get_pose()

    if any(math.isnan(val) for val in [x, y, phi, u, w, wl, wr]):
        print("NaN detected in pose or speeds!")


    print(f'Sim time: {bot.getTime():.3f}  Pose: x={x:.2f} m, y={y:.2f} m, phi={phi:.4f} rad.')

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