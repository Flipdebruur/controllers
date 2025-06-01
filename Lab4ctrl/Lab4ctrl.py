"""Lab2ctrl controller."""

# Import the controller module from Webots.
from controller import Robot, Motor
import math 
from math import pi


TIME_STEP = 64
MAX_SPEED = 6.28
lost = False
LineFront = False
LineLeft = False
LineRight = False

delta_t = TIME_STEP/1000.0 

# e-puck Physical parameters for the kinematics model (constants)
R = 0.020    # radius of the wheels: 20.5mm [m]
wheelbase = 0.057    # distance between the wheels: 52mm [m]
# distance from the center of the wheels to the point of interest [m]
A = 0.05

# Initialize pose and encoders
x, y, phi = -0.131686, 0.44569, 0.0

# Initialize desired pose
GOAL_TRIGGER_X = 0.187721
GOAL_TRIGGER_Y = 0.445134
GOAL_TRIGGER_RADIUS = 0.05
GOALS = [
    (0.46, 0.3),
    (0.46036, -0.249617),
    (-0.767044, -0.494492),
    (-0.828894, 0.523632),
    (0.0, 0.0)  # Center of the field
]
xd, yd = GOALS[0]
current_goal = 0        

# PID gains (tune these for your robot)
kp = 12.0
ki = 0.0
kd = 0.0

dist_err = 0.0
phi_err = 0.0
go_to_goal_mode = False
e_prev = 0.0   # Previous error
e_acc = 0.0    # Accumulated error (integral)

def get_wheels_speed(encoderValues, oldEncoderValues, delta_t):
    """Computes speed of the wheels based on encoder readings"""
    wl = (encoderValues[0] - oldEncoderValues[0]) / delta_t  # left wheel speed [rad/s]
    wr = (encoderValues[1] - oldEncoderValues[1]) / delta_t  # right wheel speed [rad/s]
    return [wl, wr]

def get_robot_speeds(wl, wr, r, wheelbase):
    """Computes robot linear and angular speeds"""
    u = (r / 2) * (wl + wr)  # linear speed [m/s]
    w = (r / wheelbase) * (wr - wl)  # angular speed [rad/s]
    return [u, w]

def get_robot_pose(u, w, x_old, y_old, phi_old, delta_t):
    """Updates robot pose based on heading and linear and angular speeds"""
    x = x_old + u * math.cos(phi_old) * delta_t
    y = y_old + u * math.sin(phi_old) * delta_t
    phi = phi_old + w * delta_t
    phi = (phi + math.pi) % (2 * math.pi) - math.pi
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
        print("Searching for line...")
        self.read_gsensors()
        self.LineFront = False
        self.LineLeft = False
        self.LineRight = False
        self.lost = False

        threshold = 400.0
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
        print("Obstacle detected on the Left.")
        duration = 0.3  # seconds
        steps = int(duration * 1000 / TIME_STEP)
        for _ in range(steps):
            self.moveright()
            if self.step(TIME_STEP) == -1:
                break
        self.forward()

    def backward(self):
        duration = 0.2
        steps = int(duration * 1000 / TIME_STEP)
        for _ in range(steps):
            self.left_motor.setVelocity(-MAX_SPEED)
            self.right_motor.setVelocity(-MAX_SPEED)
            if self.step(TIME_STEP) == -1:
                break

    def obstacle_avoidanceR(self):
        print("Obstacle detected on the Right.")
        duration = 0.2  # seconds
        steps = int(duration * 1000 / TIME_STEP)
        for _ in range(steps):
            self.backward()
            self.moveleft()
            if self.step(TIME_STEP) == -1:
                break     

    def moveleft(self):
        duration = 0.1  # seconds
        steps = int(duration * 1000 / TIME_STEP)
        for _ in range(steps):
            self.left_motor.setVelocity(-0.1 * MAX_SPEED)
            self.right_motor.setVelocity(0.1 * MAX_SPEED)
            if self.step(TIME_STEP) == -1:
                break
        
    def moveright(self):
        duration = 0.1  # seconds
        steps = int(duration * 1000 / TIME_STEP)
        for _ in range(steps):
            self.left_motor.setVelocity(0.1 * MAX_SPEED)
            self.right_motor.setVelocity(-0.1 * MAX_SPEED)
            if self.step(TIME_STEP) == -1:
                break
    
    def position_error(self, x, y, phi, xd, yd):
        x_err = xd - x
        y_err = yd - y
        dist_err = math.sqrt(x_err**2 + y_err**2)
        phi_d = math.atan2(y_err, x_err)
        phi_err = phi_d - phi
        phi_err_correct = math.atan2(math.sin(phi_err), math.cos(phi_err))  # Normalize angle
        return dist_err, phi_err_correct

    def wheel_speed_commands(self, u_d, w_d, wheelbase, r, MAX_SPEED):
        """Converts desired linear/angular speeds to wheel speed commands with saturation."""
        wr_d = float((2 * u_d + wheelbase * w_d) / (2 * r))
        wl_d = float((2 * u_d - wheelbase * w_d) / (2 * r))
        # If saturated, correct speeds to keep the original ratio
        if abs(wl_d) > MAX_SPEED or abs(wr_d) > MAX_SPEED:
            speed_ratio = abs(wr_d) / abs(wl_d) if abs(wl_d) != 0 else float('inf')
            if speed_ratio > 1:
                wr_d = math.copysign(MAX_SPEED, wr_d)
                wl_d = math.copysign(MAX_SPEED / speed_ratio, wl_d)
            else:
                wl_d = math.copysign(MAX_SPEED, wl_d)
                wr_d = math.copysign(MAX_SPEED * speed_ratio, wr_d)
        return wl_d, wr_d

    def go_to_goal(self, x, y, phi, xd, yd, kp, ki, kd, e_prev, e_acc, delta_t):
        self.position_error(x, y, phi, xd, yd)
        dist_err, phi_err_correct = self.position_error(x, y, phi, xd, yd)
        print(f"[DEBUG] Current pose: x={x:.3f}, y={y:.3f}, phi={phi:.3f}")
        print(f"[DEBUG] Goal pose: xd={xd:.3f}, yd={yd:.3f}")
        print(f"[DEBUG] Orientation error (phi_err): {phi_err:.4f} rad, Distance error: {dist_err:.4f} m")
        e = phi_err_correct
        e_acc = e_acc + e * delta_t  # Proper integral accumulation

        # PID controller for heading
        P = kp * e
        I = ki * e_acc
        D = kd * (e - e_prev) / delta_t
        w = P + I + D
        e_prev = e
        e_acc = I
        if abs(phi_err_correct) > 0.05:
            v = 0.0
            print(f"[DEBUG] Turning in place to face goal... phi_err_correct={phi_err_correct:.3f}")
        else:
            v = min(0.8, dist_err * 1.5)
            print(f"[DEBUG] Facing goal, moving forward... phi_err_correct={phi_err_correct:.3f}")

        u_d = v
        w_d = w
        # Differential drive inverse kinematics
        left_speed, right_speed = self.wheel_speed_commands(u_d, w_d, wheelbase, R, MAX_SPEED)
        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)
        #print(f"PID: P={P:.3f}, I={I:.3f}, D={D:.3f}, w={w:.3f}, v={v:.3f}, left_speed={left_speed:.3f}, right_speed={right_speed:.3f}")
        return dist_err, phi_err, e, e_acc  # return updated errors for next iteration 
        

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
    right_obstacle = psValues[0] > 150 or psValues[1] > 150 or psValues[2] > 150
    left_obstacle = psValues[5] > 150 or psValues[6] > 150 or psValues[7] > 150
    if not go_to_goal_mode and abs(x - GOAL_TRIGGER_X) < GOAL_TRIGGER_RADIUS and abs(y - GOAL_TRIGGER_Y) < GOAL_TRIGGER_RADIUS:
        go_to_goal_mode = True
    
    if go_to_goal_mode:
        print("Entering go-to-goal mode.")
        if right_obstacle:
            bot.obstacle_avoidanceR()
            
        elif left_obstacle:
            bot.obstacle_avoidanceL()

        # Use go-to-goal PID controller
        dist_err, phi_err, e_prev, e_acc = bot.go_to_goal(x, y, phi, xd, yd, kp, ki, kd, e_prev, e_acc, delta_t)
        print(f"Go-to-goal: Distance error: {dist_err:.3f} m, Orientation error: {phi_err:.3f} rad")
        # Optionally, exit go-to-goal mode when close enough to the target
        if dist_err < 0.05:
            # Stop at goal
            bot.left_motor.setVelocity(0)
            bot.right_motor.setVelocity(0)
            print(f"Reached goal: x={x:.2f}, y={y:.2f}, dist_err={dist_err:.3f}")
            bot.step(int(1000 / TIME_STEP) * TIME_STEP * 1)  # Wait ~1 second

            current_goal += 1
            if current_goal < len(GOALS):
                print("next goal:")
                xd, yd = GOALS[current_goal]
            else:
                go_to_goal_mode = False  # All goals reached
                # Stop robot at final goal
                bot.left_motor.setVelocity(0)
                bot.right_motor.setVelocity(0)
                print("All goals reached. Robot stopped.")
                break
    else:

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
    #print(f"Encoder values: {encoderValues}")
    #print(f"Old encoder values: {oldEncoderValues}")
    #print(f"delta_t: {delta_t}")
    # Compute speed of the wheels
    [wl, wr] = get_wheels_speed(encoderValues, oldEncoderValues, delta_t)

    # Compute robot linear and angular speeds
    [u, w] = get_robot_speeds(wl, wr, R, wheelbase)

    # Compute new robot pose
    [x, y, phi] = get_robot_pose(u, w, x, y, phi, delta_t)
    #print(f"Odometry pose: x={x:.2f} m, y={y:.2f} m, phi={phi:.4f} rad.")
    # Update old encoder values for next cycle
    oldEncoderValues = encoderValues.copy()