"""Lab2ctrl controller."""

# Import the controller module from Webots.
from controller import Robot, Motor

TIME_STEP = 64
MAX_SPEED = 6.28
lost = False
LineFront = False
LineLeft = False
LineRight = False

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
        """Read sensors. """
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
    bot.read_psensors()
    bot.Line_search()
    psValues = bot.psValues
    right_obstacle = psValues[0] > 80.0 or psValues[1] > 80.0 or psValues[2] > 80.0
    left_obstacle = psValues[5] > 80.0 or psValues[6] > 80.0 or psValues[7] > 80.0

    if right_obstacle:
        print("Obstacle detected: RIGHT")
        bot.obstacle_avoidanceL()
    elif left_obstacle:
        print("Obstacle detected: LEFT")
        bot.obstacle_avoidanceR()
    elif bot.lost:
        print("State: LOST - turning left to search")
        bot.moveleft()
    elif bot.LineLeft:
        print("State: LINE LEFT - correcting left")
        bot.moveleft()
    elif bot.LineRight:
        print("State: LINE RIGHT - correcting right")
        bot.moveright()
    else:
        print("State: FORWARD")
        bot.forward()

