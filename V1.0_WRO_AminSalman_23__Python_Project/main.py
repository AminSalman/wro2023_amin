#!f/usr/bin/env pybricks-micropython
import pybricks.parameters
from pybricks.hubs import EV3Brick
from pybricks.nxtdevices import LightSensor
from pybricks.ev3devices import Motor, GyroSensor  # ColorSensor
from pybricks.parameters import Port, Button, Number
from pybricks.tools import wait, StopWatch


class Base:
    """
    This is the class responsible for movement. To use this class, you need to specify the left and right motors,
    the wheel circumference, and the distance between the wheels in cm.
    """

    def __init__(self, left_motor_port: Port,
                 right_motor_port: Port,
                 wheel_circumference: Number,
                 ):
        self.left_motor = Motor(left_motor_port)
        self.right_motor = Motor(right_motor_port)
        self.wheel_circumference = wheel_circumference

    def start_tank(self, leftSpeed: int, rightSpeed: int):
        """
        This method gives you control of the speed of each motor separately. The base will start moving and will not
        stop automatically.

        :param leftSpeed: -100 to 100, negative for reverse
        :param rightSpeed: -100 to 100, negative for reverse
        """
        self.left_motor.run(leftSpeed)
        self.right_motor.run(-1 * rightSpeed)
        return

    def start_tank_dc(self, leftSpeed: int, rightSpeed: int):
        """
        This method gives you control of voltage of each motor separately. The base will start moving and will not stop
        automatically.
        :param leftSpeed: -100 to 100, negative for reverse
        :param rightSpeed: -100 to 100, negative for reverse
        """
        self.left_motor.dc(leftSpeed)
        self.right_motor.dc(-1 * rightSpeed)
        return

    def stop_and_hold(self):
        """
        Instantly stops the robot and holds the motors at current position.
        """
        self.left_motor.hold()
        self.right_motor.hold()
        return

    def reset_angles(self):
        """
        Resets the angles of the motors
        :return: nothing
        """
        self.left_motor.reset_angle(0)
        self.right_motor.reset_angle(0)
        return

    def get_avg_motor_deg(self, captured_motor_angles):
        """
        Returns the average number of the wheel motors degrees based on the initial angles' parameter.
        :return: int
        """
        return int((abs(int(self.right_motor.angle()) - captured_motor_angles[1]) +
                    abs(int(self.left_motor.angle()) - captured_motor_angles[0])) / 2)

    def capture_motor_angles(self):
        """
        Capture the left and right motor angles as a list of 2 items.
        :returns a list containing the left motor angle and the right motor angle [int, int]
        """
        return [int(self.left_motor.angle()), int(self.right_motor.angle())]

    def move_cm(self, distance_in_cm: float, speed: int):
        """
        Starts moving at the specified speed and stops when the distance specified is reached.
        :param distance_in_cm: Positive Integer
        :param speed: -100 to 100
        """

        self.start_tank(speed, speed)
        captured_angles = self.capture_motor_angles()
        distance_in_degrees = abs(int((distance_in_cm / self.wheel_circumference) * 360))
        while self.get_avg_motor_deg(captured_angles) < distance_in_degrees:
            pass
        self.stop_and_hold()
        return

    def start_moving(self, speed: int):
        """
        Starts moving at the specified speed.
        :param speed: -100 to 100
        """
        self.start_tank(speed, speed)
        return


class GyroController:
    """
    This class is responsible for any action that utilises the gyro sensor, such as turning accurately and moving
    straight accurately.
    """

    def __init__(self, robotBase: Base,
                 gyroPort: Port):
        self.robotBase = robotBase
        self.sensor = GyroSensor(gyroPort)

    def turn(self, endAngle: int, speed: int, mode=1):
        """
        This method turns the robot to a specific angle using the gyro sensor. If the robot doesn't turn accurately
        try reconnecting the gyro sensor or restarting the program while the robot is perfectly still. :param
        endAngle: The angle to turn to :param speed: Maximum speed of the wheel motors :param mode: 1 for
        pivoting around the center, 0 for pivoting around a wheel :return: nothing
        """
        if mode not in [0, 1]:
            return

        start_angle = self.sensor.angle()

        if endAngle > start_angle:
            while endAngle > self.sensor.angle():
                if abs(start_angle - self.sensor.angle()) > 15:
                    self.robotBase.start_tank(speed, -speed * mode)
                else:
                    self.robotBase.start_tank(15, -15 * mode)

        elif endAngle < start_angle:
            while endAngle < self.sensor.angle():
                if abs(start_angle - self.sensor.angle()) > 15:
                    self.robotBase.start_tank(-speed * mode, speed)
                else:
                    self.robotBase.start_tank(-15 * mode, 15)

        self.robotBase.stop_and_hold()
        return

    def straight_pid(self, distance_cm, speed, targetAngle):
        """
        The robot moves perfectly straight using the gyro sensor and a PID algorithm for a specific distance
        :param distance_cm: Distance to move in cm before stopping
        :param speed: Maximum speed of the wheel motors
        :param targetAngle: Angle to maintain while moving, works best if you turn to that angle first
        """
        pid, integral, d, error, last_error = 0, 0, 0, 0, 0
        kp, ki, kd = 0.5, 0.002, 3
        speed = abs(speed)
        captured_angles = self.robotBase.capture_motor_angles()
        distance_degrees = (abs(distance_cm) / self.robotBase.wheel_circumference) * 360
        while self.robotBase.get_avg_motor_deg(captured_angles) < distance_degrees:
            error = self.sensor.angle() - targetAngle
            integral += error
            pid = (kp * error) + (ki * integral) + (kd * d)
            self.robotBase.start_tank_dc(speed + pid, speed - pid)
        self.robotBase.stop_and_hold()


class LineController:
    """
    This class is responsible for any action that utilises the color sensors, such as line following and squaring.
    """

    def __init__(self, robotBase: Base,
                 left_sensor_port: Port,
                 right_sensor_port: Port,
                 black_threshold: int,
                 white_threshold: int,
                 ):
        self.robotBase = robotBase
        self.left_sensor = LightSensor(left_sensor_port)
        self.right_sensor = LightSensor(right_sensor_port)
        self.white_thres = white_threshold
        self.black_thres = black_threshold

    def black_state(self):
        """
        Gets the current state of the sensors: 0 for none black, 1 for right sensor black,
        2 for left sensor black, 3 for both black
        :return: 0, 1, 2, 3
        """
        s1 = (int(self.left_sensor.reflection() < self.black_thres) * 2)
        s2 = (int(self.right_sensor.reflection() < self.black_thres) * 1)
        return s1 + s2

    def white_state(self):
        """
        Gets the current state of the sensors: 0 for none white, 1 for right sensor white,
        2 for left sensor white, 3 for both white
        :return: 0, 1, 2, 3
        """
        s1 = (int(self.left_sensor.reflection() > self.white_thres) * 2)
        s2 = (int(self.right_sensor.reflection() > self.white_thres) * 1)
        return s1 + s2

    def stop_at_joint(self, speed: int):
        """
        Starts line following using a PID algorithm. The robot stops at the first intersection it senses.
        :param speed: The maximum speed of the robot -100% to 100%
        :return:(nothing)
        """
        pd, last_error, error = 0, 0, 0
        while self.black_state() != 3:
            error = (self.left_sensor.reflection() - self.right_sensor.reflection())
            pd = (0.25 * error) + (5 * (error - last_error))
            self.robotBase.start_tank_dc(int(speed - pd), int(speed + pd))
            last_error = error
            wait(10)
        self.robotBase.stop_and_hold()
        return

    def follow_cm(self, speed: int, distance_cm: float):
        """
        Starts line following using a PID algorithm. The robot stops when it has travelled a specific distance.
        :param speed: The maximum speed of the robot -100% to 100%
        :param distance_cm:
        :return:(nothing)
        """
        distance_cm = abs(distance_cm)
        speed = abs(speed)
        pd, last_error, error = 0, 0, 0
        captured_angles = self.robotBase.capture_motor_angles()
        distance_degrees = (distance_cm / self.robotBase.wheel_circumference) * 360
        while self.robotBase.get_avg_motor_deg(captured_angles) < distance_degrees:
            error = (self.left_sensor.reflection() - self.right_sensor.reflection())
            pd = (0.25 * error) + (5 * (error - last_error))
            self.robotBase.start_tank_dc(int(speed - pd), int(speed + pd))
            last_error = error
            wait(15)
        self.robotBase.stop_and_hold()
        return

    def turn_to_line(self, direction: str, speed: int, mode: int):
        """
        Starts turning until a line is detected. Could be used before PID line following.
        :param direction: "left" or "right"
        :param speed:  Maximum speed of the wheel motors
        :param mode: 1 for pivoting around the center, 0 for pivoting around a wheel
        :return:
        """
        if (mode not in [0, 1]) or (direction not in ["left", 'right']):
            return
        speed = abs(speed)
        if direction == "left":
            self.robotBase.start_tank(-1 * mode * speed, speed)
            while self.white_state() != 2:
                pass
            while self.white_state() not in [3, 1]:
                pass
        elif direction == "right":
            self.robotBase.start_tank(speed, -1 * mode * speed)
            while self.white_state() != 1:
                pass
            while self.white_state() not in [3, 2]:
                pass
        self.robotBase.stop_and_hold()
        return


class Arm:
    def __init__(self, claw_port, lift_port):
        self.claw = Motor(claw_port)
        self.lift = Motor(lift_port)

    def pinch(self, keep_running=0):
        self.claw.run_angle(20, 100)
        if keep_running:
            self.claw.run(10)

    def release(self):
        self.claw.run_angle(-20, 100)

    def lift(self, custom_angle=70):
        self.lift.run_angle(20, custom_angle)

    def lower(self, custom_angle=70):
        self.lift.run_angle(-20, custom_angle)


# Initialize objects:
base = Base(left_motor_port=Port.A, right_motor_port=Port.B, wheel_circumference=4)
arm = Arm(lift_port=Port.A, claw_port=Port.B)
gyro = GyroController(base, gyroPort=Port.S3)
line = LineController(base, Port.S1, Port.S2, black_threshold=10, white_threshold=55)

# Wait for center button to be pressed:
EV3Brick.light.on(pybricks.parameters.Color.ORANGE)
while EV3Brick().buttons.pressed() != [Button.CENTER]:
    EV3Brick().screen.draw_text(50, 80, "press center")
EV3Brick().screen.clear()
EV3Brick.light.on(pybricks.parameters.Color.GREEN)

StopWatch().reset()  # Start timer and run:

# Actual Run Starts Here:

base.move_cm(30, 20)
line.follow_cm(20, 40)
gyro.turn(60, 15)
gyro.straight_pid(20, 30, 60)
gyro.turn(-90, 20)
