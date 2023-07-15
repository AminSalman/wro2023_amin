#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor, ColorSensor
from pybricks.parameters import Port, Button, Color
from pybricks.tools import wait, StopWatch


class Base:
    """
    This is the class responsible for movement. To use this class, you need to specify the left and right motors,
    the wheel circumference in cm.
    """

    def __init__(self, left_motor_port: Port,
                 right_motor_port: Port,
                 wheel_circumference: float,
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
        self.left_motor.run(-1 * leftSpeed)
        self.right_motor.run(rightSpeed)
        return

    def start_tank_dc(self, leftSpeed: int, rightSpeed: int):
        """
        This method gives you control of voltage of each motor separately. The base will start moving and will not stop
        automatically.
        :param leftSpeed: -100 to 100, negative for reverse
        :param rightSpeed: -100 to 100, negative for reverse
        """
        self.left_motor.dc(-1 *leftSpeed)
        self.right_motor.dc( rightSpeed)
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
                 gyroS: GyroSensor):
        self.robotBase = robotBase
        self.sensor = gyroS

    def turn(self, endAngle: int, speed: int, mode=1):
        """
        This method turns the robot to a specific angle using the gyro sensor. If the robot doesn't turn accurately
        try reconnecting the gyro sensor or restarting the program while the robot is perfectly still. :param
        endAngle: The angle to turn to :param speed: Maximum speed of the wheel motors :param mode: 1 for
        pivoting around the center, 0 for pivoting around a wheel :return: nothing
        """
        if mode not in [0, 1]:
            return       
        for i in [1,2]:
            start_angle = self.sensor.angle()
            
            if endAngle > start_angle:
                while endAngle > self.sensor.angle():                
                    if abs(start_angle - self.sensor.angle()) > 15:
                        self.robotBase.start_tank(speed, -speed * mode)
                    else:
                        self.robotBase.start_tank(90, -90 * mode)

            elif endAngle < start_angle:
                while endAngle < self.sensor.angle():
                    if abs(start_angle - self.sensor.angle()) > 15:
                        self.robotBase.start_tank(-speed * mode, speed)
                    else:
                        self.robotBase.start_tank(-90 * mode, 90)

            self.robotBase.stop_and_hold()
            EV3Brick().screen.draw_text(70, 70, str(gyro.sensor.angle()))
        return

    def straight_pid(self, distance_cm, speed, targetAngle):
        """
        The robot moves perfectly straight using the gyro sensor and a PID algorithm for a specific distance
        :param distance_cm: Distance to move in cm before stopping
        :param speed: Maximum speed of the wheel motors
        :param targetAngle: Angle to maintain while moving, works best if you turn to that angle first
        """
        pid, integral, d, error, last_error = 0, 0, 0, 0, 0
        kp, ki, kd = 3.5, 0, 0
        speed = abs(speed)
        captured_angles = self.robotBase.capture_motor_angles()
        distance_degrees = (abs(distance_cm) / self.robotBase.wheel_circumference) * 360
        while self.robotBase.get_avg_motor_deg(captured_angles) < distance_degrees:
            error = self.sensor.angle() - targetAngle
            integral += error
            d = error - last_error
            pid = (kp * error) + (ki * integral) + (kd * d)
            self.robotBase.start_tank_dc(speed - pid, speed + pid)
            last_error = error
            EV3Brick().screen.draw_text(70, 70, str(self.sensor.angle()))
            print(str(self.sensor.angle()))
            wait(10)
            
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
        self.left_sensor = ColorSensor(left_sensor_port)
        self.right_sensor = ColorSensor(right_sensor_port)
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
        pi, i, error = 0, 0, 0
        while self.black_state() != 3:            
            error = (self.left_sensor.reflection() - self.right_sensor.reflection())
            i += error
            pi = (error) + (0.01 * i)
            self.robotBase.start_tank_dc(int(speed - pi), int(speed + pi))                
            wait(10)
        self.robotBase.start_tank_dc(-7, -7)
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
            while self.white_state() != 1:
                pass
            while self.white_state() not in [3, 2]:
                pass            
                    

        elif direction == "right":
            self.robotBase.start_tank(speed, -1 * mode * speed)
            while self.white_state() != 2:
                pass
            while self.white_state() not in [3, 1]:
                pass            
        
        while self.white_state() != 3:
                pass       
        self.robotBase.stop_and_hold()
        return


class Arm:
    def __init__(self, claw_port, lift_port):
        self.claw = Motor(claw_port)
        self.lever = Motor(lift_port)
        self.lever.reset_angle(0)
        self.claw.reset_angle(0)

    def pinch(self, keep_running=0):
        self.claw.run_until_stalled(1000)
        if keep_running:               
            self.claw.dc(50)
            return

    def release(self, custom_speed=500):        
        self.claw.run_until_stalled(-custom_speed)

    def lower(self, keep_running=0):
        self.lever.run_until_stalled(1000)
        if keep_running:
            self.lever.dc(50)


    def lift(self, keep_running=0):
        self.lever.run_until_stalled(-1000)
        if keep_running:
            self.lever.dc(-40)



# Initialize objects:
base = Base(left_motor_port=Port.D, right_motor_port=Port.B, wheel_circumference=19.6)
arm = Arm(lift_port=Port.A, claw_port=Port.C)
gyro = GyroController(base, gyroS=GyroSensor(Port.S4))
line = LineController(base, Port.S2, Port.S3, black_threshold=15, white_threshold=50)



# Wait for center button to be pressed:
EV3Brick().light.on(Color.ORANGE)
EV3Brick().screen.draw_text(30, 50, "press center")
while EV3Brick().buttons.pressed() != [Button.CENTER]:
    pass
EV3Brick().screen.clear()
EV3Brick().light.on(Color.GREEN)

StopWatch().reset()  # Start timer and run:


gyro.sensor.reset_angle(0)
EV3Brick().screen.draw_text(70, 70, str(gyro.sensor.angle()))       
# Actual Run Starts Here:

base.move_cm(5, -1000)

gyro.straight_pid(47, targetAngle=0, speed=43)
base.move_cm(distance_in_cm=42, speed=-200)


gyro.turn(34, speed = 130)
gyro.straight_pid(84.5, targetAngle=34, speed=40)
gyro.turn(0, speed = 130)
base.move_cm(distance_in_cm=2, speed=-200)

arm.pinch(keep_running=True)
arm.lift(keep_running=True)
base.move_cm(distance_in_cm=-6, speed=-200)
gyro.turn(-90, speed=130)
gyro.straight_pid(34, 45, -90)
arm.release(custom_speed=100)
base.move_cm(13, -200)
arm.claw.run_angle(190, 80, wait=False)
arm.lower()
gyro.straight_pid(12, targetAngle=-90, speed=45)
arm.pinch()
arm.claw.dc(60)
base.start_tank_dc(-100, 100)
wait(100)
gyro.turn(-260, 900)
gyro.straight_pid(30, targetAngle=-260, speed=65)
gyro.straight_pid(130, targetAngle=-270, speed=65)

#base.move_cm(20, -400)
#base.start_tank_dc(10, 80)
#while gyro.sensor.angle() > -265:
#    pass
#base.stop_and_hold()
#
while True:
    pass


#while True:
#    pass
#arm.release()
#arm.pinch(keep_running=1)
#arm.lift()
#wait(3000)
#arm.release()
#arm.lower()

