#!/usr/bin/env python3

# Import the necessary libraries
import time
import math
import random
from ev3dev2.motor import *
from ev3dev2.sound import Sound
from ev3dev2.button import Button
from ev3dev2.sensor import *
from ev3dev2.sensor.lego import *
from ev3dev2.sensor.virtual import *

# Create the sensors and motors objects

motorA = LargeMotor(OUTPUT_A)
motorB = LargeMotor(OUTPUT_B)
left_motor = motorA
right_motor = motorB
tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)
steering_drive = MoveSteering(OUTPUT_A, OUTPUT_B)

spkr = Sound()
btn = Button()
radio = Radio()

color_sensor_in1 = ColorSensor(INPUT_1)
ultrasonic_sensor_in2 = UltrasonicSensor(INPUT_2)
ultrasonic_sensor_in3 = UltrasonicSensor(INPUT_3)
ultrasonic_sensor_in4 = UltrasonicSensor(INPUT_4)
gyro_sensor_in5 = GyroSensor(INPUT_5)

motorC = LargeMotor(OUTPUT_C)  # Magnet


# Here is where your code starts

def generate_target_point(param):
    ((update_x, update_y), distance, angle) = param

    target_x = update_x + math.sin(math.radians(angle)) * distance
    target_y = update_y + math.cos(math.radians(angle)) * distance

    return math.floor(target_x), math.floor(target_y)


class Environment:
    def __init__(self, size_x, size_y):
        self.size_y = size_y
        self.size_x = size_x

        self.grid = [[(1000, False) for y in range(size_y)] for x in range(size_x)]

        return

    def update(self, sensor_front, sensor_left, sensor_right):
        # print(f"Updating environment with front sensor data {sensor_front}")
        # print(f"Updating environment with right sensor data {sensor_left}")
        # print(f"Updating environment with left sensor data {sensor_right}")

        ((front_x, front_y), front_distance, front_rotation) = sensor_front
        ((left_x, left_y), left_distance, left_rotation) = sensor_left
        ((right_x, right_y), right_distance, right_rotation) = sensor_right

        if math.fabs(front_rotation % 90) > 5:
            return

        orientation = math.fabs(front_rotation % 180) < 45

        if front_distance < 35 and left_distance < 35:
            target_front_x, target_front_y = generate_target_point(sensor_front)
            target_left_x, target_left_y = generate_target_point(sensor_left)

            if orientation:
                print(f"Left corner at {target_left_x}|{target_front_y}")
            else:
                print(f"Left corner at {target_front_x}|{target_left_y}")

        if front_distance < 35 and right_distance < 35:
            target_front_x, target_front_y = generate_target_point(sensor_front)
            target_right_x, target_right_y = generate_target_point(sensor_right)

            if orientation:
                print(f"Right corner at {target_right_x}|{target_front_y}")
            else:
                print(f"Right corner at {target_front_x}|{target_right_y}")

        return

    def fill_walls(self):

        return


class Robot:
    def __init__(self, left_motor, right_motor, color_sensor, ultrasonic_sensor_front, ultrasonic_sensor_left,
                 ultrasonic_sensor_right, gyro_sensor, magnet, start_position):
        self.location = start_position

        self.left_sensor_x_offset = -7
        self.left_sensor_y_offset = 3

        self.right_sensor_x_offset = 7
        self.right_sensor_y_offset = 3

        self.front_sensor_x_offset = 0
        self.front_sensor_y_offset = 8

        self.right_motor = right_motor
        self.left_motor = left_motor

        self.color_sensor = color_sensor
        self.ultrasonic_sensor_front = ultrasonic_sensor_front
        self.ultrasonic_sensor_left = ultrasonic_sensor_left
        self.ultrasonic_sensor_right = ultrasonic_sensor_right
        self.gyro_sensor = gyro_sensor
        self.magnet = magnet

        return

    def move_to_coordinates(self, coordinates):
        (current_x, current_y) = self.location
        (target_x, target_y) = coordinates

        rotation = self.gyro_sensor.angle

        self

        return

    def sensor_read_out(self):
        (current_x, current_y) = self.location

        front = ((current_x + self.front_sensor_x_offset), (
                current_y + self.front_sensor_y_offset)), self.ultrasonic_sensor_front.distance_centimeters, self.gyro_sensor.angle

        left = ((current_x + self.left_sensor_x_offset), (
                current_y + self.left_sensor_y_offset)), self.ultrasonic_sensor_left.distance_centimeters, self.gyro_sensor.angle - 90

        right = ((current_x + self.right_sensor_x_offset), (
                current_y + self.right_sensor_y_offset)), self.ultrasonic_sensor_right.distance_centimeters, self.gyro_sensor.angle + 90

        return front, left, right


size_x, size_y = (1000, 1000)

env = Environment(size_x, size_y)
robot = Robot(left_motor, right_motor, color_sensor_in1, ultrasonic_sensor_in2, ultrasonic_sensor_in3,
              ultrasonic_sensor_in4, gyro_sensor_in5, motorC, (size_x / 2, size_y / 2))

while True:
    # update environment

    (front, left, right) = robot.sensor_read_out()

    env.update(front, left, right)

    # localize

    # plan

    # move

    tank_drive.on(-5, 5)

    robot.move_to_coordinates((500, 550))
