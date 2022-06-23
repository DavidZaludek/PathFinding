class Robot:
    def __init__(self, left_motor, right_motor, color_sensor, ultrasonic_sensor_front, ultrasonic_sensor_left, ultrasonic_sensor_right, gyro_sensor, magnet):
        self.right_motor = right_motor
        self.left_motor = left_motor

        self.color_sensor = color_sensor
        self.ultrasonic_sensor_front = ultrasonic_sensor_front
        self.ultrasonic_sensor_left = ultrasonic_sensor_left
        self.ultrasonic_sensor_right = ultrasonic_sensor_right
        self.gyro_sensor = gyro_sensor
        self.magnet = magnet

        return
