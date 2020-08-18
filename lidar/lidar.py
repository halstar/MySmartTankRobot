import VL53L1X
import RPi.GPIO

from globals import *
from log     import *


class Lidar:

    def __init__(self, sensor_name, primary_i2c_address, primary_enable_pin, secondary_enable_pin, reset_secondary):

        self.distance = 0

        RPi.GPIO.setmode(RPi.GPIO.BCM)
        RPi.GPIO.setwarnings(False)

        RPi.GPIO.setup(primary_enable_pin  , RPi.GPIO.OUT)
        RPi.GPIO.setup(secondary_enable_pin, RPi.GPIO.OUT)

        # Hold secondary LIDAR under reset
        if reset_secondary:
            RPi.GPIO.output(secondary_enable_pin, RPi.GPIO.LOW)

        # Enable primary LIDAR
        RPi.GPIO.output(primary_enable_pin, RPi.GPIO.HIGH)

        try:

            # Try and setup primary LIDAR with its target address (case of SW restart)
            self.lidar = VL53L1X.VL53L1X(I2C_BUS_NUMBER, primary_i2c_address)
            self.lidar.open()

        except Exception:

            # Setup primary LIDAR with default address (case of HW reset)
            self.lidar = VL53L1X.VL53L1X(I2C_BUS_NUMBER, DEFAULT_LIDAR_ADDRESS)
            self.lidar.open()

            # Now we can change address, to use the target one
            self.lidar.change_address(primary_i2c_address)
            self.lidar.open()

        # UPDATE_TIME_MICROS = 66000
        # INTER_MEASUREMENT_PERIOD_MILLIS = 70
        self.lidar.set_timing(66000, 70)

        self.lidar.start_ranging(2)

        log(INFO, 'Setup {} LIDAR done'.format(sensor_name))

    def get_distance(self):

        next_distance = self.lidar.get_distance()

        if next_distance > 0:
            self.distance = next_distance / 1000.0

        return self.distance

    def print_info(self, sensor_name):

        self.get_distance()

        print('{}: timing budget  = {:6.3f} ms'.format(sensor_name, self.lidar.get_timing() / 1000.0))
        print('{}: next obstacle  @ {:6.3f} m'.format (sensor_name, self.distance))