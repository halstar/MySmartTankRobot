import utils
import os
import RPi.GPIO
import gpiozero
import pigpio

from globals import *
from log     import *

ANGLE_MIN     = -90
ANGLE_MAX     =  90
ANGLE_DEFAULT =   0


class ServoMotor:

    def __init__(self, gpio_interface, control_pin, min_pulse, max_pulse):

        self.gpio_interface = gpio_interface
        self.control_pin    = control_pin
        self.angle          = ANGLE_DEFAULT

        if gpio_interface == USE_RPI_GPIO:

            status.is_rpi_gpio_used = True

            RPi.GPIO.setmode(RPi.GPIO.BCM)
            RPi.GPIO.setwarnings(False)
            RPi.GPIO.setup(control_pin , RPi.GPIO.OUT)

            self.pwm = RPi.GPIO.PWM(control_pin, SERVO_FREQUENCY)

            servo_period   = (1 / SERVO_FREQUENCY) * 1000000
            min_duty_cycle = min_pulse * 100 / servo_period
            max_duty_cycle = max_pulse * 100 / servo_period

            self.pwm_ratio  = (max_duty_cycle - min_duty_cycle) / (ANGLE_MAX - ANGLE_MIN)
            self.center_pwm = (max_duty_cycle + min_duty_cycle) / 2

        elif gpio_interface == USE_RPI_ZERO:

            self.pwm = gpiozero.Servo(control_pin,
                                      initial_value   = ANGLE_DEFAULT,
                                      min_pulse_width = min_pulse / 1000000,
                                      max_pulse_width = max_pulse / 1000000,
                                      frame_width     = 1 / SERVO_FREQUENCY)

            self.pwm_ratio  = 2 / (ANGLE_MAX - ANGLE_MIN)
            self.center_pwm = 0

        elif gpio_interface == USE_PI_GPIO:

            if not utils.is_process_running('pigpiod'):
                log(ERROR, 'ServoMotor: pigpiod process not started')
                os._exit(3)

            self.pigpio = pigpio.pi()

            self.pigpio.set_mode(control_pin, pigpio.OUTPUT)

            self.pigpio.set_PWM_frequency(control_pin, SERVO_FREQUENCY)

            self.pwm_ratio  = (max_pulse - min_pulse) / (ANGLE_MAX - ANGLE_MIN)
            self.center_pwm = (max_pulse + min_pulse) / 2

        self.is_started = False
        self.set_angle(ANGLE_DEFAULT)

    def set_angle(self, angle):

        angle *= -1.0

        angle = utils.clamp(angle, ANGLE_MIN, ANGLE_MAX)

        pwm_value = self.center_pwm - angle * self.pwm_ratio

        if self.is_started == False:

            if self.gpio_interface == USE_RPI_GPIO:

                self.pwm.start(pwm_value)

            elif self.gpio_interface == USE_RPI_ZERO:

                self.pwm.value = pwm_value

            elif self.gpio_interface == USE_PI_GPIO:

                self.pigpio.set_servo_pulsewidth(self.control_pin, pwm_value)

            self.is_started = True

        elif angle != self.angle:

            if self.gpio_interface == USE_RPI_GPIO:

                self.pwm.ChangeDutyCycle(pwm_value)

            elif self.gpio_interface == USE_RPI_ZERO:

                self.pwm.value = pwm_value

            elif self.gpio_interface == USE_PI_GPIO:

                self.pigpio.set_servo_pulsewidth(self.control_pin, pwm_value)

        self.angle = angle

    def get_angle(self):

            return self.angle * -1.0

    def stop(self):

        if self.is_started == True:

            self.set_angle(ANGLE_DEFAULT)

            if self.gpio_interface == USE_RPI_GPIO:

                self.pwm.stop()

            elif self.gpio_interface == USE_RPI_ZERO:

                self.pwm.detach()

            elif self.gpio_interface == USE_PI_GPIO:

                self.pigpio.set_servo_pulsewidth(self.control_pin, 0)

            self.is_started = False
