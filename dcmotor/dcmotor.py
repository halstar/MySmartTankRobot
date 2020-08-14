import utils
import os
import RPi.GPIO
import gpiozero
import pigpio

from globals import *
from log     import *

STOPPED  = 0
FORWARD  = 1
BACKWARD = 2

PWM_FREQUENCY = 100


class DcMotor:

    def __init__(self, gpio_interface, enable_pin, motor_pin_1, motor_pin_2, pwm_offset):

        self.gpio_interface = gpio_interface
        self.enable_pin     = enable_pin
        self.motor_pin_1    = motor_pin_1
        self.motor_pin_2    = motor_pin_2
        self.pwm_offset     = pwm_offset

        if self.gpio_interface == USE_RPI_GPIO:

            status.is_rpi_gpio_used = True

            RPi.GPIO.setmode(RPi.GPIO.BCM)
            RPi.GPIO.setwarnings(False)
    
            RPi.GPIO.setup(enable_pin , RPi.GPIO.OUT)
            RPi.GPIO.setup(motor_pin_1, RPi.GPIO.OUT)
            RPi.GPIO.setup(motor_pin_2, RPi.GPIO.OUT)
    
            RPi.GPIO.output(motor_pin_1, RPi.GPIO.LOW)
            RPi.GPIO.output(motor_pin_2, RPi.GPIO.LOW)
            RPi.GPIO.output(enable_pin , RPi.GPIO.LOW)
    
            self.pwm = RPi.GPIO.PWM(enable_pin, PWM_FREQUENCY)
            self.pwm.start(DC_MOTOR_PWM_MIN)

        elif self.gpio_interface == USE_RPI_ZERO:

            self.motor = gpiozero.Motor(motor_pin_2, motor_pin_1, enable_pin, pwm = True)
        
        elif self.gpio_interface == USE_PI_GPIO:

            if not utils.is_process_running('pigpiod'):
                log(ERROR, 'DcMotor: pigpiod process not started')
                os._exit(3)

            self.pigpio = pigpio.pi()

            self.pigpio.set_mode(enable_pin , pigpio.OUTPUT)
            self.pigpio.set_mode(motor_pin_1, pigpio.OUTPUT)
            self.pigpio.set_mode(motor_pin_2, pigpio.OUTPUT)

            self.pigpio.write(enable_pin , 0)
            self.pigpio.write(motor_pin_1, 0)
            self.pigpio.write(motor_pin_2, 0)

            self.pigpio.set_PWM_range    (enable_pin, DC_MOTOR_PWM_MAX)
            self.pigpio.set_PWM_frequency(enable_pin, PWM_FREQUENCY   )

        self.speed     = DC_MOTOR_PWM_MIN
        self.direction = STOPPED

    def forward(self, speed):

        scaled_speed = utils.clamp(speed, DC_MOTOR_PWM_MIN, DC_MOTOR_PWM_MAX)
        scaled_speed = scaled_speed * (100 - self.pwm_offset) / 100 + self.pwm_offset

        if self.direction == STOPPED:

            if self.gpio_interface == USE_RPI_GPIO:

                self.pwm.start(scaled_speed)

            elif self.gpio_interface == USE_RPI_ZERO:

                self.motor.forward(scaled_speed / DC_MOTOR_PWM_MAX)

            elif self.gpio_interface == USE_PI_GPIO:

                self.pigpio.set_PWM_dutycycle(self.enable_pin, scaled_speed)

        elif scaled_speed != self.speed:

            if self.gpio_interface == USE_RPI_GPIO:

                self.pwm.ChangeDutyCycle(scaled_speed)

            elif self.gpio_interface == USE_RPI_ZERO:

                self.motor.forward(scaled_speed / DC_MOTOR_PWM_MAX)

            elif self.gpio_interface == USE_PI_GPIO:

                self.pigpio.set_PWM_dutycycle(self.enable_pin, scaled_speed)

        self.speed = scaled_speed

        if self.direction != FORWARD:

            if self.gpio_interface == USE_RPI_GPIO:

                RPi.GPIO.output(self.motor_pin_1, RPi.GPIO.LOW )
                RPi.GPIO.output(self.motor_pin_2, RPi.GPIO.HIGH)

            elif self.gpio_interface == USE_RPI_ZERO:

                pass

            elif self.gpio_interface == USE_PI_GPIO:

                self.pigpio.write(self.motor_pin_1, 0)
                self.pigpio.write(self.motor_pin_2, 1)

            self.direction = FORWARD

    def backward(self, speed):

        scaled_speed = utils.clamp(speed, DC_MOTOR_PWM_MIN, DC_MOTOR_PWM_MAX)
        scaled_speed = scaled_speed * (100 - self.pwm_offset) / 100 + self.pwm_offset

        if self.direction == STOPPED:

            if self.gpio_interface == USE_RPI_GPIO:

                self.pwm.start(scaled_speed)

            elif self.gpio_interface == USE_RPI_ZERO:

                self.motor.backward(scaled_speed / DC_MOTOR_PWM_MAX)

            elif self.gpio_interface == USE_PI_GPIO:

                self.pigpio.set_PWM_dutycycle(self.enable_pin, scaled_speed)

        elif scaled_speed != self.speed:

            if self.gpio_interface == USE_RPI_GPIO:

                self.pwm.ChangeDutyCycle(scaled_speed)

            elif self.gpio_interface == USE_RPI_ZERO:

                self.motor.backward(scaled_speed / DC_MOTOR_PWM_MAX)

            elif self.gpio_interface == USE_PI_GPIO:

                self.pigpio.set_PWM_dutycycle(self.enable_pin, scaled_speed)

        self.speed = scaled_speed

        if self.direction != BACKWARD:

            if self.gpio_interface == USE_RPI_GPIO:

                RPi.GPIO.output(self.motor_pin_1, RPi.GPIO.HIGH)
                RPi.GPIO.output(self.motor_pin_2, RPi.GPIO.LOW )

            elif self.gpio_interface == USE_RPI_ZERO:

                pass

            elif self.gpio_interface == USE_PI_GPIO:

                self.pigpio.write(self.motor_pin_1, 1)
                self.pigpio.write(self.motor_pin_2, 0)

            self.direction = BACKWARD

    def stop(self):

        if self.direction != STOPPED:

            if self.gpio_interface == USE_RPI_GPIO:

                self.pwm.stop()

                RPi.GPIO.output(self.motor_pin_1, RPi.GPIO.LOW)
                RPi.GPIO.output(self.motor_pin_2, RPi.GPIO.LOW)

            elif self.gpio_interface == USE_RPI_ZERO:

                self.motor.stop()

            elif self.gpio_interface == USE_PI_GPIO:

                self.pigpio.set_PWM_dutycycle(self.enable_pin, 0)

                self.pigpio.write(self.motor_pin_1, 0)
                self.pigpio.write(self.motor_pin_2, 0)

            self.speed     = DC_MOTOR_PWM_MIN
            self.direction = STOPPED

    def get_speed(self):

        return self.speed

    def print_info(self, motor_name):

        if self.direction == FORWARD:
            print('{}: Direction = FORWARD  - Speed = {:3.2f}'.format(motor_name, self.speed))
        elif self.direction == BACKWARD:
            print('{}: Direction = BACKWARD - Speed = {:3.2f}'.format(motor_name, self.speed))
        else:
            print('{}: Direction = STOPPED  - Speed = {:3.2f}'.format(motor_name, self.speed))