import utils
import servomotor
import json
import time

from globals import *

PAN_ANGLE_MIN      = -90
PAN_ANGLE_MAX      =  90
PAN_ANGLE_DEFAULT  =   0
TILT_ANGLE_MIN     = -90
TILT_ANGLE_MAX     =  90
TILT_ANGLE_DEFAULT =   0
LONG_MOVE_DURATION = 0.5


class PanTilt:

    def __init__(self, pan_control_pin, tilt_control_pin):

        with open(SETUP_FILE, 'r') as json_file:
            setup_data = json.load(json_file)

        self.pan_angle        = PAN_ANGLE_DEFAULT
        self.tilt_angle       = TILT_ANGLE_DEFAULT
        self.pan_front_angle  = PAN_ANGLE_DEFAULT
        self.pan_left_angle   = PAN_ANGLE_MIN
        self.pan_right_angle  = PAN_ANGLE_MAX
        self.tilt_front_angle = TILT_ANGLE_DEFAULT
        self.tilt_up_angle    = TILT_ANGLE_MIN
        self.tilt_down_angle  = TILT_ANGLE_MAX

        self.scanning_is_in_progress = False
        self.scanning_pan_min_angle  = PAN_ANGLE_DEFAULT
        self.scanning_pan_max_angle  = PAN_ANGLE_DEFAULT
        self.scanning_pan_step       = 0
        self.scanning_tilt_min_angle = TILT_ANGLE_DEFAULT
        self.scanning_tilt_max_angle = TILT_ANGLE_DEFAULT
        self.scanning_tilt_step      = 0

        self.pan_servo  = servomotor.ServoMotor(USE_PI_GPIO, pan_control_pin , setup_data['SERVO_MOTOR_MIN_PULSE'], setup_data['SERVO_MOTOR_MAX_PULSE'])
        self.tilt_servo = servomotor.ServoMotor(USE_PI_GPIO, tilt_control_pin, setup_data['SERVO_MOTOR_MIN_PULSE'], setup_data['SERVO_MOTOR_MAX_PULSE'])

        time.sleep(LONG_MOVE_DURATION)

    def set_pan(self, pan_angle):

        pan_angle = utils.clamp(pan_angle, PAN_ANGLE_MIN , PAN_ANGLE_MAX)

        self.pan_servo.set_angle(pan_angle)

        self.pan_angle = pan_angle

    def set_tilt(self, tilt_angle):

        tilt_angle = utils.clamp(tilt_angle, TILT_ANGLE_MIN , TILT_ANGLE_MAX)

        self.tilt_servo.set_angle(tilt_angle)

        self.tilt_angle = tilt_angle

    def get_pan(self):

        return self.pan_angle

    def get_tilt(self):

        return self.tilt_angle

    def pan_setup_front(self, front_angle):

        self.pan_front_angle = front_angle

    def pan_go_front(self):

        self.pan_servo.set_angle(self.pan_front_angle)
        self.pan_angle = self.pan_front_angle

        time.sleep(LONG_MOVE_DURATION)

    def pan_setup_left(self, left_angle):

        self.pan_left_angle = left_angle

    def pan_go_left(self):

        self.pan_servo.set_angle(self.pan_left_angle)
        self.pan_angle = self.pan_left_angle

        time.sleep(LONG_MOVE_DURATION)

    def pan_setup_right(self, right_angle):

        self.pan_right_angle = right_angle

    def pan_go_right(self):

        self.pan_servo.set_angle(self.pan_right_angle)
        self.pan_angle = self.pan_right_angle

        time.sleep(LONG_MOVE_DURATION)

    def tilt_setup_front(self, front_angle):

        self.tilt_front_angle = front_angle

    def tilt_go_front(self):

        self.tilt_servo.set_angle(self.tilt_front_angle)
        self.tilt_angle = self.tilt_front_angle

        time.sleep(LONG_MOVE_DURATION)

    def tilt_setup_up(self, up_angle):

        self.tilt_up_angle = up_angle

    def tilt_go_up(self):

        self.tilt_servo.set_angle(self.tilt_up_angle)
        self.tilt_angle = self.tilt_up_angle

        time.sleep(LONG_MOVE_DURATION)

    def tilt_setup_down(self, down_angle):

        self.tilt_down_angle = down_angle

    def tilt_go_down(self):

        self.tilt_servo.set_angle(self.tilt_down_angle)
        self.tilt_angle = self.tilt_down_angle

        time.sleep(LONG_MOVE_DURATION)

    def reset(self):

        self.pan_go_front ()
        self.tilt_go_front()

        time.sleep(LONG_MOVE_DURATION)

    def start_scanning_area(self,
                            pan_min_angle,
                            pan_max_angle,
                            pan_step,
                            tilt_min_angle,
                            tilt_max_angle,
                            tilt_step):

        self.scanning_is_in_progress = True
        self.scanning_pan_min_angle  = pan_min_angle
        self.scanning_pan_max_angle  = pan_max_angle
        self.scanning_pan_step       = pan_step
        self.scanning_tilt_min_angle = tilt_min_angle
        self.scanning_tilt_max_angle = tilt_max_angle
        self.scanning_tilt_step      = tilt_step

        self.set_pan (pan_min_angle )
        self.set_tilt(tilt_min_angle)

        time.sleep(LONG_MOVE_DURATION)

    def step_scanning_area(self):

        if (self.scanning_pan_step != 0) and (self.pan_angle + self.scanning_pan_step <= self.scanning_pan_max_angle):

            self.set_pan(self.pan_angle + self.scanning_pan_step)

        else:

            self.set_pan(self.scanning_pan_min_angle)

            if (self.scanning_tilt_step != 0) and (self.tilt_angle + self.scanning_tilt_step <= self.scanning_tilt_max_angle):

                self.set_tilt(self.tilt_angle + self.scanning_tilt_step)

            else:

                self.scanning_is_in_progress = False
                self.reset()

        return not self.scanning_is_in_progress

    def stop(self):

        self.pan_servo.stop ()
        self.tilt_servo.stop()

        time.sleep(LONG_MOVE_DURATION)

    def print_info(self, position):

        print('{} PAN SERVO:'.format(position))
        print('   Left    angle = {:6.2f} - Front angle = {:6.2f} - Right angle = {:6.2f}'.format(self.pan_left_angle, self.pan_front_angle, self.pan_right_angle))
        print('   Current angle = {:6.2f}'.format(self.pan_angle))
        print('{} TILT SERVO:'.format(position))
        print('   Up      angle = {:6.2f} - Front angle = {:6.2f} - Down  angle = {:6.2f}'.format(self.tilt_up_angle, self.tilt_front_angle, self.tilt_down_angle))
        print('   Current angle = {:6.2f}'.format(self.tilt_angle))
