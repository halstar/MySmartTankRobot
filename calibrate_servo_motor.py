import os
import json
import pigpio

from globals import *

CONTROL_PIN     = 23
MID_PULSE_WIDTH = 1500
PULSE_STEP      = 10


def main():

    os.system('clear')

    print('')
    print('/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\ ')
    print('| Servo motor calibration starting... |')
    print('\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/ ')
    print('')

    with open(SETUP_FILE, 'r') as json_file:
        setup_data = json.load(json_file)

    pwm = pigpio.pi()
    pwm.set_mode(CONTROL_PIN, pigpio.OUTPUT)
    pwm.set_PWM_frequency(CONTROL_PIN, SERVO_FREQUENCY)

    pulse_width         = MID_PULSE_WIDTH
    is_calibration_done = False

    while not is_calibration_done:

        pwm.set_servo_pulsewidth(CONTROL_PIN, pulse_width)

        print('Seeking min pulse - Current: {}. Hit Enter as long as motor moves, else type OK: '.format(pulse_width), end = '', flush = True)
        user_input = input()
        if user_input == 'OK':
            is_calibration_done = True
        else:
            pulse_width -= PULSE_STEP

    setup_data['SERVO_MOTOR_MIN_PULSE'] = pulse_width + PULSE_STEP

    print('')

    pulse_width         = MID_PULSE_WIDTH
    is_calibration_done = False

    while not is_calibration_done:

        pwm.set_servo_pulsewidth(CONTROL_PIN, pulse_width)

        print('Seeking max pulse - Current: {}. Hit Enter as long as motor moves, else type OK: '.format(pulse_width), end = '', flush = True)
        user_input = input()
        if user_input == 'OK':
            is_calibration_done = True
        else:
            pulse_width += PULSE_STEP

    setup_data['SERVO_MOTOR_MAX_PULSE'] = pulse_width - PULSE_STEP

    with open(SETUP_FILE, 'w') as json_file:
        json.dump(setup_data, json_file, indent=4)

    print('')
    print('/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\ ')
    print('| Servo motor calibration done! |')
    print('\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/ ')
    print('')


if __name__ == '__main__':

    try:

        main()

    except KeyboardInterrupt:
        print('Keyboard interrupt...')

    except Exception as e:
        print('Error: ' + str(e))
