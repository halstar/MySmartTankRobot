import json
import os
import queue
import threading
import time
import RPi.GPIO
import picamera
import robot
import dcmotor
import encoder
import imu
import lidar
import pantilt
import pid
import remotectrl
import streamer

from globals import *
from log     import *

setup_data = None


def robot_control_thread(robot):

    robot.start()


def run_obstacles_avoidance(robot, front_pan_tilt, front_lidar):

    speed_samples     = queue.Queue()
    number_of_samples = 0
    robot_seems_stuck = False

    log(INFO, 'Starting obstacles avoidance mode')

    front_pan_tilt.reset()

    return

    while (control.main_mode == MODE.AVOID_OBSTACLES) and (control.is_mode_aborted == False):

        next_obstacle_distance = front_lidar.get_distance()
        next_obstacle_distance *= 1000

        next_obstacle_distance = 1.0

        speed_samples.put(robot.get_speed())
        number_of_samples += 1

        if number_of_samples >= 3:

            if max(list(speed_samples.queue)) - min(list(speed_samples.queue)) < 0.1:
                robot_seems_stuck = True
            else:
                robot_seems_stuck = False

            speed_samples.get()

        if robot_seems_stuck == True:

            log(INFO, 'Robot seems stuck')

            robot.stop()
            time.sleep(1.0)
            robot.backward(TARGET_SPEED_STEP)
            time.sleep(1.5)
            robot.stop()
            time.sleep(1.0)
            robot.left_with_strength(TURNING_ANGLE_STEP)
            time.sleep(2.0)

            robot_seems_stuck = False
            number_of_samples = 0
            speed_samples.empty()

        elif next_obstacle_distance < OBSTACLE_DISTANCE:

            log(DEBUG, 'Obstacle ahead')

            robot.stop()
            time.sleep(1.0)
            robot.right_with_strength(TURNING_ANGLE_STEP)
            time.sleep(2.0)

        else:

            log(DEBUG, 'Path is clear')

            robot.forward(TARGET_SPEED_STEP)
            time.sleep(0.1)

    front_pan_tilt.reset()


def run_line_following(robot):

    log(INFO, 'Starting line following mode')

    while (control.main_mode == MODE.FOLLOW_LINE) and (control.is_mode_aborted == False):

        time.sleep(0.1)


def run_corridor_following(robot, front_pan_tilt, front_lidar, back_pan_tilt, back_lidar):

    log(INFO, 'Starting corridor following mode')

    front_pan_tilt.reset()
    back_pan_tilt.reset ()

    is_ready_to_go = False

    # Get the best possible centered position in corridor
    while (is_ready_to_go == False) and (control.main_mode == MODE.FOLLOW_CORRIDOR) and (control.is_mode_aborted == False):

        front_pan_tilt.pan_go_left()
        back_pan_tilt.pan_go_left ()

        left_wall_distance  = front_lidar.get_distance()
        right_wall_distance = back_lidar.get_distance ()

        is_ready_to_go = True

        time.sleep(0.1)

    # Move along corridor and stay centered
    while (control.main_mode == MODE.FOLLOW_CORRIDOR) and (control.is_mode_aborted == False):

        time.sleep(0.1)

    front_pan_tilt.reset()
    back_pan_tilt.reset ()


def run_along_obstacle(robot, front_pan_tilt, front_lidar, back_pan_tilt, back_lidar):

    log(INFO, 'Starting along obstacle mode')

    front_pan_tilt.reset()
    back_pan_tilt.reset ()

    while (control.main_mode == MODE.ALONG_OBSTACLE) and (control.is_mode_aborted == False):

        time.sleep(0.1)

    front_pan_tilt.reset()
    back_pan_tilt.reset ()


def autonomous_mode_thread(robot, front_pan_tilt, front_lidar, back_pan_tilt, back_lidar, data_reporter):

    log(INFO, 'Initiating autonomous mode thread')

    while True:

        if control.main_mode == MODE.AVOID_OBSTACLES:

            run_obstacles_avoidance(robot, front_pan_tilt, front_lidar)

        elif control.main_mode == MODE.FOLLOW_LINE:

            run_line_following(robot)

        elif control.main_mode == MODE.FOLLOW_CORRIDOR:

            run_corridor_following(robot, front_pan_tilt, front_lidar, back_pan_tilt, back_lidar)

        elif control.main_mode == MODE.ALONG_OBSTACLE:

            run_along_obstacle(robot, front_pan_tilt, front_lidar, back_pan_tilt, back_lidar)

        else:

            time.sleep(1.0)


def scanning_control_thread(pan_tilt, lidar, is_front, data_reporter):

    if is_front == True:
        scan_name = 'front'
    else:
        scan_name = ' back'

    log(INFO, 'Initiating {} scanning control thread'.format(scan_name))

    while True:

        if (control.display_mode != DISPLAY.SCAN_2D and control.display_mode != DISPLAY.SCAN_3D) \
             or (control.is_mode_aborted == True):

            time.sleep(1.0)

        else:

            if control.display_mode == DISPLAY.SCAN_2D:
                log(INFO, '{} 2D scan starting'.format(scan_name))
                pan_tilt.start_scanning_area(-90, 89, OBSTACLE_FOV, 0, 0, 0)
            else:
                log(INFO, '{} 3D scan starting'.format(scan_name))
                pan_tilt.start_scanning_area(-90, 89, OBSTACLE_FOV, 0, 89, OBSTACLE_FOV)

            is_scanning_done = False

            while (is_scanning_done == False) and (control.display_mode == DISPLAY.SCAN_2D or control.display_mode == DISPLAY.SCAN_3D):

                time.sleep(0.2)
                is_scanning_done, pan_angle, tilt_angle = pan_tilt.step_scanning_area()

                log(DEBUG, '{} scan step: {:6.2f} / {}'.format(scan_name, pan_angle, tilt_angle))

                obstacle_distance  = lidar.get_distance()
                obstacle_distance *= 1000

                obstacle_distance = 1.0

                if is_front == True:
                    pan_angle += 90
                else:
                    pan_angle += 270

                data_reporter.enqueue_data('{:06.2f}:{:06.2f}:{:05.2f}'.format(tilt_angle, pan_angle, obstacle_distance))

                if (control.is_mode_aborted == True):
                    is_scanning_done = True
                    pan_tilt.reset()

            if control.display_mode == DISPLAY.SCAN_2D:
                log(INFO, '{} 2D scan done'.format(scan_name))
            else:
                log(INFO, '{} 3D scan done'.format(scan_name))


def cmds_server_thread(robot, data_reporter):

    global setup_data

    cmds_server = remotectrl.CommandsServer(robot, setup_data['COMMANDS_SERVER_PORT'], data_reporter)

    cmds_server.start()


def data_reporter_thread(data_reporter):

    data_reporter.start()


def camera_control_thread():

    global setup_data

    log(INFO, 'Initiating camera control thread')

    camera = picamera.PiCamera(resolution = str(setup_data['CAMERA_IMAGE_WIDTH']) + 'x' + str(setup_data['CAMERA_IMAGE_HEIGHT']), framerate=25)

    is_camera_recording = False

    while True:

        if control.display_mode == DISPLAY.CAMERA and not is_camera_recording == True:

            log(INFO, 'Camera starting to record')

            camera.start_recording(streamer.streaming_output, format='mjpeg')
            is_camera_recording = True

        elif control.display_mode != DISPLAY.CAMERA and is_camera_recording == True:

            log(INFO, 'Camera stopping to record')

            camera.stop_recording()
            is_camera_recording = False

        else:

            time.sleep(0.1)


def streaming_control_thread():

    steamer = streamer.StreamingServer(('', setup_data['CAMERA_STREAMING_PORT']), streamer.StreamingHandler)

    steamer.serve_forever()


def live_3d_control_thread(front_pan_tilt, front_lidar, back_pan_tilt, back_lidar, data_reporter):

    log(INFO, 'Initiating live 3D control thread')

    while True:

        if control.display_mode != DISPLAY.LIVE_3D:

            time.sleep(1.0)

        else:

            obstacle_distance  = front_lidar.get_distance()
            obstacle_distance *= 1000

            obstacle_distance = 1.0

            pan_angle, tilt_angle = front_pan_tilt.get_position()
            tilt_angle = 0
            pan_angle += 90

            data_reporter.enqueue_data('{:06.2f}:{:06.2f}:{:05.2f}'.format(tilt_angle, pan_angle, obstacle_distance))

            obstacle_distance  = back_lidar.get_distance()
            obstacle_distance *= 1000

            obstacle_distance = 2.0

            pan_angle, tilt_angle = back_pan_tilt.get_position()
            pan_angle += 270

            data_reporter.enqueue_data('{:06.2f}:{:06.2f}:{:05.2f}'.format(pan_angle, tilt_angle, obstacle_distance))

            time.sleep(0.1)


def print_help():

    print('')
    print('Enter 1 to select speed PID')
    print('Enter selected PID values like: p=12, i=0.15, d=100, w=0.05')
    print('')
    print('Enter forward/backward speed   value like: s=50')
    print('Enter left/right turn angle    value like: a=-30')
    print('Enter left/right turn strength value like: g=15')
    print('')
    print('Enter front pan  angle value like: fp=50')
    print('Enter front tilt angle value like: ft=-25')
    print('Enter back  pan  angle value like: bp=50')
    print('Enter back  tilt angle value like: bt=-25')
    print('')
    print('Press m to display DC    motors\'s data')
    print('Press v to display servo motors\'s data')
    print('Press c to display PIDs\'          data')
    print('Press u to display IMU\'s          data')
    print('Press r to display robot\'s        data')
    print('')
    print('Set log level: l=0 > NO_LOG , 1 > ERROR,')
    print('                 2 > WARNING, 3 > INFO , 4 > DEBUG')
    print('')
    print('Press q to go for operational mode')
    print('Press x to exit with no error')
    print('Press h to display this help')
    print('')


def console_thread(robot, left_motor, right_motor, imu_device, front_pan_tilt, front_lidar, back_pan_tilt, back_lidar, speed_pid_controller):

    print_help()

    is_console_on = True
    selected_pid  = 1

    while is_console_on == True:

        print('> ', end = '')

        user_input = input()

        if len(user_input) == 1:

            command = user_input[0]

            if command == '1':
                print('')
                print('Speed PID selected')
                print('')
                selected_pid = 1
            elif command == 'm':
                print('')
                left_motor.print_info ('LEFT  DC MOTOR')
                right_motor.print_info('RIGHT DC MOTOR')
                print('')
            elif command == 'v':
                print('')
                front_pan_tilt.print_info('FRONT')
                print('')
                back_pan_tilt.print_info ('BACK' )
                print('')
            elif command == 'c':
                print('')
                print('SPEED PID:')
                speed_pid_controller.print_info()
                print('')
            elif command == 'u':
                print('')
                imu_device.print_info()
                print('')
            elif command == 'r':
                print('')
                robot.print_info()
                print('')
                front_lidar.print_info('FRONT LIDAR')
                back_lidar.print_info ('BACK  LIDAR')
                print('')
            elif command == 'q':
                print('')
                print('***** GOING TO OPERATIONAL MODE *****')
                is_console_on = False
            elif command == 'x':
                print('***** EXITING GRACEFULLY *****')
                if status.is_rpi_gpio_used:
                    RPi.GPIO.cleanup()
                os._exit(0)
            elif command == 'h':
                print_help()

        elif len(user_input) > 2 and user_input[1] == '=':

            command = user_input[0]
            value   = float(user_input[2:])

            if command == 'l':
                if value == 0:
                    log_set_level(NO_LOG )
                elif value == 1:
                    log_set_level(ERROR  )
                elif value == 2:
                    log_set_level(WARNING)
                elif value == 3:
                    log_set_level(INFO   )
                elif value == 4:
                    log_set_level(DEBUG  )
                else:
                    print('Invalid log level')
            elif command == 'p':
                if selected_pid == 1:
                    speed_pid_controller.set_kp(value)
            elif command == 'i':
                if selected_pid == 1:
                    speed_pid_controller.set_ki(value)
            elif command == 'd':
                if selected_pid == 1:
                    speed_pid_controller.set_kd(value)
            elif command == 'w':
                if selected_pid == 1:
                    speed_pid_controller.set_anti_wind_up(value)
            elif command == 's':
                if value > 0:
                    robot.forward(value)
                elif value < 0:
                    robot.backward(-value)
                else:
                    robot.stop()
            elif command == 'a':
                if value > 0:
                    robot.right_with_angle(value)
                else:
                    robot.left_with_angle(-value)
            elif command == 'g':
                if value > 0:
                    robot.right_with_strength(value)
                else:
                    robot.left_with_strength(-value)

        elif len(user_input) > 2 and user_input[2] == '=':

            command = user_input[0:2]
            value   = float(user_input[3:])

            print("command ", command)
            print("value ", value)

            if command == 'fp':
                front_pan_tilt.set_pan(value)
            elif command == 'ft':
                front_pan_tilt.set_tilt(value)
            elif command == 'bp':
                back_pan_tilt.set_pan(value)
            elif command == 'bt':
                back_pan_tilt.set_tilt(value)


def main():

    global setup_data

    with open(SETUP_FILE, 'r') as json_file:
        setup_data = json.load(json_file)

    log_init(setup_data['LOG_LEVEL'])

    if setup_data['START_CONSOLE'] == 1:
        os.system('clear')
        print('')
        print('     |     /=*|||||||||*=\     | ')
        print('     |-----|-------------|-----| ')
        print('     |  /- /(O) ------- (O)\   | ')
        print('     | ||  \   -| (*) |-   /   | ')
        print('     |-----|-------------|-----| ')
        print('     |/ / / / / /   \ \ \ \ \  | ')
        print('     /-----\  ||     ||  /-----\ ')
        print('     |- - -|*****   *****|- - -| ')
        print('     |-----|*****   *****|-----| ')
        print('     |- - -|             |- - -| ')
        print('     \-----/             \-----/ ')
        print('')

    if setup_data['START_CONSOLE'] == 1:
        print('*****   CURRENTLY RUNNING IN CONSOLE MODE   *****')
    else:
        print('***** CURRENTLY RUNNING IN OPERATIONAL MODE *****')

    print('')

    # Setup motors with their dedicated GPIOs and offsets
    left_motor  = dcmotor.DcMotor(USE_PI_GPIO, LEFT_MOTOR_ENABLE , LEFT_MOTOR_PIN_1 , LEFT_MOTOR_PIN_2 , setup_data['DC_MOTOR_LEFT_OFFSET' ])
    right_motor = dcmotor.DcMotor(USE_PI_GPIO, RIGHT_MOTOR_ENABLE, RIGHT_MOTOR_PIN_1, RIGHT_MOTOR_PIN_2, setup_data['DC_MOTOR_RIGHT_OFFSET'])

    # Setup left/right motors encoders with their dedicated GPIOs
    left_encoder  = encoder.Encoder(USE_PI_GPIO, LEFT_MOTOR_ENCODER_PIN_1 , LEFT_MOTOR_ENCODER_PIN_2 )
    right_encoder = encoder.Encoder(USE_PI_GPIO, RIGHT_MOTOR_ENCODER_PIN_1, RIGHT_MOTOR_ENCODER_PIN_2)

    imu_device             = imu.ImuDevice()
    data_reporter          = remotectrl.DataReporter(setup_data['DATA_REPORTING_PORT'])
    front_lidar            = lidar.Lidar(FRONT_LIDAR_ADDRESS)
    back_lidar             = lidar.Lidar(BACK_LIDAR_ADDRESS )
    front_pan_tilt         = pantilt.PanTilt(FRONT_PAN_CONTROL_PIN, FRONT_TILT_CONTROL_PIN)
    back_pan_tilt          = pantilt.PanTilt(BACK_PAN_CONTROL_PIN , BACK_TILT_CONTROL_PIN )
    speed_pid_controller   = pid.Pid(SPEED_PID_KP  , SPEED_PID_KI  , SPEED_PID_KD  , SPEED_PID_TARGET  , SPEED_PID_MIN  , SPEED_PID_MAX  , SPEED_PID_WINDUP  )

    robot_device = robot.Robot(setup_data, speed_pid_controller, left_motor, right_motor, left_encoder, right_encoder, imu_device)

    robot_control = threading.Thread(target = robot_control_thread, name = 'robot_control',  args = [robot_device])
    robot_control.start()

    cmds_server = threading.Thread(target = cmds_server_thread, args = [robot_device, data_reporter])
    cmds_server.start()

    data_report = threading.Thread(target = data_reporter_thread, args = [data_reporter])
    data_report.start()

    autonomous_mode_control = threading.Thread(target = autonomous_mode_thread,  name = 'obstacles_avoidance',  args = [robot_device, front_pan_tilt, front_lidar, back_pan_tilt, back_lidar, data_reporter])
    autonomous_mode_control.start()

    scanning_front_control = threading.Thread(target = scanning_control_thread,  name = 'front_scanning_control',  args = [front_pan_tilt, front_lidar, True, data_reporter])
    scanning_front_control.start()

    scanning_back_control = threading.Thread(target = scanning_control_thread,  name = 'back_scanning_control',  args = [back_pan_tilt, back_lidar, False, data_reporter])
    scanning_back_control.start()

    camera_control = threading.Thread(target = camera_control_thread, name = 'camera_control', args = [])
    camera_control.start()

    live_3d_control = threading.Thread(target = live_3d_control_thread, name = 'live_3d_control', args = [front_pan_tilt, front_lidar, back_pan_tilt, back_lidar, data_reporter])
    live_3d_control.start()

    streaming_control = threading.Thread(target = streaming_control_thread, name = 'streaming_control', args = [])
    streaming_control.start()

    if setup_data['START_CONSOLE'] == 1:
        console = threading.Thread(target = console_thread, name = 'console', args = [robot_device, left_motor, right_motor, imu_device, front_pan_tilt, front_lidar, back_pan_tilt, back_lidar, speed_pid_controller])
        console.start()

    robot_control.join          ()
    cmds_server.join            ()
    data_report.join            ()
    autonomous_mode_control.join()
    scanning_front_control.join ()
    scanning_back_control.join ()
    camera_control.join         ()
    live_3d_control.join        ()
    streaming_control.join      ()

    if setup_data['START_CONSOLE'] == 1:
        console.join()


if __name__ == '__main__':

    try:

        main()

    except KeyboardInterrupt:
        log(ERROR, 'Keyboard interrupt...')
        os._exit(1)

    except Exception as e:
        log(ERROR, 'Caught exception')
        log(ERROR, e                 )
        os._exit(2)

    finally:
        if status.is_rpi_gpio_used:
            RPi.GPIO.cleanup()
        os._exit(0)