import json
import os
import threading
import time
import RPi.GPIO
import picamera
import robot
import dcmotor
import encoder
import imu
import lidar
import pid
import remotectrl
import streamer

from pantilt import *
from globals import *
from log     import *

setup_data = None


def robot_control_thread(robot):

    robot.start()


def find_min_max_obstacles(front_pan_tilt, front_lidar, back_pan_tilt, back_lidar):

    log(INFO, 'Finding closest & farthest obstacles')

    front_pan_tilt.start_scanning_area(PAN_ANGLE_MIN, PAN_ANGLE_MAX, OBSTACLE_DETAILED_STEP, 0, 0, 0)
    back_pan_tilt.start_scanning_area (PAN_ANGLE_MIN, PAN_ANGLE_MAX, OBSTACLE_DETAILED_STEP, 0, 0, 0)

    obstacles = []

    is_scanning_done = False

    while is_scanning_done == False:

        start_time = time.time()

        pan_angle  = front_pan_tilt.get_pan()
        tilt_angle = front_pan_tilt.get_tilt()

        log(DEBUG, 'Scan step: {:6.2f} / {}'.format(pan_angle, tilt_angle))

        distance = front_lidar.get_distance()
        obstacles.append({'angle': pan_angle, 'distance': distance})

        # Compute back pan tilt angle in front pan tilt referential
        if 0 <= pan_angle <= 90:
            pan_angle -= 180
        else:
            pan_angle += 180

        distance = back_lidar.get_distance()
        obstacles.append({'angle': pan_angle, 'distance': distance})

        is_scanning_done = front_pan_tilt.step_scanning_area()
        is_scanning_done = back_pan_tilt.step_scanning_area()

        elapsed_time = time.time() - start_time

        if elapsed_time < LIDAR_LOOP_TIME_STEP:
            time.sleep(LIDAR_LOOP_TIME_STEP - elapsed_time)

    min_distance = 100
    min_angle    = 0
    max_distance = 0
    max_angle    = 0

    for obstacle in obstacles:

        distance = obstacle['distance']
        angle    = obstacle['angle'   ]

        if distance < min_distance:
            min_distance = distance
            min_angle    = angle

        if distance > max_distance:
            max_distance = distance
            max_angle    = angle

    log(INFO, 'Closest  obstacle : {:6.3f} m @ {:6.2f}'.format(min_distance, min_angle))
    log(INFO, 'Farthest obstacle : {:6.3f} m @ {:6.2f}'.format(max_distance, max_angle))

    return min_distance, min_angle, max_distance, max_angle


def find_closest_obstacle(front_pan_tilt, front_lidar, back_pan_tilt, back_lidar):

    min_distance, min_angle, max_distance, max_angle = find_min_max_obstacles(front_pan_tilt, front_lidar, back_pan_tilt, back_lidar)

    return min_distance, min_angle


def find_farthest_obstacle(front_pan_tilt, front_lidar, back_pan_tilt, back_lidar):

    min_distance, min_angle, max_distance, max_angle = find_min_max_obstacles(front_pan_tilt, front_lidar, back_pan_tilt, back_lidar)

    return max_distance, max_angle


def run_obstacles_avoidance(robot, front_pan_tilt, front_lidar, back_pan_tilt, back_lidar):

    log(INFO, 'Starting obstacles avoidance mode')

    front_pan_tilt.reset()
    back_pan_tilt.reset ()

    search_direction = True

    while (control.main_mode == MODE.AVOID_OBSTACLES) and (control.is_mode_aborted == False):

        if search_direction == True:

            robot.stop()

            # ####################################### #
            # Find direction of the farthest obstacle #
            # ####################################### #

            farthest_obstacle_distance, farthest_obstacle_angle = find_farthest_obstacle(front_pan_tilt, front_lidar, back_pan_tilt, back_lidar)

            # ############################# #
            # Aim at this farthest obstacle #
            # ############################# #

            if farthest_obstacle_angle < 0:

                robot.right_with_angle(-farthest_obstacle_angle)

            else:

                robot.left_with_angle(farthest_obstacle_angle)

            search_direction = False

        else:

            # ############################# #
            # Get distance of next obstacle #
            # ############################# #

            next_obstacle_distance = front_lidar.get_distance()

        # ################################ #
        #  Now move forward that direction #
        # ################################ #

        if robot.is_stuck() == True:

            log(DEBUG, 'Robot seems stuck: changing direction')

            robot.reset_stuck_status()

            search_direction = True

        elif next_obstacle_distance < AVOID_OBSTACLE_STOP_DISTANCE:

            log(DEBUG, 'Obstacle ahead: changing direction')

            search_direction = True

        else:

            log(DEBUG, 'Path is clear: moving on...')

            robot.forward_step()

    front_pan_tilt.reset()
    back_pan_tilt.reset ()


def run_line_following(robot):

    log(INFO, 'Starting line following mode')

    while (control.main_mode == MODE.FOLLOW_LINE) and (control.is_mode_aborted == False):

        time.sleep(0.1)


def run_corridor_following(robot, front_pan_tilt, front_lidar, back_pan_tilt, back_lidar):

    log(INFO, 'Starting corridor following mode')

    front_pan_tilt.reset()
    back_pan_tilt.reset ()

    # ###################################### #
    # Find direction of the closest obstacle #
    # ###################################### #

    closest_obstacle_distance, closest_obstacle_angle = find_closest_obstacle(front_pan_tilt, front_lidar, back_pan_tilt, back_lidar)

    # ############################ #
    # Aim at this closest obstacle #
    # ############################ #

    if closest_obstacle_angle < 0:

        robot.right_with_angle(-closest_obstacle_angle)

    else:

        robot.left_with_angle(closest_obstacle_angle)

    # Let time for the turn to be done
    time.sleep(2.0)

    log(INFO, 'Now aiming at the closest obstacle')

    # ############################ #
    # Get centered in the corridor #
    # ############################ #

    front_pan_tilt.reset()
    back_pan_tilt.reset ()

    is_centered = False

    while is_centered == False:

        front_wall_distance  = front_lidar.get_distance()
        back_wall_distance   = back_lidar.get_distance ()

        delta_distance = front_wall_distance - back_wall_distance

        if -CORRIDOR_FOLLOWING_PRECISION < delta_distance < CORRIDOR_FOLLOWING_PRECISION:

            is_centered = True

        elif delta_distance > 0:
            robot.forward_step ()
        else:
            robot.backward_step()

    log(INFO, 'Now centered in corridor')

    # ############################## #
    # Get into the forward direction #
    # ############################## #

    robot.right_with_angle(90.0)

    # Let time for the turn to be done
    time.sleep(2.0)

    log(INFO, 'Ready to move in corridor!')

    # ######################################## #
    #      Now move forward in corridor and    #
    # keep the best possible centered position #
    # ######################################## #

    front_pan_tilt.pan_go_left()
    back_pan_tilt.pan_go_left ()

    while (control.main_mode == MODE.FOLLOW_CORRIDOR) and (control.is_mode_aborted == False):

        left_wall_distance  = front_lidar.get_distance()
        right_wall_distance = back_lidar.get_distance ()

        delta_distance = left_wall_distance - right_wall_distance

        if -CORRIDOR_FOLLOWING_PRECISION < delta_distance < CORRIDOR_FOLLOWING_PRECISION:

            robot.forward_step()

        elif delta_distance > 0:

            robot.left_step   ()
            robot.forward_step()
            robot.right_step  ()

        else:

            robot.right_step  ()
            robot.forward_step()
            robot.left_step   ()

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

            run_obstacles_avoidance(robot, front_pan_tilt, front_lidar, back_pan_tilt, back_lidar)

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
                pan_tilt.start_scanning_area(PAN_ANGLE_MIN, PAN_ANGLE_MAX, OBSTACLE_SCANNING_STEP, 0, 0, 0)
            else:
                log(INFO, '{} 3D scan starting'.format(scan_name))
                pan_tilt.start_scanning_area(PAN_ANGLE_MIN, PAN_ANGLE_MAX, OBSTACLE_SCANNING_STEP, TILT_ANGLE_MIN, 0, OBSTACLE_SCANNING_STEP)

            is_scanning_done = False

            while (is_scanning_done == False) and (control.display_mode == DISPLAY.SCAN_2D or control.display_mode == DISPLAY.SCAN_3D):

                start_time = time.time()

                pan_angle  = pan_tilt.get_pan ()
                tilt_angle = pan_tilt.get_tilt()

                log(DEBUG, '{} scan step: {:6.2f} / {}'.format(scan_name, pan_angle, tilt_angle))

                obstacle_distance  = lidar.get_distance()

                # Compute pan angle in the app's referential
                if is_front == True:
                    pan_angle += 90
                else:
                    pan_angle += 270

                data_reporter.enqueue_data('{:06.2f}:{:06.2f}:{:06.3f}'.format(-tilt_angle, pan_angle, obstacle_distance))

                is_scanning_done = pan_tilt.step_scanning_area()

                if (control.is_mode_aborted == True):
                    is_scanning_done = True
                    pan_tilt.reset()

                elapsed_time = time.time() - start_time

                if elapsed_time < LIDAR_LOOP_TIME_STEP:
                    time.sleep(LIDAR_LOOP_TIME_STEP - elapsed_time)
                else:
                    log(WARNING, 'Scanning control thread is late!')

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

            start_time = time.time()

            front_pan_tilt.set_tilt(0)
            back_pan_tilt.set_tilt (0)

            obstacle_distance = front_lidar.get_distance()
            pan_angle         = front_pan_tilt.get_pan () + 90
            tilt_angle        = front_pan_tilt.get_tilt()

            data_reporter.enqueue_data('{:06.2f}:{:06.2f}:{:06.3f}'.format(tilt_angle, pan_angle, obstacle_distance))

            obstacle_distance = back_lidar.get_distance()
            pan_angle         = back_pan_tilt.get_pan () +  270
            tilt_angle        = back_pan_tilt.get_tilt()

            data_reporter.enqueue_data('{:06.2f}:{:06.2f}:{:06.3f}'.format(tilt_angle, pan_angle, obstacle_distance))

            elapsed_time = time.time() - start_time

            if elapsed_time < LIDAR_LOOP_TIME_STEP:
                time.sleep(LIDAR_LOOP_TIME_STEP - elapsed_time)
            else:
                log(WARNING, 'Live 3D control thread is late!')


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
    print('Press m to display DC    motors data')
    print('Press v to display servo motors data')
    print('Press c to display PIDs         data')
    print('Press u to display IMU          data')
    print('Press r to display robot        data')
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
                if value < 0:
                    robot.right_with_angle(-value)
                else:
                    robot.left_with_angle(value)
            elif command == 'g':
                if value < 0:
                    robot.right_with_strength(-value)
                else:
                    robot.left_with_strength(value)

        elif len(user_input) > 2 and user_input[2] == '=':

            command = user_input[0:2]
            value   = float(user_input[3:])

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

    log(INFO, 'Robot >>> initiating HW devices...')

    # Setup motors with their dedicated GPIOs and offsets
    left_motor  = dcmotor.DcMotor(USE_PI_GPIO, LEFT_MOTOR_ENABLE , LEFT_MOTOR_PIN_1 , LEFT_MOTOR_PIN_2 , setup_data['DC_MOTOR_LEFT_OFFSET' ])
    right_motor = dcmotor.DcMotor(USE_PI_GPIO, RIGHT_MOTOR_ENABLE, RIGHT_MOTOR_PIN_1, RIGHT_MOTOR_PIN_2, setup_data['DC_MOTOR_RIGHT_OFFSET'])

    # Setup left/right motors encoders with their dedicated GPIOs
    left_encoder  = encoder.Encoder(USE_PI_GPIO, LEFT_MOTOR_ENCODER_PIN_1 , LEFT_MOTOR_ENCODER_PIN_2 )
    right_encoder = encoder.Encoder(USE_PI_GPIO, RIGHT_MOTOR_ENCODER_PIN_1, RIGHT_MOTOR_ENCODER_PIN_2)

    imu_device             = imu.ImuDevice()
    data_reporter          = remotectrl.DataReporter(setup_data['DATA_REPORTING_PORT'])
    front_lidar            = lidar.Lidar('front', FRONT_LIDAR_ADDRESS, FRONT_LIDAR_ENABLE, BACK_LIDAR_ENABLE , True )
    back_lidar             = lidar.Lidar(' back', BACK_LIDAR_ADDRESS , BACK_LIDAR_ENABLE , FRONT_LIDAR_ENABLE, False)
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