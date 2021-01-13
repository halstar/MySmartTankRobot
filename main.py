import json
import os
import threading
import time
import RPi.GPIO
import picamera
import picamera.array
import robot
import dcmotor
import encoder
import imu
import lidar
import pid
import remotectrl
import streamer
import cv2
import numpy

from pantilt import *
from globals import *
from log     import *

setup_data          = None
camera_image_mutex  = None
patched_image_mutex = None
camera_image        = None
patched_image       = None


def robot_control_thread(robot):

    robot.start()


def find_min_max_obstacles(min_angle, max_angle, step, front_pan_tilt, front_lidar, back_pan_tilt, back_lidar):

    log(INFO, 'Finding closest & farthest obstacles')

    if front_pan_tilt is not None:
       front_pan_tilt.start_scanning_area(min_angle, max_angle, step, 0, 0, 0)

    if back_pan_tilt is not None:
        back_pan_tilt.start_scanning_area (min_angle, max_angle, step, 0, 0, 0)

    obstacles = []

    is_scanning_done = False

    while is_scanning_done == False:

        start_time = time.time()

        if front_pan_tilt is not None:

            pan_angle  = front_pan_tilt.get_pan ()
            tilt_angle = front_pan_tilt.get_tilt()

        else:

            pan_angle  = back_pan_tilt.get_pan ()
            tilt_angle = back_pan_tilt.get_tilt()

        log(DEBUG, 'Scan step: {:6.2f} / {}'.format(pan_angle, tilt_angle))

        if front_pan_tilt is not None:

            distance = front_lidar.get_distance()
            obstacles.append({'angle': pan_angle, 'distance': distance})

        if back_pan_tilt is not None:

            # Compute back pan tilt angle in front pan tilt referential
            if 0 <= pan_angle <= 90:
                pan_angle -= 180
            else:
                pan_angle += 180

            distance = back_lidar.get_distance()
            obstacles.append({'angle': pan_angle, 'distance': distance})

        if front_pan_tilt is not None:
            is_scanning_done = front_pan_tilt.step_scanning_area()

        if back_pan_tilt is not None:
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


def find_closest_obstacle(min_angle, max_angle, step, front_pan_tilt, front_lidar, back_pan_tilt, back_lidar):

    min_distance, min_angle, max_distance, max_angle = find_min_max_obstacles(min_angle, max_angle, step, front_pan_tilt, front_lidar, back_pan_tilt, back_lidar)

    return min_distance, min_angle


def find_farthest_obstacle(min_angle, max_angle, step, front_pan_tilt, front_lidar, back_pan_tilt, back_lidar):

    min_distance, min_angle, max_distance, max_angle = find_min_max_obstacles(min_angle, max_angle, step, front_pan_tilt, front_lidar, back_pan_tilt, back_lidar)

    return max_distance, max_angle


def run_obstacles_avoidance(robot, front_pan_tilt, front_lidar, back_pan_tilt, back_lidar):

    log(INFO, 'Starting obstacles avoidance mode')

    front_pan_tilt.reset()
    back_pan_tilt.reset ()

    # ####################################### #
    # Find direction of the farthest obstacle #
    # ####################################### #

    log(INFO, 'Looking for the best direction to start')

    farthest_obstacle_distance, farthest_obstacle_angle = find_farthest_obstacle(PAN_ANGLE_MIN, PAN_ANGLE_MAX, OBSTACLE_FAST_SCANNING_STEP, front_pan_tilt, front_lidar, back_pan_tilt, back_lidar)

    # ############################# #
    # Aim at this farthest obstacle #
    # ############################# #

    if farthest_obstacle_angle < 0:

        robot.right_with_angle(-farthest_obstacle_angle, True)

    else:

        robot.left_with_angle(farthest_obstacle_angle, True)

    # ############################## #
    #  Now start avoiding obstacles  #
    # ############################## #

    search_direction = False

    while (control.main_mode == MODE.AVOID_OBSTACLES) and (control.is_mode_aborted == False):

        if search_direction == True:

            robot.stop()
            robot.forward_step()

            # ####################################### #
            # Find direction of the farthest obstacle #
            # ####################################### #

            log(INFO, 'Looking for the best direction to continue')

            back_pan_tilt.reset()
            farthest_obstacle_distance, farthest_obstacle_angle = find_farthest_obstacle(PAN_ANGLE_MIN, PAN_ANGLE_MAX, OBSTACLE_FAST_SCANNING_STEP, front_pan_tilt, front_lidar, None, None)

            if farthest_obstacle_distance < AVOID_OBSTACLE_STOP_DISTANCE:

                front_pan_tilt.reset()
                farthest_obstacle_distance, farthest_obstacle_angle = find_farthest_obstacle(PAN_ANGLE_MIN, PAN_ANGLE_MAX, OBSTACLE_FAST_SCANNING_STEP, None, None, back_pan_tilt, back_lidar)

            # ############################# #
            # Aim at this farthest obstacle #
            # ############################# #

            if farthest_obstacle_angle < 0:

                robot.right_with_angle(-farthest_obstacle_angle, True)

            else:

                robot.left_with_angle(farthest_obstacle_angle, True)

            log(INFO, 'Aiming at the farthest obstacle')

            search_direction = False

        else:

            # ############################# #
            # Get distance of next obstacle #
            # ############################# #

            next_obstacle_distance = front_lidar.get_distance()

            # ################################ #
            #  Now move forward that direction #
            # ################################ #

            if next_obstacle_distance < AVOID_OBSTACLE_STOP_DISTANCE:

                log(INFO, 'Obstacle ahead: changing direction')

                search_direction = True

            else:

                log(DEBUG, 'Path is clear: moving on...')

                robot.forward(TARGET_SPEED_STEP / 2)

    robot.stop()

    front_pan_tilt.reset()
    back_pan_tilt.reset ()


def run_along_obstacle(robot, front_pan_tilt, front_lidar, back_pan_tilt, back_lidar):

    log(INFO, 'Starting along obstacle mode')

    front_pan_tilt.reset()
    back_pan_tilt.reset ()

    # ###################################### #
    # Find direction of the closest obstacle #
    # ###################################### #

    log(INFO, 'Looking for the best direction to start')

    closest_obstacle_distance, closest_obstacle_angle = find_closest_obstacle(PAN_ANGLE_MIN, PAN_ANGLE_MAX, OBSTACLE_FAST_SCANNING_STEP, front_pan_tilt, front_lidar, back_pan_tilt, back_lidar)

    # ##################################### #
    # Get parallel to this closest obstacle #
    # ##################################### #

    if closest_obstacle_angle < 0:

        log(INFO, 'Closest obstacle is on the right')

        is_obstacle_on_the_right = True

        if -90 < closest_obstacle_angle < 0:

            robot.left_with_angle(90 + closest_obstacle_angle, True)

        else:

            robot.right_with_angle(-closest_obstacle_angle - 90, True)

    else:

        log(INFO, 'Closest obstacle is on the left')

        is_obstacle_on_the_right = False

        if 0 < closest_obstacle_angle < 90:

            robot.right_with_angle(90 - closest_obstacle_angle, True)

        else:

            robot.left_with_angle(closest_obstacle_angle - 90, True)

    log(INFO, 'Now parallel to the closest obstacle')

    # ############################## #
    #     Now move along obstacle    #
    # ############################## #

    if is_obstacle_on_the_right == True:

        back_pan_tilt.pan_go_left()

    else:

        back_pan_tilt.pan_go_right()

    reference_distance = back_lidar.get_distance()

    log(INFO, 'Reference distance: {:06.3f}'.format(reference_distance))

    while (control.main_mode == MODE.ALONG_OBSTACLE) and (control.is_mode_aborted == False):

        robot.forward(TARGET_SPEED_STEP)

        front_distance = front_lidar.get_distance()
        side_distance  = back_lidar.get_distance()

        delta_distance = side_distance - reference_distance

        if front_distance < AVOID_OBSTACLE_STOP_DISTANCE:

            log(INFO, 'Obstacle ahead: changing direction')

            robot.stop()

            if is_obstacle_on_the_right == True:

                robot.left_with_angle(45, True)

            else:

                robot.right_with_angle(45, True)

            log(INFO, 'Ready to restart along obstacle')

        elif side_distance > 2 * reference_distance:

            log(INFO, 'Lost obstacle: changing direction')

            robot.stop()

            if is_obstacle_on_the_right == True:

                robot.right_with_angle(45, True)

            else:

                robot.left_with_angle(45, True)

            robot.forward(TARGET_SPEED_STEP)

            # Wait a bit for a move to be done
            time.sleep(1)

            log(INFO, 'Ready to restart along obstacle')

        elif -ALONG_OBSTACLE_PRECISION < delta_distance < ALONG_OBSTACLE_PRECISION:

            log(DEBUG, 'Obstacle is at the expected distance: moving on...')

            robot.stop_turn()

        elif delta_distance > 0:

            log(DEBUG, 'Going away from obstacle: correcting...')

            if is_obstacle_on_the_right == True:

                robot.right_with_strength(LEFT_RIGHT_TURN_STRENGTH)

            else:

                robot.left_with_strength(LEFT_RIGHT_TURN_STRENGTH)

        else:

            log(DEBUG, 'Going away from obstacle: correcting...')

            if is_obstacle_on_the_right == True:

                robot.left_with_strength(LEFT_RIGHT_TURN_STRENGTH)

            else:

                robot.right_with_strength(LEFT_RIGHT_TURN_STRENGTH)

    robot.stop()

    front_pan_tilt.reset()
    back_pan_tilt.reset ()


def run_corridor_following(robot, front_pan_tilt, front_lidar, back_pan_tilt, back_lidar):

    log(INFO, 'Starting corridor following mode')

    front_pan_tilt.reset()
    back_pan_tilt.reset ()

    # ###################################### #
    # Find direction of the closest obstacle #
    # ###################################### #

    log(INFO, 'Looking for the best direction to start')

    closest_obstacle_distance, closest_obstacle_angle = find_closest_obstacle(PAN_ANGLE_MIN, PAN_ANGLE_MAX, OBSTACLE_FAST_SCANNING_STEP, front_pan_tilt, front_lidar, back_pan_tilt, back_lidar)

    # ############################ #
    # Aim at this closest obstacle #
    # ############################ #

    if closest_obstacle_angle < 0:

        robot.right_with_angle(-closest_obstacle_angle, True)

    else:

        robot.left_with_angle(closest_obstacle_angle, True)

    log(INFO, 'Now aiming at the closest obstacle')

    # ############################ #
    # Get centered in the corridor #
    # ############################ #

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

    robot.right_with_angle(90, True)

    log(INFO, 'Ready to move in corridor')

    # ######################################## #
    #      Now move forward in corridor and    #
    # keep the best possible centered position #
    # ######################################## #

    front_pan_tilt.pan_go_left()
    back_pan_tilt.pan_go_left ()

    while (control.main_mode == MODE.FOLLOW_CORRIDOR) and (control.is_mode_aborted == False):

        robot.forward(TARGET_SPEED_STEP)

        left_wall_distance  = front_lidar.get_distance()
        right_wall_distance = back_lidar.get_distance ()

        delta_distance = left_wall_distance - right_wall_distance

        if -CORRIDOR_FOLLOWING_PRECISION < delta_distance < CORRIDOR_FOLLOWING_PRECISION:

            log(DEBUG, 'Correctly centered in corridor: moving on...')

            robot.stop_turn()

            # Allow u-turn only when robot is centered
            if control.is_u_turn_requested == True:

                log(DEBUG, 'Performing u-turn as per requested')

                robot.stop()
                robot.right_with_angle(180, True)

                control.is_u_turn_requested = False

        elif delta_distance > 0:

            log(DEBUG, 'Going away from center: correcting to the left...')

            robot.left_with_strength(LEFT_RIGHT_TURN_STRENGTH / 2)

        else:

            log(DEBUG, 'Going away from center: correcting to the right...')

            robot.right_with_strength(LEFT_RIGHT_TURN_STRENGTH / 2)

    robot.stop()

    front_pan_tilt.reset()
    back_pan_tilt.reset ()


def get_best_shape(input_image, mask_top_left_corner, mask_bottom_rigt_corner, threshold):

    gray_image = cv2.cvtColor(input_image, cv2.COLOR_BGR2GRAY)

    cv2.rectangle(gray_image, mask_top_left_corner, mask_bottom_rigt_corner, (255, 255, 255), -1)
    return_value, thresholded = cv2.threshold(gray_image, threshold, 255, cv2.THRESH_BINARY_INV)

    # Get contours
    contours, hierarchy = cv2.findContours(thresholded, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    highest_area     = 0.0
    selected_contour = None

    # Keep biggest contour only
    for contour in contours:

        contour_area = cv2.contourArea(contour)

        if cv2.contourArea(contour) > highest_area:
            highest_area = contour_area
            selected_contour = contour

    if highest_area < 500:
        selected_contour = None

    return selected_contour, thresholded


def run_line_following(robot):

    global camera_image_mutex, camera_image, patched_image

    log(INFO, 'Starting line following mode')

    # Deactivate the use of PID as it provides better results
    # with those little steps with we use this mode
    robot.use_pid(False)

    # ################################ #
    #  Read reference stop and u-turn  #
    # images and shapes, for later use #
    # ################################ #

    stop_image   = cv2.imread('stop.png'  , 0)
    u_turn_image = cv2.imread('u-turn.png', 0)

    return_value, stop_thresholded   = cv2.threshold(stop_image  , 127, 255, 0)
    return_value, u_turn_thresholded = cv2.threshold(u_turn_image, 127, 255, 0)

    stop_contours  , hierarchy = cv2.findContours(stop_thresholded  , cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    u_turn_contours, hierarchy = cv2.findContours(u_turn_thresholded, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    stop_contour   = stop_contours  [0]
    u_turn_contour = u_turn_contours[0]

    was_center_on_the_left = None

    while control.main_mode == MODE.FOLLOW_LINE:

        # Deal with startup, when no camera image is ready yet
        if camera_image is None:
            continue

        camera_image_mutex.acquire()
        patched_image_mutex.acquire()

        patched_image = camera_image.copy()

        camera_image_mutex.release()

        # Turn image to gray, separate lower and upper parts, apply threshold & get best shape
        lower_part_shape, thresholded_image = get_best_shape(patched_image, (0,   0), (367, 169), control.line_threshold)
        upper_part_shape, thresholded_image = get_best_shape(patched_image, (0, 170), (367, 239), control.line_threshold)

        # Draw a line to show separation between lower and upper parts
        cv2.line(patched_image, (0, 170), (367, 170), (255, 255, 255), 1)

        # ############################# #
        # Deal with upper part of image #
        # ############################# #

        got_a_stop_match   = False
        got_a_u_turn_match = False

        stop_draw_color   = (255, 255, 255)
        u_turn_draw_color = (255, 255, 255)
        sign_draw_color   = (255, 255, 255)

        if upper_part_shape is None:

            log(DEBUG, 'Found no shape in upper part')

        else:

            # Get rotated rectangle of upper shape
            rectangle = cv2.minAreaRect(upper_part_shape)
            box = cv2.boxPoints(rectangle)
            box = numpy.int0(box)

            # Compute match of that shape against stop sign
            stop_match_value = cv2.matchShapes(upper_part_shape, stop_contour, cv2.CONTOURS_MATCH_I1, 0)

            # Compute match of that shape against u-turn sign
            u_turn_match_value = cv2.matchShapes(upper_part_shape, u_turn_contour, cv2.CONTOURS_MATCH_I1, 0)

            # Deal with upper part shape against stop sign
            if stop_match_value < 0.50:

                log(DEBUG, 'Found STOP sign')

                got_a_stop_match = True
                stop_draw_color  = (0, 255, 0)
                sign_draw_color  = (0, 255, 0)

            # Deal with upper part shape against u-turn sign
            if u_turn_match_value < 0.75:

                log(DEBUG, 'Found U-TURN sign')

                got_a_u_turn_match = True
                u_turn_draw_color  = (0, 255, 0)
                sign_draw_color    = (0, 255, 0)

            cv2.putText(patched_image, 'Stop @ {:04.2f}'.format  (stop_match_value  ), (10, 20), cv2.FONT_HERSHEY_PLAIN, 1, stop_draw_color  , 1, cv2.FILLED)
            cv2.putText(patched_image, 'U-turn @ {:04.2f}'.format(u_turn_match_value), (10, 40), cv2.FONT_HERSHEY_PLAIN, 1, u_turn_draw_color, 1, cv2.FILLED)

            # Draw rotated rectangle of upper shape
            patched_image = cv2.drawContours(patched_image, [box], 0, sign_draw_color, 2)

        # ############################# #
        # Deal with lower part of image #
        # ############################# #

        # Ignore a malformed shape; consider it as if no shape was found
        if lower_part_shape is not None and len(lower_part_shape) < 5:

            log(DEBUG, 'Found malformed shape in lower part')

            lower_part_shape = None

        if lower_part_shape is None:

            log(DEBUG, 'Found no actual shape in lower part')

            patched_image_mutex.release()

            robot.stop()

            # Deal with upper part shape against stop sign
            if got_a_stop_match == True:

                log(DEBUG, 'Executing STOP sign')

                # No need to stop robot at it's already stopped
                was_center_on_the_left = None

            # Deal with upper part shape against u-turn sign
            elif got_a_u_turn_match == True:

                log(DEBUG, 'Executing U-TURN sign')

                robot.left_with_angle(180, True)
                robot.stop_for_period(1.0)
                was_center_on_the_left = not(was_center_on_the_left)

            # Try and get back on the track; this first test deals with startup
            elif was_center_on_the_left is None:

                pass

            elif was_center_on_the_left == True:

                log(DEBUG, 'Last known position of line was on the left: correcting to the left')

                robot.left_step()
                robot.stop_for_period(0.2)

            else:

                log(DEBUG, 'Last known position of line was on the right: correcting to the right')

                robot.right_step()
                robot.stop_for_period(0.2)

        else:

            log(DEBUG, 'Found an actual shape in lower part')

            # Get rotated rectangle of lower shape
            rectangle = cv2.minAreaRect(lower_part_shape)
            box = cv2.boxPoints(rectangle)
            box = numpy.int0(box)

            # Ge the center of lower shape
            M = cv2.moments(lower_part_shape)
            center_x = int(M["m10"] / M["m00"])
            center_y = int(M["m01"] / M["m00"])

            if center_x < 183:
                was_center_on_the_left = True
            else:
                was_center_on_the_left = False

            if 113 < center_x  < 253:
                center_draw_color = (0, 255, 0)
            else:
                center_draw_color = (0, 0, 255)

            # Draw rotated rectangle and center of lower shape
            patched_image = cv2.drawContours(patched_image, [box], 0, center_draw_color, 2)
            cv2.circle (patched_image, (center_x, center_y), 5, center_draw_color, -1)
            cv2.putText(patched_image, 'Center: {:03}'.format(center_x), (250, 230), cv2.FONT_HERSHEY_PLAIN, 1, center_draw_color, 1, cv2.FILLED)

            patched_image_mutex.release()

            # Implement line following based on lower part lateral position
            if 113 < center_x  < 253:

                log(DEBUG, 'Correctly aligned with line: moving on')

                robot.forward_step()
                robot.stop_for_period(0.2)

            elif center_x < 113:

                log(DEBUG, 'Going away from line: correcting to the left')

                robot.left_step()
                robot.stop_for_period(0.2)

            else:

                log(DEBUG, 'Going away from line: correcting to the right')

                robot.right_step()
                robot.stop_for_period(0.2)

    robot.stop()

    # Restore the use of PID for other modes
    robot.use_pid(True)


def autonomous_mode_thread(robot, front_pan_tilt, front_lidar, back_pan_tilt, back_lidar, data_reporter):

    log(INFO, 'Initiating autonomous mode thread')

    while True:

        if control.main_mode == MODE.AVOID_OBSTACLES:

            run_obstacles_avoidance(robot, front_pan_tilt, front_lidar, back_pan_tilt, back_lidar)

        elif control.main_mode == MODE.ALONG_OBSTACLE:

            run_along_obstacle(robot, front_pan_tilt, front_lidar, back_pan_tilt, back_lidar)

        elif control.main_mode == MODE.FOLLOW_CORRIDOR:

            run_corridor_following(robot, front_pan_tilt, front_lidar, back_pan_tilt, back_lidar)

        elif control.main_mode == MODE.FOLLOW_LINE:

            run_line_following(robot)

        else:

            time.sleep(IDLE_LOOP_SLEEP_TIME)


def scanning_control_thread(pan_tilt, lidar, is_front, data_reporter):

    if is_front == True:
        scan_name = 'Front'
    else:
        scan_name = ' Back'

    log(INFO, 'Initiating {} scanning control thread'.format(scan_name))

    while True:

        if (control.display_mode != DISPLAY.SCAN_2D and control.display_mode != DISPLAY.SCAN_3D) \
             or (control.is_mode_aborted == True):

            time.sleep(IDLE_LOOP_SLEEP_TIME)

        else:

            if control.display_mode == DISPLAY.SCAN_2D:
                log(INFO, '{} 2D scan starting'.format(scan_name))
                pan_tilt.start_scanning_area(PAN_ANGLE_MIN, PAN_ANGLE_MAX, OBSTACLE_DETAILED_SCANNING_STEP, 0, 0, 0)
            else:
                log(INFO, '{} 3D scan starting'.format(scan_name))
                pan_tilt.start_scanning_area(PAN_ANGLE_MIN, PAN_ANGLE_MAX, OBSTACLE_DETAILED_SCANNING_STEP, TILT_ANGLE_MIN, 0, OBSTACLE_DETAILED_SCANNING_STEP)

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

    global setup_data, camera_image_mutex, patched_image_mutex, camera_image, patched_image

    log(INFO, 'Initiating camera control thread')

    camera = picamera.PiCamera(resolution = str(setup_data['CAMERA_IMAGE_WIDTH']) + 'x' + str(setup_data['CAMERA_IMAGE_HEIGHT']), framerate=25)

    raw_buffer = picamera.array.PiRGBArray(camera, size=(setup_data['CAMERA_IMAGE_WIDTH'], setup_data['CAMERA_IMAGE_HEIGHT']))

    while True:

        if control.display_mode == DISPLAY.CAMERA:

            log(INFO, 'Camera starting to record')

            # Continuously Capture frames from the camera
            for frame in camera.capture_continuous(raw_buffer, format = 'bgr', use_video_port = True):

                camera_image_mutex.acquire()

                # Deal with one captured image
                camera_image = frame.array

                # Clear raw buffer in preparation for the next frame
                raw_buffer.truncate(0)

                camera_image_mutex.release()

                patched_image_mutex.acquire()

                # Use patched image in line following mode only
                if control.main_mode != MODE.FOLLOW_LINE:
                    patched_image = camera_image.copy()
                # Deal with startup, when no patched image is ready yet
                elif patched_image is None:
                    continue

                # Prepare image for streaming
                return_value, buffer = cv2.imencode('.jpg', patched_image)

                patched_image_mutex.release()

                streamer.streaming_output.write(bytearray(buffer))

                if control.display_mode != DISPLAY.CAMERA:
                    break

            log(INFO, 'Camera stopping to record')

        else:

            time.sleep(IDLE_LOOP_SLEEP_TIME)


def encoding_thread():

    global patched_image_mutex, patched_image

    log(INFO, 'Initiating encoding thread')

    while True:

        if control.display_mode == DISPLAY.CAMERA:

            patched_image_mutex.acquire()

            # Deal with startup, when no patched image is ready yet
            if camera_image is None:
                log(DEBUG, 'No camera image to encode')
                patched_image_mutex.release()
                continue
            # Use patched image in line following mode only
            elif control.main_mode != MODE.FOLLOW_LINE:
                patched_image = camera_image.copy()
            # Deal with startup, when no patched image is ready yet
            elif patched_image is None:
                log(DEBUG, 'No patched image to encode')
                patched_image_mutex.release()
                continue

            # Prepare image for streaming
            return_value, buffer = cv2.imencode('.jpg', patched_image)

            patched_image_mutex.release()

            streamer.streaming_output.write(bytearray(buffer))

            if control.display_mode != DISPLAY.CAMERA:
                break

        else:

            time.sleep(IDLE_LOOP_SLEEP_TIME)


def streaming_control_thread():

    steamer = streamer.StreamingServer(('', setup_data['CAMERA_STREAMING_PORT']), streamer.StreamingHandler)

    steamer.serve_forever()


def live_3d_control_thread(front_pan_tilt, front_lidar, back_pan_tilt, back_lidar, data_reporter):

    log(INFO, 'Initiating live 3D control thread')

    while True:

        if control.display_mode != DISPLAY.LIVE_3D:

            time.sleep(IDLE_LOOP_SLEEP_TIME)

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
    print('Run forward  step with: sf')
    print('Run backward step with: sb')
    print('Run left     step with: sl')
    print('Run right    step with: sr')
    print('')
    print('Enter forward/backward speed   value like: s=50')
    print('Enter left/right turn angle    value like: a=-30')
    print('Enter left/right turn strength value like: g=15')
    print('Enter line following threshold value like: t=20')
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
                print('Line following threshold: {}'.format(control.line_threshold))
                print('')
            elif command == 'q':
                print('')
                print('***** GOING TO OPERATIONAL MODE *****')
                is_console_on = False
            elif command == 'x':
                print('***** EXITING GRACEFULLY *****')
                robot.stop()
                front_pan_tilt.stop()
                back_pan_tilt.stop ()
                if status.is_rpi_gpio_used:
                    RPi.GPIO.cleanup()
                os._exit(0)
            elif command == 'h':
                print_help()

        elif len(user_input) == 2:

            command = user_input[0:2]

            if command == 'sf':
                robot.forward_step()
            elif command == 'sb':
                robot.backward_step()
            elif command == 'sl':
                robot.left_step()
            elif command == 'sr':
                robot.right_step()

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
                    robot.right_with_angle(-value, True)
                else:
                    robot.left_with_angle(value, True)
            elif command == 'g':
                if value < 0:
                    robot.right_with_strength(-value)
                else:
                    robot.left_with_strength(value)
            elif command == 't':
                control.line_threshold = value

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

    global setup_data, camera_image_mutex, patched_image_mutex

    with open(SETUP_FILE, 'r') as json_file:
        setup_data = json.load(json_file)

    log_init(setup_data['LOG_LEVEL'])

    camera_image_mutex  = threading.Lock()
    patched_image_mutex = threading.Lock()

    if setup_data['START_CONSOLE'] == 1:
        os.system('clear')
        print('')
        print('***** STARTING MY SMART TANK ROBOT CONTROL  *****')
        print('')
        print('                   /---\            ')
        print('                   | * | -\         ')
        print('          /--------\| |/  |         ')
        print('       //---------- |||  |/         ')
        print('     |-----|--------===-----|-----| ')
        print('     |  /-------/|//|\|\|------\  | ')
        print('     | ||     (O) --(*)-- (O)   \ | ')
        print('     |-----|----------------|-----| ')
        print('     /-----\  ||        ||  /-----\ ')
        print('     |- - -|*****      *****|- - -| ')
        print('     |- - -|*****      *****|- - -| ')
        print('     \-----/                \-----/ ')
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
    front_lidar            = lidar.Lidar('Front', FRONT_LIDAR_ADDRESS, FRONT_LIDAR_ENABLE, BACK_LIDAR_ENABLE , True )
    back_lidar             = lidar.Lidar(' Back', BACK_LIDAR_ADDRESS , BACK_LIDAR_ENABLE , FRONT_LIDAR_ENABLE, False)
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

    encoding = threading.Thread(target = encoding_thread, name = 'encoding', args = [])
    encoding.start()

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