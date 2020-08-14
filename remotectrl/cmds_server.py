import socket

from globals import *
from log     import *


class CommandsServer:

    def __init__(self, robot, port, data_reporter):

        self.port          = port
        self.robot         = robot
        self.data_reporter = data_reporter

    def __decode_cmd__(self, decoded_data):

        if decoded_data == 'DO':

            log(DEBUG, 'Received DISPLAY OFF command')

            control.display_mode    = DISPLAY.OFF
            control.is_mode_aborted = False

        elif decoded_data == 'DC':

            log(DEBUG, 'Received DISPLAY CAMERA command')

            control.display_mode    = DISPLAY.CAMERA
            control.is_mode_aborted = False

        elif decoded_data == 'D2':

            log(DEBUG, 'Received DISPLAY SCAN 2D command')

            control.display_mode    = DISPLAY.SCAN_2D
            control.is_mode_aborted = False

        elif decoded_data == 'D3':

            log(DEBUG, 'Received DISPLAY SCAN 3D command')

            control.display_mode    = DISPLAY.SCAN_3D
            control.is_mode_aborted = False

        elif decoded_data == 'L3':

            log(DEBUG, 'Received DISPLAY LIVE 3D command')

            control.display_mode    = DISPLAY.LIVE_3D
            control.is_mode_aborted = False

        elif decoded_data == 'MU':

            log(DEBUG, 'Received USER CONTROL MODE command')

            control.main_mode       = MODE.USER_CONTROL
            control.is_mode_aborted = False

        elif decoded_data == 'MO':

            log(DEBUG, 'Received AVOID OBSTACLES MODE command')

            control.main_mode       = MODE.AVOID_OBSTACLES
            control.is_mode_aborted = False

        elif decoded_data == 'ML':

            log(DEBUG, 'Received FOLLOW LINE MODE command')

            control.main_mode       = MODE.FOLLOW_LINE
            control.is_mode_aborted = False

        elif decoded_data == 'MC':

            log(DEBUG, 'Received FOLLOW CORRIDOR MODE command')

            control.main_mode       = MODE.FOLLOW_CORRIDOR
            control.is_mode_aborted = False

        elif decoded_data == 'MA':

            log(DEBUG, 'Received ALONG OBSTACLE MODE command')

            control.main_mode       = MODE.ALONG_OBSTACLE
            control.is_mode_aborted = False

        elif decoded_data == 'CD':

            log(DEBUG, 'Received CONTROL DIRECTIONS command')

            control.user_control = USER_CONTROL.DIRECTIONS

        elif decoded_data == 'CJ':

            log(DEBUG, 'Received CONTROL JOYSTICK command')

            control.user_control = USER_CONTROL.JOYSTICK

        elif decoded_data == 'CS':

            log(DEBUG, 'Received CONTROL SENSORS command')

            control.user_control = USER_CONTROL.SENSORS

        elif decoded_data == 'L1':

            log(DEBUG, 'Received LEFT 1 command')

            self.robot.left_with_angle(TURNING_ANGLE_STEP * 1)

        elif decoded_data == 'L2':

            log(DEBUG, 'Received LEFT 2 command')

            self.robot.left_with_angle(TURNING_ANGLE_STEP * 2)

        elif decoded_data == 'R1':

            log(DEBUG, 'Received RIGHT 1 command')

            self.robot.right_with_angle(TURNING_ANGLE_STEP * 1)

        elif decoded_data == 'R2':

            log(DEBUG, 'Received RIGHT 2 command')

            self.robot.right_with_angle(TURNING_ANGLE_STEP * 2)

        elif decoded_data == 'F1':

            log(DEBUG, 'Received FORWARD 1 command')

            self.robot.forward(TARGET_SPEED_STEP * 1)

        elif decoded_data == 'F2':

            log(DEBUG, 'Received FORWARD 2 command')

            self.robot.forward(TARGET_SPEED_STEP * 2)

        elif decoded_data == 'B1':

            log(DEBUG, 'Received BACKWARD 1 command')

            self.robot.backward(TARGET_SPEED_STEP * 1)

        elif decoded_data == 'B2':

            log(DEBUG, 'Received BACKWARD 2 command')

            self.robot.backward(TARGET_SPEED_STEP * 2)

        elif decoded_data == 'S':

            log(DEBUG, 'Received STOP command')

            self.robot.stop()

            control.is_mode_aborted = True

        else:

            if ':' in decoded_data:
                splitted_data = decoded_data.split(":")
                if len(splitted_data) == 3:
                    yaw   = int(splitted_data[0])
                    roll  = int(splitted_data[1])
                    pitch = int(splitted_data[2])
                else:
                    return
            else:
                return

            log(DEBUG, 'Received EVENT: {:6.2f} / {:6.2f} / {:6.2f}'.format(roll, pitch, yaw))

            forward_value = float(pitch) / 90
            turn_value    = float(roll)  / 90

            if forward_value > 0.0:

                self.robot.forward(forward_value * TARGET_SPEED_STEP * 2)

            else:

                self.robot.backward(-forward_value * TARGET_SPEED_STEP * 2)

            if turn_value > 0.0:

                self.robot.left_with_strength(turn_value * TURNING_SPEED_STEP * 2)

            else:

                self.robot.right_with_strength(-turn_value * TURNING_SPEED_STEP * 2)

    def start(self):

        while True:

            log(INFO, 'Initiating commands server')

            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:

                server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

                try:
                    server_socket.bind(('', self.port))
                except Exception as e:
                    log(ERROR, 'Command server connection error')
                    log(ERROR, e)
                    server_socket.close()
                    continue

                log(DEBUG, 'Server bound on port '+ str(self.port))

                server_socket.listen()

                log(INFO, 'Waiting fot a socket connection...')

                connection, client_address = server_socket.accept()

                with connection:

                    log(INFO, 'Got a connection from ' + str(client_address))

                    status.controlling_app_ip, controlling_app_port  = client_address

                    self.data_reporter.connect(status.controlling_app_ip)

                    log(INFO, 'Waiting for commands...')

                    while True:

                        try:
                            raw_data = connection.recv(32)

                            if not raw_data:
                                log(WARNING, 'Client disconnected (no data)')
                                server_socket.close()
                                break

                            decoded_data = raw_data.decode('ascii', 'ignore')

                        except Exception as e:
                            log(WARNING, 'Client disconnected (with error)')
                            log(WARNING, e)
                            server_socket.close()
                            break

                        self.__decode_cmd__(decoded_data)
