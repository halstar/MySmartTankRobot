import time

from globals import *
from log     import *


class Robot:

    def __init__(self, setup_data, speed_pid_controller, left_motor, right_motor, left_encoder, right_encoder, imu_device):

        self.speed_pid_controller = speed_pid_controller

        self.left_motor  = left_motor
        self.right_motor = right_motor

        self.left_encoder  = left_encoder
        self.right_encoder = right_encoder

        self.imu_device = imu_device

        self.left_counter       = 0
        self.right_counter      = 0
        self.filtered_pitch     = 0.0
        self.relative_yaw_angle = 0.0
        self.turn_angle_order   = 0.0
        self.turn_speed_step    = 0.0
        self.current_speed      = 0.0
        self.target_speed       = 0.0
        self.applied_speed      = 0.0

        # Setup IMU
        self.imu_device.reset        ()
        self.imu_device.reset_offsets()

        # Setup acceleration offsets
        self.imu_device.set_x_acceleration_offset(setup_data['ACCELERATION_X_OFFSET'])
        self.imu_device.set_y_acceleration_offset(setup_data['ACCELERATION_Y_OFFSET'])
        self.imu_device.set_z_acceleration_offset(setup_data['ACCELERATION_Z_OFFSET'])

        # Setup gyroscope offsets
        self.imu_device.set_x_gyroscope_offset(setup_data['GYROSCOPE_X_OFFSET'])
        self.imu_device.set_y_gyroscope_offset(setup_data['GYROSCOPE_Y_OFFSET'])
        self.imu_device.set_z_gyroscope_offset(setup_data['GYROSCOPE_Z_OFFSET'])

        # Setup gyroscope drift correction
        self.imu_device.set_x_gyroscope_drift_correction(setup_data['GYROSCOPE_X_DRIFT_CORRECTION'])
        self.imu_device.set_y_gyroscope_drift_correction(setup_data['GYROSCOPE_Y_DRIFT_CORRECTION'])
        self.imu_device.set_z_gyroscope_drift_correction(setup_data['GYROSCOPE_Z_DRIFT_CORRECTION'])

    def stop(self):

        log(INFO, 'Robot >>> stop')

        self.turn_angle_order = 0.0
        self.turn_speed_step  = 0.0
        self.target_speed     = 0.0

    def stop_turn(self):

        log(INFO, 'Robot >>> stop turn')

        self.turn_angle_order = 0.0
        self.turn_speed_step  = 0.0

    def forward(self, speed):

        log(INFO, 'Robot >>> forward @ speed = {:3.2f}'.format(speed))

        self.target_speed = speed

    def backward(self, speed):

        log(INFO, 'Robot >>> backward @ speed = {:3.2f}'.format(speed))

        self.target_speed = -speed

    def forward_step(self):

        log(DEBUG, 'Robot >>> forward step')

        self.target_speed = TARGET_SPEED_STEP / 2
        time.sleep(ROBOT_FORWARD_BACKWARD_STEP_TIME)
        self.target_speed = 0.0

    def backward_step(self):

        log(DEBUG, 'Robot >>> backward step')

        self.target_speed = -TARGET_SPEED_STEP / 2
        time.sleep(ROBOT_FORWARD_BACKWARD_STEP_TIME)
        self.target_speed = 0.0

    def left_with_angle(self, angle):

        log(INFO, 'Robot >>> turning left @ angle = {:3.2f}'.format(angle))

        self.turn_angle_order = self.relative_yaw_angle + angle
        self.turn_speed_step  = TURNING_SPEED_STEP / 2

    def right_with_angle(self, angle):

        log(INFO, 'Robot >>> turning right @ angle = {:3.2f}'.format(angle))

        self.turn_angle_order = self.relative_yaw_angle - angle
        self.turn_speed_step  = -TURNING_SPEED_STEP / 2

    def left_with_strength(self, strength):

        log(INFO, 'Robot >>> turning left @ strength = {:3.2f}'.format(strength))

        self.turn_angle_order = 0.0
        self.turn_speed_step  = strength

    def right_with_strength(self, strength):

        log(INFO, 'Robot >>> turning right @ strength = {:3.2f}'.format(strength))

        self.turn_angle_order = 0.0
        self.turn_speed_step  = -strength

    def left_step(self):

        log(DEBUG, 'Robot >>> left step')

        self.turn_angle_order = 0.0
        self.turn_speed_step  = TURNING_SPEED_STEP
        time.sleep(ROBOT_LEFT_RIGHT_TURN_STEP_TIME)
        self.turn_speed_step  = 0.0

    def right_step(self):

        log(DEBUG, 'Robot >>> right step')

        self.turn_angle_order = 0.0
        self.turn_speed_step  = -TURNING_SPEED_STEP
        time.sleep(ROBOT_LEFT_RIGHT_TURN_STEP_TIME)
        self.turn_speed_step  = 0.0

    def get_speed(self):

        return self.current_speed

    def print_info(self):

        print('Left encoder     = {:6.2f} / Right encoder  = {:6.2f}'.format                          (self.left_counter    , self.right_counter                          ))
        print('Target speed     = {:6.2f} / Current speed  = {:6.2f} / Applied speed = {:6.2f}'.format(self.target_speed    , self.current_speed , self.applied_speed     ))
        print('Turn angle order = {:6.2f} / Filtered pitch = {:6.2f} / Relative yaw  = {:6.2f}'.format(self.turn_angle_order, self.filtered_pitch, self.relative_yaw_angle))

    def start(self):

        distance_readings = []

        log(INFO, 'Initiating robot control thread')

        while True:

            start_time = time.time()

            # ######################### #
            # Speed control computation #
            # ######################### #

            self.speed_pid_controller.set_target(self.target_speed)

            self.left_counter  = self.left_encoder.get_counter()
            self.right_counter = self.right_encoder.get_counter()

            distance_readings.append((self.left_counter + self.right_counter) / 2)

            if len(distance_readings) > DISTANCE_SAMPLES_NB:
                distance_readings.pop(0)
                self.current_speed = sum(distance_readings) / SPEED_COMPUTATION_DIVIDER
            else:
                self.current_speed = self.target_speed

            self.applied_speed = self.speed_pid_controller.update(self.current_speed, DISTANCE_SAMPLES_NB * POSITION_LOOP_TIME_STEP)

            self.left_encoder.reset_counter ()
            self.right_encoder.reset_counter()

            # ####################### #
            # Situational computation #
            # ####################### #

            try:
                self.imu_device.read_acceleration_data()
                self.imu_device.read_gyroscope_data()
                pass
            except Exception as e:
                log(ERROR, 'IMU/I2C error detected')
                log(ERROR, e)
                continue

            self.imu_device.correct_gyroscope_data()

            self.imu_device.compute_angles()
            self.imu_device.compute_rates ()

            pitch      = self.imu_device.get_pitch     ()
            pitch_rate = self.imu_device.get_pitch_rate()
            yaw_rate   = self.imu_device.get_yaw_rate  ()

            self.filtered_pitch      = COMPLEMENTARY_FILTER_FACTOR * (self.filtered_pitch + pitch_rate * POSITION_LOOP_TIME_STEP) + (1 - COMPLEMENTARY_FILTER_FACTOR) * pitch
            self.relative_yaw_angle += yaw_rate * POSITION_LOOP_TIME_STEP

            # ################################### #
            # Left/right ordered turn computation #
            # ################################### #

            if self.turn_angle_order != 0:

                if self.turn_speed_step > 0 and self.relative_yaw_angle > self.turn_angle_order - 3.0:

                    self.stop_turn()

                    log(DEBUG, 'Ordered left turn is over')

                elif self.turn_speed_step < 0 and self.relative_yaw_angle < self.turn_angle_order + 3.0:

                    self.stop_turn()

                    log(DEBUG, 'Ordered right turn is over')

            # ########################## #
            # Overall speeds computation #
            # ########################## #

            overall_left_speed  = self.applied_speed - self.turn_speed_step
            overall_right_speed = self.applied_speed + self.turn_speed_step

            # ######################### #
            # Update motors with speeds #
            # ######################### #

            if (-1 < overall_left_speed < 1) and (-1 < overall_right_speed < 1):

                self.left_motor.stop ()
                self.right_motor.stop()

            else:

                if overall_left_speed > 0:
                    self.left_motor.forward(overall_left_speed)
                else:
                    self.left_motor.backward(-overall_left_speed)

                if overall_right_speed > 0:
                    self.right_motor.forward(overall_right_speed)
                else:
                    self.right_motor.backward(-overall_right_speed)

            elapsed_time = time.time() - start_time

            if elapsed_time < POSITION_LOOP_TIME_STEP:
                time.sleep(POSITION_LOOP_TIME_STEP - elapsed_time)
            else:
                log(WARNING, 'Robot thread is late!')
