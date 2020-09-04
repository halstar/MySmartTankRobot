from enum import Enum

USE_RPI_GPIO = 0
USE_RPI_ZERO = 1
USE_PI_GPIO  = 2

SETUP_FILE = 'setup.json'


class MODE(Enum):

    USER_CONTROL    = 1
    AVOID_OBSTACLES = 2
    FOLLOW_LINE     = 3
    FOLLOW_CORRIDOR = 4
    ALONG_OBSTACLE  = 5


class USER_CONTROL(Enum):

    DIRECTIONS = 1
    JOYSTICK   = 2
    SENSORS    = 3


class DISPLAY(Enum):

    OFF     = 1
    CAMERA  = 2
    SCAN_2D = 3
    SCAN_3D = 4
    LIVE_3D = 5


I2C_BUS_NUMBER = 1

POSITION_LOOP_TIME_STEP = 0.005

IDLE_LOOP_SLEEP_TIME = 0.10

COMPLEMENTARY_FILTER_FACTOR = 0.998
SPEED_COMPUTATION_DIVIDER   =    10

TARGET_SPEED_STEP  = 45
TURNING_SPEED_STEP = 60
TURNING_ANGLE_STEP = 90

OBSTACLE_DETAILED_SCANNING_STEP =  2
OBSTACLE_FAST_SCANNING_STEP     =  5
DISTANCE_SAMPLES_NB             = 50

SERVO_FREQUENCY  =  50
DC_MOTOR_PWM_MIN =   0
DC_MOTOR_PWM_MAX = 100

# Default constants used by the speed PID controller
SPEED_PID_KP     = 1.00
SPEED_PID_KI     = 0.00
SPEED_PID_KD     = 0.00
SPEED_PID_TARGET = 0.00
SPEED_PID_MIN    = -DC_MOTOR_PWM_MAX
SPEED_PID_MAX    =  DC_MOTOR_PWM_MAX
SPEED_PID_WINDUP = 0.025

LEFT_MOTOR_ENABLE = 16
LEFT_MOTOR_PIN_1  = 20
LEFT_MOTOR_PIN_2  = 21

RIGHT_MOTOR_ENABLE = 13
RIGHT_MOTOR_PIN_1  = 26
RIGHT_MOTOR_PIN_2  = 19

LEFT_MOTOR_ENCODER_PIN_1 = 12
LEFT_MOTOR_ENCODER_PIN_2 =  6

RIGHT_MOTOR_ENCODER_PIN_1 = 22
RIGHT_MOTOR_ENCODER_PIN_2 = 23

DEFAULT_LIDAR_ADDRESS = 0x29
FRONT_LIDAR_ADDRESS   = 0x27
BACK_LIDAR_ADDRESS    = 0x28

FRONT_LIDAR_ENABLE = 18
BACK_LIDAR_ENABLE  = 14

FRONT_PAN_CONTROL_PIN  = 10
FRONT_TILT_CONTROL_PIN = 25

BACK_PAN_CONTROL_PIN  = 11
BACK_TILT_CONTROL_PIN =  7

LIDAR_POSITION_OFFSET         = 0.08
LIDAR_TIMING_BUDGET_IN_US     = 66000
LIDAR_INTER_MEASUREMENT_IN_MS =    70
LIDAR_LOOP_TIME_STEP          = (LIDAR_TIMING_BUDGET_IN_US / 1000000.0 + LIDAR_INTER_MEASUREMENT_IN_MS / 1000.0)

ROBOT_FORWARD_BACKWARD_STEP_TIME = 0.3
ROBOT_LEFT_RIGHT_TURN_STEP_TIME  = 0.1

CORRIDOR_FOLLOWING_PRECISION = 0.03
ALONG_OBSTACLE_PRECISION     = 0.03
AVOID_OBSTACLE_STOP_DISTANCE = 0.30