from wpilib import ADXRS450_Gyro, AnalogGyro, SPI

from constants import *

class Drivetrain:

    speed_mult: float = 0

    _gyro = ADXRS450_Gryo(SPI.Port.kOnboard.CS0)
    _level = AnalogGyro(0)

    _left_motor = WPI_TalonFX(DRIVE_LB_MOTOR)
    _left_follower = WPI_TalonFX(DRIVE_LF_MOTOR)

    _right_motor = WPI_TalonFX(DRIVE_RB_MOTOR)
    _right_follower = WPI_TalonFX(DRIVE_LF_MOTOR)

    _left_controller = PIDController(**DRIVE_PID)
    _right_controller = PIDController(**DRIVE_PID)

    _ff = SimpleMotorFeedforward(**DRIVE_FF)

    def __init__(self):
        _left_motor.configFactoryDefault()
        _left_follower.configFactoryDefault()
        _right_motor.configFactoryDefault()
        _right_follower.configFactoryDefault()