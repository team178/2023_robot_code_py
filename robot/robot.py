from commands2 import TimedCommandRobot
from commands2.button import CommandXboxController
from wpilib import SmartDashboard

from robot.subsystems.arm import Arm, ArmPosition
from robot.subsystems.drivetrain import Drivetrain
from robot.subsystems.claw import Claw
from robot.constants import *
from robot.auto import AutoSelector


class Robot(TimedCommandRobot):
    arm = Arm()
    claw = Claw()
    drivetrain = Drivetrain()

    driver = CommandXboxController(0)
    aux = CommandXboxController(1)

    _auto_cmd: None = None

    def __init__(self):
        super().__init__()

        # Why no RobotContainer? Because it's unneeded boilerplate - Patrick

        # * Aux controller bindings
        self.aux.B().onTrue(self.arm.set_position(ArmPosition.HOME))

        self.aux.Y().onTrue(self.arm.set_position(ArmPosition.SUBSTATION))

        self.aux.A().onTrue(self.arm.set_position(ArmPosition.LOW))

        self.aux.X().onTrue(self.arm.set_position(ArmPosition.HIGH))

        self.aux.leftBumper().onTrue(self.claw.toggle())

        # * Drive controller bindings
        self.drivetrain.setDefaultCommand(
            self.drivetrain.arcade_drive(self.driver.getLeftY, self.driver.getRightX)
        )

        SmartDashboard.putData("auto", AutoSelector(self))

    def teleopInit(self):
        if self._auto_cmd is not None:
            self._auto_cmd.cancel()
        self.drivetrain.set_wheel_speeds(0, 0) 


    def autonomousInit(self):
        self._auto_cmd = AutoSelector.get_selected()
        self.drivetrain.set_wheel_speeds(0, 0)

        if self._auto_cmd is not None:
            self._auto_cmd.schedule()
        else:
            print("NO AUTO ROUTE SELECTED")

    def disabledInit(self) -> None:
        if self._auto_cmd is not None:
            self._auto_cmd.cancel()

        # added after the robot punched a locker after re-enabling autonomous
        # always remember the turn OFF the motors if you want it to stay still
        self.drivetrain.set_wheel_speeds(0, 0)
