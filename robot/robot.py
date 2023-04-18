from wpilib import TimedRobot, XboxController

from robot.subsystems.arm import Arm, ArmPosition


class Robot(TimedRobot):
    arm = Arm()

    driver = XboxController(0)
    aux = XboxController(1)

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        if self.aux.getBButtonPressed():
            self.arm.set_position(ArmPosition.HOME)

        if self.aux.getYButtonPressed():
            self.arm.set_position(ArmPosition.SUBSTATION)

        if self.aux.getAButtonPressed():
            self.arm.set_position(ArmPosition.LOW)

        if self.aux.getXButtonPressed():
            self.arm.set_position(ArmPosition.HIGH)
        
        self.arm.periodic()

        print(self.arm._lower_setpoint, self.arm._upper_setpoint)

    def autoInit(self):
        pass

    def autoPeriodic(self):
        pass
