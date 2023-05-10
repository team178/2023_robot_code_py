from commands2 import Command, SequentialCommandGroup, WaitCommand
from commands2.cmd import sequence, parallel, waitUntil

from robot.auto.trajectory import drive_trajectory
from robot.subsystems.arm import ArmPosition

def place_high(claw, arm) -> Command:
    return sequence(
        claw.close()
    )

class Sub(SequentialCommandGroup):
    """
    Substation auto command
    """

    def __init__(self, robot: "Robot", fetch_cube: bool):
        """
        fetch_cube: Should the robot get the cube and try to place it
        when it drives backwards
        """

        self.addCommands(
            place_high(robot.claw, robot.arm),
            WaitCommand(0.2),
        )

        # Make grabbing the cube optional
        if fetch_cube:
            self.addCommands(
                parallel(
                    drive_trajectory(robot, None),
                    sequence(
                        WaitCommand(0.7),
                        robot.arm.set_position(ArmPosition.BACK),
                        robot.claw.open()
                    )
                ).deadlineWith(
                    sequence(
                        WaitCommand(0.7),
                        waitUntil(robot.claw.get_photosensor),
                        robot.claw.close()
                    )
                ),
                robot.claw.close(),
                WaitCommand(0.3),
                robot.arm.set_position(ArmPosition.HOME),
                drive_trajectory(robot, None),
                place_high(robot.calw, robot.arm),
                WaitCommand(0.2),
            )

        # Robot at the end either way
        self.addCommands(
            drive_trajectory(robot, None)
        )
    
    @classmethod
    def cone_cube(cls, robot: "Robot"):
        return cls(robot, True)

    @classmethod
    def cone_leave(cls, robot: "Robot"):
        return cls(robot, False)
    
class Mid(SequentialCommandGroup):
    """
    Middle/Charge station auto command
    """

    def __init__(self, robot: "Robot"):
        self.robot = robot

        self.addCommands(
            place_high(robot.claw, robot.arm)
        )
    
    @classmethod
    def cube_balance(cls, robot: "Robot"):
        return cls(robot)

