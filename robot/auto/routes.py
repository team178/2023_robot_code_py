from commands2 import Command, WaitCommand
from commands2.cmd import sequence, parallel, waitUntil

from robot.auto.trajectory import drive_trajectory
from robot.subsystems.arm import ArmPosition
from robot.auto.selector import AutoSelector as auto


@auto.route("PlaceHigh")
def place_high(robot) -> Command:
    return sequence(
        robot.claw.close()
    )

def sub(robot, fetch_cube: bool):
    seq = sequence(
        place_high(robot),
        WaitCommand(0.2),
    )

    # Makes grabbing the cube optional
    if fetch_cube:
        seq = seq.andThen(
            drive_trajectory(robot, None).deadlineWith(
                    sequence(
                        WaitCommand(0.7),
                        robot.arm.set_position(ArmPosition.BACK),
                        robot.claw.open(),
                        waitUntil(robot.claw.get_photosensor),
                        robot.claw.close()
                    )
                ),
                robot.claw.close(),
                WaitCommand(0.3),
                robot.arm.set_position(ArmPosition.HOME),
                drive_trajectory(robot, None),
                place_high(robot),
                WaitCommand(0.2)
        )
    
    # Robot drives back at the end either way
    return seq.andThen(
        drive_trajectory(robot, None)
    )

@auto.route("SubConeCube")
def sub_cone_cube(robot):
    return sub(robot, True)

@auto.route("SubConeCube")
def sub_cone_leave(robot):
    return sub(robot, False)

    
@auto.route("MidCubeBalance")
def cube_balance(robot: "Robot"):
    return sequence(
        place_high(robot)
    )

def bump(robot, fetch_cube: bool):
    seq = sequence(
        place_high(robot),
        WaitCommand(0.2),
    )

    # Makes grabbing the cube optional
    if fetch_cube:
        seq = seq.andThen(
            drive_trajectory(robot, None).deadlineWith(
                    sequence(
                        WaitCommand(0.7),
                        robot.arm.set_position(ArmPosition.BACK),
                        robot.claw.open(),
                        waitUntil(robot.claw.get_photosensor),
                    )
                ),
                robot.claw.close(),
                WaitCommand(0.3),
                robot.arm.set_position(ArmPosition.HOME),
                drive_trajectory(robot, None),
                place_high(robot),
                WaitCommand(0.2)
        )
    
    # Robot drives back at the end either way
    return seq.andThen(
        drive_trajectory(robot, None)
    )

@auto.route("BumpConeCube")
def bump_cone_cube(robot):
    return bump(robot, True)

@auto.route("BumpConeCube")
def bump_cone_leave(robot):
    return bump(robot, False)
