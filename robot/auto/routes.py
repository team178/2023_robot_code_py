from commands2 import Command, WaitCommand
from commands2.cmd import sequence, waitUntil

from robot.auto.trajectory import drive_trajectory
from robot.subsystems.arm import ArmPosition
from robot.auto.selector import AutoSelector as auto


@auto.route("PlaceHigh")
def place_high(robot) -> Command:
    return sequence(
        robot.claw.close(),
        robot.arm.set_position(ArmPosition.HIGH),
        WaitCommand(1.5),
        robot.claw.open(),
        WaitCommand(0.3),
        robot.arm.set_position(ArmPosition.HOME),
    )


@auto.route("MidCubeBalance")
def cube_balance(robot: "Robot"):
    return sequence(place_high(robot))


@auto.route("SubConeCube")
def sub_cone_cube(robot):
    return sequence(
        place_high(robot),
        WaitCommand(0.2),
        drive_trajectory(robot, None).deadlineWith(
            sequence(
                WaitCommand(0.7),
                robot.arm.set_position(ArmPosition.BACK),
                robot.claw.open(),
                waitUntil(robot.claw.get_photosensor),
                robot.claw.close(),
            )
        ),
        robot.claw.close(),
        WaitCommand(0.3),
        robot.arm.set_position(ArmPosition.HOME),
        drive_trajectory(robot, None),
        place_high(robot),
        WaitCommand(0.2),
        drive_trajectory(robot, None),
    )


@auto.route("SubConeLeave")
def sub_cone_leave(robot):
    # fmt: off
    return sequence(
        place_high(robot),
        WaitCommand(0.2),
        drive_trajectory(robot, None))
    # fmt: on


@auto.route("BumpConeCube")
def bump_cone_cube(robot):
    return sequence(
        place_high(robot),
        WaitCommand(0.2),
        drive_trajectory(robot, None).deadlineWith(
            sequence(
                WaitCommand(0.7),
                robot.arm.set_position(ArmPosition.BACK),
                robot.claw.open(),
                waitUntil(robot.claw.get_photosensor),
                robot.claw.close(),
            )
        ),
        robot.claw.close(),
        WaitCommand(0.3),
        robot.arm.set_position(ArmPosition.HOME),
        drive_trajectory(robot, None),
        place_high(robot),
        WaitCommand(0.2),
        drive_trajectory(robot, None),
    )


@auto.route("BumpConeLeave")
def bump_cone_leave(robot):
    # fmt: off
    return sequence(
        place_high(robot),
        WaitCommand(0.2),
        drive_trajectory(robot, None))
    # fmt: on
