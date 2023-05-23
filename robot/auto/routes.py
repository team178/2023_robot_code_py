from commands2 import Command, WaitCommand
from commands2.cmd import sequence, waitUntil

from robot.auto.trajectory import DriveTrajectory, load_trajectory
from robot.subsystems.arm import ArmPosition
from robot.auto.selector import AutoSelector as auto

"""
All of our autonomous routes are defined here.

To create an auto route, the static `route` decorator of the `AutoSelector`
class can be used on a function or class, which takes `robot` as it's first
parameter. See `AutoSelector.route`'s docstring in `selector.py` to see more.

---

Why did I do it this way? Well, for one, it's a lot cleaner than making a bunch
of classes for each individual command.

It's also a way to quickly make new auto commands, without having to hardcode 
function or class Command into our AutoSelector to show it as an option.
Python's decorators make it really easy to make easily expandable systems such
as this.
"""

# Trajectories
SUB_TO_CUBE = load_trajectory("SubDriveToCube", 1.75, 5, True)
FUNNY = load_trajectory("SubDriveToCube", 1.75, 5, True)
SUB_TO_GRID = load_trajectory("SubDriveToGrid", 1.5, 5)

BUMP_TO_CUBE = load_trajectory("BumpDriveToCube", 1.0, 5, True)
BUMP_TO_GRID = load_trajectory("BumpDriveToGrid", 1.0, 5)

GET_ON_CHARGE = load_trajectory("MidGetOnCharge", 1.75, True)


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
def cube_balance(robot) -> Command:
    # fmt: off
    return sequence(
        place_high(robot),
        WaitCommand(1),
        robot.drivetrain.reset_level(),
        DriveTrajectory(robot, GET_ON_CHARGE),
        robot.drivetrain.drive_until_level(-0.4)
    )
    # fmt: on


@auto.route("SubConeCube")
def sub_cone_cube(robot) -> Command:
    return sequence(
        robot.drivetrain.reset_pose(SUB_TO_CUBE.get_initial_state),
        place_high(robot),
        WaitCommand(0.2),
        DriveTrajectory(robot, SUB_TO_CUBE).deadlineWith(
            sequence(
                WaitCommand(0.7),
                robot.arm.set_position(ArmPosition.BACK),
                robot.claw.open(),
                waitUntil(robot.claw.get_photosensor).andThen(robot.claw.close())
            )
        ),
        robot.claw.close(),
        WaitCommand(0.3),
        robot.arm.set_position(ArmPosition.HOME),
        DriveTrajectory(robot, SUB_TO_GRID),
        place_high(robot),
        WaitCommand(0.2),
        DriveTrajectory(robot, FUNNY),
    )


@auto.route("SubConeLeave")
def sub_cone_leave(robot) -> Command:
    # fmt: off
    return sequence(
        place_high(robot),
        WaitCommand(0.2),
        DriveTrajectory(robot, SUB_TO_CUBE))
    # fmt: on


@auto.route("BumpConeCube")
def bump_cone_cube(robot) -> Command:
    return sequence(
        place_high(robot),
        WaitCommand(0.2),
        DriveTrajectory(robot, BUMP_TO_CUBE).deadlineWith(
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
        DriveTrajectory(robot, BUMP_TO_GRID),
        place_high(robot),
    )


@auto.route("BumpConeLeave")
def bump_cone_leave(robot) -> Command:
    # fmt: off
    return sequence(
        place_high(robot),
        WaitCommand(0.2),
        DriveTrajectory(robot, BUMP_TO_CUBE))
    # fmt: on
