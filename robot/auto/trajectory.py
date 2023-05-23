import math

from commands2 import CommandBase
from wpilib import DriverStation, Timer
from wpimath.controller import RamseteController
from wpimath.kinematics import ChassisSpeeds
from wpimath.trajectory import Trajectory
from wpimath.geometry import Pose2d, Rotation2d
from pathplannerlib import PathPlanner

from robot.constants import *


def mirror_trajectory(traj: Trajectory) -> Trajectory:
    """
    Mirror a Trajectory across the Y axis of the field. Requires the FIELD_LENGTH
    constant to be adjusted per-season.
    """
    return Trajectory(
        [
            Trajectory.State(
                state.t,
                state.velocity,
                state.acceleration,
                Pose2d(
                    FIELD_LENGTH - state.pose.X(),
                    state.pose.Y(),
                    (state.pose.rotation() * -1) + Rotation2d(math.radians(180)),
                ),
                state.curvature,
            )
            for state in traj.states()
        ]
    )


class Trajectories:
    """
    Creates a pair of trajectories. Can be used to switch between a trajectory and it's
    mirror when the drive command is run instead of when it is instantiated.
    """

    def __init__(self, trajectory: Trajectory):
        self._blue = trajectory
        self._red = mirror_trajectory(trajectory)

    @property
    def trajectory(self) -> Trajectory:
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            return self._red
        else:
            return self._blue
        
    def get_initial_state(self) -> Pose2d:
        return self.trajectory.initialPose()


def load_trajectory(name: str, *args, **kwargs) -> Trajectories:
    return Trajectories(
        PathPlanner.loadPath(name, *args, **kwargs).asWPILibTrajectory()
    )


class DriveTrajectory(CommandBase):
    """
    Uses a RamseteController to drive along a provided Trajectory, or along
    a Trajectory provided by a Trajectories object.
    """

    def __init__(self, robot, trajectory: Trajectories | Trajectory):
        super().__init__()
        self.robot = robot

        self.trajectory = trajectory

        self._timer = Timer()
        self._controller = RamseteController()

    def initialize(self):
        if isinstance(self.trajectory, Trajectories):
            self.trajectory = self.trajectory.trajectory

        self._prev_time = -1
        initial_state = self.trajectory.sample(0)
        self._prev_speeds = DRIVE_KINEMATICS.toWheelSpeeds(
            ChassisSpeeds(
                initial_state.velocity,
                0,
                initial_state.curvature * initial_state.velocity,
            )
        )
        self._timer.restart()

    def execute(self):
        cur_time = self._timer.get()

        if self._prev_time < 0:
            self.robot.drivetrain.set_wheel_speeds(0, 0)
            self._prev_time = cur_time
            return

        target_whl_spds = DRIVE_KINEMATICS.toWheelSpeeds(
            self._controller.calculate(
                self.robot.drivetrain.get_pose(), self.trajectory.sample(cur_time)
            )
        )

        self.robot.drivetrain.use_wheel_speeds(target_whl_spds)
        self._prev_speeds = target_whl_spds
        self._prev_time = cur_time

    def end(self, interupt: bool):
        self._timer.stop()

        if interupt:
            self.robot.drivetrain.set_wheel_speeds(0, 0)

    def isFinished(self):
        return self._timer.hasElapsed(self.trajectory.totalTime())
