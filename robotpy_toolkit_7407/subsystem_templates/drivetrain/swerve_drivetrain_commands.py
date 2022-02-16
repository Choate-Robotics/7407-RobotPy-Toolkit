import math
import time

from wpimath.controller import HolonomicDriveController, PIDController, ProfiledPIDControllerRadians
from wpimath.geometry import Rotation2d
from wpimath.trajectory import TrapezoidProfileRadians, Trajectory

from robotpy_toolkit_7407.command import SubsystemCommand
from robotpy_toolkit_7407.subsystem_templates.drivetrain.swerve_drivetrain import SwerveDrivetrain
from robotpy_toolkit_7407.utils import logger
from robotpy_toolkit_7407.utils.math import rotate_vector, bounded_angle_diff
from robotpy_toolkit_7407.utils.units import m, s, rad


class DriveSwerve(SubsystemCommand[SwerveDrivetrain]):
    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        dx, dy, d_theta = self.subsystem.axis_dx.value, self.subsystem.axis_dy.value, self.subsystem.axis_rotation.value

        # if abs(dx) < 0.1:
        #     dx = 0
        # if abs(dy) < 0.1:
        #     dy = 0
        # if abs(d_theta) < 0.1:
        #     d_theta = 0

        # TODO normalize this to circle somehow
        dx *= self.subsystem.max_vel.asUnit(m/s)
        dy *= -self.subsystem.max_vel.asUnit(m/s)

        self.subsystem.set((dx, dy), -d_theta * self.subsystem.max_angular_vel)

    def end(self, interrupted: bool) -> None:
        self.subsystem.stop()

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False


class FollowPath(SubsystemCommand[SwerveDrivetrain]):
    def __init__(self, subsystem: SwerveDrivetrain, trajectory: Trajectory):
        super().__init__(subsystem)
        self.trajectory = trajectory
        self.controller = HolonomicDriveController(
            PIDController(1, 0, 0),
            PIDController(1, 0, 0),
            ProfiledPIDControllerRadians(
                8, 0, 0, TrapezoidProfileRadians.Constraints(
                    subsystem.max_angular_vel.asNumber(rad/s),
                    (subsystem.max_angular_vel / (.01 * s)).asNumber(rad/(s**2))
                )
            )
        )
        self.start_time = 0
        self.t = 0
        self.duration = trajectory.totalTime()
        self.theta_i = trajectory.initialPose().rotation().radians() * rad
        self.theta_f = trajectory.sample(self.duration).pose.rotation().radians() * rad
        self.theta_diff = bounded_angle_diff(self.theta_i.asNumber(rad), self.theta_f.asNumber(rad)) * rad
        self.omega = self.theta_diff / (self.duration * s)

    def initialize(self) -> None:
        self.start_time = time.perf_counter()

    def execute(self) -> None:
        self.t = time.perf_counter() - self.start_time
        if self.t > self.duration:
            self.t = self.duration
        goal = self.trajectory.sample(self.t)
        goal_theta = self.theta_i + self.omega * self.t * s
        speeds = self.controller.calculate(self.subsystem.odometry.getPose(), goal, Rotation2d(goal_theta.asNumber(rad)))
        vx, vy = rotate_vector(
            speeds.vx * m/s, speeds.vy * m/s,
            self.subsystem.odometry.getPose().rotation().radians() * rad
        )
        self.subsystem.set((vx, vy), speeds.omega * rad/s)

    def end(self, interrupted: bool) -> None:
        pass

    def isFinished(self) -> bool:
        return self.t > self.trajectory.totalTime()

    def runsWhenDisabled(self) -> bool:
        return False
