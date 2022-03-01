from dataclasses import dataclass

from fast_unit import Unum
from wpimath.geometry import Pose2d, Translation2d
from wpimath.trajectory import TrajectoryGenerator, TrajectoryConfig, Trajectory

from robotpy_toolkit_7407.utils.units import m, rad, s


def translation(x: Unum, y: Unum) -> Translation2d:
    return Translation2d(x.as_number(m), y.as_number(m))


@dataclass
class TrajectoryEndpoint:
    x: Unum = 0 * m
    y: Unum = 0 * m
    angle: Unum = 0 * rad
    vel: Unum = 0 * m/s

    def as_pose(self):
        return Pose2d(self.x.as_number(m), self.y.as_number(m), self.angle.as_number(rad))


class SimTrajectory:
    @staticmethod
    def generate_trajectory(start: TrajectoryEndpoint, waypoints: list[Translation2d], end: TrajectoryEndpoint,
                            max_vel: Unum, max_accel: Unum) -> Trajectory:
        config = TrajectoryConfig(max_vel.as_number(m/s), max_accel.as_number(m/(s*s)))
        config.setStartVelocity(start.vel.as_number(m/s))
        config.setEndVelocity(end.vel.as_number(m/s))
        return TrajectoryGenerator.generateTrajectory(start.as_pose(), waypoints, end.as_pose(), config)
