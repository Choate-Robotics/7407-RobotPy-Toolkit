import math

from robotpy_toolkit_7407.unum import Unum
from wpimath.geometry import Rotation2d, Pose2d, Translation2d
from wpimath.kinematics import SwerveDrive4Odometry, SwerveDrive4Kinematics, SwerveModuleState, ChassisSpeeds

from robotpy_toolkit_7407.oi.joysticks import JoystickAxis
from robotpy_toolkit_7407.subsystem import Subsystem
from robotpy_toolkit_7407.utils import logger
from robotpy_toolkit_7407.utils.math import rotate_vector, bounded_angle_diff
from robotpy_toolkit_7407.utils.units import s, m, deg, rad, hour, mile, rev, meters, meters_per_second, \
    radians_per_second, radians


class SwerveNode:
    _motor_reversed: bool
    _motor_sensor_offset: radians

    def init(self):
        self._motor_reversed = False
        self._motor_sensor_offset = 0

    def set(self, vel: meters_per_second, angle_radians: radians_per_second):
        self._set_angle(angle_radians, self.get_current_motor_angle() + self._motor_sensor_offset)
        self.set_motor_velocity(vel if not self._motor_reversed else -vel)

    # OVERRIDDEN FUNCTIONS
    def set_motor_angle(self, pos: radians): ...
    def get_current_motor_angle(self) -> radians: ...
    def set_motor_velocity(self, vel: meters_per_second): ...
    def get_motor_velocity(self) -> meters_per_second: ...

    # 0 degrees is facing right
    def _set_angle(self, target_angle: radians, initial_angle: radians):
        target_sensor_angle, flipped, flip_sensor_offset = SwerveNode._resolve_angles(target_angle, initial_angle)

        target_sensor_angle -= self._motor_sensor_offset

        if flipped:
            self._motor_reversed = not self._motor_reversed
            self._motor_sensor_offset += flip_sensor_offset

        self.set_motor_angle(target_sensor_angle)

    @staticmethod
    def _resolve_angles(target_angle: radians, initial_angle: radians) -> tuple[float, bool, float]:
        """
        :param target_angle: Target node angle
        :param initial_angle: Initial node sensor angle
        :return: (target_sensor_angle, flipped, flip_sensor_offset)
        """

        # Actual angle difference in radians
        diff = bounded_angle_diff(initial_angle, target_angle)

        # Should we flip
        if abs(diff) > 0.65 * math.pi:
            flip_sensor_offset = math.pi if diff > 0 else -math.pi
            diff -= flip_sensor_offset
            return diff + initial_angle, True, flip_sensor_offset

        return diff + initial_angle, False, 0


class SwerveGyro:
    def init(self): ...
    def get_robot_heading(self) -> radians: ...
    def reset_angle(self): ...


class SwerveDrivetrain(Subsystem):
    n_00: SwerveNode  # Top Left
    n_01: SwerveNode  # Bottom Left
    n_10: SwerveNode  # Top Right
    n_11: SwerveNode  # Bottom Right
    gyro: SwerveGyro
    axis_dx: JoystickAxis
    axis_dy: JoystickAxis
    axis_rotation: JoystickAxis
    track_width: meters = 1
    max_vel: meters_per_second = (20 * mile/hour).asNumber(m/s)
    max_angular_vel: radians_per_second = (4 * rev/s).asNumber(rad/s)
    deadzone_velocity: meters_per_second = 0.05
    deadzone_angular_velocity: radians_per_second = (5 * deg/s).asNumber(rad/s)
    start_pose: Pose2d = Pose2d(0, 0, 0)

    def __init__(self):
        super().__init__()
        self.kinematics: SwerveDrive4Kinematics | None = None
        self.odometry: SwerveDrive4Odometry | None = None
        self.chassis_speeds: ChassisSpeeds | None = None
        self._omega: radians_per_second = 0

    def init(self):
        logger.info("initializing swerve drivetrain", "[swerve_drivetrain]")
        self.n_00.init()
        self.n_01.init()
        self.n_10.init()
        self.n_11.init()
        self.gyro.init()
        logger.info("initializing odometry", "[swerve_drivetrain]")
        self.kinematics = SwerveDrive4Kinematics(
            Translation2d(-.5 * self.track_width, -.5 * self.track_width),
            Translation2d(-.5 * self.track_width, .5 * self.track_width),
            Translation2d(.5 * self.track_width, -.5 * self.track_width),
            Translation2d(.5 * self.track_width, .5 * self.track_width)
        )
        self.odometry = SwerveDrive4Odometry(
            self.kinematics,
            Rotation2d(self.gyro.get_robot_heading()),
            self.start_pose
        )
        logger.info("initialization complete", "[swerve_drivetrain]")

    def set_driver_centric(self, vel: (meters_per_second, meters_per_second), angular_vel: radians_per_second):
        vel = rotate_vector(vel[0], vel[1], -self.gyro.get_robot_heading())
        self.set_robot_centric(vel, angular_vel)

    def set_robot_centric(self, vel: (meters_per_second, meters_per_second), angular_vel: radians_per_second):
        self._omega = angular_vel  # For simulation

        if abs(vel[0]) < self.deadzone_velocity and abs(vel[1]) < self.deadzone_velocity and \
                abs(angular_vel) < self.deadzone_angular_velocity:
            self.n_00.set_motor_velocity(0)
            self.n_01.set_motor_velocity(0)
            self.n_10.set_motor_velocity(0)
            self.n_11.set_motor_velocity(0)
        else:
            self.n_00.set(*self._calculate_swerve_node(
                -.5 * self.track_width, -.5 * self.track_width,
                vel[0], vel[1], angular_vel
            ))
            self.n_01.set(*self._calculate_swerve_node(
                -.5 * self.track_width, .5 * self.track_width,
                vel[0], vel[1], angular_vel
            ))
            self.n_10.set(*self._calculate_swerve_node(
                .5 * self.track_width, -.5 * self.track_width,
                vel[0], vel[1], angular_vel
            ))
            self.n_11.set(*self._calculate_swerve_node(
                .5 * self.track_width, .5 * self.track_width,
                vel[0], vel[1], angular_vel
            ))

        module_states = (
            SwerveModuleState(
                self.n_00.get_motor_velocity(),
                Rotation2d(self.n_00.get_current_motor_angle())
            ), SwerveModuleState(
                self.n_01.get_motor_velocity(),
                Rotation2d(self.n_01.get_current_motor_angle())
            ), SwerveModuleState(
                self.n_10.get_motor_velocity(),
                Rotation2d(self.n_10.get_current_motor_angle())
            ), SwerveModuleState(
                self.n_11.get_motor_velocity(),
                Rotation2d(self.n_11.get_current_motor_angle())
            )
        )

        self.odometry.update(Rotation2d(self.gyro.get_robot_heading()), *module_states)
        self.chassis_speeds = self.kinematics.toChassisSpeeds(module_states)

    def stop(self):
        self.n_00.set(0, 0)
        self.n_01.set(0, 0)
        self.n_10.set(0, 0)
        self.n_11.set(0, 0)

    @staticmethod
    def _calculate_swerve_node(node_x: meters, node_y: meters, dx: meters_per_second, dy: meters_per_second, d_theta: radians_per_second) -> (meters_per_second, radians):
        tangent_x, tangent_y = -node_y, node_x
        tangent_m = math.sqrt(tangent_x**2 + tangent_y**2)
        tangent_x /= tangent_m
        tangent_y /= tangent_m

        r = math.sqrt(2) / 2
        sx = dx + r * d_theta * tangent_x
        sy = dy + r * d_theta * tangent_y

        theta = math.atan2(sy, sx)
        magnitude = math.sqrt(sx ** 2 + sy ** 2)
        return magnitude, theta
