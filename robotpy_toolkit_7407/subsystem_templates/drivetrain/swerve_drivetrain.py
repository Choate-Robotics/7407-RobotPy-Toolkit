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
    """
    Extendable class for swerve node.
    """
    motor_reversed: bool = False
    motor_sensor_offset: radians = 0

    def init(self):
        """
        Initialize the swerve node.
        """
        ...

    def set(self, vel: meters_per_second, angle_radians: radians_per_second):
        """
        Set the velocity and angle of the swerve node.

        Args:
            vel (meters_per_second): velocity of the swerve node
            angle_radians (radians_per_second): turning swerve node velocity in radians per second
        """
        self._set_angle(angle_radians, self.get_current_motor_angle() + self.motor_sensor_offset)
        self.set_motor_velocity(vel if not self.motor_reversed else -vel)

    # OVERRIDDEN FUNCTIONS
    def set_motor_angle(self, pos: radians):
        """
        Set the angle of the swerve node. Must be overridden.

        Args:
            pos (radians): angle of the swerve node in radians
        """
        ...

    def get_current_motor_angle(self) -> radians:
        """
        Get the current angle of the swerve node. Must be overridden. Must return radians.
        """
        ...

    def set_motor_velocity(self, vel: meters_per_second):
        """
        Set the velocity of the swerve node. Must be overridden.
        Args:
            vel (meters_per_second): velocity of the swerve node in meters per second
        """
        ...

    def get_motor_velocity(self) -> meters_per_second:
        """
        Get the velocity of the swerve node. Must be overridden. Must return meters per second.
        """
        ...

    # 0 degrees is facing right | "ethan is our FRC lord and saviour" - sid
    def _set_angle(self, target_angle: radians, initial_angle: radians):
        target_sensor_angle, flipped, flip_sensor_offset = SwerveNode._resolve_angles(target_angle, initial_angle)

        target_sensor_angle -= self.motor_sensor_offset

        if flipped:
            self.motor_reversed = not self.motor_reversed
            self.motor_sensor_offset += flip_sensor_offset

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
    """
    Extendable class for swerve gyro.
    """

    def init(self):
        """
        Initialize the swerve gyro. Overridden class.
        """
        ...

    def get_robot_heading(self) -> radians:
        """
        Get the robot heading in radians. Overridden class. Must return radians.
        """
        ...

    def reset_angle(self):
        """
        Reset the robot heading. Overridden class.
        """
        ...


class SwerveDrivetrain(Subsystem):
    """
    Swerve Drivetrain Extendable class. Contains driving functions.
    """
    n_00: SwerveNode  # Top Left
    n_01: SwerveNode  # Bottom Left
    n_10: SwerveNode  # Top Right
    n_11: SwerveNode  # Bottom Right
    gyro: SwerveGyro
    axis_dx: JoystickAxis
    axis_dy: JoystickAxis
    axis_rotation: JoystickAxis
    track_width: meters = 1
    max_vel: meters_per_second = (20 * mile / hour).asNumber(m / s)  # Maximum velocity
    max_angular_vel: radians_per_second = (4 * rev / s).asNumber(rad / s)  # Maximum angular velocity
    deadzone_velocity: meters_per_second = 0.05  # Does not run within this speed
    deadzone_angular_velocity: radians_per_second = (5 * deg / s).asNumber(rad / s)  # Will not turn within this speed
    start_pose: Pose2d = Pose2d(0, 0, 0)  # Starting pose of the robot from wpilib Pose (x, y, rotation)

    def __init__(self):
        super().__init__()
        self.kinematics: SwerveDrive4Kinematics | None = None
        self.odometry: SwerveDrive4Odometry | None = None
        self.chassis_speeds: ChassisSpeeds | None = None
        self._omega: radians_per_second = 0

    def init(self):
        """
        Initialize the swerve drivetrain, kinematics, odometry, and gyro.
        """
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
        """
        Set the driver centric velocity and angular velocity. Driver centric runs with perspective of driver.

        Args:
            vel: velocity in x and y direction as (meters per second, meters per second)
            angular_vel: angular velocity in radians per second
        """
        vel = rotate_vector(vel[0], vel[1], -self.gyro.get_robot_heading())
        self.set_robot_centric(vel, angular_vel)

    def set_robot_centric(self, vel: (meters_per_second, meters_per_second), angular_vel: radians_per_second):
        """
        Set the robot centric velocity and angular velocity. Robot centric runs with perspective of robot.
        Args:
            vel: velocity in x and y direction as (meters per second, meters per second)
            angular_vel: angular velocity in radians per second
        """
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
        """
        Stop the drivetrain and all pods.
        """
        self.n_00.set(0, 0)
        self.n_01.set(0, 0)
        self.n_10.set(0, 0)
        self.n_11.set(0, 0)

    @staticmethod
    def _calculate_swerve_node(node_x: meters, node_y: meters, dx: meters_per_second, dy: meters_per_second,
                               d_theta: radians_per_second) -> (meters_per_second, radians):
        tangent_x, tangent_y = -node_y, node_x
        tangent_m = math.sqrt(tangent_x ** 2 + tangent_y ** 2)
        tangent_x /= tangent_m
        tangent_y /= tangent_m

        r = math.sqrt(2) / 2
        sx = dx + r * d_theta * tangent_x
        sy = dy + r * d_theta * tangent_y

        theta = math.atan2(sy, sx)
        magnitude = math.sqrt(sx ** 2 + sy ** 2)
        return magnitude, theta
