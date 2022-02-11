import math

from unum import Unum

from robotpy_toolkit_7407.oi.joysticks import JoystickAxis
from robotpy_toolkit_7407.subsystem import Subsystem
from robotpy_toolkit_7407.utils import logger
from robotpy_toolkit_7407.utils.math import rotate_vector, bounded_angle_diff
from robotpy_toolkit_7407.utils.units import s, m, deg, rad, hour, mile, rev


class SwerveNode:
    _motor_reversed: bool
    _motor_sensor_offset: Unum = rad

    def init(self):
        self._motor_reversed = False
        self._motor_sensor_offset = 0 * rad

    def set(self, vel: Unum, angle_radians: Unum):
        self._set_angle(angle_radians, self.get_current_motor_angle() + self._motor_sensor_offset)
        self.set_motor_velocity(vel if not self._motor_reversed else -vel)

    # OVERRIDDEN FUNCTIONS
    def set_motor_angle(self, pos: Unum): ...
    def get_current_motor_angle(self) -> Unum: ...
    def set_motor_velocity(self, vel: Unum): ...
    def get_motor_velocity(self) -> Unum: ...

    # 0 degrees is facing right
    def _set_angle(self, target_angle: Unum, initial_angle: Unum):
        target_sensor_angle, flipped, flip_sensor_offset = SwerveNode._resolve_angles(target_angle, initial_angle)

        target_sensor_angle -= self._motor_sensor_offset

        if flipped:
            self._motor_reversed = not self._motor_reversed
            self._motor_sensor_offset += flip_sensor_offset

        self.set_motor_angle(target_sensor_angle)

    @staticmethod
    def _resolve_angles(target_angle: Unum, initial_angle: Unum) -> (float, bool, float):
        """
        :param target_angle: Target node angle
        :param initial_angle: Initial node sensor angle
        :return: (target_sensor_angle, flipped, flip_sensor_offset)
        """

        target_rad = target_angle.asNumber(rad)
        initial_rad = initial_angle.asNumber(rad)

        # Actual angle difference in radians
        diff = bounded_angle_diff(initial_rad, target_rad)

        # Should we flip
        if abs(diff) > 0.65 * math.pi:
            flip_sensor_offset = math.pi if diff > 0 else -math.pi
            diff -= flip_sensor_offset
            return (diff + initial_rad) * rad, True, flip_sensor_offset * rad

        return (diff + initial_rad) * rad, False, 0 * rad


class SwerveOdometry:
    def init(self): ...
    def get_robot_heading(self) -> Unum: ...
    def reset_angle(self): ...


class SwerveDrivetrain(Subsystem):
    n_00: SwerveNode  # Top Left
    n_01: SwerveNode  # Bottom Left
    n_10: SwerveNode  # Top Right
    n_11: SwerveNode  # Bottom Right
    odometry: SwerveOdometry
    axis_dx: JoystickAxis
    axis_dy: JoystickAxis
    axis_rotation: JoystickAxis
    track_width: Unum = 1 * m
    max_vel: Unum = 20 * mile/hour
    max_angular_vel: Unum = 4 * rev/s
    deadzone_velocity: Unum = 0.05 * m/s
    deadzone_angular_velocity: Unum = 5 * deg/s

    def init(self):
        logger.info("initializing swerve drivetrain", "[swerve_drivetrain]")
        self.n_00.init()
        self.n_01.init()
        self.n_10.init()
        self.n_11.init()
        self.odometry.init()
        logger.info("initialization complete", "[swerve_drivetrain]")

    def set(self, vel: (Unum, Unum), angular_vel: Unum):
        vel = rotate_vector(vel[0], vel[1], -self.odometry.get_robot_heading())

        # logger.info(f"ROBOT AT {self.odometry.get_robot_angle_degrees()} degrees offset")
        # logger.info(f"({vel_tw_per_second[0]} tw/sec, {vel_tw_per_second[1]} tw/sec, {angular_vel} rad/sec)")

        if abs(vel[0]) < self.deadzone_velocity and abs(vel[1]) < self.deadzone_velocity and \
                abs(angular_vel) < self.deadzone_angular_velocity:
            self.n_00.set_motor_velocity(0 * m/s)
            self.n_01.set_motor_velocity(0 * m/s)
            self.n_10.set_motor_velocity(0 * m/s)
            self.n_11.set_motor_velocity(0 * m/s)
            return

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

    def stop(self):
        self.n_00.set(0 * m/s, 0 * rad/s)
        self.n_01.set(0 * m/s, 0 * rad/s)
        self.n_10.set(0 * m/s, 0 * rad/s)
        self.n_11.set(0 * m/s, 0 * rad/s)

    @staticmethod
    def _calculate_swerve_node(node_x: Unum, node_y: Unum, dx: Unum, dy: Unum, d_theta: Unum) -> (Unum, Unum):
        tangent_x, tangent_y = -node_y, node_x
        tangent_m = math.sqrt(tangent_x.asNumber(m)**2 + tangent_y.asNumber(m)**2) * m
        tangent_x /= tangent_m / m
        tangent_y /= tangent_m / m

        r = math.sqrt(2) / 2
        sx = dx + r * d_theta * tangent_x
        sy = dy + r * d_theta * tangent_y

        sx_u = sx.asNumber(m/s)
        sy_u = sy.asNumber(m/s)

        theta = math.atan2(sy_u, sx_u) * rad
        magnitude = math.sqrt(sx_u ** 2 + sy_u ** 2) * m/s
        return magnitude, theta
