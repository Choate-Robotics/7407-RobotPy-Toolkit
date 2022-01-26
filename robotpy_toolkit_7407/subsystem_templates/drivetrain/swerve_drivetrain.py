import math

from robotpy_toolkit_7407.oi.joysticks import JoystickAxis
from robotpy_toolkit_7407.subsystem import Subsystem
from robotpy_toolkit_7407.utils import logger
from robotpy_toolkit_7407.utils.math import rotate_vector, bounded_angle_diff


class SwerveNode:
    motor_reversed: bool
    motor_sensor_offset: float

    def init(self):
        self.motor_reversed = False
        self.motor_sensor_offset = 0

    def set(self, vel_tw_per_second: float, angle_radians: float):
        self._set_angle_radians(angle_radians, self.get_current_angle_raw() + self.motor_sensor_offset)
        self.set_velocity_raw(vel_tw_per_second)

    # OVERRIDDEN FUNCTIONS
    def set_angle_raw(self, pos: float): ...
    def get_current_angle_raw(self) -> float: ...
    def set_velocity_raw(self, vel_tw_per_second: float): ...
    def get_current_velocity(self) -> float: ...

    # 0 degrees is facing right
    def _set_angle_radians(self, target_radians: float, initial_radians: float):
        target_sensor_angle, flipped, flip_sensor_offset = SwerveNode._resolve_angles(target_radians, initial_radians)

        target_sensor_angle -= self.motor_sensor_offset

        if flipped:
            self.motor_reversed = not self.motor_reversed
            self.motor_sensor_offset += flip_sensor_offset

        self.set_angle_raw(target_sensor_angle)

    @staticmethod
    def _resolve_angles(target_angle, initial_angle) -> (float, bool, float):
        """
        :param target_angle: Target node angle
        :param initial_angle: Initial node sensor angle
        :return: (target_sensor_angle, flipped, flip_sensor_offset)
        """

        # Actual angle difference in radians
        diff = bounded_angle_diff(initial_angle, target_angle)

        # Should we flip
        if abs(diff) > 0.65 * math.pi:
            # logger.info(f"FLIPPED initial={initial_angle} target={target_angle} diff={diff}")
            flip_sensor_offset = math.pi if diff > 0 else -math.pi
            diff -= flip_sensor_offset
            return diff + initial_angle, True, flip_sensor_offset

        # logger.info(f"initial={initial_angle} target={target_angle} diff={diff}")
        return diff + initial_angle, False, 0


class SwerveOdometry:
    def init(self): ...
    def get_robot_angle_degrees(self) -> float: ...
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
    track_width: float

    def init(self):
        logger.info("initializing swerve drivetrain", "[swerve_drivetrain]")
        self.n_00.init()
        self.n_01.init()
        self.n_10.init()
        self.n_11.init()
        self.odometry.init()
        logger.info("initialization complete", "[swerve_drivetrain]")

    def set(self, vel_tw_per_second: tuple[float, float], angular_vel: float):
        vel_tw_per_second = rotate_vector(
            vel_tw_per_second[0],
            vel_tw_per_second[1],
            -self.odometry.get_robot_angle_degrees() * (math.pi / 180)
        )

        # logger.info(f"ROBOT AT {self.odometry.get_robot_angle_degrees()} degrees offset")
        # logger.info(f"({vel_tw_per_second[0]} tw/sec, {vel_tw_per_second[1]} tw/sec, {angular_vel} rad/sec)")

        if abs(vel_tw_per_second[0]) < 0.1 and abs(vel_tw_per_second[1]) < 0.1 and abs(angular_vel) < 0.1:
            self.n_00.set_velocity_raw(0)
            self.n_01.set_velocity_raw(0)
            self.n_10.set_velocity_raw(0)
            self.n_11.set_velocity_raw(0)
            return

        self.n_00.set(*self._swerve_displacement(-1, -1, vel_tw_per_second[0], vel_tw_per_second[1], angular_vel, 0))
        self.n_01.set(*self._swerve_displacement(-1, 1, vel_tw_per_second[0], vel_tw_per_second[1], angular_vel, 0))
        self.n_10.set(*self._swerve_displacement(1, -1, vel_tw_per_second[0], vel_tw_per_second[1], angular_vel, 0))
        self.n_11.set(*self._swerve_displacement(1, 1, vel_tw_per_second[0], vel_tw_per_second[1], angular_vel, 0))

    def stop(self):
        self.n_00.set(0, 0)
        self.n_01.set(0, 0)
        self.n_10.set(0, 0)
        self.n_11.set(0, 0)

    @staticmethod
    def _swerve_displacement(node_x: float, node_y: float, dx: float, dy: float, d_theta: float, angle_offset: float) -> tuple[float, float]:
        tangent_x, tangent_y = -node_y, node_x
        tangent_m = math.sqrt(tangent_x*tangent_x + tangent_y*tangent_y)
        tangent_x /= tangent_m
        tangent_y /= tangent_m

        r = math.sqrt(2) / 2
        sx = dx + r * d_theta * tangent_x
        sy = dy + r * d_theta * tangent_y

        theta = math.atan2(sy, sx)
        magnitude = math.sqrt(sx * sx + sy * sy)
        return magnitude, theta + angle_offset
