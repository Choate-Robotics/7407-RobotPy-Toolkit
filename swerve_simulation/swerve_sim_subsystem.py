import math

from robotpy_toolkit_7407.unum import Unum
from wpimath.geometry import Pose2d

from robotpy_toolkit_7407.motors import TestMotor
from robotpy_toolkit_7407.oi import JoystickAxis
from robotpy_toolkit_7407.subsystem_templates.drivetrain import SwerveNode, SwerveGyro, SwerveDrivetrain, \
    DriveSwerve
from robotpy_toolkit_7407.utils.units import rad, m, s, rev, mile, hour, radians, radians_per_meter, radians_per_second, \
    radians_per_second_squared, meters_per_second


#   ----   DIRECT CONTROL   ----
# class TestSwerveNode(SwerveNode):
#     vel: float
#     angle: float
#
#     def __init__(self, angle_offset: float = 0):
#         self.angle_offset = angle_offset
#
#     def init(self):
#         super().init()
#         self.vel = 0
#         self.angle = 0
#
#     def update(self, dt=0.02):
#         pass
#
#     def set_angle_raw(self, pos: float):
#         self.angle = pos - self.angle_offset
#
#     def set_velocity_raw(self, vel_tw_per_second: float):
#         self.vel = vel_tw_per_second
#
#     def get_current_angle_raw(self) -> float:
#         return self.angle
#
#     def get_current_velocity(self) -> float:
#         return self.vel


#   ----   REALISTIC CONTROL   ----


class TestSwerveNode(SwerveNode):
    m_turn: TestMotor
    m_move: TestMotor
    angle_offset: radians
    gear_ratio: radians_per_meter = 1

    def __init__(self, angle_offset: radians = 0, max_angular_vel: radians_per_second = 20,
                 max_vel: radians_per_second = 5, max_accel: radians_per_second_squared = 50):
        self.angle_offset = angle_offset
        self.m_turn = TestMotor(max_angular_vel)
        self.m_move = TestMotor(max_vel, max_accel)

    def init(self):
        super().init()
        self.m_turn.init()
        self.m_move.init()

    def update(self, dt=0.02):
        self.m_turn.update(dt)
        self.m_move.update(dt)

    def set_motor_angle(self, pos: radians):
        self.m_turn.set_target_position(pos - self.angle_offset)

    def set_motor_velocity(self, vel: meters_per_second):
        self.m_move.set_target_velocity(vel * self.gear_ratio)

    def get_current_motor_angle(self) -> radians:
        return self.m_turn.get_sensor_position() + self.angle_offset

    def get_motor_velocity(self) -> meters_per_second:
        return self.m_move.get_sensor_velocity() / self.gear_ratio


class TestGyro(SwerveGyro):
    orientation: radians

    def init(self):
        self.orientation = 0

    def get_robot_heading(self) -> radians:
        return self.orientation

    def reset_angle(self):
        self.orientation = 0


class TestJoystickAxis(JoystickAxis):
    def __init__(self):
        self.val = 0

    @property
    def value(self) -> float:
        return self.val


class SimDrivetrain(SwerveDrivetrain):
    n_00 = TestSwerveNode()
    n_01 = TestSwerveNode()
    n_10 = TestSwerveNode()
    n_11 = TestSwerveNode()
    axis_dx = TestJoystickAxis()
    axis_dy = TestJoystickAxis()
    axis_rotation = TestJoystickAxis()
    gyro = TestGyro()
    max_vel = (20 * mile/hour).asNumber(m/s)
    max_angular_vel = (2 * rev/s).asNumber(rad/s)
    start_pose = Pose2d(2, 2, 0)

