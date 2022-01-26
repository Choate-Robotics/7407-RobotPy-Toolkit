import math

from robotpy_toolkit_7407.motors import TestMotor
from robotpy_toolkit_7407.oi import JoystickAxis
from robotpy_toolkit_7407.subsystem_templates.drivetrain import SwerveNode, SwerveOdometry, SwerveDrivetrain, \
    DriveSwerve


#   ----   DIRECT CONTROL   ----
class TestSwerveNode(SwerveNode):
    vel: float
    angle: float

    def __init__(self, angle_offset: float = 0):
        self.angle_offset = angle_offset

    def init(self):
        super().init()
        self.vel = 0
        self.angle = 0

    def update(self, dt=0.02):
        pass

    def set_angle_raw(self, pos: float):
        self.angle = pos - self.angle_offset

    def set_velocity_raw(self, vel_tw_per_second: float):
        self.vel = vel_tw_per_second

    def get_current_angle_raw(self) -> float:
        return self.angle

    def get_current_velocity(self) -> float:
        return self.vel


#   ----   REALISTIC CONTROL   ----
# class TestSwerveNode(SwerveNode):
#     m_turn: TestMotor
#     m_move: TestMotor
#     angle_offset: float
#
#     def __init__(self, angle_offset: float = 0, max_angular_vel: float = 6*math.pi, max_vel: float = 5):
#         self.angle_offset = angle_offset
#         self.m_turn = TestMotor(max_angular_vel)
#         self.m_move = TestMotor(max_vel)
#
#     def init(self):
#         super().init()
#         self.m_turn.init()
#         self.m_move.init()
#
#     def update(self, dt=0.02):
#         self.m_turn.update(dt)
#         self.m_move.update(dt)
#
#     def set_angle_raw(self, pos: float):
#         self.m_turn.set_target_position(pos - self.angle_offset)
#
#     def set_velocity_raw(self, vel_tw_per_second: float):
#         if self.motor_reversed:
#             self.m_move.set_target_velocity(-vel_tw_per_second)
#         else:
#             self.m_move.set_target_velocity(vel_tw_per_second)
#
#     def get_current_angle_raw(self) -> float:
#         return self.m_turn.get_sensor_position() + self.angle_offset
#
#     def get_current_velocity(self) -> float:
#         return abs(self.m_move.get_sensor_velocity())


class TestOdometry(SwerveOdometry):
    angle_radians: float

    def init(self):
        self.angle_radians = 0

    def get_robot_angle_degrees(self) -> float:
        return self.angle_radians * (180 / math.pi)

    def reset_angle(self):
        self.angle_radians = 0


class TestJoystickAxis(JoystickAxis):
    def __init__(self):
        self.val = 0

    @property
    def value(self) -> float:
        return self.val


class Drivetrain(SwerveDrivetrain):
    n_00 = TestSwerveNode(0)
    n_01 = TestSwerveNode(0)
    n_10 = TestSwerveNode(0)
    n_11 = TestSwerveNode(0)
    axis_dx = TestJoystickAxis()
    axis_dy = TestJoystickAxis()
    axis_rotation = TestJoystickAxis()
    odometry = TestOdometry()


drivetrain = Drivetrain()
drivetrain.init()

drive_command = DriveSwerve(drivetrain)
drive_command.initialize()

