from robotpy_toolkit_7407.unum import Unum

from robotpy_toolkit_7407.motor import PIDMotor
from robotpy_toolkit_7407.utils.math import clamp
from robotpy_toolkit_7407.utils.units import rev, s, minute, rad, radians_per_second, radians_per_second_squared, \
    radians


class TestMotor(PIDMotor):
    class ControlMode:
        PERCENT = 0
        POSITION = 1
        VELOCITY = 2

    control_mode: int
    set_point: float | radians | radians_per_second
    current_pos: radians
    current_vel: radians_per_second
    prev_pos: radians
    prev_dt: float

    def __init__(self, max_vel: radians_per_second = 100, max_accel: radians_per_second_squared = 10):
        self.max_vel = max_vel
        self.max_accel = max_accel

    def init(self):
        self.control_mode = self.ControlMode.PERCENT
        self.set_point = 0
        self.current_pos = 0
        self.current_vel = 0
        self.prev_pos = 0
        self.prev_dt = 0

    def update(self, dt: float = 0.02):
        self.prev_pos = self.current_pos
        self.prev_dt = dt
        if self.control_mode == self.ControlMode.PERCENT:
            diff = self.set_point * self.max_vel - self.current_vel
            self.current_vel += clamp(diff, -self.max_accel * dt, self.max_accel * dt)
            self.current_pos += self.current_vel * dt
        if self.control_mode == self.ControlMode.POSITION:
            diff = self.set_point - self.current_pos
            self.current_pos += clamp(diff, -self.max_vel * dt, self.max_vel * dt)
        if self.control_mode == self.ControlMode.VELOCITY:
            diff = self.set_point - self.current_vel
            self.current_vel += clamp(diff, -self.max_accel * dt, self.max_accel * dt)
            self.current_pos += self.current_vel * dt

    def set_raw_output(self, x: float):
        self.control_mode = self.ControlMode.PERCENT
        self.set_point = x

    def set_target_position(self, pos: radians):
        self.control_mode = self.ControlMode.POSITION
        self.set_point = pos

    def set_target_velocity(self, vel: radians_per_second):
        self.control_mode = self.ControlMode.VELOCITY
        self.set_point = vel

    def get_sensor_position(self) -> radians:
        return self.current_pos

    def set_sensor_position(self, pos: radians):
        self.current_pos = pos

    def get_sensor_velocity(self) -> radians_per_second:
        if self.control_mode == self.ControlMode.VELOCITY:
            return self.current_vel
        if self.prev_dt == 0:
            return 0
        return (self.current_pos - self.prev_pos) / self.prev_dt
