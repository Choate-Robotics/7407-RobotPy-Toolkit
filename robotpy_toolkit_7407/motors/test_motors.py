from robotpy_toolkit_7407.unum import Unum

from robotpy_toolkit_7407.motor import PIDMotor
from robotpy_toolkit_7407.utils.math import clamp
from robotpy_toolkit_7407.utils.units import rev, s, minute, rad


class TestMotor(PIDMotor):
    class ControlMode:
        PERCENT = 0
        POSITION = 1
        VELOCITY = 2

    control_mode: int
    set_point: Unum | float
    current_pos: Unum
    current_vel: Unum
    prev_pos: Unum
    prev_dt: Unum

    def __init__(self, max_vel: Unum = 100 * rad/s, max_accel: Unum = 10 * rad/s**2):
        self.max_vel = max_vel
        self.max_accel = max_accel

    def init(self):
        self.control_mode = self.ControlMode.PERCENT
        self.set_point = 0
        self.current_pos = 0 * rev
        self.current_vel = 0 * rev / s
        self.prev_pos = 0 * rev
        self.prev_dt = 0 * s

    def update(self, dt: Unum = 0.02 * s):
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

    def set_target_position(self, pos: Unum):
        self.control_mode = self.ControlMode.POSITION
        self.set_point = pos.asUnit(rev)

    def set_target_velocity(self, vel: Unum):
        self.control_mode = self.ControlMode.VELOCITY
        self.set_point = vel.asUnit(rev / s)

    def get_sensor_position(self) -> Unum:
        return self.current_pos

    def set_sensor_position(self, pos: Unum):
        self.current_pos = pos.asUnit(rev)

    def get_sensor_velocity(self) -> Unum:
        if self.control_mode == self.ControlMode.VELOCITY:
            return self.current_vel
        if self.prev_dt == 0:
            return 0 * rev/s
        return (self.current_pos - self.prev_pos) / self.prev_dt
