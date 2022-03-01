from fast_unit import Unum


class Motor:
    def init(self): ...
    def set_raw_output(self, x: float): ...


class EncoderMotor(Motor):
    def get_sensor_position(self) -> Unum: ...
    def get_sensor_velocity(self) -> Unum: ...
    def set_sensor_position(self, pos: Unum): ...


class PIDMotor(EncoderMotor):
    def set_target_position(self, pos: Unum): ...
    def set_target_velocity(self, vel: Unum): ...
