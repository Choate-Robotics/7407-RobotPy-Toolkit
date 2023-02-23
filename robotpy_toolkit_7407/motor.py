from abc import abstractmethod

from robotpy_toolkit_7407.utils.units import radians, radians_per_second



class Motor():

    def init(self): ...

    def set_raw_output(self, x: float): ...


class EncoderMotor(Motor):
    @abstractmethod
    def get_sensor_position(self) -> radians: ...

    @abstractmethod
    def get_sensor_velocity(self) -> radians_per_second: ...

    @abstractmethod
    def set_sensor_position(self, pos: radians): ...


class PIDMotor(EncoderMotor):
    @abstractmethod
    def set_target_position(self, pos: radians): ...

    @abstractmethod
    def set_target_velocity(self, vel: radians_per_second): ...
