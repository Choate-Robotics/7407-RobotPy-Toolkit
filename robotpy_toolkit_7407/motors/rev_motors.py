from dataclasses import dataclass
from typing import Optional

from rev import CANSparkMax, SparkMaxPIDController, SparkMaxRelativeEncoder
from robotpy_toolkit_7407.unum import Unum

from robotpy_toolkit_7407.motor import PIDMotor
from robotpy_toolkit_7407.utils.units import rev, minute, radians, radians_per_second, rad, s


@dataclass
class SparkMaxConfig:
    k_P: Optional[float] = None
    k_I: Optional[float] = None
    k_D: Optional[float] = None
    k_F: Optional[float] = None
    output_range: Optional[tuple[float, float]] = None
    idle_mode: Optional[CANSparkMax.IdleMode] = None


k_sensor_pos_to_radians = rev.asNumber(rad)
k_radians_to_sensor_pos = rad.asNumber(rev)
k_sensor_vel_to_rad_per_sec = (rev/minute).asNumber(rad/s)
k_rad_per_sec_to_sensor_vel = (rad/s).asNumber(rev/minute)


class SparkMax(PIDMotor):
    _motor: CANSparkMax
    __pid_controller: SparkMaxPIDController
    __encoder: SparkMaxRelativeEncoder

    def __init__(self, can_id: int, inverted: bool = True, brushless: bool = True, config: SparkMaxConfig = None):
        super().__init__()
        self._can_id = can_id
        self._inverted = inverted
        self._brushless = brushless
        self._config = config

    def init(self):
        self._motor = CANSparkMax(
            self._can_id,
            CANSparkMax.MotorType.kBrushless if self._brushless else CANSparkMax.MotorType.kBrushed
        )
        self._motor.setInverted(self._inverted)
        self.__pid_controller = self._motor.getPIDController()
        self.__encoder = self._motor.getEncoder()
        self._set_config(self._config)

    def set_raw_output(self, x: float):
        self._motor.set(x)

    def set_target_position(self, pos: radians):
        self.__pid_controller.setReference(pos * k_radians_to_sensor_pos, CANSparkMax.ControlType.kPosition)

    def set_target_velocity(self, vel: radians_per_second):
        self.__pid_controller.setReference(vel * k_rad_per_sec_to_sensor_vel, CANSparkMax.ControlType.kVelocity)

    def get_sensor_position(self) -> radians:
        return self.__encoder.getPosition() * k_sensor_pos_to_radians

    def set_sensor_position(self, pos: radians):
        self.__encoder.setPosition(pos * k_radians_to_sensor_pos)

    def get_sensor_velocity(self) -> radians_per_second:
        return self.__encoder.getVelocity() * k_sensor_vel_to_rad_per_sec

    def _set_config(self, config: SparkMaxConfig):
        if config is None:
            return
        if config.k_P is not None:
            self.__pid_controller.setP(config.k_P)
        if config.k_I is not None:
            self.__pid_controller.setI(config.k_I)
        if config.k_D is not None:
            self.__pid_controller.setD(config.k_D)
        if config.k_F is not None:
            self.__pid_controller.setFF(config.k_F)
        if config.output_range is not None:
            self.__pid_controller.setOutputRange(config.output_range[0], config.output_range[1])
        if config.idle_mode is not None:
            self._motor.setIdleMode(config.idle_mode)
