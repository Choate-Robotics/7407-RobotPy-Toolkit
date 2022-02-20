from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import ctre
from robotpy_toolkit_7407.unum import Unum

from robotpy_toolkit_7407.motor import PIDMotor
from robotpy_toolkit_7407.utils.units import rad, rev, s


@dataclass
class TalonConfig:
    k_P: Optional[float] = None
    k_I: Optional[float] = None
    k_D: Optional[float] = None
    k_F: Optional[float] = None
    closed_loop_peak_output: Optional[float] = None
    motion_cruise_velocity: Optional[Unum] = None
    motion_acceleration: Optional[Unum] = None
    neutral_brake: Optional[bool] = None
    integral_zone: Optional[float] = None
    max_integral_accumulator: Optional[float] = None


talon_sensor_unit = Unum.unit("talon_sensor_u", rev / 2048, "talon sensor unit")
hundred_ms = Unum.unit("100ms", s / 10, "100 milliseconds")
talon_sensor_vel_unit = talon_sensor_unit / hundred_ms
talon_sensor_accel_unit = talon_sensor_vel_unit / s


class _Talon(PIDMotor):
    _motor: ctre.BaseTalon

    def __init__(self, can_id: int, inverted: bool = False, config: TalonConfig = None):
        super().__init__()
        self._can_id = can_id
        self._config = config
        self._inverted = inverted

    def get_sensor_position(self) -> Unum:
        return self._motor.getSelectedSensorPosition(0) * talon_sensor_unit

    def set_sensor_position(self, pos: Unum):
        self._motor.setSelectedSensorPosition(pos.asNumber(talon_sensor_unit))

    def get_sensor_velocity(self) -> Unum:
        return self._motor.getSelectedSensorVelocity(0) * talon_sensor_vel_unit

    def set_raw_output(self, x: float):
        self._motor.set(ctre.ControlMode.PercentOutput, x)

    def set_target_position(self, pos: Unum):
        self._motor.set(ctre.ControlMode.MotionMagic, pos.asNumber(talon_sensor_unit))

    def set_target_velocity(self, vel: Unum):
        self._motor.set(ctre.ControlMode.Velocity, vel.asNumber(talon_sensor_vel_unit))

    def follow(self, master: _Talon):
        self._motor.follow(master._motor)

    def _set_config(self, config: Optional[TalonConfig]):
        if config is None:
            return
        if config.k_P is not None:
            self._motor.config_kP(0, config.k_P)
        if config.k_I is not None:
            self._motor.config_kI(0, config.k_I)
        if config.k_D is not None:
            self._motor.config_kD(0, config.k_D)
        if config.k_F is not None:
            self._motor.config_kF(0, config.k_F)
        if config.closed_loop_peak_output is not None:
            self._motor.configClosedLoopPeakOutput(0, config.closed_loop_peak_output)
        if config.motion_cruise_velocity is not None:
            self._motor.configMotionCruiseVelocity(config.motion_cruise_velocity.asNumber(talon_sensor_vel_unit))
        if config.motion_acceleration is not None:
            self._motor.configMotionAcceleration(config.motion_acceleration.asNumber(talon_sensor_accel_unit))
        if config.neutral_brake is not None:
            self._motor.setNeutralMode(ctre.NeutralMode.Brake if config.neutral_brake else ctre.NeutralMode.Coast)
        if config.integral_zone is not None:
            self._motor.config_IntegralZone(0, config.integral_zone)
        if config.max_integral_accumulator is not None:
            self._motor.configMaxIntegralAccumulator(0, config.max_integral_accumulator)


class TalonFX(_Talon):
    def init(self):
        self._motor = ctre.TalonFX(self._can_id)
        self._set_config(self._config)
        self._motor.setInverted(self._inverted)


class TalonSRX(_Talon):
    def init(self):
        self._motor = ctre.TalonSRX(self._can_id)
        self._set_config(self._config)
        self._motor.setInverted(self._inverted)


class VictorSPX(_Talon):
    def init(self):
        self._motor = ctre.VictorSPX(self._can_id)
        self._set_config(self._config)
        self._motor.setInverted(self._inverted)


class TalonGroup(PIDMotor):
    motors: list[_Talon]

    def __init__(self, *motors: _Talon, config: TalonConfig = None, leader_idx: int = 0):
        super().__init__()
        self.motors = list(motors)
        for m in self.motors:
            m._config = config
        self._leader_idx = leader_idx

    def init(self):
        self.motors[self._leader_idx].init()
        for idx, motor in enumerate(self.motors):
            if idx != self._leader_idx:
                motor.init()
                motor.follow(self.motors[self._leader_idx])

    def set_leader_idx(self, idx: int):
        self._leader_idx = idx
        # self.motors[self._leader_idx].set_raw_output(0)  # Maybe?
        for idx, motor in enumerate(self.motors):
            if idx != self._leader_idx:
                motor.follow(self.motors[self._leader_idx])

    def get_sensor_position(self) -> Unum:
        return self.motors[self._leader_idx].get_sensor_position()

    def set_sensor_position(self, pos: Unum):
        self.motors[self._leader_idx].set_sensor_position(pos)

    def get_sensor_velocity(self) -> Unum:
        return self.motors[self._leader_idx].get_sensor_velocity()

    def set_raw_output(self, x: float):
        self.motors[self._leader_idx].set_raw_output(x)

    def set_target_position(self, pos: Unum):
        self.motors[self._leader_idx].set_target_position(pos)

    def set_target_velocity(self, vel: Unum):
        self.motors[self._leader_idx].set_target_velocity(vel)
