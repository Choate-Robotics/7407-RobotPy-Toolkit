from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import ctre
from robotpy_toolkit_7407.unum import Unum

from robotpy_toolkit_7407.motor import PIDMotor
from robotpy_toolkit_7407.utils.units import rad, rev, s, radians_per_second, radians_per_second_squared, radians


@dataclass
class TalonConfig:
    """
    Configuration for a TalonSRX/Falcon motor controller

    Args:
        kP: Proportional gain
        kI: Integral gain
        kD: Derivative gain
        kF: Feedforward gain
        closed_loop_peak_output: The maximum output of the controller
        neutral_brake: Whether to brake or coast when the motor is not moving
    """
    k_P: Optional[float] = None
    k_I: Optional[float] = None
    k_D: Optional[float] = None
    k_F: Optional[float] = None
    closed_loop_peak_output: Optional[float] = None
    motion_cruise_velocity: Optional[radians_per_second] = None
    motion_acceleration: Optional[radians_per_second_squared] = None
    neutral_brake: Optional[bool] = None
    integral_zone: Optional[float] = None
    max_integral_accumulator: Optional[float] = None


talon_sensor_unit = Unum.unit("talon_sensor_u", rev / 2048, "talon sensor unit")
hundred_ms = Unum.unit("100ms", s / 10, "100 milliseconds")
talon_sensor_vel_unit = talon_sensor_unit / hundred_ms
talon_sensor_accel_unit = talon_sensor_vel_unit / s

k_sensor_pos_to_radians = talon_sensor_unit.asNumber(rad)
k_radians_to_sensor_pos = rad.asNumber(talon_sensor_unit)
k_sensor_vel_to_rad_per_sec = talon_sensor_vel_unit.asNumber(rad / s)
k_rad_per_sec_to_sensor_vel = (rad / s).asNumber(talon_sensor_unit / hundred_ms)
k_sensor_accel_to_rad_per_sec_sq = talon_sensor_accel_unit.asNumber(rad / (s * s))
k_rad_per_sec_sq_to_sensor_accel = (rad / (s * s)).asNumber(talon_sensor_unit / (s * hundred_ms))


class _Talon(PIDMotor):
    _motor: ctre.BaseTalon

    def __init__(self, can_id: int, inverted: bool = False, config: TalonConfig = None):
        super().__init__()
        self._can_id = can_id
        self._config = config
        self._inverted = inverted

    def get_sensor_position(self) -> radians:
        return self._motor.getSelectedSensorPosition(0) * k_sensor_pos_to_radians

    def set_sensor_position(self, pos: radians):
        self._motor.setSelectedSensorPosition(pos * k_radians_to_sensor_pos)

    def get_sensor_velocity(self) -> radians_per_second:
        return self._motor.getSelectedSensorVelocity(0) * k_sensor_vel_to_rad_per_sec

    def set_raw_output(self, x: float):
        self._motor.set(ctre.ControlMode.PercentOutput, x)

    def set_target_position(self, pos: radians):
        self._motor.set(ctre.ControlMode.MotionMagic, pos * k_radians_to_sensor_pos)

    def set_target_velocity(self, vel: radians_per_second):
        self._motor.set(ctre.ControlMode.Velocity, vel * k_rad_per_sec_to_sensor_vel)

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
            self._motor.configMotionCruiseVelocity(config.motion_cruise_velocity * k_rad_per_sec_to_sensor_vel)
        if config.motion_acceleration is not None:
            self._motor.configMotionAcceleration(config.motion_acceleration * k_rad_per_sec_sq_to_sensor_accel)
        if config.neutral_brake is not None:
            self._motor.setNeutralMode(ctre.NeutralMode.Brake if config.neutral_brake else ctre.NeutralMode.Coast)
        if config.integral_zone is not None:
            self._motor.config_IntegralZone(0, config.integral_zone)
        if config.max_integral_accumulator is not None:
            self._motor.configMaxIntegralAccumulator(0, config.max_integral_accumulator)


class TalonFX(_Talon):
    """
    TalonFX motor controller wrapper
    """

    def init(self):
        """
        Initialize the motor controller
        """
        self._motor = ctre.TalonFX(self._can_id)
        self._set_config(self._config)
        self._motor.setInverted(self._inverted)


class TalonSRX(_Talon):
    """
    TalonSRX motor controller wrapper
    """

    def init(self):
        """
        Initialize the motor controller
        """
        self._motor = ctre.TalonSRX(self._can_id)
        self._set_config(self._config)
        self._motor.setInverted(self._inverted)


class VictorSPX(_Talon):
    """
    VictorSPX motor controller wrapper
    """

    def init(self):
        """
        Initialize the motor controller
        """
        self._motor = ctre.VictorSPX(self._can_id)
        self._set_config(self._config)
        self._motor.setInverted(self._inverted)


class TalonGroup(PIDMotor):
    """
    Group of Talon motor controllers. Used when multiple motors act as a single unit with a leader motor
    """
    motors: list[_Talon]

    def __init__(self, *motors: _Talon, config: TalonConfig = None, leader_idx: int = 0):
        super().__init__()
        self.motors = list(motors)
        for m in self.motors:
            m._config = config
        self._leader_idx = leader_idx

    def init(self):
        """
        Initialize the motor controllers
        """
        self.motors[self._leader_idx].init()
        for idx, motor in enumerate(self.motors):
            if idx != self._leader_idx:
                motor.init()
                motor.follow(self.motors[self._leader_idx])

    def set_leader_idx(self, idx: int):
        """
        Set the leader motor index (in the list of motors)

        Args:
            idx (int): Index of the leader motor
        """
        self._leader_idx = idx
        for idx, motor in enumerate(self.motors):
            if idx != self._leader_idx:
                motor.follow(self.motors[self._leader_idx])

    def get_sensor_position(self) -> radians:
        """
        Get the sensor position of the leader motor

        Returns:
            position (radians): Sensor position of the leader motor in radians
        """
        return self.motors[self._leader_idx].get_sensor_position()

    def set_sensor_position(self, pos: radians):
        """
        Set the sensor position of the leader motor
        Args:
            pos (radians): Sensor position of the leader motor in radians
        """
        self.motors[self._leader_idx].set_sensor_position(pos)

    def get_sensor_velocity(self) -> radians_per_second:
        """
        Get the sensor velocity of the leader motor
        Returns:
            velocity (radians_per_second): Sensor velocity of the leader motor in radians per second
        """
        return self.motors[self._leader_idx].get_sensor_velocity()

    def set_raw_output(self, x: float):
        """
        Set the raw output of the leader motor
        Args:
            x (float): Raw output of the leader motor
        """
        self.motors[self._leader_idx].set_raw_output(x)

    def set_target_position(self, pos: radians):
        """
        Set the target position of the leader motor
        Args:
            pos (radians): Target position of the leader motor in radians
        """
        self.motors[self._leader_idx].set_target_position(pos)

    def set_target_velocity(self, vel: radians_per_second):
        """
        Set the target velocity of the leader motor
        Args:
            vel (radians_per_second): Target velocity of the leader motor in radians per second
        """
        self.motors[self._leader_idx].set_target_velocity(vel)
