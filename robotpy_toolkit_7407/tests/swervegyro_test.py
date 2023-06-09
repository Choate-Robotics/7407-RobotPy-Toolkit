import math
from unittest.mock import MagicMock

import pytest
from ctre.sensors import Pigeon2
from pytest import MonkeyPatch

from robotpy_toolkit_7407.sensors.gyro import SwerveGyro, BaseGyro, PigeonIMUGyro_Wrapper


@pytest.fixture
def pigeon() -> PigeonIMUGyro_Wrapper:
    gyro = PigeonIMUGyro_Wrapper(3)
    gyro._gyro = MagicMock()
    return gyro


def test_swerve_gyro() -> None:
    # setup

    # action
    gyro = SwerveGyro()
    # assert
    assert isinstance(gyro, BaseGyro)


def test_pigeon_gyro_wrapper_init(pigeon) -> None:
    # setup
    # gyro = PigeonIMUGyro_Wrapper(1)
    syaw = pigeon._gyro.setYaw
    # action
    pigeon.init(10)
    # assert
    assert syaw.called_once_with(10 / (2 * math.pi) * 360)


def test_pigeon_get_robot_heading(monkeypatch: MonkeyPatch) -> None:
    # setup
    gyro = PigeonIMUGyro_Wrapper(3)

    def mock_getYaw(self):
        return 90

    monkeypatch.setattr(Pigeon2, "getYaw", mock_getYaw)

    # action
    value = gyro.get_robot_heading()
    # assert
    assert value == math.pi / 2


def test_pigeon_robot_pitch(monkeypatch: MonkeyPatch) -> None:
    # setup
    gyro = PigeonIMUGyro_Wrapper(3)

    def mock_get_Pitch(self):
        return 90

    monkeypatch.setattr(Pigeon2, "getPitch", mock_get_Pitch)

    # action
    value = gyro.get_robot_pitch()
    # assert
    assert value == math.pi / 2


def test_pigeon_robot_roll(monkeypatch: MonkeyPatch) -> None:
    # setup
    gyro = PigeonIMUGyro_Wrapper(3)

    def mock_get_Roll(self):
        return 90

    monkeypatch.setattr(Pigeon2, "getRoll", mock_get_Roll)

    # action
    value = gyro.get_robot_roll()
    # assert
    assert value == math.pi / 2


def test_reset_angle(pigeon) -> None:
    # setup
    setyaw = pigeon._gyro.setYaw
    # action
    pigeon.reset_angle(math.pi / 6)
    # assert
    assert setyaw.called_once_with(30)
