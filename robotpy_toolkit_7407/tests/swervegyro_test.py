import pytest

from unittest.mock import MagicMock
from robotpy_toolkit_7407.sensors.gyro import SwerveGyro, BaseGyro, PigeonIMUGyro_Wrapper

@pytest.fixture
def pigeon()->PigeonIMUGyro_Wrapper:
    gyro=PigeonIMUGyro_Wrapper(3)
    gyro._gyro=MagicMock()
    return gyro

def test_swerve_gyro() -> None:
    # setup

    # action
    gyro = SwerveGyro()
    # assert
    assert isinstance(gyro, BaseGyro)

def test_pigeon_gyro_wrapper_init(pigeon)->None:
    # setup
    gyro=PigeonIMUGyro_Wrapper(3)
    # action
    gyro.init(10)
    # assert
    assert gyro.get_robot_heading()==10
