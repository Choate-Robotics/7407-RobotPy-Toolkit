from robotpy_toolkit_7407.sensors.gyro import SwerveGyro, BaseGyro


def test_swerve_gyro() -> None:
    # setup

    # action
    gyro = SwerveGyro()
    # assert
    assert isinstance(gyro, BaseGyro)
