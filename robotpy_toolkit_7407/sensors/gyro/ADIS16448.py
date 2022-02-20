from robotpy_toolkit_7407.unum import Unum
from wpilib import ADIS16448_IMU

from robotpy_toolkit_7407.utils.units import deg


class GyroADIS16448:
    def __init__(self) -> None:
        self._gyro = ADIS16448_IMU()
        self.__offset = 0 * deg

    @property
    def angle(self) -> Unum:
        return (self._gyro.getGyroAngleZ() + self.__offset) * deg

    def reset(self):
        self.__offset = self.angle
