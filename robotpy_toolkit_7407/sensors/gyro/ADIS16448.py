import math
from wpilib import ADIS16448_IMU

from robotpy_toolkit_7407.utils.units import radians


class GyroADIS16448:
    def __init__(self) -> None:
        self._gyro = ADIS16448_IMU()
        self.__offset = 0

    @property
    def angle(self) -> radians:
        """
        Returns:
            theta: The angle of the robot in radians.
        """
        return math.radians(self._gyro.getGyroAngleZ() + self.__offset)

    def reset(self):
        self.__offset = self.angle
