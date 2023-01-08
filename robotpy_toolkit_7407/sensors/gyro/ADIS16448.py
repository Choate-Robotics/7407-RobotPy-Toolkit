import math
from wpilib import ADIS16448_IMU

from robotpy_toolkit_7407.utils.units import radians

from robotpy_toolkit_7407.sensors.gyro.base_gyro import Gyro


class GyroADIS16448(Gyro):
    """
    Wrapper class for the ADIS16448 Gyro
    """
    def __init__(self) -> None:
        self._gyro = ADIS16448_IMU()
        self.__offset = 0

    def init(self):
        """
        Initialize the gyro
        """
        self.reset_angle()
        self.__offset = 0

    def get_robot_heading(self) -> radians:
        """
        Returns the angle of the robot's heading in radians (yaw)
        :return: Robot heading (radians)
        """
        return math.radians(self._gyro.getGyroAngleZ() + self.__offset)

    def reset_angle(self):
        """
        Resets the gyro's yaw.
        """
        self.__offset = self.get_robot_heading()
