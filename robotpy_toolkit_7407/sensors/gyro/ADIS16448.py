import math
from wpilib import ADIS16448_IMU

from robotpy_toolkit_7407.utils.units import radians

from robotpy_toolkit_7407.sensors.gyro.base_gyro import BaseGyro


class GyroADIS16448(BaseGyro):
    """
    Wrapper class for the ADIS16448 Gyro
    """
    def __init__(self) -> None:
        self._gyro = ADIS16448_IMU()
        self.__offset = 0

    def init(self, gyro_start_angle=0):
        """
        Initialize the gyro
        """
        self.reset_angle()
        self.__offset = gyro_start_angle

    def get_robot_heading(self) -> radians:
        """
        Returns the angle of the robot's heading in radians (yaw)
        :return: Robot heading (radians)
        """
        return math.radians(self._gyro.getGyroAngleZ() + self.__offset)

    def get_robot_pitch(self) -> radians:
        """
        Returns the angle of the robot's pitch in radians
        :return: Robot pitch (radians)
        """
        return math.radians(self._gyro.getGyroAngleX())

    def get_robot_roll(self) -> radians:
        """
        Returns the angle of the robot's roll in radians
        :return: Robot roll (radians)
        """
        return math.radians(self._gyro.getGyroAngleY())

    def reset_angle(self, angle: radians = 0):
        """
        Resets the gyro's yaw.
        """
        self.__offset = self.get_robot_heading() - angle
