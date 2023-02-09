import ctre
import math

from robotpy_toolkit_7407.utils.units import radians
from robotpy_toolkit_7407.sensors.gyro.base_gyro import BaseGyro


class PigeonIMUGyro_Wrapper(BaseGyro):
    """
    Wrapper class for the Pigeon2 IMU gyro.
    """

    def __init__(self, port):
        """
        Args:
            port (int): CAN ID of the Pigeon gyro
        """
        self._gyro = ctre.Pigeon2(port)
        self._gyro.configMountPose(0, 0, 0)

    def init(self, gyro_start_angle=0):
        """
        Initialize gyro
        """
        self.reset_angle(gyro_start_angle)

    def get_robot_heading(self) -> radians:
        """
        Returns the angle of the robot's heading in radians (yaw)
        :return: Robot heading (radians)
        """
        return math.radians(self._gyro.getYaw())

    def get_robot_pitch(self) -> radians:
        """
        Returns the angle of the robot's pitch in radians
        :return: Robot pitch (radians)
        """
        return math.radians(self._gyro.getPitch())

    def get_robot_roll(self) -> radians:
        """
        Returns the angle of the robot's roll in radians
        :return: Robot roll (radians)
        """
        return math.radians(self._gyro.getRoll())

    # reset the gyro
    def reset_angle(self, angle: radians = 0):
        """
        Resets the gyro's yaw.
        """
        self._gyro.setYaw(math.degrees(angle))
