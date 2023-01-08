import ctre
import math

from robotpy_toolkit_7407.utils.units import radians
from robotpy_toolkit_7407.sensors.gyro.base_gyro import Gyro


class PigeonIMUGyro_Wrapper(Gyro):
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

    def init(self):
        """
        Initialize gyro
        """
        self.reset_angle()

    def get_robot_heading(self) -> radians:
        """
        Returns the angle of the robot's heading in radians (yaw)
        :return: Robot heading (radians)
        """
        return math.radians(self._gyro.getYaw())

    # reset the gyro
    def reset_angle(self):
        """
        Resets the gyro's yaw.
        """
        self._gyro.setYaw(0)
