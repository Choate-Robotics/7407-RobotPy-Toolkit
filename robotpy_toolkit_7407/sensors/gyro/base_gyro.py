from robotpy_toolkit_7407.utils.units import radians


class Gyro:
    """
    Extendable class for swerve gyro.
    """

    def init(self):
        """
        Initialize the swerve gyro. Overridden class.
        """
        ...

    def get_robot_heading(self) -> radians:
        """
        Get the robot heading in radians. Overridden class. Must return radians.
        """
        ...

    def reset_angle(self):
        """
        Reset the robot heading. Overridden class.
        """
        ...
