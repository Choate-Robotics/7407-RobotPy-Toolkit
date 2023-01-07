import wpilib


class LimitSwitch:
    """
    Wrapper class for I2C Limit Switches
    """

    def __init__(self, port: int, inverted: bool = True):
        """Wrapper class for I2C Limit Switches
        Args:
            port (int): I2C port of the limit switch.
            inverted (bool, optional): Return the inverted boolean of output. Defaults to True.
        """
        self.limit_switch = wpilib.DigitalInput(port)
        self.reverse = inverted

    def get_value(self):
        """Return if the limit switch is pressed or if object is detected (in the case of non-tactile sensors).

        Returns:
            bool: True if pressed, False if not.
        """
        if self.reverse:
            return not self.limit_switch.get()
        return self.limit_switch.get()


class MagneticLimitSwitch(LimitSwitch):
    """
    Wrapper class for I2C Magnetic Limit Switches
    """

    def __init__(self, port: int, inverted: bool = True):
        """Wpilib I2C Magnetic Limit Switch Wrapper extends LimitSwitch

        Args:
            port (int): I2C port of the limit switch.
            inverted (bool, optional): Return the inverted boolean of output. Defaults to True.
        """
        super().__init__(port, inverted)


class PhotoElectricSwitch(LimitSwitch):
    """
    Wrapper class for I2C PhotoElectric Switches
    """

    def __init__(self, port: int, inverted: bool = True):
        """Wpilib I2C PhotoElectric Limit Switch Wrapper extends LimitSwitch

        Args:
            port (int): I2C port of the limit switch.
            inverted (bool, optional): Return the inverted boolean of output. Defaults to True.
        """
        super().__init__(port, inverted)
