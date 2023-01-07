from rev import ColorSensorV3
from wpilib import I2C
from robotpy_toolkit_7407.utils.logger import Logger


class REVColorSensor:
    """
    REVColor Sensor Wrapper for usage with I2C Multiplexer
    """

    def __init__(self,
                 sensor_port: int,
                 I2C_address: int = 0x71,
                 threshold_blue: float = 500,
                 threshold_red: float = 500,
                 threshold_green: float = 400,
                 debug: bool = False):
        """
        REVColor Sensor Wrapper for usage with I2C Multiplexer

        Args:
            sensor_port (int): Initialize with the sensor's port on multiplexer (0b0001, 0b0010, 0b0100, or 0b1000).
            I2C_address (int, optional): I2C Address of the multiplexer on RoboRio. Defaults to 0x71. Ranges from 0x70 to 0x77.
            threshold_blue (float, optional): Threshold for classification as Blue. Defaults to 500.
            threshold_red (float, optional): Threshold for classification as Red. Defaults to 500.
            threshold_green (float, optional): Used to counteract field lighting issues. Defaults to 400.
            debug (bool, optional): Use to enable debugging flags. Defaults to False.
        """

        self.port = sensor_port
        self.I2C_address = I2C_address
        self.threshold_blue = threshold_blue
        self.threshold_red = threshold_red
        self.threshold_green = threshold_green
        self.debug = debug

        self.multiplexer = I2C(I2C.Port.kMXP, self.I2C_address)
        self.sensor = ColorSensorV3(I2C.Port.kMXP)

        self.logger = Logger("ColorSensor")

    def get_val(self) -> tuple[float, float, float, int]:
        """
        Return the raw values of the color sensor's response.

        Returns:
            tuple[float, float, float, int]: R, G, B, Proximity.
        """

        self.multiplexer.writeBulk(bytes([self.port]))
        c = self.sensor.getRawColor()
        return c.red, c.green, c.blue, self.sensor.getProximity()

    def color(self) -> str:
        """Returns the color of the detected object.

        Returns:
            str: "red", "blue", or "none".
        """

        vals = self.get_val()

        if vals[0] == 0:
            if self.debug:
                self.logger.log_warning("Values not found, reinitializing color sensor...")

            self.multiplexer = I2C(I2C.Port.kMXP, self.I2C_address)
            self.sensor = ColorSensorV3(I2C.Port.kMXP)

        if vals[0] - vals[2] > self.threshold_red:
            return "red"
        elif vals[2] - vals[0] > self.threshold_blue and vals[1] > self.threshold_green:
            return "blue"

        return "none"
