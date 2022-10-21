from rev import ColorSensorV3
from wpilib import I2C


class ColorSensor:

    def __init__(self, sensor_port, I2C_address: int = 0x71, threshold_blue=500, threshold_red=500, threshold_green=400,
                 debug=False):
        """WPILIB Color Sensor Wrapper for usage with I2C Multiplexer
        Args:
            sensor_port (_type_): Initialize with the sensor's port on multiplexer (0b0001, 0b0010, 0b0100, or 0b1000).
            I2C_address (int, optional): I2C Address of the multiplexer on RoboRio. Defaults to 0x71. Ranges from 0x70 to 0x77.
            threshold_blue (int, optional): Threshold for classification as Blue. Defaults to 500.
            threshold_red (int, optional): Threshold for classification as Red. Defaults to 500.
            threshold_green (int, optional): Used to counteract field lighting issues. Defaults to 400.
            debug (bool, optional): Use to enable debugging flags. Defaults to False.
        """

        self.multiplexer = I2C(I2C.Port.kMXP, I2C_address)
        self.port = sensor_port
        self.I2C_address = I2C_address
        self.sensor = ColorSensorV3(I2C.Port.kMXP)

        self.threshold_blue = threshold_blue
        self.threshold_red = threshold_red
        self.threshold_green = threshold_green
        self.debug = debug

    def get_val(self) -> tuple[float, float, float, int]:
        """Return the raw values of the color sensor's response.

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
                print("Values not found, reinitializing color sensor...")

            self.multiplexer = I2C(I2C.Port.kMXP, self.I2C_address)
            self.sensor = ColorSensorV3(I2C.Port.kMXP)

        if vals[0] - vals[2] > self.threshold_red:
            return "red"
        elif vals[2] - vals[0] > self.threshold_blue and vals[1] > self.threshold_green:
            return "blue"

        return "none"
