import math
from networktables import NetworkTables
from robotpy_toolkit_7407.utils.units import m, deg, ft, inch, rad, radians, meters


class Limelight:
    """
    Wrapper for the Limelight sensor.
    Connect, get, and modify limelight values and settings through the NetworkTables interface.
    """

    def __init__(self, cam_height: float, cam_angle: float, target_height: float = None):
        """
        Args:
            cam_height (float): Height of the limelight camera from the ground in meters.
            cam_angle (float): Camera angle from the horizontal in degrees.
            target_height (float, optional): Height of the target from the ground in meters. Defaults to camera height.
        """

        NetworkTables.initialize()
        self.table = NetworkTables.getTable("limelight")
        self.tx = 0
        self.ty = 0
        self.refs = 0
        self.k_cam_height = (cam_height * m).asNumber(m)  # Height from ground
        self.k_cam_angle: radians = (cam_angle * deg).asNumber(rad)  # Angle from horizontal
        if target_height is not None:
            self.k_h_target_height = (target_height * m).asNumber(m)
        else:
            self.k_h_target_height = self.k_cam_height

    def led_on(self):
        """Turn limelight LEDs on. Recommended to use ref_on instead.
        """
        self.table.putNumber("ledMode", 3)

    def led_off(self):
        """Turn limelight LEDs off. Recommended to use ref_off instead.
        """
        self.table.putNumber("ledMode", 1)

    def ref_on(self):
        """
        Turn on the limelight LEDs and increment the reference counter.
        """
        if self.refs == 0:
            self.led_on()
        self.refs += 1

    def ref_off(self):
        """
        Turn off the limelight LEDs and decrement the reference counter.
        """
        self.refs -= 1
        if self.refs == 0:
            self.led_off()

    def update(self):
        """Update Limelight Values to NetworkTables. Run this every loop to ensure fresh values.
        """
        c_tx = self.table.getNumber('tx', None)
        c_ty = self.table.getNumber('ty', None)
        if c_tx is None or c_ty is None:
            return "No target found."
        self.tx = c_tx
        self.ty = c_ty

    def calculate_distance(self) -> float:
        """
        Calculate distance of limelight from target. This function is not useful when the limelight and target are of
        similar height.

        Returns:
            float: Distance from target in meters
        """
        true_angle = self.k_cam_angle + self.ty
        distance = (self.k_h_target_height - self.k_cam_height) / math.tan(true_angle)
        return distance

    def get_x_offset(self) -> radians:
        """Offset on the x plane from the target in radians

        Returns:
            radians: Radian offset of the target from the center of the camera
        """
        return math.radians(self.tx)
