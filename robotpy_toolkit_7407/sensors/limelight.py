import math
from networktables import NetworkTables
from robotpy_toolkit_7407.utils.units import m, deg, ft, inch, rad, radians, meters

class Limelight:
    def __init__(self, cam_height: float, cam_angle: float, target_height: float):
        """Connect, get, and modify limelight values and settings through the networktables interface

        Args:
            cam_height (float): Height of the limelight camera from the ground in meters.
            cam_angle (float): Camera angle from the horizontal in degrees.
            target_height (float): Height of the target from the ground in meters.
        """
        
        NetworkTables.initialize()
        self.table = NetworkTables.getTable("limelight")
        self.tx = 0
        self.ty = 0
        self.refs = 0
        self.k_cam_height = (cam_height * m).asNumber(m) # Height from ground
        self.k_cam_angle: radians = (cam_angle * deg).asNumber(rad)  # Angle from horizontal
        self.k_h_target_height = (target_height*m).asNumber(m)

    def led_on(self):
        """Turn limelight LEDs on
        """
        self.table.putNumber("ledMode", 3)

    def led_off(self):
        """Turn limelight LEDs off
        """
        self.table.putNumber("ledMode", 1)

    def ref_on(self):
        if self.refs == 0:
            self.led_on()
        self.refs += 1

    def ref_off(self):
        self.refs -= 1
        if self.refs == 0:
            self.led_off()

    def update(self):
        """Update Limelight Values
        """
        c_tx = self.table.getNumber('tx', None)
        c_ty = self.table.getNumber('ty', None)
        if c_tx is None or c_ty is None:
            return "No target found."
        self.tx = c_tx
        self.ty = c_ty

    def calculate_distance(self) -> float:
        """Calculate distance from target

        Returns:
            float: Distance from target in meters
        """
        true_angle = self.k_cam_angle + self.ty
        distance = (self.k_h_target_height - self.k_cam_height) / math.tan(true_angle)
        return distance

    def get_x_offset(self) -> radians:
        """Offset on the x plane from the target

        Returns:
            radians: Radian offset of the target from the center of the camera
        """
        return math.radians(self.tx)