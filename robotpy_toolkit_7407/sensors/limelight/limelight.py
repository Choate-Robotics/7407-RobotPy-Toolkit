import math
from networktables import NetworkTables
from wpimath.geometry import Pose3d, Translation3d, Rotation3d

from robotpy_toolkit_7407.utils.units import m, deg, rad, radians
from robotpy_toolkit_7407.sensors.odometry import VisionEstimator

from wpilib import Timer


class Limelight:
    """
    Wrapper for the Limelight sensor.
    Connect, get, and modify limelight values and settings through the NetworkTables interface.
    """

    def __init__(self, cam_height: float, cam_angle: float, target_height: float = None, robot_ip: str = "10.74.07.2"):
        """
        Args:
            cam_height (float): Height of the limelight camera from the ground in meters.
            cam_angle (float): Camera angle from the horizontal in degrees.
            target_height (float, optional): Height of the target from the ground in meters. Defaults to camera height.
        """

        NetworkTables.initialize(server=robot_ip)
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

    def get_bot_pose(self, round_to: int = None) -> list | None:
        """
        Get the robot's pose from the limelight's perspective.
        """
        bot_pose = self.table.getValue("botpose", None)
        if round_to is not None and bot_pose is not None:
            bot_pose = [round(i, round_to) for i in bot_pose]
        return bot_pose


class LimelightController(VisionEstimator):
    def __init__(self, limelight_list: list[Limelight]):
        super().__init__()
        self.limelights = limelight_list

    def get_estimated_robot_pose(self) -> list[Pose3d, float] | None:
        """
        Returns the robot's pose relative to the field, estimated by the limelight.
        :return: Limelight estimate of robot pose.
        :rtype: Pose3d | None
        """
        pose_list = []

        for limelight in self.limelights:
            est_pose = limelight.get_bot_pose()
            if est_pose:
                pose_list.append(
                    (
                        Pose3d(
                            Translation3d(est_pose[0], est_pose[1], est_pose[2]),
                            Rotation3d(est_pose[3], est_pose[4], est_pose[5])
                        ),
                        Timer.getFPGATimestamp()
                    )
                )
            else:
                pose_list.append((None, None))

        return pose_list if pose_list else None
