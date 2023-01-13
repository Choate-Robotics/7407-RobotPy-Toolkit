"""
IMPORTANT:
 - From the perspective of the driver:
   - Positive X is forward
   - Positive Y is left
   - Positive theta is counterclockwise
"""

import math

from robotpy_toolkit_7407.sensors.photonvision.photon_target import PhotonTarget, AprilTag
from robotpy_toolkit_7407.sensors.photonvision.photon_camera import PhotonCamera
from robotpy_apriltag import AprilTagFieldLayout
from robotpy_toolkit_7407.sensors.gyro import Gyro
from wpimath.geometry import Pose3d, Translation3d, Rotation3d


def LoadFieldLayout(json_path: str):
    return AprilTagFieldLayout(json_path)


class PhotonOdometry:
    def __init__(self, camera: PhotonCamera, field_layout: dict, gyro: Gyro, start_pose=Pose3d(Translation3d(0, 0, 0), Rotation3d(roll=0, pitch=0, yaw=0))):
        self.camera = camera
        self.field_layout = self.parse_field_layout(field_layout)
        self.gyro = gyro
        self.pose_estimate = start_pose

    def refresh(self):
        self.camera.refresh()
        self.pose_estimate = self.getRobotPose()

    def getRobotPose(self, target: PhotonTarget = None):
        if target is None:
            target = self.camera.latest_best_target

        if target is None:
            return None

        try:
            field_to_target = self.field_layout.getTagPose(target.ID)
        except:
            return None

        gyro_angle = self.gyro.get_robot_heading()
        camera_to_target = target.relative_pose
        robot_to_camera = self.camera.camera_to_robot_pose

        theta = math.atan2(robot_to_camera.x, robot_to_camera.y) - gyro_angle
        h = (robot_to_camera.x ** 2 + robot_to_camera.y ** 2) ** .5

        field_to_camera = (
            field_to_target.x - camera_to_target.x,
            field_to_target.y - camera_to_target.y
        )

        field_to_robot = (
            field_to_camera[0] - h * math.sin(theta),
            field_to_camera[1] - h * math.cos(theta)
        )

        self.pose = field_to_robot

        return field_to_robot

    def parse_field_layout(self, field_layout: dict):
        new_layout: AprilTagFieldLayout = None
        april_tags: list[AprilTag] = []
        length: float = None  # meters
        width: float = None  # meters

        for i in field_layout['apriltags'].keys():
            april_tags.append(
                AprilTag(
                    ID=i,
                    pose=field_layout['apriltags'][i]
                )
            )

        length = field_layout['fieldLength']
        width = field_layout['fieldWidth']
        new_layout = AprilTagFieldLayout(
            apriltags=april_tags,
            fieldLength=length,
            fieldWidth=width
        )

        return new_layout
