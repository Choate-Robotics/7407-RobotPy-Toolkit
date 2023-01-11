from wpimath.geometry import Transform3d, Pose3d, Rotation3d, Translation3d, Transform2d, Translation2d, Pose2d

from robotpy_toolkit_7407.sensors.photonvision.photon_target import PhotonTarget, AprilTag
from robotpy_toolkit_7407.sensors.photonvision.photon_camera import PhotonCamera
from photonvision import PhotonUtils, PhotonTrackedTarget
from robotpy_apriltag import AprilTagFieldLayout
from robotpy_toolkit_7407.sensors.gyro import Gyro
from robotpy_apriltag import AprilTagPoseEstimator


def LoadFieldLayout(json_path: str):
    return AprilTagFieldLayout(json_path)


class PhotonOdometry:
    def __init__(self, camera: PhotonCamera, field_layout: dict, gyro: Gyro):
        self.camera = camera
        self.field_layout = self.parse_field_layout(field_layout)
        self.gyro = gyro

    def refresh(self):
        self.camera.refresh()

    def getRobotPose(self, target: PhotonTarget = None):
        if target is None:
            target = self.camera.latest_best_target

        if target is None:
            return None

        try:
            field_to_target = self.field_layout.getTagPose(target.ID).translation().toTranslation2d()
        except:
            return None

        # field_to_target = self.field_layout.getTagPose(target.ID).translation().toTranslation2d()  # Coords of target relative to field
        camera_to_target = target.relative_pose.translation().toTranslation2d() * self.camera.scale_constant  # Coords of target relative to camera
        camera_to_robot = self.camera.camera_to_robot_pose.translation().toTranslation2d()  # Coords of camera relative to robot

        field_to_robot = field_to_target - camera_to_target - camera_to_robot  # Reverse coords from field to target to camera to robot

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
