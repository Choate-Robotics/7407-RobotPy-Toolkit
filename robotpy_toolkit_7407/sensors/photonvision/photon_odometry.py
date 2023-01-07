from photon_camera import PhotonCamera
from photonvision import PhotonUtils
from wpimath.geometry import Pose2d, Pose3d, Rotation2d, Translation2d
import math

april_tag_field_layout = [
    [0, Pose3d()]
]

class PhotonOdometry:
    def __init__(self, camera: PhotonCamera, AprilTagFieldLayout: list, CAMERA_HEIGHT_METERS: float, CAMERA_PITCH_RADIANS: float):
        self.camera = camera
        self.april_field = AprilTagFieldLayout
        self.CAMERA_HEIGHT_METERS = CAMERA_HEIGHT_METERS
        self.CAMERA_PITCH_RADIANS = CAMERA_PITCH_RADIANS

    def refresh(self):
        self.camera.refresh()

    def getRobotPose(self):
        target = self.camera.latest_target

        if target.relative_pose is None:
            return None

        return PhotonUtils.estimateFieldToRobot(
            cameraToTarget=target.relative_pose,
            fieldToTarget=target.field_to_target_pose,
            cameraToRobot=self.camera.camera_to_robot_pose
        )

