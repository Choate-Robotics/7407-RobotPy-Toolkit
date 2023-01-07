from photon_camera import PhotonCamera
from photonvision import PhotonUtils
from wpimath.geometry import Pose2d, Pose3d, Rotation2d, Translation2d
from robotpy_apriltag import AprilTagFieldLayout
from robotpy_toolkit_7407.sensors.gyro import Gyro


def LoadFieldLayout(json_path: str):
    return AprilTagFieldLayout(json_path)

class PhotonOdometry:
    def __init__(self, camera: PhotonCamera, field_layout: AprilTagFieldLayout, gyro: Gyro):
        self.camera = camera
        self.field_layout = field_layout
        self.gyro = gyro

    def refresh(self):
        self.camera.refresh()

    def getRobotPose(self):
        target = self.camera.latest_target

        if target.relative_pose is None:
            return None

        return PhotonUtils.estimateFieldToRobot(
            # cameraHeight=self.camera.height,
            # targetHeight=self.field_layout.getTagPose(target.id).Y,
            # cameraPitch=self.camera.pitch,
            # targetPitch=target.pitch,
            # targetYaw=target.yaw,
            # gyroAngle=self.gyro.get_robot_heading(),
            cameraToTarget=target.relative_pose,
            fieldToTarget=self.field_layout.getTagPose(target.id),
            cameraToRobot=self.camera.camera_to_robot_pose
        )

    PhotonUtils.estimateFieldToRobot()
