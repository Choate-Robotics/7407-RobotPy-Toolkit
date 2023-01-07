from photon_camera import PhotonCamera
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import SwerveDrive4Odometry

class PhotonOdometry:
    def __init__(self, camera: PhotonCamera):
        self.camera = camera

    def getRobotPose(self):
        self.camera.refresh()

