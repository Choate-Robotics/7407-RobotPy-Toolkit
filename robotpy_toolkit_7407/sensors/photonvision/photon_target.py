import photonvision
from robotpy_apriltag import AprilTag
from wpimath.geometry import Pose3d


class PhotonTarget:
    def __init__(self, target: photonvision.PhotonTrackedTarget):
        self.raw_target = target
        self.relative_pose = self.raw_target.getBestCameraToTarget()
        self.ID = self.raw_target.getFiducialId()
        self.pitch = self.raw_target.getPitch()
        self.yaw = self.raw_target.getYaw()


class AprilTag(AprilTag):
    def __init__(self, ID: int, pose: Pose3d):
        super().__init__()
        self.ID = ID
        self.pose = pose
