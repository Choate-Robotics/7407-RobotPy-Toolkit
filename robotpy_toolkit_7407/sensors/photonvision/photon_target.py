import photonvision
import math

class PhotonTarget:
    def __init__(self, target: photonvision.PhotonTrackedTarget, AprilTagFieldLayout: list):
        self.raw_target = target
        self.field_to_target_pose = AprilTagFieldLayout[self.getTargetID()]
        self.relative_pose = self.raw_target.getBestCameraToTarget()
        self.pitch = math.radians(self.raw_target.getPitch())

    def getTargetID(self):
        return self.raw_target.getFiducialId()

    def getTargetRelativePose(self):
        return self.relative_pose
