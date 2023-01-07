import photonvision
import math

class PhotonTarget:
    def __init__(self, target: photonvision.PhotonTrackedTarget):
        self.raw_target = target
        self.relative_pose = self.raw_target.getBestCameraToTarget()
        self.id = self.raw_target.getFiducialId()
        self.pitch = self.raw_target.getPitch()
        self.yaw = self.raw_target.getYaw()
