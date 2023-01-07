import photonvision
from photon_target import PhotonTarget
from wpimath.geometry import Pose3d

class PhotonCamera:
    def __init__(self, name, poseRelativeToRobot: Pose3d):
        self.camera = photonvision.PhotonCamera(cameraName=name)
        self.latest_target: PhotonTarget = None
        self.camera_to_robot_pose = poseRelativeToRobot

    def getLatestResult(self):
        return self.camera.getLatestResult()

    def hasTargets(self):
        return self.camera.hasTargets()

    def refresh(self):
        self.latest_target = PhotonTarget(self.getLatestResult().getBestTarget()) if self.hasTargets() else None
