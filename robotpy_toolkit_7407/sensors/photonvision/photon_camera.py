import photonvision
from photon_target import PhotonTarget

class PhotonCamera:
    def __init__(self, name):
        self.camera = photonvision.PhotonCamera(cameraName=name)
        self.latest_target: photonvision.PhotonTrackedTarget = None

    def getLatestResult(self):
        return self.camera.getLatestResult()

    def hasTargets(self):
        return self.camera.hasTargets()

    def refresh(self):
        self.latest_target = PhotonTarget(self.getLatestResult().getBestTarget()) if self.hasTargets() else None
