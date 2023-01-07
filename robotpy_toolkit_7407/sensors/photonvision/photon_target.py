import photonvision

poses = {
    1: (0, 0, 0),
}

class PhotonTarget:
    def __init__(self, target: photonvision.PhotonTrackedTarget):
        self.target = target
        self.pose = poses[self.getTargetID()]

    def getPose(self):
        return self.target.getBestTarget()

    def getTargetID(self):
        return self.target.getFiducialId()
