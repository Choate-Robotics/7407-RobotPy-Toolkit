import photonvision
from robotpy_toolkit_7407.sensors.photonvision.photon_target import PhotonTarget
from wpimath.geometry import Pose3d


class PhotonCamera:
    def __init__(self, name: str, poseRelativeToRobot: Pose3d, scale_constant: float = 1, height=None, pitch=None):
        self.camera = photonvision.PhotonCamera(cameraName=name)
        self.latest_best_target: PhotonTarget = None
        self.latest_targets_all: list[PhotonTarget] = None
        self.camera_to_robot_pose = poseRelativeToRobot
        self.height = height
        self.pitch = pitch
        self.scale_constant = scale_constant

    def hasTargets(self):
        return self.camera.hasTargets()

    def get_scaled_relative_pose(self):
        if self.latest_best_target is not None:
            return self.latest_best_target.relative_pose.translation().toTranslation2d() * self.scale_constant

    def refresh(self):  # Call at the beginning of every loop. Saves target for the loop to optimize lookup times.
        self.latest_targets_all = [PhotonTarget(target) for target in
                                   self.camera.getLatestResult().getTargets()] if self.hasTargets() else None
        self.latest_best_target = PhotonTarget(
            self.camera.getLatestResult().getBestTarget()) if self.hasTargets() else None
