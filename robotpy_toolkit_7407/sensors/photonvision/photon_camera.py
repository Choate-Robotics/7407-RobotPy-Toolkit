from typing import Optional

from photonvision import PhotonCamera as BasePhotonCamera
from robotpy_toolkit_7407.sensors.photonvision.photon_target import PhotonTarget
from wpimath.geometry import Pose3d


class PhotonCamera(BasePhotonCamera):
    def __init__(self, name: str, poseRelativeToRobot: Pose3d, scale_constant: float = 1, height=None, pitch=None):
        BasePhotonCamera.__init__(self, cameraName=name)
        self.latest_best_target: Optional[PhotonTarget] = None
        self.latest_targets_all: Optional[list[PhotonTarget]] = None
        self.camera_to_robot_pose = poseRelativeToRobot
        self.height = height
        self.pitch = pitch
        self.scale_constant = scale_constant

    def get_scaled_relative_pose(self):
        if self.latest_best_target is not None:
            return self.latest_best_target.relative_pose.translation().toTranslation2d() * self.scale_constant

    def refresh(self):  # Call at the beginning of every loop. Saves target for the loop to optimize lookup times.
        self.latest_targets_all = [PhotonTarget(target) for target in
                                   self.getLatestResult().getTargets()] if self.hasTargets() else None
        self.latest_best_target = PhotonTarget(
            self.getLatestResult().getBestTarget()) if self.hasTargets() else None
