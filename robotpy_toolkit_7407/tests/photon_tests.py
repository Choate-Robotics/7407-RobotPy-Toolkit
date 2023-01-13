from photonvision import PhotonTrackedTarget
from robotpy_apriltag import AprilTagFieldLayout
from wpimath.geometry import Pose3d, Translation3d, Rotation3d, Transform3d

from robotpy_toolkit_7407.sensors.gyro import PigeonIMUGyro_Wrapper
from robotpy_toolkit_7407.sensors.photonvision import PhotonCamera, PhotonTarget, PhotonOdometry, AprilTag

gyro = PigeonIMUGyro_Wrapper(13)

camera = PhotonCamera("globalshuttercamera",
                      Pose3d(Translation3d(x=1, y=1, z=.65),
                             Rotation3d(roll=0, pitch=0, yaw=0)),
                      scale_constant=1)

current_target = PhotonTarget(
    PhotonTrackedTarget(
        1, 1, 1, 1, 1,
        Transform3d(
            Pose3d(0, 0, 0, Rotation3d(0, 0, 0)),
            Pose3d(1, 1, 1, Rotation3d(0, 0, 0))),
        Transform3d(
            Pose3d(1, 1, 1, Rotation3d(1, 1, 1)),
            Pose3d(0, 0, 0, Rotation3d(0, 0, 0))),
        .1,
        [(1, 1), (1, 1), (1, 1), (1, 1)]))

field_layout = {
    'apriltags': {
        1: Pose3d(
            Translation3d(x=4.1783, y=1.2065, z=0.70485),
            Rotation3d(roll=0, pitch=0, yaw=0)
        ),
    },
    'fieldLength': 50,
    'fieldWidth': 30
}

odometry = PhotonOdometry(
    camera,
    field_layout,
    gyro
)

odometry.refresh()
print(odometry.getRobotPose(target=current_target))
