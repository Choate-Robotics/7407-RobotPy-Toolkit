import math
from wpimath.geometry import Pose3d, Rotation3d, Translation3d

dist_scalar = 1

field_to_target = Pose3d(Translation3d(2, -2, 1), Rotation3d(0, 0, 0))
camera_to_target = Pose3d(Translation3d(1, -1, 0), Rotation3d(0, 0, math.radians(45)))
roc = (.5, -.5)
robot_to_camera = Pose3d(Translation3d(roc[0], roc[1], 1), Rotation3d(0, 0, math.atan2(roc[0], roc[1])))

gyro_angle = math.radians(45)

field_to_camera = (
    field_to_target.x - camera_to_target.x,
    field_to_target.y - camera_to_target.y
)

print("FIELD TO CAMERA: ", field_to_camera)

theta = robot_to_camera.rotation().angle - gyro_angle

print("THETA: ", math.degrees(theta))

h = (robot_to_camera.x ** 2 + robot_to_camera.y ** 2) ** .5

field_to_robot = (
    field_to_camera[0] - h * math.sin(theta),
    field_to_camera[1] - h * math.cos(theta)
)

print("FIELD TO ROBOT: ", field_to_robot)
