import math

import cv2
import numpy as np
import pygame
from numpy import ndarray
from robotpy_toolkit_7407.unum import Unum

from wpimath.geometry import Pose2d, Translation2d
from wpimath.trajectory import TrajectoryGenerator, TrajectoryConfig, Trajectory

from robotpy_toolkit_7407.subsystem_templates.drivetrain import DriveSwerve
from robotpy_toolkit_7407.subsystem_templates.drivetrain.swerve_drivetrain_commands import FollowPath
from robotpy_toolkit_7407.utils import logger
from robotpy_toolkit_7407.utils.math import rotate_vector, bounded_angle_diff
from robotpy_toolkit_7407.utils.units import m, s, ms, rad, mile, hour
from swerve_simulation.swerve_sim_aim import DriveSwerveAim
from swerve_simulation.swerve_sim_subsystem import SimDrivetrain, TestSwerveNode
from swerve_simulation.swerve_sim_trajectory import SimTrajectory, TrajectoryEndpoint, translation

pixel = Unum.unit("px", m/50, "pixel")
frame = Unum.unit("frame", 0, "frame")
update = Unum.unit("update", 0, "update")


class Simulation:
    def __init__(self, command: DriveSwerve, subsystem: SimDrivetrain,
                 trajectory: Trajectory = None, hub_pos: tuple[Unum, Unum] = None):
        self.drive_command = command
        self.subsystem = subsystem
        self.hub_pos = hub_pos

        pygame.init()
        pygame.joystick.init()
        self.joystick_connected = False
        self.joystick: pygame.joystick.Joystick | None = None

        self.img_w = 800
        self.img_h = 600
        self.img = np.zeros((self.img_h, self.img_w, 3))
        if trajectory is not None:
            self.draw_trajectory(trajectory, (255, 255, 0), (0, 255, 255))
        if hub_pos is not None:
            self.circle(hub_pos, 0.2 * m, (128, 255, 128))
        self.bg_img: ndarray = self.img.copy()

        self.robot_x = 0 * m
        self.robot_y = 0 * m

        self.time_scale = (1 / 0.02) * update/s
        self.frame_scale = 1 * frame/update

        self.wait_duration = int((1 / (self.time_scale * self.frame_scale)).asNumber(ms/frame))
        self.dt = 1 * update/self.time_scale

        self.max_vel_arrow_length = 0.5 * self.subsystem.track_width
        self.vel_multiplier = self.max_vel_arrow_length / self.subsystem.max_vel

    def run(self):
        while cv2.waitKey(self.wait_duration) != ord("q"):
            if isinstance(self.drive_command, DriveSwerveAim):
                dx = self.hub_pos[0] - self.robot_x
                dy = self.hub_pos[1] - self.robot_y
                theta = math.atan2(dy.asNumber(m), dx.asNumber(m))
                self.drive_command.offset = bounded_angle_diff(self.subsystem.odometry.getPose().rotation().radians() + math.pi / 2, theta) * rad
            self.drive_command.execute()
            self.subsystem.n_00.update(self.dt)
            self.subsystem.n_01.update(self.dt)
            self.subsystem.n_10.update(self.dt)
            self.subsystem.n_11.update(self.dt)

            self.img = self.bg_img.copy()

            self.get_joystick_vals()

            pose = self.subsystem.odometry.getPose()

            self.robot_x = pose.X() * m
            self.robot_y = pose.Y() * m
            self.subsystem.gyro.orientation += self.subsystem._omega * self.dt

            self.draw_robot()

            def format_num(n: Unum, unit: Unum, decimal_places: int = 2) -> str:
                return f"{n.asNumber(unit):.2f} {unit.strUnit()}"

            self.text(
                # f"v=({format_num(vx, m/s)}, {format_num(vy, m/s)}, {format_num(vr, rad/s)})",
                f"{self.subsystem.odometry.getPose()}",
                10 * pixel, 10 * pixel, (255, 255, 255)
            )

            cv2.imshow("swerve", self.img)

    def draw_robot(self):
        pts = [
            rotate_vector(-0.5 * self.subsystem.track_width, -0.5 * self.subsystem.track_width, self.subsystem.gyro.orientation),
            rotate_vector(+0.5 * self.subsystem.track_width, -0.5 * self.subsystem.track_width, self.subsystem.gyro.orientation),
            rotate_vector(+0.5 * self.subsystem.track_width, +0.5 * self.subsystem.track_width, self.subsystem.gyro.orientation),
            rotate_vector(-0.5 * self.subsystem.track_width, +0.5 * self.subsystem.track_width, self.subsystem.gyro.orientation)
        ]

        def draw_motor(motor: TestSwerveNode, px: Unum, py: Unum):
            x1, y1 = self.robot_x + px, self.robot_y + py
            dx, dy = rotate_vector(
                motor.get_motor_velocity(), 0 * m/s,
                motor.get_current_motor_angle() + self.subsystem.gyro.orientation
            )
            x2, y2 = x1 + dx * self.vel_multiplier, y1 + dy * self.vel_multiplier
            self.line((x1, y1), (x2, y2), (0, 0, 255) if not motor._motor_reversed else (255, 0, 0))

        self.poly(pts, (255, 128, 0))

        draw_motor(self.subsystem.n_00, *pts[0])
        draw_motor(self.subsystem.n_10, *pts[1])
        draw_motor(self.subsystem.n_01, *pts[3])
        draw_motor(self.subsystem.n_11, *pts[2])

    @staticmethod
    def deadzone(v: float, dz_amt: float = 0.1) -> float:
        return 0 if abs(v) < dz_amt else v

    def get_joystick_vals(self):
        try:
            pygame.event.pump()
            self.joystick = pygame.joystick.Joystick(0)
            if not self.joystick_connected:  # If we haven't initialized before
                self.joystick.init()
                self.joystick_connected = True
                logger.info("joystick connected")
            self.subsystem.axis_dx.val = self.deadzone(self.joystick.get_axis(0))
            self.subsystem.axis_dy.val = self.deadzone(self.joystick.get_axis(1))
            self.subsystem.axis_rotation.val = self.deadzone(self.joystick.get_axis(3))
        except pygame.error:
            if self.joystick_connected:
                self.joystick_connected = False
                logger.info("joystick disconnected")

    def line(self, pt1: tuple[Unum, Unum], pt2: tuple[Unum, Unum], color: tuple[int, int, int], thickness: int = 3):
        color = tuple(i / 255 for i in color)
        cv2.line(
            self.img,
            (int(pt1[0].asNumber(pixel)), self.img_h - int(pt1[1].asNumber(pixel))),
            (int(pt2[0].asNumber(pixel)), self.img_h - int(pt2[1].asNumber(pixel))),
            color, thickness
        )

    def circle(self, pos: tuple[Unum, Unum], radius: Unum, color: tuple[int, int, int]):
        color = tuple(i / 255 for i in color)
        cv2.circle(
            self.img,
            (int(pos[0].asNumber(pixel)), self.img_h - int(pos[1].asNumber(pixel))),
            int(radius.asNumber(pixel)),
            color, -1
        )

    def poly(self, pts: list[tuple[Unum, Unum]], color: tuple[int, int, int]):
        color = tuple(i / 255 for i in color)
        p_pts = []
        for a, b in pts:
            p_pts.append((
                int((a + self.robot_x).asNumber(pixel)),
                self.img_h - int((b + self.robot_y).asNumber(pixel))
            ))

        cv2.fillPoly(self.img, [np.array(p_pts)], color)

    def text(self, text: str, x: Unum, y: Unum, color: tuple[int, int, int], scale: float = 0.5, thickness: float = 1):
        color = tuple(i / 255 for i in color)
        cv2.putText(
            self.img, text,
            (int(x.asNumber(pixel)), self.img_h - int(y.asNumber(pixel))),
            cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness
        )

    def draw_trajectory(self, trajectory: Trajectory, color1: tuple[int, int, int], color2: tuple[int, int, int], samples=1000):
        total_time = trajectory.totalTime()
        dt = total_time / samples
        t = 0
        last_pose = trajectory.initialPose()

        def interp(x):
            return (
                color2[0] * x + color1[0] * (1 - x),
                color2[1] * x + color1[1] * (1 - x),
                color2[2] * x + color1[2] * (1 - x)
            )

        while t < total_time:
            t += dt
            new_pose = trajectory.sample(t).pose
            self.line(
                (last_pose.X() * m, last_pose.Y() * m),
                (new_pose.X() * m, new_pose.Y() * m),
                interp(t / total_time), 1
            )
            last_pose = new_pose


drivetrain = SimDrivetrain()
drivetrain.init()

test_trajectory = SimTrajectory.generate_trajectory(
    TrajectoryEndpoint(2 * m, 2 * m, 0 * rad),
    [
        translation(6.61 * m, 3.52 * m),
        translation(5.61 * m, 5.52 * m),
        translation(4.05 * m, 8.40 * m)
    ],
    TrajectoryEndpoint(1.27 * m, 7.02 * m, 2.88 * rad),
    20 * mile/hour, (20 * mile/hour) / (3 * s)
)

drive_command = FollowPath(drivetrain, test_trajectory)
drive_command.initialize()
sim = Simulation(drive_command, drivetrain, trajectory=test_trajectory)

# drive_command = DriveSwerve(drivetrain)
# drive_command.initialize()
# sim = Simulation(drive_command, drivetrain, hub_pos=(4 * m, 4 * m))

# drive_command = DriveSwerveAim(drivetrain)
# drive_command.initialize()
# sim = Simulation(drive_command, drivetrain, hub_pos=(4 * m, 4 * m))

sim.run()
