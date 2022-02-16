import cv2
import numpy as np
import pygame
from unum import Unum

from robotpy_toolkit_7407.subsystem_templates.drivetrain import DriveSwerve
from robotpy_toolkit_7407.utils import logger
from robotpy_toolkit_7407.utils.math import rotate_vector
from robotpy_toolkit_7407.utils.units import m, s, ms, rad
from swerve_simulation.swerve_sim_subsystem import TestDrivetrain, TestSwerveNode

pixel = Unum.unit("px", m/50, "pixel")
frame = Unum.unit("frame", 0, "frame")
update = Unum.unit("update", 0, "update")


class Simulation:
    def __init__(self, command: DriveSwerve, subsystem: TestDrivetrain):
        self.drive_command = command
        self.subsystem = subsystem

        pygame.init()
        pygame.joystick.init()
        self.joystick_connected = False
        self.joystick: pygame.joystick.Joystick | None = None

        self.img_w = 800
        self.img_h = 600
        self.img = np.zeros((self.img_h, self.img_w, 3))

        self.robot_x = 0.5 * m
        self.robot_y = 0.5 * m

        self.time_scale = (1 / 0.02) * update/s
        self.frame_scale = 1 * frame/update

        self.wait_duration = int((1 / (self.time_scale * self.frame_scale)).asNumber(ms/frame))
        self.dt = 1 * update/self.time_scale

        self.max_vel_arrow_length = 0.5 * self.subsystem.track_width
        self.vel_multiplier = self.max_vel_arrow_length / self.subsystem.max_vel

    def run(self):
        while cv2.waitKey(self.wait_duration) != ord("q"):
            # Execute command and update swerve nodes
            self.drive_command.execute()
            self.subsystem.n_00.update(self.dt)
            self.subsystem.n_01.update(self.dt)
            self.subsystem.n_10.update(self.dt)
            self.subsystem.n_11.update(self.dt)

            self.img.fill(0)

            self.get_joystick_vals()

            vx = self.subsystem.axis_dx.val * self.subsystem.max_vel
            vy = -self.subsystem.axis_dy.val * self.subsystem.max_vel
            vr = -self.subsystem.axis_rotation.val * self.subsystem.max_angular_vel

            self.robot_x += vx * self.dt
            self.robot_y += vy * self.dt
            self.subsystem.odometry.orientation += vr * self.dt

            self.draw_robot()

            def format_num(n: Unum, unit: Unum, decimal_places: int = 2) -> str:
                return f"{n.asNumber(unit):.2f} {unit.strUnit()}"

            self.text(
                f"v=({format_num(vx, m/s)}, {format_num(vy, m/s)}, {format_num(vr, rad/s)})",
                10 * pixel, 10 * pixel, (255, 255, 255)
            )

            cv2.imshow("swerve", self.img)

    def draw_robot(self):
        pts = [
            rotate_vector(-0.5 * self.subsystem.track_width, -0.5 * self.subsystem.track_width, self.subsystem.odometry.orientation),
            rotate_vector(+0.5 * self.subsystem.track_width, -0.5 * self.subsystem.track_width, self.subsystem.odometry.orientation),
            rotate_vector(+0.5 * self.subsystem.track_width, +0.5 * self.subsystem.track_width, self.subsystem.odometry.orientation),
            rotate_vector(-0.5 * self.subsystem.track_width, +0.5 * self.subsystem.track_width, self.subsystem.odometry.orientation)
        ]

        def draw_motor(motor: TestSwerveNode, px: Unum, py: Unum):
            x1, y1 = self.robot_x + px, self.robot_y + py
            dx, dy = rotate_vector(
                motor.get_motor_velocity(), 0 * m/s,
                motor.get_current_motor_angle() + self.subsystem.odometry.orientation
            )
            x2, y2 = x1 + dx * self.vel_multiplier, y1 + dy * self.vel_multiplier
            self.line((x1, y1), (x2, y2), (0, 0, 255) if not motor._motor_reversed else (255, 0, 0))

        self.poly(pts, (128, 128, 0))

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
        cv2.line(
            self.img,
            (int(pt1[0].asNumber(pixel)), self.img_h - int(pt1[1].asNumber(pixel))),
            (int(pt2[0].asNumber(pixel)), self.img_h - int(pt2[1].asNumber(pixel))),
            color, thickness
        )

    def poly(self, pts: list[tuple[Unum, Unum]], color: tuple[int, int, int]):
        p_pts = []
        for a, b in pts:
            p_pts.append((
                int((a + self.robot_x).asNumber(pixel)),
                self.img_h - int((b + self.robot_y).asNumber(pixel))
            ))

        cv2.fillPoly(self.img, [np.array(p_pts)], color)

    def text(self, text: str, x: Unum, y: Unum, color: tuple[int, int, int], scale: float = 0.5, thickness: float = 1):
        cv2.putText(
            self.img, text,
            (int(x.asNumber(pixel)), self.img_h - int(y.asNumber(pixel))),
            cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness
        )


drivetrain = TestDrivetrain()
drivetrain.init()

drive_command = DriveSwerve(drivetrain)
drive_command.initialize()

sim = Simulation(drive_command, drivetrain)

sim.run()
