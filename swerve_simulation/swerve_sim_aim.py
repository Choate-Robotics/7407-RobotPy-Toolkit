from wpimath.controller import ProfiledPIDControllerRadians
from wpimath.trajectory import TrapezoidProfileRadians

from robotpy_toolkit_7407.command import SubsystemCommand
from robotpy_toolkit_7407.subsystem_templates.drivetrain import SwerveDrivetrain
from robotpy_toolkit_7407.unum import Unum
from robotpy_toolkit_7407.utils.units import rad, s, m, radians


def curve_abs(x):
    return x ** 2


def curve(x):
    if x < 0:
        return -curve_abs(x)
    return curve_abs(x)


class DriveSwerveAim(SubsystemCommand[SwerveDrivetrain]):
    controller: ProfiledPIDControllerRadians

    def __init__(self, subsystem: SwerveDrivetrain):
        super().__init__(subsystem)
        self.offset: radians = 0

    def initialize(self) -> None:
        self.controller = ProfiledPIDControllerRadians(9, 0, 0, TrapezoidProfileRadians.Constraints(5, 20))
        self.controller.reset(0)

    def execute(self) -> None:
        dx, dy = self.subsystem.axis_dx.value, self.subsystem.axis_dy.value
        omega = self.controller.calculate(0, self.offset)

        dx = curve(dx)
        dy = curve(dy)

        dx *= self.subsystem.max_vel
        dy *= -self.subsystem.max_vel

        self.subsystem.set_driver_centric((dx, dy), omega)

    def end(self, interrupted: bool) -> None:
        if not interrupted:
            self.subsystem.n_00.set(0, 0)
            self.subsystem.n_01.set(0, 0)
            self.subsystem.n_10.set(0, 0)
            self.subsystem.n_11.set(0, 0)

    def isFinished(self) -> bool:
        return False

    def runsWhenDisabled(self) -> bool:
        return False
