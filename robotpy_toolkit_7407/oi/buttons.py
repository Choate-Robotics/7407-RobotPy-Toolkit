from dataclasses import dataclass

import commands2 as commands
import commands2.button

from robotpy_toolkit_7407.oi.joysticks import _Joysticks


@dataclass
class _Button:
    controller_id: int

    def __call__(self) -> commands.button.Button: ...


@dataclass
class DefaultButton(_Button):
    """
    Wrapper for wpilib button
    """
    button_id: int

    def __call__(self) -> commands.button.Button:
        if self.button_id < 0:
            return commands.button.Button(
                lambda: _Joysticks.joysticks[self.controller_id].getRawAxis(-self.button_id) > 0.8
            )
        return commands.button.JoystickButton(_Joysticks.joysticks[self.controller_id], self.button_id)


@dataclass
class AxisButton(_Button):
    """
    Wrapper for wpilib axis button
    """
    axis_id: int
    range_min: float = -1
    range_max: float = 1

    def __call__(self) -> commands.button.Button:
        return commands.button.Button(
            lambda: self.range_min <= _Joysticks.joysticks[self.controller_id].getRawAxis(self.axis_id) <= self.range_max
        )
