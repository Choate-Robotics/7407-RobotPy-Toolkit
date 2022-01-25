from dataclasses import dataclass

import commands2 as commands

from robotpy_toolkit_7407.oi.joysticks import Joysticks


@dataclass
class Button:
    controller_id: int
    def __call__(self) -> commands._impl.button.Button: ...


@dataclass
class DefaultButton(Button):
    button_id: int

    def __call__(self) -> commands._impl.button.Button:
        if self.button_id < 0:
            return commands._impl.button.Button(
                lambda: Joysticks.joysticks[self.controller_id].getRawAxis(-self.button_id) > 0.8
            )
        return commands._impl.button.Button.JoystickButton(Joysticks.joysticks[self.controller_id], self.button_id)


@dataclass
class AxisButton(Button):
    axis_id: int
    range_min: float = -1
    range_max: float = 1

    def __call__(self) -> commands._impl.button.Button:
        return commands._impl.button.Button(
            lambda: self.range_min <= Joysticks.joysticks[self.controller_id].getRawAxis(self.axis_id) <= self.range_max
        )
