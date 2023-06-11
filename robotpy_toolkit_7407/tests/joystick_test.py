import pytest
import wpilib
from pytest import MonkeyPatch

from robotpy_toolkit_7407.oi.joysticks import Joysticks, JoystickAxis


def test_joysticks() -> None:
    # setup
    # action
    jystks = Joysticks()

    # assert
    assert jystks.joysticks[0].getPort() == 0
    assert jystks.joysticks[1].getPort() == 1


@pytest.mark.parametrize(("shift", "joystick", "axis"), [
    (3, 0, 1),
    (2, 1, 2),
    (0, 1, 3),
    (1, 0, 4)
])
def test_joystickaxis(shift, joystick, axis, monkeypatch: MonkeyPatch) -> None:
    # setup
    def mock_getRawAxis(self, id):
        return id + shift

    monkeypatch.setattr(wpilib.Joystick, "getRawAxis", mock_getRawAxis)
    # action
    jystk = JoystickAxis(controller_id=joystick, axis_id=axis)
    value = jystk.value

    # assert
    assert value == axis + shift
    assert joystick == jystk.controller_id
    assert axis == jystk.axis_id
