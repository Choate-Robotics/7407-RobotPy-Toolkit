import wpilib
from pytest import MonkeyPatch

from robotpy_toolkit_7407.sensors.limit_switches import *


def test_limit_switch_init() -> None:
    # setup
    # action
    lsw = LimitSwitch(1)
    # assert
    assert lsw.limit_switch.getChannel() == 1

    # action
    lsw.reverse = False
    # assert
    assert lsw.reverse == False


def test_get_unreversed(monkeypatch: MonkeyPatch) -> None:
    # setup
    lsw = LimitSwitch(3, False)

    # lsw.limit_switch=MagicMock()
    def mock_get(self) -> bool:
        return True

    monkeypatch.setattr(wpilib.DigitalInput, "get", mock_get)

    value = lsw.get_value()

    # assert
    assert value == True


def test_get_reversed(monkeypatch: MonkeyPatch) -> None:
    # setup
    lsw = LimitSwitch(3, True)

    # lsw.limit_switch=MagicMock()
    def mock_get(self) -> bool:
        return True

    monkeypatch.setattr(wpilib.DigitalInput, "get", mock_get)

    value = lsw.get_value()

    # assert
    assert value == False


def test_PhotoElectricSwitch() -> None:
    # setup
    # action
    pes = PhotoElectricSwitch(3)

    assert isinstance(pes, LimitSwitch)


def test_MagneticLimitSwitch() -> None:
    # setup
    # action
    mes = MagneticLimitSwitch(3)

    assert isinstance(mes, LimitSwitch)
