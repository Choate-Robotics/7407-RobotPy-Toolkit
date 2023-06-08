from unittest.mock import MagicMock
from robotpy_toolkit_7407.pneumatics.pistons.double_solenoid import DoubleSolenoidPiston
import wpilib
import pytest
@pytest.fixture
def solenoid() -> DoubleSolenoidPiston:
    # Create a Solenoid, but it has mock
    # classes for its dependencies
    solenoid = DoubleSolenoidPiston(1, 2, 3 )
    solenoid.solenoid = MagicMock()

    return solenoid
def test_extend(solenoid: DoubleSolenoidPiston ):
    # setup
    extend=solenoid.solenoid.set

    # action
    solenoid.extend()

    # assert
    extend.assert_called_with(wpilib.DoubleSolenoid.Value.kForward)

def test_retract(solenoid: DoubleSolenoidPiston ):
    # setup
    retract=solenoid.solenoid.set

    # action
    solenoid.retract()

    # assert
    retract.assert_called_with(wpilib.DoubleSolenoid.Value.kReverse)

def test_toggle(solenoid: DoubleSolenoidPiston ):
    # setup
    toggle=solenoid.solenoid.toggle

    # action
    solenoid.toggle()

    # assert
    toggle.assert_called_once()

def test_get(solenoid: DoubleSolenoidPiston ):
    # setup
    get=solenoid.solenoid.get

    # action
    value=solenoid.get_value()

    # assert
    get.assert_called_once()

