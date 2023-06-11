from robotpy_toolkit_7407.subsystem import Subsystem
import commands2
def subsystem_test()->None:
    # Setup

    system = Subsystem()
    system.init()
    assert type(system) == commands2.SubsystemBase


