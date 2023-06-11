from robotpy_toolkit_7407.command import SubsystemCommand
from robotpy_toolkit_7407.subsystem import Subsystem


def test_subsystemcommand()->None:
    '''
    A test to make sure that a SubsystemCommand adds the necessary
    required subsystem.

    Asserts: That the Command has the required subsystem

    '''
    # Setup
    my_subsystem=Subsystem()

    # Action
    my_command = SubsystemCommand(my_subsystem)

    # Assert
    assert my_subsystem in my_command.getRequirements()