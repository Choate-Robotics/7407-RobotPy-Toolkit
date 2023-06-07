import pytest
from pytest import MonkeyPatch

from robotpy_toolkit_7407.subsystem_templates.drivetrain.swerve_drivetrain import *
#from robotpy_toolkit_7407.utils.units import s, m, deg, rad, hour, mile, rev, meters, meters_per_second, \
#    radians_per_second, radians

# def test_swerveNode_set()->None:
#     #TODO Finish finding the right assert
#     # Setup
#     node=SwerveNode()
#
#     # Action
#     node.set(vel=10, angle_radians=1.2)
#
#     # Assert
#     assert True
@pytest.mark.parametrize("initial_angle, target_angle, answer", [
    (math.pi/4 , math.pi/3, (math.pi/3, False, 0)),
    (3, 6, (3-math.pi+3, True, math.pi)),
     (3, -3, (3.2831853071, False, 0))
])
def test_resolve_angles(target_angle, initial_angle, answer)->None:
    # Setup
    node=SwerveNode()

    # Action

    result = SwerveNode._resolve_angles(target_angle=target_angle, initial_angle=initial_angle)

    # Assert
    assert result[2]==answer[2]
    assert result[1]==answer[1]
    assert result[0]==pytest.approx(answer[0], abs=.00001)



def test_get_node_state(monkeypatch: MonkeyPatch)->None:
    # Setup
    node=SwerveNode()

    def mock_get_motor_velocity(self):
        return 8
    def mock_get_turn_motor_angle(self):
        return math.pi/4
    monkeypatch.setattr(SwerveNode, "get_motor_velocity", mock_get_motor_velocity)
    monkeypatch.setattr(SwerveNode, "get_turn_motor_angle", mock_get_turn_motor_angle)

    # Action
    state=node.get_node_state()
    # Assert
    assert type(state)==SwerveModuleState
    assert state.speed==8
    assert state.angle.radians()==pytest.approx(math.pi/4, abs=.0001)

def test_get_node_positions(monkeypatch: MonkeyPatch)->None:
    # Setup
    node=SwerveNode()

    def mock_get_drive_motor_traveled_distance(self):
        return 8
    def mock_get_turn_motor_angle(self):
        return math.pi/4
    monkeypatch.setattr(SwerveNode, "get_drive_motor_traveled_distance", mock_get_drive_motor_traveled_distance)
    monkeypatch.setattr(SwerveNode, "get_turn_motor_angle", mock_get_turn_motor_angle)

    # Action
    position=node.get_node_position()
    # Assert
    assert type(position)==SwerveModulePosition
    assert position.distance==8
    assert position.angle.radians()==pytest.approx(math.pi/4, abs=.0001)

# TODO Parameterize and make more cases
def test_set_angle(monkeypatch: MonkeyPatch)->None:
    #setup
    node=SwerveNode()
    node.motor_sensor_offset=1
    def mock_set_motor_angle(self, pos):
        node.target=pos

    monkeypatch.setattr(SwerveNode, "set_motor_angle", mock_set_motor_angle)
    #action
    node._set_angle(-3, 3) # returns (3.2831853071, False, 0)

    #assert
    assert node.target==pytest.approx(2.2831853071)