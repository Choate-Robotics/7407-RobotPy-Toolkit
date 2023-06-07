import pytest
import math

from robotpy_toolkit_7407.utils.math import bounded_angle_diff, rotate_vector, clamp

@pytest.mark.parametrize("theta_from, theta_to, answer",[
    (3,1,-2),
    (4,-1,1.2831853),
    (-1.1, 4.2,-0.983185),
    (5, 9, -2.2831853),
    (-10, 1, -1.566370614),
    (0, -2*math.pi, 0),
    (4*math.pi, 0, 0),
    (3*math.pi/2, 7*math.pi/4, math.pi/4)
])
def test_bounded_angle_diff(theta_from, theta_to, answer)->None:
    angle=bounded_angle_diff(theta_from,theta_to)
    assert angle==pytest.approx(answer, abs=1e-06)

@pytest.mark.parametrize("x, y, theta, answer",[
    (1,1,math.pi/4, (0, math.sqrt(2))),
    (1,0,math.pi, (-1,0)),
    (-1,1, math.pi/3, (math.sqrt(2)*math.cos(13*math.pi/12), math.sqrt(2)*math.sin(13*math.pi/12))),
    (3,-2, 13*math.pi/6, (math.sqrt(13)*math.cos(math.atan2(-2, 3)+math.pi/6), math.sqrt(13)*math.sin(math.atan2(-2, 3)+math.pi/6))),
    (-5.2,-2.3, 4.7, (math.sqrt(5.2**2+2.3**2)*math.cos(math.atan2(-2.3, -5.2)+4.7), math.sqrt(5.2**2+2.3**2)*math.sin(math.atan2(-2.3, -5.2)+4.7)))
    ])
def test_rotate_vector(x, y, theta, answer)->None:
    vector=rotate_vector(x,y, theta)
    assert vector[0]==pytest.approx(answer[0], abs=1e-06)
    assert vector[1]==pytest.approx(answer[1], abs=1e-06)

@pytest.mark.parametrize("val, _min, _max, answer", [
    (14, 1, 20, 14),
    (-2, 3, 12, 3),
    (-2, -12, -3, -3),
    (-2, -3, 12, -2),
    (18, 3, 12, 12)
    ])
def test_clamp(val: float, _min: float, _max: float, answer):
    num=clamp(val, _min, _max)
    assert num == pytest.approx(answer, abs=1e-06)
