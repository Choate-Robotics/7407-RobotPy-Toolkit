import math

from robotpy_toolkit_7407.unum import Unum

from robotpy_toolkit_7407.utils.units import rad


def bounded_angle_diff(theta_from: float, theta_too: float) -> float:
    """
    Finds the bounded (from -π to π) angle difference between two unbounded angles
    """
    res = math.fmod(theta_too - theta_from, 6.283185307179586)
    if res > math.pi:
        res -= 6.283185307179586
    if res < -math.pi:
        res += 6.283185307179586
    return res


def rotate_vector(x: Unum | float, y: Unum | float, theta: Unum) -> tuple[Unum, Unum]:
    theta_rad = theta.asNumber(rad)

    return (
        x * math.cos(theta_rad) - y * math.sin(theta_rad),
        x * math.sin(theta_rad) + y * math.cos(theta_rad)
    )


def clamp(val, _min, _max):
    if val < _min:
        return _min
    if val > _max:
        return _max
    return val


def ft_to_m(ft: float):
    return ft * 0.3048


def sensor_units_to_inches(sensor_units: float, low_gear: bool) -> float:
    motor_rotations = sensor_units / 2048.0

    if low_gear:
        wheelbase_rotations = motor_rotations / 15.45  # Low gear
    else:
        wheelbase_rotations = motor_rotations / 8.21  # High gear

    inches = wheelbase_rotations * (6 * math.pi)

    return inches


def sensor_units_to_meters(sensor_units: float, low_gear: bool) -> float:
    return sensor_units_to_inches(sensor_units, low_gear) * 0.0254


def meters_to_sensor_units(meters: float, low_gear: bool) -> float:
    return inches_to_sensor_units(meters / 0.0254, low_gear)


def inches_to_sensor_units(inches: float, low_gear: bool) -> float:
    wheelbase_rotations = inches / (6 * math.pi)

    if low_gear:
        motor_rotations = wheelbase_rotations * 15.45  # Low gear
    else:
        motor_rotations = wheelbase_rotations * 8.21  # High gear

    sensor_units = motor_rotations * 2048.0

    return sensor_units
