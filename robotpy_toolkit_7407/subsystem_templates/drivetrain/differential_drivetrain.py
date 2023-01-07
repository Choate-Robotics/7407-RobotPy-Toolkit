from robotpy_toolkit_7407.unum import Unum

from robotpy_toolkit_7407.motor import PIDMotor
from robotpy_toolkit_7407.oi.joysticks import JoystickAxis
from robotpy_toolkit_7407.subsystem import Subsystem
from robotpy_toolkit_7407.utils import logger
from robotpy_toolkit_7407.utils.units import rad, m, radians_per_meter, meters_per_second
class DifferentialDrivetrain(Subsystem):
    """
    Extendable differential drivetrain class.

    Args:
        m_left: leading left motor
        m_right: leading right motor
        axis_x: x-axis of the joystick
        axis_y: y-axis of the joystick
        gear_ratio: gear ratio of the drivetrain
    """
    m_left: PIDMotor = None
    m_right: PIDMotor = None
    axis_x: JoystickAxis = None
    axis_y: JoystickAxis = None
    gear_ratio: radians_per_meter

    def init(self):
        """
        Initialize the drivetrain.
        """
        logger.info("initializing differential drivetrain", "[differential_drivetrain]")
        self.m_left.init()
        self.m_right.init()
        logger.info("initialization complete", "[differential_drivetrain]")

    def set_motor_percent_output(self, left: float, right: float):
        """
        Set the percent output of the motors between -1 and 1

        Args:
            left (float): percent output of the left motor
            right (float): percent output of the right motor
        """
        self.m_left.set_raw_output(left)
        self.m_right.set_raw_output(right)

    def set_motor_velocity(self, left_vel: meters_per_second, right_vel: meters_per_second):
        """
        Set the velocity of the motors in meters per second

        Args:
            left_vel (meters_per_second): velocity of the left motor
            right_vel (meters_per_second): velocity of the right motor
        """
        self.m_left.set_target_velocity(left_vel * self.gear_ratio)
        self.m_right.set_target_velocity(right_vel * self.gear_ratio)
