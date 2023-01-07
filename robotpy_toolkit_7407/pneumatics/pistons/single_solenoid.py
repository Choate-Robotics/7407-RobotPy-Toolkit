import wpilib


class SingleSolenoidPiston:
    """
    Wrapper class for a single solenoid piston.
    """

    def __init__(self, module, channel, ctre_hub=False):
        """
        Args:
            module (int): The Pneumatics Control Module (PCM) CAN ID
            channel (int): The channel on the PCM that the solenoid is connected to
            ctre_hub (bool): Whether the PCM is connected to a CTRE Pneumatics Control Module or not.
        """
        module_type = wpilib.PneumaticsModuleType.CTREPCM if ctre_hub else wpilib.PneumaticsModuleType.REVPH
        self.solenoid = wpilib.Solenoid(module, module_type, channel)

    def extend(self):
        """
        Extend the piston.
        """
        self.solenoid.set(True)

    def retract(self):
        """
        Retract the piston.
        """
        self.solenoid.set(False)

    def toggle(self):
        """
        Toggle the piston.
        """
        self.solenoid.toggle()

    def get_value(self) -> bool:
        """
        Get the piston state (True for extended, False for retracted).
        """
        return self.solenoid.get()
