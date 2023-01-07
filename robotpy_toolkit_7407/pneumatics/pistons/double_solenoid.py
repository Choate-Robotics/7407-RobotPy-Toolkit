import wpilib


class DoubleSolenoidPiston:
    """
    Wrapper class for a double solenoid piston.
    """

    def __init__(self, module, forward_channel, reverse_channel, ctre_hub=False):
        """
        Args:
            module (int): The Pneumatics Control Module (PCM) CAN ID
            forward_channel (int): The channel on the PCM that the forward channel of the solenoid is connected to
            reverse_channel (int): The channel on the PCM that the reverse channel of the solenoid is connected to
            ctre_hub (bool): Whether the PCM is connected to a CTRE Pneumatics Control Module or not.
        """
        module_type = wpilib.PneumaticsModuleType.CTREPCM if ctre_hub else wpilib.PneumaticsModuleType.REVPH
        self.solenoid = wpilib.DoubleSolenoid(module, module_type, forward_channel, reverse_channel)

    def extend(self):
        """
        Extend the piston.
        """
        self.solenoid.set(wpilib.DoubleSolenoid.Value.kForward)

    def retract(self):
        """
        Retract the piston.
        """
        self.solenoid.set(wpilib.DoubleSolenoid.Value.kReverse)

    def toggle(self):
        """
        Toggle the piston.
        """
        self.solenoid.toggle()

    def get_value(self) -> wpilib.DoubleSolenoid.Value:
        """
        Get the piston state (wpilib.DoubleSolenoid.Value.kReverse, kForward, or kOff).
        """
        return self.solenoid.get()
