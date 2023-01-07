import wpilib


class Compressor:
    """
    Wrapper class for a wpilib compressor.
    """

    def __init__(self, module, ctre_hub=False):
        """
        Args:
            module (int): The Pneumatics Control Module (PCM) CAN ID
            ctre_hub (bool): Whether the PCM is connected to a CTRE Pneumatics Control Module or not
        """
        module_type = wpilib.PneumaticsModuleType.CTREPCM if ctre_hub else wpilib.PneumaticsModuleType.REVPH
        self.compressor = wpilib.Compressor(module, module_type)

    def get_compressor_enabled(self) -> bool:
        """
        Get whether the compressor is enabled.
        """
        return self.compressor.enabled()

    def get_current(self) -> float:
        """
        Get the current draw of the compressor.
        """
        return self.compressor.getCurrent()

    def enable_analog(self, minimum_psi: float, maximum_psi: float):
        """
        Enable the compressor.

        Args:
            minimum_psi (float): The minimum pressure in PSI to enable the compressor
            maximum_psi (float): The maximum pressure in PSI to disable the compressor
        """
        self.compressor.enableAnalog(minimum_psi, maximum_psi)

    def start(self):
        """
        Start the compressor.
        """
        self.compressor.start()

    def stop(self):
        """
        Stop the compressor.
        """
        self.compressor.stop()
