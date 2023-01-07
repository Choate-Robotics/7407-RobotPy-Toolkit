import commands2

from robotpy_toolkit_7407.network.objects import Sendable, SubsystemNetworkObject


class Subsystem(commands2.SubsystemBase, Sendable[SubsystemNetworkObject]):
    """
    Extendable subsystem class. Needs to be extended by every subsystem.
    """
    def init(self):
        """
        Overridable method for initializing the subsystem. Place motor inits/other physical initialization here,
        for example re-zeroing a gyro.
        """
        ...

    def get_network_object(self) -> SubsystemNetworkObject:
        return SubsystemNetworkObject(
            name=self.__class__.__name__,
            motors=[]
        )

    def network_update(self, data: SubsystemNetworkObject): ...

