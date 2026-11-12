from abc import ABC, abstractmethod

from ..parameters import Port
from sim.models.model import Model


class Device(ABC):
    '''
    Represent a simulated electronic device.

    Attributes:
        port (Port): Port the device is connected to.
        _model (Model | None): Simulator model, or None before a concrete
            device creates
            one.
    '''

    def __init__(self, port: Port) -> None:
        '''
        Create a device on a validated port.

        Arg:
            port (Port): Port the device is plugged into.
        '''
        if not isinstance(port, Port):
            raise TypeError('port must be a Port')
        self.port: Port = port
        self._model: Model | None = None
