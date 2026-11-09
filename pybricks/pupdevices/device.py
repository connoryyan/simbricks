from abc import ABC, abstractmethod

from ..parameters import Port
from sim.models.model import Model

class Device(ABC):
    '''
    Simulate an electronic component of the robot. 
    '''

    def __init__(self, port: Port):
        self.port = port
        self._model: Model | None = None