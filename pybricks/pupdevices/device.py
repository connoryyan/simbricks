from abc import ABC, abstractmethod

from ..parameters import Direction, Port, Stop

class Device(ABC):
    '''
    Simulate an electronic component of the robot. 
    '''

    def __init__(self, port: Port):
        self.port = port