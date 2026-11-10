from sim.initialization import requires_init
from .parameters import Axis
from .pupdevices.sensor import IMU
from sim.hub import get_hub


class _Buttons:
    def __init__(self, model):
        self._model = model

    @requires_init
    def pressed(self):
        '''
        Return the currently pressed hub buttons.
        '''
        return set(self._model.pressed_buttons)


class PrimeHub:
    '''
    Provide a level Prime Hub with a shared IMU and simulated buttons.
    '''
    def __init__(self, top_side=Axis.Z, front_side=Axis.X):
        '''
        Connect to the simulated hub.

        Arg:
            top_side (Axis): Hub top direction. Only Axis.Z is supported.
            front_side (Axis): Hub front direction. Only Axis.X is supported.
        '''
        if top_side != Axis.Z or front_side != Axis.X:
            raise NotImplementedError('Only the default level hub mounting is simulated')
        self._model = get_hub()
        self.imu = IMU(lambda: self._model.imu)
        self.buttons = _Buttons(self._model)

    @property
    def model(self):
        '''
        Return the hub model for simulator configuration.
        '''
        return self._model


InventorHub = PrimeHub
