from sim.initialization import requires_init
from .parameters import Axis, Button
from .pupdevices.sensor import IMU
from sim.hub import HubModel, get_hub


class _Buttons:
    '''
    Read pressed buttons from the shared hub model.

    Attributes:
        _model (HubModel): Shared hub model containing the scripted button set.
    '''

    def __init__(self, model: HubModel) -> None:
        if not isinstance(model, HubModel):
            raise TypeError('model must be a HubModel')
        self._model: HubModel = model

    @requires_init
    def pressed(self) -> set[Button]:
        '''
        Return the currently pressed hub buttons.
        '''
        if not isinstance(self._model.pressed_buttons, set):
            raise TypeError('pressed_buttons must be a set')
        if not all(isinstance(button, Button)
                   for button in self._model.pressed_buttons):
            raise TypeError('pressed_buttons must contain Button values')
        return set(self._model.pressed_buttons)


class PrimeHub:
    '''
    Provide a level hub with a shared IMU and buttons.

    Attributes:
        _model (HubModel): Shared simulator hub model.
        imu (IMU): IMU interface reading the shared model.
        buttons (_Buttons): Interface returning a copy of the pressed button
            set.
        model (HubModel): Shared hub model exposed for simulator setup.
    '''

    def __init__(
        self,
        top_side: Axis = Axis.Z,
        front_side: Axis = Axis.X,
    ) -> None:
        '''
        Connect to the simulated hub.

        Arg:
            top_side (Axis): Hub top direction. Only Axis.Z is supported.
            front_side (Axis): Hub front direction. Only Axis.X is supported.
        '''
        if not isinstance(top_side, Axis) or not isinstance(front_side, Axis):
            raise TypeError('top_side and front_side must be Axis values')
        if top_side != Axis.Z or front_side != Axis.X:
            raise NotImplementedError(
                'Only the default level hub mounting is simulated')
        self._model: HubModel = get_hub()
        self.imu: IMU = IMU(lambda: self._model.imu)
        self.buttons: _Buttons = _Buttons(self._model)

    @property
    def model(self) -> HubModel:
        '''
        Return the hub model for simulator configuration.
        '''
        return self._model


InventorHub = PrimeHub
