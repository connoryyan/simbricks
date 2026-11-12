from .clock import clock
from .models.model import Model
from .models.imu_model import IMUModel
from .validation import number
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from pybricks.parameters import Button
    from .models.drive_base_model import DriveBaseModel


class HubModel(Model):
    '''
    Store the shared hub, IMU and scripted buttons.

    Attributes:
        step_priority (float): Clock update phase; smaller phases run first.
        _time_constant_s (float): Stored device response time constant in
            seconds.
        time_constant_s (float): Read-only device response time constant in
            seconds.
        imu (IMUModel): Shared IMU model used by hub and drive base interfaces.
        body (DriveBaseModel | None): Drive base body supplying motion, or None
            for an
            idle hub.
        pressed_buttons (set): Set of Button values currently pressed in the
            simulator.
    '''

    def __init__(self) -> None:
        '''
        Create a shared hub with an idle IMU and no pressed buttons.
        '''
        self.imu: IMUModel = IMUModel()
        self.body: DriveBaseModel | None = None
        self.pressed_buttons: set[Button] = set()
        super().__init__()

    def step(self, dt_s: float) -> None:
        '''
        Validate the hub step without sampling its separate IMU model.

        Arg:
            dt_s (float): Positive simulation step in seconds.
        '''
        number(dt_s, 'dt_s', positive=True)

    def bind_body(self, body: 'DriveBaseModel') -> None:
        '''
        Connect a drive body to the hub without resetting IMU calibration.
        '''
        from .models.drive_base_model import DriveBaseModel
        if not isinstance(body, DriveBaseModel):
            raise TypeError('body must be a DriveBaseModel')
        if (self.body is not None
                and self.body is not body
                and self.body in clock.steppables):
            raise ValueError(
                'One PrimeHub can only measure one active drive body')
        if body._imu is not None and body._imu is not self.imu:
            clock.remove_steppable(self.imu)
            self.imu = body._imu
        else:
            body._imu = self.imu
        self.body = body
        self.imu.body = body
        self.imu.rebase_pose()
        clock.add_steppable(self.imu)


def get_active_hub() -> HubModel | None:
    '''
    Return the hub registered on the current clock, or None.
    '''
    return next(
        (item for item in clock.steppables if isinstance(item, HubModel)),
        None)


def get_hub() -> HubModel:
    '''
    Get the shared hub and bind an existing drive body when possible.
    '''
    hub = get_active_hub()
    if hub is None:
        from .models.drive_base_model import DriveBaseModel
        bodies = [item for item in clock.steppables if isinstance(
            item, DriveBaseModel)]
        if len(bodies) > 1:
            raise ValueError(
                'Create PrimeHub before creating multiple simulated bodies')
        hub = HubModel()
        if bodies:
            hub.bind_body(bodies[0])
    return hub
