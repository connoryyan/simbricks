from .clock import clock
from .models.model import Model
from .models.imu_model import IMUModel


class HubModel(Model):
    '''
    Store one physical hub and its shared IMU in the current simulation.
    '''
    def __init__(self):
        self.imu = IMUModel()
        self.body = None
        self.pressed_buttons = set()
        super().__init__()

    def step(self, dt_s):
        pass

    def bind_body(self, body):
        '''
        Connect a drive body to the hub without resetting IMU calibration.
        '''
        if self.body is not None and self.body is not body and self.body in clock.steppables:
            raise ValueError('One PrimeHub can only measure one active drive body')
        if body._imu is not None and body._imu is not self.imu:
            clock.remove_steppable(self.imu)
            self.imu = body._imu
        else:
            body._imu = self.imu
        self.body = body
        self.imu.body = body
        self.imu.rebase_pose()
        clock.add_steppable(self.imu)


def get_active_hub():
    '''
    Return the hub registered on the current clock, or None.
    '''
    return next((item for item in clock.steppables if isinstance(item, HubModel)), None)


def get_hub():
    '''
    Get the shared hub and bind an existing drive body when possible.
    '''
    hub = get_active_hub()
    if hub is None:
        from .models.drive_base_model import DriveBaseModel
        bodies = [item for item in clock.steppables if isinstance(item, DriveBaseModel)]
        if len(bodies) > 1:
            raise ValueError('Create PrimeHub before creating multiple simulated bodies')
        hub = HubModel()
        if bodies:
            hub.bind_body(bodies[0])
    return hub
