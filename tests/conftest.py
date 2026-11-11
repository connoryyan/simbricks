import pytest

import sim
import world
from maps.flat_white.world import FlatWhiteWorld
from pybricks.hubs import PrimeHub
from pybricks.parameters import Direction, Port
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from sim.clock import clock


@pytest.fixture(autouse=True)
def clean_simulation():
    '''
    Keep global simulation state separate for each test.
    '''
    previous_groups = {priority: group.copy() for priority, group in clock._steppables.items()}
    previous_time = clock.time_ms
    previous_initialization = clock._initialization
    previous_world = world.current_world
    clock._steppables.clear()
    clock.time_ms = 0
    clock._initialization = None
    world.current_world = None
    try:
        yield
    finally:
        clock._steppables.clear()
        clock._steppables.update(previous_groups)
        clock.time_ms = previous_time
        clock._initialization = previous_initialization
        world.current_world = previous_world


@pytest.fixture
def initialized():
    def init():
        pass

    sim.initialize(init)


@pytest.fixture
def robot(initialized):
    left = Motor(Port.B, Direction.COUNTERCLOCKWISE)
    right = Motor(Port.D)
    return DriveBase(left, right, 56, 112)


@pytest.fixture
def hub_robot(initialized):
    hub = PrimeHub()
    left = Motor(Port.B, Direction.COUNTERCLOCKWISE)
    right = Motor(Port.D)
    robot = DriveBase(left, right, 56, 112)
    return hub, robot


@pytest.fixture
def flat_field():
    field = FlatWhiteWorld()
    world.current_world = field
    return field


@pytest.fixture
def white_field():
    field = FlatWhiteWorld(ambient_light=0)
    world.current_world = field
    return field
