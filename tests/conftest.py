import pytest

import sim
import world
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
    yield
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
