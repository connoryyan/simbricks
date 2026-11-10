import sim
import world
from sim.initialization import require_initialized
from maps.flat_white.world import FlatWhiteWorld


def initialize_existing():
    '''
    Enter the unit test initializer before exercising robot APIs.
    '''
    try:
        require_initialized()
        return
    except sim.NotInitializedError:
        pass

    def init():
        if world.current_world is None:
            world.current_world = FlatWhiteWorld()

    sim.initialize(init)
