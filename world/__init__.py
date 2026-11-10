from .world import World
from .line import Line, Polyline
from .obstacle import Obstacle, BoxObstacle
from .surface import SurfaceHit


current_world: World | None = None


def get_current_world() -> World:
    '''
    Return the active map and create a white map when none is selected.
    '''
    global current_world
    if current_world is None:
        from maps.flat_white.world import FlatWhiteWorld
        current_world = FlatWhiteWorld()
    return current_world
