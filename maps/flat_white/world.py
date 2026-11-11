from world import World


class FlatWhiteWorld(World):
    '''
    Provide a uniform white field without lines or obstacles.

    Attributes:
        ambient_light (float): Uniform ambient light signal from 0 to 100.
        reflection (float): Ground reflection outside lines, from 0 to 100.
        bounds (tuple): Preview rectangle (xmin, ymin, xmax, ymax) in world mm.
        lines (list): Ground lines contributing to reflection readings.
        obstacles (list): Obstacle shapes tested by surface rays.
        friction_coefficient (float): Uniform non-negative ground friction coefficient.
        _friction_coefficient (float): Stored coefficient backing friction_coefficient.
        _ambient_light (float): Stored ambient signal backing ambient_light.
    '''
    def __init__(self, reflection: float = 85, *, ambient_light: float = 5.0) -> None:
        '''
        Create a uniform white field without obstacles.

        Arg:
            reflection (float): Ground reflection from 0 to 100.
            ambient_light (float): Ambient light from 0 to 100.
        '''
        super().__init__(ambient_light, reflection=reflection)
