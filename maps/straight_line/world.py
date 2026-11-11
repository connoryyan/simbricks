from world import World, Polyline


class LineWorld(World):
    '''
    Provide a straight black line along the world x axis.

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
    def __init__(self, line_width_mm: float = 20.0, *, ambient_light: float = 5.0) -> None:
        '''
        Create a straight black line along the world x axis.

        Arg:
            line_width_mm (float): Line width in mm.
            ambient_light (float): Ambient light from 0 to 100.
        '''
        super().__init__(ambient_light)
        self.add_line(Polyline([(-1000, 0), (3000, 0)], width_mm=line_width_mm))
