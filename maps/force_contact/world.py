from world import BoxObstacle, Polyline, World


class ForceContactWorld(World):
    '''
    Provide a slippery floor and a wall for a front force sensor.

    Attributes:
        ambient_light (float): Uniform ambient signal from 0 to 100.
        _ambient_light (float): Stored ambient signal.
        reflection (float): Background ground reflection from 0 to 100.
        bounds (tuple): Preview rectangle in world mm.
        lines (list): Gray route used as a visual reference.
        obstacles (list): Wall with a front face at x = 1350 mm.
        friction_coefficient (float): Low ground friction coefficient.
        _friction_coefficient (float): Stored ground friction coefficient.
    '''

    def __init__(self) -> None:
        '''
        Create a low-friction route ending at a 150 mm high wall.
        '''
        super().__init__(
            ambient_light=5,
            bounds=(-150, -160, 1550, 560),
            friction_coefficient=0.04,
        )
        self.add_line(Polyline(
            [(0, 0), (700, 0), (700, 400), (1350, 400)],
            width_mm=12,
            reflection=70,
            edge_width_mm=2,
        ))
        self.add_obstacle(BoxObstacle(1350, 200, 1490, 550, 150))
