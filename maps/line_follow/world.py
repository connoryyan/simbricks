from world import World, Polyline, BoxObstacle


class LineFollowWorld(World):
    '''
    Provide a winding black track ending at a 150 mm high box.

    Attributes:
        ambient_light (float): Uniform ambient light signal from 0 to 100.
        reflection (float): Ground reflection outside lines, from 0 to 100.
        bounds (tuple): Preview rectangle (xmin, ymin, xmax, ymax) in world mm.
        lines (list): Ground lines contributing to reflection readings.
        obstacles (list): Obstacle shapes tested by surface rays.
        friction_coefficient (float): Uniform non-negative ground friction
            coefficient.
        _friction_coefficient (float): Stored coefficient backing
            friction_coefficient.
        _ambient_light (float): Stored ambient signal backing ambient_light.
        line_width_mm (float): Configured track width in mm.
        black_reflection (int): Configured black line reflection from 0 to 100.
        edge_width_mm (float): Width of the reflection blend at the line edge,
            in mm.
        path (list): Connected track center points (x, y) in world mm.
    '''

    def __init__(self, *, ambient_light: float = 5.0) -> None:
        '''
        Create a winding black line ending at a 150 mm obstacle.

        Arg:
            ambient_light (float): Ambient light from 0 to 100.
        '''
        super().__init__(reflection=85, ambient_light=ambient_light)
        self.line_width_mm: float = 20.0
        self.black_reflection: int = 10
        self.edge_width_mm: float = 4.0
        self.path: list[tuple[float, float]] = [(0.0, 0.0), (250.0, 0.0)]
        curves = (
            ((250, 0), (450, 0), (450, 250), (650, 250)),
            ((650, 250), (850, 250), (900, -150), (1100, -150)),
            ((1100, -150), (1300, -150), (1350, 120), (1550, 120)),
            ((1550, 120), (1700, 120), (1750, 0), (1900, 0)),
        )
        for points in curves:
            for i in range(1, 61):
                t = i / 60.0
                weights = ((1 - t)**3, 3 * (1 - t)**2
                           * t, 3 * (1 - t) * t * t, t**3)
                self.path.append(
                    tuple(
                        sum(w * p[axis] for w, p in zip(weights, points))
                        for axis in (0, 1)))
        self.path.append((2300.0, 0.0))
        self.add_line(
            Polyline(
                self.path,
                width_mm=self.line_width_mm,
                reflection=self.black_reflection,
                edge_width_mm=self.edge_width_mm))
        self.add_obstacle(BoxObstacle(2300, -100, 2450, 100, 150))

    def distance_to_line(self, x_mm: float, y_mm: float) -> float:
        '''
        Return the distance to the track centre in mm.
        '''
        return self.lines[0].distance_to(x_mm, y_mm)
