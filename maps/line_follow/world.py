from world import World, Polyline, BoxObstacle


class LineFollowWorld(World):
    '''
    Simulate a winding 20 mm black line with a 150 mm obstacle at the end.
    '''

    def __init__(self, *, ambient_light: float = 5.0):
        super().__init__(reflection=85, ambient_light=ambient_light)
        self.line_width_mm = 20.0
        self.black_reflection = 10
        self.edge_width_mm = 4.0
        self.path = [(0.0, 0.0), (250.0, 0.0)]
        curves = (
            ((250, 0), (450, 0), (450, 250), (650, 250)),
            ((650, 250), (850, 250), (900, -150), (1100, -150)),
            ((1100, -150), (1300, -150), (1350, 120), (1550, 120)),
            ((1550, 120), (1700, 120), (1750, 0), (1900, 0)),
        )
        for points in curves:
            for i in range(1, 61):
                t = i / 60.0
                weights = ((1-t)**3, 3*(1-t)**2*t, 3*(1-t)*t*t, t**3)
                self.path.append(tuple(sum(w*p[axis] for w, p in zip(weights, points))
                                       for axis in (0, 1)))
        self.path.append((2300.0, 0.0))
        self.add_line(Polyline(self.path, width_mm=self.line_width_mm,
                               reflection=self.black_reflection, edge_width_mm=self.edge_width_mm))
        self.add_obstacle(BoxObstacle(2300, -100, 2450, 100, 150))

    def distance_to_line(self, x_mm, y_mm):
        '''
        Return the distance to the track centre in mm.
        '''
        return self.lines[0].distance_to(x_mm, y_mm)
