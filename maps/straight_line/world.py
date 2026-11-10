from world import World, Polyline


class LineWorld(World):
    '''
    Simulate a straight black line along the x axis.
    '''
    def __init__(self, line_width_mm=20.0, *, ambient_light=5.0):
        super().__init__(ambient_light)
        self.add_line(Polyline([(-1000, 0), (3000, 0)], width_mm=line_width_mm))
