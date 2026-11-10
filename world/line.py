import math
from abc import ABC, abstractmethod


class Line(ABC):
    '''
    Define a ground line and its preview.
    '''
    def __init__(self, width_mm=20.0, reflection=10, edge_width_mm=4.0):
        if not all(math.isfinite(v) and v > 0 for v in (width_mm, edge_width_mm)):
            raise ValueError("Line widths must be finite and positive")
        if edge_width_mm > width_mm:
            raise ValueError("Edge width cannot exceed line width")
        if not math.isfinite(reflection) or not 0 <= reflection <= 100:
            raise ValueError("Line reflection must be between 0 and 100")
        self.width_mm = width_mm
        self.reflection = reflection
        self.edge_width_mm = edge_width_mm

    @abstractmethod
    def distance_to(self, x_mm: float, y_mm: float) -> float:
        '''
        Return the shortest distance to the line centre in mm.
        '''
        raise NotImplementedError

    def get_reflection_at(self, x_mm, y_mm, background):
        '''
        Blend the line reflection into the background at its edge.
        '''
        distance = self.distance_to(x_mm, y_mm)
        blend = (distance - self.width_mm/2 + self.edge_width_mm/2) / self.edge_width_mm
        blend = max(0.0, min(1.0, blend))
        return round(self.reflection + blend * (background-self.reflection))

    @abstractmethod
    def to_svg(self) -> str:
        '''
        Return an SVG element for the line.
        '''
        raise NotImplementedError


class Polyline(Line):
    '''
    Represent a line with connected points in mm.
    '''
    def __init__(self, points, width_mm=20.0, reflection=10, edge_width_mm=4.0):
        super().__init__(width_mm, reflection, edge_width_mm)
        self.points = tuple(tuple(float(v) for v in point) for point in points)
        if len(self.points) < 2 or any(len(p) != 2 or not all(math.isfinite(v) for v in p)
                                      for p in self.points):
            raise ValueError("A line needs at least two finite 2D points")
        self._segments = []
        for (x, y), (end_x, end_y) in zip(self.points, self.points[1:]):
            dx, dy = end_x-x, end_y-y
            length_sq = dx*dx + dy*dy
            if length_sq == 0:
                raise ValueError("Consecutive line points must be different")
            self._segments.append((x, y, dx, dy, length_sq))

    def distance_to(self, x_mm, y_mm):
        '''
        Return the shortest distance to any segment in mm.
        '''
        nearest = math.inf
        for x, y, dx, dy, length_sq in self._segments:
            t = max(0.0, min(1.0, ((x_mm-x)*dx + (y_mm-y)*dy) / length_sq))
            nearest = min(nearest, (x_mm-x-t*dx)**2 + (y_mm-y-t*dy)**2)
        return math.sqrt(nearest)

    def to_svg(self):
        '''
        Return the line as an SVG element.
        '''
        points = ' '.join(f'{x:.2f},{-y:.2f}' for x, y in self.points)
        shade = round(self.reflection * 255 / 100)
        return (f'<polyline points="{points}" fill="none" stroke="rgb({shade},{shade},{shade})" '
                f'stroke-width="{self.width_mm}" stroke-linejoin="round" stroke-linecap="round"/>')
