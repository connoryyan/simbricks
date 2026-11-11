import math
from abc import ABC, abstractmethod
from sim.validation import number, sequence, vector


class Line(ABC):
    '''
    Define a ground line with a blended reflection edge.

    Attributes:
        width_mm (float): Total line width in mm.
        reflection (float): Reflection from 0 to 100.
        edge_width_mm (float): Width of the reflection blend at the line edge, in mm.
    '''
    def __init__(self, width_mm: float = 20.0, reflection: float = 10, edge_width_mm: float = 4.0) -> None:
        '''
        Create the shared ground line appearance.

        Arg:
            width_mm (float): Positive line width in mm.
            reflection (float): Line reflection from 0 to 100.
            edge_width_mm (float): Positive edge blend width, no wider than the line.
        '''
        number(width_mm, 'width_mm', positive=True)
        number(edge_width_mm, 'edge_width_mm', positive=True)
        if edge_width_mm > width_mm:
            raise ValueError("Edge width cannot exceed line width")
        number(reflection, 'reflection', minimum=0, maximum=100)
        self.width_mm: float = width_mm
        self.reflection: float = reflection
        self.edge_width_mm: float = edge_width_mm

    @abstractmethod
    def distance_to(self, x_mm: float, y_mm: float) -> float:
        '''
        Return the shortest distance to the line centre in mm.
        '''
        raise NotImplementedError

    def get_reflection_at(self, x_mm: float, y_mm: float, background) -> float:
        '''
        Blend the line reflection into the background at its edge.
        '''
        number(background, 'background', minimum=0, maximum=100)
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
    Represent a ground line as connected straight segments.

    Attributes:
        width_mm (float): Total line width in mm.
        reflection (float): Reflection from 0 to 100.
        edge_width_mm (float): Width of the reflection blend at the line edge, in mm.
        points (tuple): Connected (x, y) world points in mm.
        _segments (list): Cached (x, y, dx, dy, squared_length) tuples for each segment.
    '''
    def __init__(self, points, width_mm: float = 20.0, reflection: float = 10, edge_width_mm: float = 4.0) -> None:
        '''
        Create a line from connected two-dimensional points.

        Arg:
            points (list | tuple): At least two distinct neighboring points in mm.
            width_mm (float): Positive line width in mm.
            reflection (float): Line reflection from 0 to 100.
            edge_width_mm (float): Positive edge blend width in mm.
        '''
        super().__init__(width_mm, reflection, edge_width_mm)
        sequence(points, 'points')
        if len(points) < 2:
            raise ValueError('A line needs at least two points')
        self.points: tuple[tuple[float, float], ...] = tuple(vector(point, 'point', 2) for point in points)
        self._segments: list[tuple[float, float, float, float, float]] = []
        for (x, y), (end_x, end_y) in zip(self.points, self.points[1:]):
            dx, dy = end_x-x, end_y-y
            length_sq = dx*dx + dy*dy
            number(length_sq, 'Segment squared length')
            if length_sq == 0:
                raise ValueError("Consecutive line points must be different")
            self._segments.append((x, y, dx, dy, length_sq))

    def distance_to(self, x_mm: float, y_mm: float) -> float:
        '''
        Return the shortest distance to any segment in mm.
        '''
        number(x_mm, 'x_mm')
        number(y_mm, 'y_mm')
        nearest = math.inf
        for x, y, dx, dy, length_sq in self._segments:
            t = max(0.0, min(1.0, ((x_mm-x)*dx + (y_mm-y)*dy) / length_sq))
            nearest = min(nearest, (x_mm-x-t*dx)**2 + (y_mm-y-t*dy)**2)
        return math.sqrt(nearest)

    def to_svg(self) -> str:
        '''
        Return the line as an SVG element.
        '''
        points = ' '.join(f'{x:.2f},{-y:.2f}' for x, y in self.points)
        shade = round(self.reflection * 255 / 100)
        return (f'<polyline points="{points}" fill="none" stroke="rgb({shade},{shade},{shade})" '
                f'stroke-width="{self.width_mm}" stroke-linejoin="round" stroke-linecap="round"/>')
