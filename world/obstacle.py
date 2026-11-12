import math
from abc import ABC, abstractmethod

from .surface import SurfaceHit
from sim.validation import number, unit_vector, vector


class Obstacle(ABC):
    '''
    Define ray intersections and a preview for an obstacle shape.
    '''
    @abstractmethod
    def raycast(self, origin, direction, max_mm: float) -> SurfaceHit | None:
        '''
        Return the nearest surface hit along a unit ray.

        Arg:
            origin (tuple): World ray origin in mm.
            direction (tuple): Unit ray direction.
            max_mm (float): Maximum ray distance in mm.

        Returns:
            SurfaceHit | None: First surface hit, or None.
        '''
        raise NotImplementedError

    @abstractmethod
    def to_svg(self) -> str:
        '''
        Return an SVG element for the obstacle footprint.
        '''
        raise NotImplementedError


class BoxObstacle(Obstacle):
    '''
    Represent an axis-aligned box standing on the ground.

    Attributes:
        xmin (float): Minimum world x coordinate in mm.
        ymin (float): Minimum world y coordinate in mm.
        xmax (float): Maximum world x coordinate in mm.
        ymax (float): Maximum world y coordinate in mm.
        height_mm (float): Box height above the ground in mm.
        reflection (float): Reflection from 0 to 100.
    '''

    def __init__(
        self,
        xmin: float,
        ymin: float,
        xmax: float,
        ymax: float,
        height_mm: float,
        reflection: int = 40,
    ) -> None:
        '''
        Create an obstacle box.

        Arg:
            xmin (float): Minimum x coordinate in mm.
            ymin (float): Minimum y coordinate in mm.
            xmax (float): Maximum x coordinate in mm.
            ymax (float): Maximum y coordinate in mm.
            height_mm (float): Height above the ground in mm.
            reflection (int): Surface reflection from 0 to 100.
        '''
        for name, value in zip(('xmin', 'ymin', 'xmax', 'ymax', 'height_mm'),
                               (xmin, ymin, xmax, ymax, height_mm)):
            number(value, name)
        if xmin >= xmax or ymin >= ymax or height_mm <= 0:
            raise ValueError(
                "Box dimensions must form a finite positive volume")
        number(reflection, 'reflection', minimum=0, maximum=100)
        self.xmin: float = xmin
        self.ymin: float = ymin
        self.xmax: float = xmax
        self.ymax: float = ymax
        self.height_mm: float = height_mm
        self.reflection: float = reflection

    def raycast(self, origin, direction, max_mm: float) -> SurfaceHit | None:
        '''
        Use Slab Algorithm to find where the ray first hits a box.

        Arg:
            origin (tuple): Ray origin in mm.
            direction (tuple): Unit ray direction.
            max_mm (float): Maximum ray distance in mm.

        Returns:
            SurfaceHit | None: First surface hit, or None.
        '''
        origin = vector(origin, 'origin', 3)
        direction = unit_vector(direction, 'direction')
        number(max_mm, 'max_mm', minimum=0)
        entry, exit = -math.inf, math.inf
        entry_cosine = exit_cosine = 0.0
        for o, d, lo, hi in zip(origin, direction,
                                (self.xmin, self.ymin, 0.0),
                                (self.xmax, self.ymax, self.height_mm)):
            if abs(d) < 1e-12:
                if o < lo or o > hi:
                    return None
                continue
            near, far = sorted(((lo - o) / d, (hi - o) / d))
            if near > entry:
                entry, entry_cosine = near, abs(d)
            if far < exit:
                exit, exit_cosine = far, abs(d)
            if entry > exit:
                return None
        distance, cosine = ((entry, entry_cosine) if entry >= 0
                            else (exit, exit_cosine))
        if distance < 0 or distance > max_mm:
            return None
        x, y, z = (o + distance * d for o, d in zip(origin, direction))
        return SurfaceHit(x, y, z, distance, self.reflection, cosine)

    def to_svg(self) -> str:
        '''
        Return the obstacle footprint as an SVG element.
        '''
        return (
            f'<rect x="{self.xmin}" y="{-self.ymax}" '
            f'width="{self.xmax - self.xmin}" '
            f'height="{self.ymax - self.ymin}" fill="#e57235"/>'
        )
