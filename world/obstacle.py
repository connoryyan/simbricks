import math
from abc import ABC, abstractmethod

from .surface import SurfaceHit


class Obstacle(ABC):
    '''
    Define an obstacle surface and its preview.
    '''
    @abstractmethod
    def raycast(self, origin, direction, max_mm) -> SurfaceHit | None:
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
    Store an obstacle box in mm.
    '''
    def __init__(self, xmin: float, ymin: float, xmax: float, ymax: float,
                 height_mm: float, reflection: int = 40):
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
        if (not all(math.isfinite(v) for v in (xmin, ymin, xmax, ymax, height_mm))
                or xmin >= xmax or ymin >= ymax or height_mm <= 0):
            raise ValueError("Box dimensions must form a finite positive volume")
        if not math.isfinite(reflection) or not 0 <= reflection <= 100:
            raise ValueError("Obstacle reflection must be between 0 and 100")
        self.xmin = xmin
        self.ymin = ymin
        self.xmax = xmax
        self.ymax = ymax
        self.height_mm = height_mm
        self.reflection = reflection

    def raycast(self, origin, direction, max_mm) -> SurfaceHit | None:
        '''
        Find the first box surface along a unit ray.

        Arg:
            origin (tuple): Ray origin in mm.
            direction (tuple): Unit ray direction.
            max_mm (float): Maximum ray distance in mm.

        Returns:
            SurfaceHit | None: First surface hit, or None.
        '''
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
        return (f'<rect x="{self.xmin}" y="{-self.ymax}" '
                f'width="{self.xmax-self.xmin}" height="{self.ymax-self.ymin}" '
                'fill="#e57235"/>')
