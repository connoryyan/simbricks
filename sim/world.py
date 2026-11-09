from abc import ABC, abstractmethod
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class SurfaceHit:
    '''
    Store a surface hit in world coordinates.
    '''
    x_mm: float
    y_mm: float
    z_mm: float
    distance_mm: float
    reflection: int
    incidence_cosine: float


class World(ABC):
    def __init__(self, ambient_light: float = 15.0):
        self.ambient_light = ambient_light

    def get_ambient_light(self) -> float:
        return self._ambient_light

    def set_ambient_light(self, value: float) -> None:
        value = float(value)
        if not math.isfinite(value) or not 0 <= value <= 100:
            raise ValueError("Ambient light must be finite and between 0 and 100")
        self._ambient_light = value

    ambient_light = property(get_ambient_light, set_ambient_light)

    def get_ambient_at(self, x_mm: float, y_mm: float, z_mm: float) -> float:
        '''
        Return uniform ambient light at the sensor position.

        Arg:
            x_mm (float): World x coordinate in mm.
            y_mm (float): World y coordinate in mm.
            z_mm (float): Height above the ground in mm.

        Returns:
            float: Ambient light from 0 to 100 in the simulated signal scale.
        '''
        return self.ambient_light

    def raycast_surface(self, x_mm: float, y_mm: float, z_mm: float,
                        dx: float, dy: float, dz: float,
                        max_mm: float) -> SurfaceHit | None:
        '''
        Find the ground surface along a unit ray.

        Arg:
            x_mm (float): Ray origin x in mm.
            y_mm (float): Ray origin y in mm.
            z_mm (float): Ray origin height in mm.
            dx (float): Unit direction along x.
            dy (float): Unit direction along y.
            dz (float): Unit direction along z.
            max_mm (float): Maximum ray distance in mm.

        Returns:
            SurfaceHit | None: Nearest surface hit, or None when no surface is seen.
        '''
        if z_mm <= 0 or dz >= -1e-9:
            return None
        distance = -z_mm / dz
        if distance > max_mm:
            return None
        x = x_mm + distance * dx
        y = y_mm + distance * dy
        return SurfaceHit(x, y, 0.0, distance,
                          self.get_reflection_at(x, y), -dz)

    @abstractmethod
    def get_reflection_at(self, x_mm: float, y_mm: float) -> int:
        '''
        Return the ground reflection at a point.

        Arg:
            x_mm (float): x coordinate in mm.
            y_mm (float): y coordinate in mm.

        Returns:
            int: Ground reflection from 0 to 100.
        '''
        raise NotImplementedError

    @abstractmethod
    def raycast_distance(self, x_mm: float, y_mm: float, heading_deg: float,
                         max_mm: float = 2000, *, z_mm: float = 0.0,
                         pitch_deg: float = 0.0) -> float:
        '''
        Return the distance to the nearest object along a ray.

        Arg:
            x_mm (float): x coordinate in mm.
            y_mm (float): y coordinate in mm.
            heading_deg (float): Counter-clockwise ray direction in degrees.
            max_mm (float): When no object is detected, return this value.
            z_mm (float): Ray origin height in mm.
            pitch_deg (float): Downward ray pitch in degrees.

        Returns:
            float: Distance in mm, or max_mm when no object is detected.
        '''
        raise NotImplementedError


class FlatWhiteWorld(World):
    '''
    Simulate a white field with no obstacles.
    '''

    def __init__(self, reflection: int = 85, *, ambient_light: float = 5.0):
        super().__init__(ambient_light)
        self.reflection = reflection

    def get_reflection_at(self, x_mm: float, y_mm: float) -> int:
        return self.reflection

    def raycast_distance(self, x_mm: float, y_mm: float, heading_deg: float,
                         max_mm: float = 2000, *, z_mm: float = 0.0,
                         pitch_deg: float = 0.0) -> float:
        yaw = math.radians(heading_deg)
        pitch = math.radians(pitch_deg)
        hit = self.raycast_surface(
            x_mm, y_mm, z_mm,
            math.cos(pitch) * math.cos(yaw),
            math.cos(pitch) * math.sin(yaw), -math.sin(pitch), max_mm,
        )
        return max_mm if hit is None else hit.distance_mm


class ScriptableWorld(World):
    '''
    Provide fixed sensor readings for tests.
    '''

    def __init__(self, *, ambient_light: float = 5.0):
        super().__init__(ambient_light)
        self._reflection = 50
        self._distance = 2000.0

    def set_reflection(self, value: int) -> None:
        self._reflection = value

    def set_distance(self, value_mm: float) -> None:
        self._distance = value_mm

    def get_reflection_at(self, x_mm: float, y_mm: float) -> int:
        return self._reflection

    def raycast_distance(self, x_mm: float, y_mm: float, heading_deg: float,
                         max_mm: float = 2000, *, z_mm: float = 0.0,
                         pitch_deg: float = 0.0) -> float:
        # This test world deliberately returns a scripted range without geometry.
        return min(self._distance, max_mm)


class LineWorld(FlatWhiteWorld):
    '''
    Simulate a straight black line along the x axis.
    '''

    def __init__(self, line_width_mm: float = 20.0, *, ambient_light: float = 5.0):
        super().__init__(reflection=85, ambient_light=ambient_light)
        self.line_width_mm = line_width_mm
        self.black_reflection = 10
        self.edge_width_mm = 4.0

    def get_reflection_at(self, x_mm: float, y_mm: float) -> int:
        distance = abs(y_mm)
        half_width = self.line_width_mm / 2.0

        # Blend black into white across the line edge.
        blend = (
            distance - half_width + self.edge_width_mm / 2.0
        ) / self.edge_width_mm
        blend = max(0.0, min(1.0, blend))

        return round(
            self.black_reflection
            + blend * (self.reflection - self.black_reflection)
        )


# Default field. When launching the sim it can be replaced.
current_world: World = FlatWhiteWorld()

