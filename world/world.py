from abc import ABC, abstractmethod
import math
from pathlib import Path

from .line import Line
from .obstacle import Obstacle
from .surface import SurfaceHit


class World(ABC):
    '''
    Define a map with ground lines, obstacles and ambient light.
    '''
    @abstractmethod
    def __init__(self, ambient_light: float = 15.0, *, reflection=85,
                 bounds=(-100, -380, 2600, 320)):
        '''
        Create the common map state.

        Arg:
            ambient_light (float): Ambient signal from 0 to 100.
            reflection (float): Ground reflection from 0 to 100.
            bounds (tuple): Preview xmin, ymin, xmax and ymax in mm.
        '''
        if not math.isfinite(reflection) or not 0 <= reflection <= 100:
            raise ValueError("Ground reflection must be between 0 and 100")
        if (len(bounds) != 4 or not all(math.isfinite(v) for v in bounds)
                or bounds[0] >= bounds[2] or bounds[1] >= bounds[3]):
            raise ValueError("Preview bounds must form a finite rectangle")
        self.ambient_light = ambient_light
        self.reflection = reflection
        self.bounds = bounds
        self.lines: list[Line] = []
        self.obstacles: list[Obstacle] = []

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
        Find the nearest ground or obstacle surface along a unit ray.

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
        hit = None
        if z_mm > 0 and dz < -1e-9:
            distance = -z_mm / dz
            if distance <= max_mm:
                x = x_mm + distance * dx
                y = y_mm + distance * dy
                hit = SurfaceHit(x, y, 0.0, distance,
                                 self.get_reflection_at(x, y), -dz)
        for obstacle in self.obstacles:
            candidate = obstacle.raycast((x_mm, y_mm, z_mm), (dx, dy, dz), max_mm)
            if candidate is not None and (hit is None or candidate.distance_mm < hit.distance_mm):
                hit = candidate
        return hit

    def get_reflection_at(self, x_mm: float, y_mm: float) -> int:
        '''
        Return the ground reflection at a point.

        Arg:
            x_mm (float): x coordinate in mm.
            y_mm (float): y coordinate in mm.

        Returns:
            int: Ground reflection from 0 to 100.
        '''
        return min((line.get_reflection_at(x_mm, y_mm, self.reflection)
                    for line in self.lines), default=self.reflection)

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
        yaw = math.radians(heading_deg)
        pitch = math.radians(pitch_deg)
        hit = self.raycast_surface(x_mm, y_mm, z_mm,
                                  math.cos(pitch)*math.cos(yaw),
                                  math.cos(pitch)*math.sin(yaw), -math.sin(pitch), max_mm)
        return max_mm if hit is None else hit.distance_mm


    def add_line(self, line: Line) -> None:
        '''
        Add a ground line to the map.
        '''
        if not isinstance(line, Line):
            raise TypeError("line must be a Line")
        self.lines.append(line)

    def add_obstacle(self, obstacle: Obstacle) -> None:
        '''
        Add an obstacle to the map.
        '''
        if not isinstance(obstacle, Obstacle):
            raise TypeError("obstacle must be an Obstacle")
        self.obstacles.append(obstacle)

    def save_preview(self, path, trace=()) -> None:
        '''
        Save the map and optional robot path as an SVG preview.

        Arg:
            path (Path): Output image path.
            trace (tuple): Robot poses as x, y and heading in degrees.
        '''
        xmin, ymin, xmax, ymax = self.bounds
        parts = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="{xmin} {-ymax} {xmax-xmin} {ymax-ymin}">',
                 f'<rect x="{xmin}" y="{-ymax}" width="{xmax-xmin}" height="{ymax-ymin}" fill="white"/>']
        parts.extend(line.to_svg() for line in self.lines)
        parts.extend(obstacle.to_svg() for obstacle in self.obstacles)
        if trace:
            points = ' '.join(f'{x:.2f},{-y:.2f}' for x, y, _ in trace)
            parts.append(f'<polyline points="{points}" fill="none" stroke="#168dc1" stroke-width="3"/>')
            x, y, yaw = trace[-1]
            ax, ay = x+70*math.cos(math.radians(yaw)), y+70*math.sin(math.radians(yaw))
            parts.append(f'<circle cx="{x}" cy="{-y}" r="65" fill="#168dc1" fill-opacity="0.25"/>')
            parts.append(f'<path d="M{x},{-y} L{ax},{-ay}" stroke="#168dc1" stroke-width="5"/>')
        parts.append('</svg>')
        Path(path).write_text(''.join(parts), encoding='utf-8')
