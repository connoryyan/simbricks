from abc import ABC, abstractmethod
import math
from pathlib import Path

from .line import Line
from .obstacle import Obstacle
from .surface import SurfaceHit
from sim.validation import number, sequence, unit_vector, vector


class World(ABC):
    '''
    Store ground appearance, lines, obstacles and friction.

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
        preview_error (OSError | None): Last SVG write error, or None after
            a successful write.
    '''
    @abstractmethod
    def __init__(
        self,
        ambient_light: float = 15.0,
        *,
        reflection: float = 85,
        bounds=(-100, -380, 2600, 320),
        friction_coefficient: float = 0.8,
    ) -> None:
        '''
        Create the common map state.

        Arg:
            ambient_light (float): Ambient signal from 0 to 100.
            reflection (float): Ground reflection from 0 to 100.
            bounds (tuple): Preview xmin, ymin, xmax and ymax in mm.
            friction_coefficient (float): Ground friction coefficient.
        '''
        number(reflection, 'reflection', minimum=0, maximum=100)
        bounds = vector(bounds, 'bounds', 4)
        if bounds[0] >= bounds[2] or bounds[1] >= bounds[3]:
            raise ValueError("Preview bounds must form a finite rectangle")
        self.ambient_light: float = ambient_light
        self.reflection: float = reflection
        self.bounds: tuple[float, float, float, float] = bounds
        self.lines: list[Line] = []
        self.obstacles: list[Obstacle] = []
        self.friction_coefficient: float = friction_coefficient
        self.preview_error: OSError | None = None

    @property
    def friction_coefficient(self) -> float:
        '''
        Return the non-negative ground friction coefficient.
        '''
        return self._friction_coefficient

    @friction_coefficient.setter
    def friction_coefficient(self, value: float) -> None:
        '''
        Set the uniform ground friction coefficient.

        Arg:
            value (float): Finite non-negative friction coefficient.
        '''
        number(value, 'friction_coefficient', minimum=0)
        self._friction_coefficient: float = value

    def get_friction_at(self, x_mm: float, y_mm: float) -> float:
        '''
        Return ground friction at a wheel contact point.

        Arg:
            x_mm (float): World x coordinate in mm.
            y_mm (float): World y coordinate in mm.

        Returns:
            float: Non-negative friction coefficient.
        '''
        number(x_mm, 'x_mm')
        number(y_mm, 'y_mm')
        return self.friction_coefficient

    def get_ambient_light(self) -> float:
        '''
        Return uniform ambient light from 0 to 100.
        '''
        return self._ambient_light

    def set_ambient_light(self, value: float) -> None:
        '''
        Set the uniform ambient light signal.

        Arg:
            value (float): Ambient light from 0 to 100.
        '''
        number(value, 'ambient_light', minimum=0, maximum=100)
        self._ambient_light: float = value

    @property
    def ambient_light(self) -> float:
        '''
        Return uniform ambient light from 0 to 100.
        '''
        return self.get_ambient_light()

    @ambient_light.setter
    def ambient_light(self, value: float) -> None:
        '''
        Set uniform ambient light.

        Arg:
            value (float): Ambient light from 0 to 100.
        '''
        self.set_ambient_light(value)

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
        number(x_mm, 'x_mm')
        number(y_mm, 'y_mm')
        number(z_mm, 'z_mm', minimum=0)
        return self.ambient_light

    def raycast_surface(
        self,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        dx: float,
        dy: float,
        dz: float,
        max_mm: float,
    ) -> SurfaceHit | None:
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
            SurfaceHit | None: Nearest surface hit, or None when no surface is
                seen.
        '''
        vector((x_mm, y_mm, z_mm), 'origin', 3)
        unit_vector((dx, dy, dz), 'direction')
        number(max_mm, 'max_mm', minimum=0)
        hit = None
        if z_mm > 0 and dz < -1e-9:
            distance = -z_mm / dz
            if distance <= max_mm:
                x = x_mm + distance * dx
                y = y_mm + distance * dy
                hit = SurfaceHit(x, y, 0.0, distance,
                                 self.get_reflection_at(x, y), -dz)
        for obstacle in self.obstacles:
            candidate = obstacle.raycast(
                (x_mm, y_mm, z_mm), (dx, dy, dz), max_mm)
            if candidate is not None and (
                    hit is None or candidate.distance_mm < hit.distance_mm):
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
        number(x_mm, 'x_mm')
        number(y_mm, 'y_mm')
        return min((line.get_reflection_at(x_mm, y_mm, self.reflection)
                    for line in self.lines), default=self.reflection)

    def raycast_distance(
        self,
        x_mm: float,
        y_mm: float,
        heading_deg: float,
        max_mm: float = 2000,
        *,
        z_mm: float = 0.0,
        pitch_deg: float = 0.0,
    ) -> float:
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
        number(heading_deg, 'heading_deg')
        number(pitch_deg, 'pitch_deg')
        yaw = math.radians(heading_deg)
        pitch = math.radians(pitch_deg)
        hit = self.raycast_surface(
            x_mm, y_mm, z_mm,
            math.cos(pitch) * math.cos(yaw),
            math.cos(pitch) * math.sin(yaw), -math.sin(pitch), max_mm,
        )
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

    def save_preview(self, path: str | Path, trace=()) -> bool:
        '''
        Save the map and optional robot path as an SVG preview.

        Arg:
            path (str | Path): Output image path.
            trace (tuple): Robot poses as x, y and heading in degrees.

        Returns:
            bool: True after saving, or False after an OSError. The write
                error is stored in preview_error. Invalid arguments still
                raise TypeError or ValueError.
        '''
        sequence(trace, 'trace')
        trace = tuple(vector(pose, 'trace pose', 3) for pose in trace)
        path = Path(path)
        xmin, ymin, xmax, ymax = self.bounds
        parts = [
            f'<svg xmlns="http://www.w3.org/2000/svg" '
            f'viewBox="{xmin} {-ymax} {xmax - xmin} {ymax - ymin}">',
            f'<rect x="{xmin}" y="{-ymax}" width="{xmax - xmin}" '
            f'height="{ymax - ymin}" fill="white"/>',
        ]
        parts.extend(line.to_svg() for line in self.lines)
        parts.extend(obstacle.to_svg() for obstacle in self.obstacles)
        if trace:
            points = ' '.join(f'{x:.2f},{-y:.2f}' for x, y, _ in trace)
            parts.append(
                f'<polyline points="{points}" fill="none" '
                'stroke="#168dc1" stroke-width="3"/>')
            x, y, yaw = trace[-1]
            ax, ay = x + 70 * \
                math.cos(math.radians(yaw)), y + 70 * \
                math.sin(math.radians(yaw))
            parts.append(
                f'<circle cx="{x}" cy="{-y}" r="65" '
                'fill="#168dc1" fill-opacity="0.25"/>')
            parts.append(
                f'<path d="M{x},{-y} L{ax},{-ay}" '
                'stroke="#168dc1" stroke-width="5"/>')
        parts.append('</svg>')
        self.preview_error = None
        try:
            path.write_text(''.join(parts), encoding='utf-8')
        except OSError as error:
            self.preview_error = error
            return False
        return True
