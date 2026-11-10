import math

from sim.world import FlatWhiteWorld, SurfaceHit


class BoxObstacle:
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


class LineFollowWorld(FlatWhiteWorld):
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
        self.obstacles = [BoxObstacle(2300, -100, 2450, 100, 150)]
        self._segments = []
        for (x, y), (end_x, end_y) in zip(self.path, self.path[1:]):
            dx, dy = end_x - x, end_y - y
            self._segments.append((x, y, dx, dy, dx*dx + dy*dy))

    def distance_to_line(self, x_mm: float, y_mm: float) -> float:
        '''
        Return the shortest distance to the line centre.

        Arg:
            x_mm (float): World x coordinate in mm.
            y_mm (float): World y coordinate in mm.

        Returns:
            float: Distance in mm.
        '''
        nearest = math.inf
        for x, y, dx, dy, length_sq in self._segments:
            t = max(0.0, min(1.0, ((x_mm-x)*dx + (y_mm-y)*dy) / length_sq))
            nearest = min(nearest, (x_mm-x-t*dx)**2 + (y_mm-y-t*dy)**2)
        return math.sqrt(nearest)

    def get_reflection_at(self, x_mm: float, y_mm: float) -> int:
        '''
        Return ground reflection across the black line and its soft edge.
        '''
        distance = self.distance_to_line(x_mm, y_mm)
        blend = (distance - self.line_width_mm/2 + self.edge_width_mm/2) / self.edge_width_mm
        blend = max(0.0, min(1.0, blend))
        return round(self.black_reflection + blend*(self.reflection-self.black_reflection))

    def raycast_surface(self, x_mm, y_mm, z_mm, dx, dy, dz, max_mm):
        '''
        Return the nearest ground or obstacle surface along a unit ray.
        '''
        hit = super().raycast_surface(x_mm, y_mm, z_mm, dx, dy, dz, max_mm)
        for obstacle in self.obstacles:
            candidate = obstacle.raycast((x_mm, y_mm, z_mm), (dx, dy, dz), max_mm)
            if candidate is not None and (hit is None or candidate.distance_mm < hit.distance_mm):
                hit = candidate
        return hit
