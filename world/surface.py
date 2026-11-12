from sim.validation import number


class SurfaceHit:
    '''
    Store one surface hit from a ray.

    Attributes:
        x_mm (float): World x coordinate in mm.
        y_mm (float): World y coordinate in mm.
        z_mm (float): World height in mm.
        distance_mm (float): Non-negative distance from the ray origin to a
            surface hit,
            in mm.
        reflection (float): Reflection from 0 to 100.
        incidence_cosine (float): Cosine of the ray incidence angle, from 0 to
            1.
    '''

    def __init__(
        self,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        distance_mm: float,
        reflection: float,
        incidence_cosine: float,
    ) -> None:
        '''
        Create a validated surface hit.

        Arg:
            x_mm (float): World x coordinate in mm.
            y_mm (float): World y coordinate in mm.
            z_mm (float): World height in mm.
            distance_mm (float): Non-negative ray distance in mm.
            reflection (float): Surface reflection from 0 to 100.
            incidence_cosine (float): Incidence cosine from 0 to 1.
        '''
        number(x_mm, 'x_mm')
        number(y_mm, 'y_mm')
        number(z_mm, 'z_mm')
        number(distance_mm, 'distance_mm', minimum=0)
        number(reflection, 'reflection', minimum=0, maximum=100)
        number(incidence_cosine, 'incidence_cosine', minimum=0, maximum=1)

        self.x_mm: float = x_mm
        self.y_mm: float = y_mm
        self.z_mm: float = z_mm
        self.distance_mm: float = distance_mm
        self.reflection: float = reflection
        self.incidence_cosine: float = incidence_cosine
