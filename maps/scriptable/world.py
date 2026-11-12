from world import World
from sim.validation import number


class ScriptableWorld(World):
    '''
    Provide fixed sensor readings controlled by the test or user.

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
        _reflection (float): Scripted ground reflection from 0 to 100.
        _distance (float): Scripted non-negative ultrasonic range in mm.
    '''

    def __init__(self, *, ambient_light: float = 5.0) -> None:
        '''
        Create a field with configurable fixed sensor readings.

        Arg:
            ambient_light (float): Ambient light from 0 to 100.
        '''
        super().__init__(ambient_light)
        self._reflection: float = 50
        self._distance: float = 2000.0

    def set_reflection(self, value: int) -> None:
        '''
        Set the scripted ground reflection.

        Arg:
            value (float): Reflection from 0 to 100.
        '''
        number(value, 'reflection', minimum=0, maximum=100)
        self._reflection = value

    def set_distance(self, value_mm: float) -> None:
        '''
        Set the scripted ultrasonic range.

        Arg:
            value_mm (float): Finite non-negative distance in mm.
        '''
        number(value_mm, 'value_mm', minimum=0)
        self._distance = value_mm

    def get_reflection_at(self, x_mm: float, y_mm: float) -> int:
        '''
        Return the scripted reflection regardless of position.

        Arg:
            x_mm (float): Finite world x coordinate in mm.
            y_mm (float): Finite world y coordinate in mm.

        Returns:
            float: Scripted reflection from 0 to 100.
        '''
        number(x_mm, 'x_mm')
        number(y_mm, 'y_mm')
        return self._reflection

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
        # This test world deliberately returns a scripted range without
        # geometry.
        '''
        Return the scripted distance capped by the requested range.

        Validate the pose arguments without using geometric intersections.

        Arg:
            x_mm (float): World x coordinate in mm.
            y_mm (float): World y coordinate in mm.
            heading_deg (float): Ray heading in degrees.
            max_mm (float): Non-negative maximum distance in mm.
            z_mm (float): Non-negative ray height in mm.
            pitch_deg (float): Downward pitch in degrees.

        Returns:
            float: Scripted distance or max_mm, whichever is smaller.
        '''
        for name, value in zip(('x_mm', 'y_mm', 'heading_deg', 'pitch_deg'),
                               (x_mm, y_mm, heading_deg, pitch_deg)):
            number(value, name)
        number(z_mm, 'z_mm', minimum=0)
        number(max_mm, 'max_mm', minimum=0)
        return min(self._distance, max_mm)
