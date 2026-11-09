from abc import ABC, abstractmethod


class World(ABC):
    @abstractmethod
    def get_reflection_at(self, x_mm: float, y_mm: float) -> int:
        '''
        Reflection at a certain point on the gaming field

        Args:
            x_mm (float): x coordinate in mm.
            y_mm (float): y coordinate in mm.

        Returns:
            int: Reflection at (x, y)
        '''
        raise NotImplementedError

    @abstractmethod
    def raycast_distance(self, x_mm: float, y_mm: float, heading_deg: float, max_mm: float = 2000) -> float:
        '''
        Distance between point (x, y) and the nearest object at the direction of heading_deg

        Args:
            x_mm (float): x coordinate in mm.
            y_mm (float): y coordinate in mm.
            heading_deg (int): direction where the object is expected to be detected.
            max_mm (float): When no object is detected, return this value.
        '''
        raise NotImplementedError


class FlatWhiteWorld(World):
    '''
    A placeholder gaming field with white surface and no obstacles.
    '''

    def __init__(self, reflection: int = 85):
        self.reflection = reflection

    def get_reflection_at(self, x_mm: float, y_mm: float) -> int:
        return self.reflection

    def raycast_distance(self, x_mm: float, y_mm: float, heading_deg: float, max_mm: float = 2000) -> float:
        return max_mm


class ScriptableWorld(World):
    '''
    Tunable gaming field mainly for testing.
    '''

    def __init__(self):
        self._reflection = 50
        self._distance = 2000.0

    def set_reflection(self, value: int) -> None:
        self._reflection = value

    def set_distance(self, value_mm: float) -> None:
        self._distance = value_mm

    def get_reflection_at(self, x_mm: float, y_mm: float) -> int:
        return self._reflection

    def raycast_distance(self, x_mm: float, y_mm: float, heading_deg: float, max_mm: float = 2000) -> float:
        return min(self._distance, max_mm)


class LineWorld(FlatWhiteWorld):
    '''
    A straight black line along the x axis.
    '''

    def __init__(self, line_width_mm: float = 20.0):
        super().__init__(reflection=85)
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

