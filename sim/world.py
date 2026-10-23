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


# Default field. When launching the sim it can be replaced.
current_world: World = FlatWhiteWorld()
