from world import World


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


