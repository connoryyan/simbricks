import pytest
import xml.etree.ElementTree as ET
from pathlib import Path
import world
from world import World, Line, Obstacle, Polyline, BoxObstacle
from maps.flat_white.world import FlatWhiteWorld
from maps.straight_line.world import LineWorld
from maps.scriptable.world import ScriptableWorld
from maps.line_follow.world import LineFollowWorld

class CustomWorld(World):

    def __init__(self):
        super().__init__(ambient_light=7)
        self.add_line(Polyline([(0, 0), (500, 0)], width_mm=20))
        self.add_line(Polyline([(0, 100), (500, 100)], width_mm=20))
        self.add_obstacle(BoxObstacle(200, -50, 250, 50, 150))
        self.add_obstacle(BoxObstacle(100, -50, 150, 50, 150))

class TestWorldPackage:

    def test_base_classes_are_abstract(self):
        for cls in (World, Line, Obstacle):
            with pytest.raises(TypeError):
                cls()

    def test_custom_map_only_needs_geometry_configuration(self):
        field = CustomWorld()
        assert field.get_reflection_at(50, 0) == 10
        assert field.get_reflection_at(50, 100) == 10
        assert field.get_reflection_at(50, 50) == 85
        assert field.get_ambient_at(0, 0, 20) == 7
        assert field.raycast_distance(0, 0, 0, z_mm=60) == 100
        assert field.raycast_distance(0, 0, 0, z_mm=160) == 2000

    def test_invalid_geometry_is_rejected(self):
        field = CustomWorld()
        with pytest.raises(TypeError):
            field.add_line(object())
        with pytest.raises(TypeError):
            field.add_obstacle(object())
        for points in ([], [(0, 0)], [(0, 0), (0, 0)], [(0, 0), (1, float('nan'))]):
            with pytest.raises(ValueError):
                Polyline(points)
        with pytest.raises(ValueError):
            BoxObstacle(10, 0, 0, 100, 150)
        with pytest.raises(ValueError):
            BoxObstacle(0, 0, 100, 100, -1)

    def test_default_map_uses_one_active_reference(self):
        previous = world.current_world
        try:
            world.current_world = None
            assert isinstance(world.get_current_world(), FlatWhiteWorld)
            field = CustomWorld()
            world.current_world = field
            assert world.get_current_world() is field
        finally:
            world.current_world = previous

    def test_each_map_has_a_subclass_and_valid_preview(self):
        root = Path(__file__).resolve().parents[1] / 'maps'
        namespace = '{http://www.w3.org/2000/svg}'
        for name, cls in (('flat_white', FlatWhiteWorld), ('straight_line', LineWorld), ('scriptable', ScriptableWorld), ('line_follow', LineFollowWorld)):
            assert issubclass(cls, World)
            assert (root / name / 'world.py').is_file()
            image = ET.parse(root / name / 'preview.svg').getroot()
            assert image.tag == namespace + 'svg'
            field = cls()
            assert len(image.findall(namespace + 'polyline')) == len(field.lines)
            assert len(image.findall(namespace + 'rect')) == 1 + len(field.obstacles)
