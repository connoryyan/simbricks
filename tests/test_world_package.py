import unittest
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


class WorldPackageTests(unittest.TestCase):
    def test_base_classes_are_abstract(self):
        for cls in (World, Line, Obstacle):
            with self.subTest(cls=cls):
                with self.assertRaises(TypeError):
                    cls()

    def test_custom_map_only_needs_geometry_configuration(self):
        field = CustomWorld()
        self.assertEqual(field.get_reflection_at(50, 0), 10)
        self.assertEqual(field.get_reflection_at(50, 100), 10)
        self.assertEqual(field.get_reflection_at(50, 50), 85)
        self.assertEqual(field.get_ambient_at(0, 0, 20), 7)
        self.assertEqual(field.raycast_distance(0, 0, 0, z_mm=60), 100)
        self.assertEqual(field.raycast_distance(0, 0, 0, z_mm=160), 2000)

    def test_invalid_geometry_is_rejected(self):
        field = CustomWorld()
        with self.assertRaises(TypeError):
            field.add_line(object())
        with self.assertRaises(TypeError):
            field.add_obstacle(object())
        for points in ([], [(0, 0)], [(0, 0), (0, 0)], [(0, 0), (1, float('nan'))]):
            with self.assertRaises(ValueError):
                Polyline(points)
        with self.assertRaises(ValueError):
            BoxObstacle(10, 0, 0, 100, 150)
        with self.assertRaises(ValueError):
            BoxObstacle(0, 0, 100, 100, -1)

    def test_default_map_uses_one_active_reference(self):
        previous = world.current_world
        try:
            world.current_world = None
            self.assertIsInstance(world.get_current_world(), FlatWhiteWorld)
            field = CustomWorld()
            world.current_world = field
            self.assertIs(world.get_current_world(), field)
        finally:
            world.current_world = previous

    def test_each_map_has_a_subclass_and_valid_preview(self):
        root = Path(__file__).resolve().parents[1] / 'maps'
        namespace = '{http://www.w3.org/2000/svg}'
        for name, cls in (('flat_white', FlatWhiteWorld), ('straight_line', LineWorld),
                          ('scriptable', ScriptableWorld), ('line_follow', LineFollowWorld)):
            with self.subTest(name=name):
                self.assertTrue(issubclass(cls, World))
                self.assertTrue((root / name / 'world.py').is_file())
                image = ET.parse(root / name / 'preview.svg').getroot()
                self.assertEqual(image.tag, namespace + 'svg')
                field = cls()
                self.assertEqual(len(image.findall(namespace + 'polyline')), len(field.lines))
                self.assertEqual(len(image.findall(namespace + 'rect')), 1 + len(field.obstacles))


if __name__ == '__main__':
    unittest.main()
