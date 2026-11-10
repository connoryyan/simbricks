import contextlib
import io
import math
from pathlib import Path
import subprocess
import sys
import unittest
import xml.etree.ElementTree as ET

from examples.line_follow.line_follow import run, STOP_DISTANCE_MM, SPEED_MM_S
import world
from sim.clock import clock
from maps.line_follow.world import LineFollowWorld


class LineFollowTests(unittest.TestCase):
    def test_direct_script_runs_outside_repository_and_saves_svg(self):
        script = Path(__file__).resolve().parents[1] / 'examples/line_follow/line_follow.py'
        result = subprocess.run(
            [sys.executable, '-B', str(script)],
            cwd=script.parent, capture_output=True, text=True, timeout=30,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('Stopped for obstacle', result.stdout)
        root = ET.parse(script.with_name('map_visual.svg')).getroot()
        self.assertEqual(root.tag, '{http://www.w3.org/2000/svg}svg')
        self.assertEqual(len(root.findall('{http://www.w3.org/2000/svg}polyline')), 2)

    def setUp(self):
        self.previous_models = clock._steppables.copy()
        self.previous_time = clock.time_ms
        self.previous_world = world.current_world
        clock._steppables.clear()
        clock.time_ms = 0

    def tearDown(self):
        clock._steppables.clear()
        clock._steppables.update(self.previous_models)
        clock.time_ms = self.previous_time
        world.current_world = self.previous_world

    def test_line_width_and_winding_path(self):
        field = LineFollowWorld()
        self.assertEqual(field.line_width_mm, 20)
        self.assertEqual(field.get_reflection_at(100, 0), 10)
        self.assertEqual(field.get_reflection_at(100, 8), 10)
        self.assertEqual(field.get_reflection_at(100, 12), 85)
        self.assertEqual(field.get_reflection_at(100, 10),
                         field.get_reflection_at(100, -10))
        self.assertAlmostEqual(field.distance_to_line(100, 10), 10)
        for x, y in field.path:
            self.assertEqual(field.get_reflection_at(x, y), 10)
        self.assertGreater(max(y for _, y in field.path), 200)
        self.assertLess(min(y for _, y in field.path), -100)

    def test_obstacle_is_at_line_end_and_has_150_mm_height(self):
        field = LineFollowWorld()
        obstacle = field.obstacles[0]
        self.assertEqual(field.path[-1], (obstacle.xmin, 0))
        self.assertEqual(obstacle.height_mm, 150)
        self.assertAlmostEqual(field.raycast_distance(2200, 0, 0, z_mm=60), 100)
        self.assertEqual(field.raycast_distance(2200, 0, 0, z_mm=151), 2000)
        self.assertEqual(field.raycast_distance(2200, 0, 180, z_mm=60), 2000)
        self.assertEqual(field.raycast_distance(2200, 101, 0, z_mm=60), 2000)

    def test_rays_hit_box_top_or_side_before_ground(self):
        field = LineFollowWorld()
        hit = field.raycast_surface(2350, 0, 200, 0, 0, -1, 2000)
        self.assertEqual(hit.z_mm, 150)
        self.assertEqual(hit.distance_mm, 50)
        self.assertEqual(hit.reflection, field.obstacles[0].reflection)
        self.assertAlmostEqual(field.raycast_distance(
            2200, 0, 0, z_mm=200, pitch_deg=45), math.sqrt(20000))
        self.assertEqual(field.raycast_distance(2200, 0, 0, max_mm=50, z_mm=60), 50)

    def test_example_follows_bends_and_stops_before_obstacle(self):
        with contextlib.redirect_stdout(io.StringIO()):
            field, db, trace, distance = run()
        self.assertGreater(max(y for _, y, _ in trace), 200)
        self.assertLess(min(y for _, y, _ in trace), -100)
        self.assertGreater(db.pose()[0], 2100)
        self.assertGreater(distance, STOP_DISTANCE_MM - SPEED_MM_S * 0.2)
        self.assertLessEqual(distance, STOP_DISTANCE_MM)
        self.assertAlmostEqual(db.state()[1], 0, delta=0.01)
        # Check a conservative 65 mm body radius against the obstacle footprint.
        box = field.obstacles[0]
        max_error = 0
        for x, y, _ in trace:
            dx = max(box.xmin-x, 0, x-box.xmax)
            dy = max(box.ymin-y, 0, y-box.ymax)
            self.assertGreater(math.hypot(dx, dy), 65)
            max_error = max(max_error, field.distance_to_line(x, y))
        self.assertLess(max_error, 20)


if __name__ == '__main__':
    unittest.main()
