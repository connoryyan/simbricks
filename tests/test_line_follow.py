from tests.initialization_helpers import initialize_existing
import contextlib
import io
import math
from pathlib import Path
import subprocess
import sys
import shutil
import importlib.util
import pytest
from unittest.mock import patch
from examples.line_follow import line_follow as example
import xml.etree.ElementTree as ET
from examples.line_follow.line_follow import run, STOP_DISTANCE_MM, SPEED_MM_S
import world
from sim.clock import clock
from maps.line_follow.world import LineFollowWorld

class TestLineFollow:

    @pytest.mark.skipif(not shutil.which('ffmpeg') or not importlib.util.find_spec('PIL'), reason='Video export requires FFmpeg and Pillow')
    def test_direct_script_runs_outside_repository_and_saves_video(self):
        script = Path(__file__).resolve().parents[1] / 'examples/line_follow/line_follow.py'
        result = subprocess.run([sys.executable, '-B', str(script)], cwd=script.parent, capture_output=True, text=True, timeout=60)
        assert result.returncode == 0
        assert 'Stopped for obstacle' in result.stdout
        video = script.with_name('line_follow.mp4')
        assert video.read_bytes()[4:8] == b'ftyp'
        decoded = subprocess.run([shutil.which('ffmpeg'), '-v', 'error', '-i', str(video), '-frames:v', '1', '-f', 'image2pipe', '-vcodec', 'png', 'pipe:1'], capture_output=True, timeout=30)
        assert decoded.returncode == 0
        assert decoded.stdout.startswith(b'\x89PNG')

    def test_recording_uses_simulation_time_and_is_detached(self):
        frames = []
        with contextlib.redirect_stdout(io.StringIO()):
            field, db, trace, distance = run(frames)
        assert frames[0][0] == 0
        assert len(frames) > 1
        assert frames[-1][0] == pytest.approx(clock.time_ms / 1000, abs=0.04, rel=0)
        for previous, current in zip(frames, frames[1:]):
            assert current[0] - previous[0] == pytest.approx(1 / 30, abs=0.006, rel=0)
        assert all((len(frame[2]) == 2 for frame in frames))
        assert not any((type(item).__name__ == 'PoseRecorder' for item in clock.steppables))

    def test_timeout_is_caught_and_recorded_frames_are_saved(self):
        errors = io.StringIO()
        with patch.object(example, 'TIMEOUT_MS', 60), patch.object(example, 'save_video') as save, contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(errors):
            example.main()
        assert 'Simulation timeout:' in errors.getvalue()
        save.assert_called_once()
        path, field, frames = save.call_args.args
        assert path.name == 'line_follow.mp4'
        assert field is world.current_world
        assert len(frames) > 1
        assert frames[-1][0] > 0.5
        initialize_existing()
        assert example.db.state()[1] == pytest.approx(0, abs=0.01, rel=0)
        assert not any((type(item).__name__ == 'PoseRecorder' for item in clock.steppables))

    def setup_method(self):
        self.previous_models = {priority: group.copy() for priority, group in clock._steppables.items()}
        self.previous_time = clock.time_ms
        self.previous_world = world.current_world
        clock._steppables.clear()
        clock.time_ms = 0
        self.previous_initialization = clock._initialization
        left = example.Motor(example.Port.A, example.Direction.COUNTERCLOCKWISE)
        right = example.Motor(example.Port.B)
        robot = example.DriveBase(left, right, 56, 112)
        self.fixture = patch.multiple(example, left=left, right=right, db=robot, color=example.ColorSensor(example.Port.C), ultrasonic=example.UltrasonicSensor(example.Port.D), field=None)
        self.fixture.start()
        example.field = example.sim.initialize(example.init)

    def teardown_method(self):
        self.fixture.stop()
        clock._steppables.clear()
        clock._steppables.update(self.previous_models)
        clock.time_ms = self.previous_time
        world.current_world = self.previous_world
        clock._initialization = self.previous_initialization

    def test_line_width_and_winding_path(self):
        field = LineFollowWorld()
        assert field.line_width_mm == 20
        assert field.get_reflection_at(100, 0) == 10
        assert field.get_reflection_at(100, 8) == 10
        assert field.get_reflection_at(100, 12) == 85
        assert field.get_reflection_at(100, 10) == field.get_reflection_at(100, -10)
        assert field.distance_to_line(100, 10) == pytest.approx(10, abs=5e-08, rel=0)
        for x, y in field.path:
            assert field.get_reflection_at(x, y) == 10
        assert max((y for _, y in field.path)) > 200
        assert min((y for _, y in field.path)) < -100

    def test_obstacle_is_at_line_end_and_has_150_mm_height(self):
        field = LineFollowWorld()
        obstacle = field.obstacles[0]
        assert field.path[-1] == (obstacle.xmin, 0)
        assert obstacle.height_mm == 150
        assert field.raycast_distance(2200, 0, 0, z_mm=60) == pytest.approx(100, abs=5e-08, rel=0)
        assert field.raycast_distance(2200, 0, 0, z_mm=151) == 2000
        assert field.raycast_distance(2200, 0, 180, z_mm=60) == 2000
        assert field.raycast_distance(2200, 101, 0, z_mm=60) == 2000

    def test_rays_hit_box_top_or_side_before_ground(self):
        field = LineFollowWorld()
        hit = field.raycast_surface(2350, 0, 200, 0, 0, -1, 2000)
        assert hit.z_mm == 150
        assert hit.distance_mm == 50
        assert hit.reflection == field.obstacles[0].reflection
        assert field.raycast_distance(2200, 0, 0, z_mm=200, pitch_deg=45) == pytest.approx(math.sqrt(20000), abs=5e-08, rel=0)
        assert field.raycast_distance(2200, 0, 0, max_mm=50, z_mm=60) == 50

    def test_example_follows_bends_and_stops_before_obstacle(self):
        with contextlib.redirect_stdout(io.StringIO()):
            field, db, trace, distance = run()
        assert max((y for _, y, _ in trace)) > 200
        assert min((y for _, y, _ in trace)) < -100
        assert db.pose()[0] > 2100
        assert distance > STOP_DISTANCE_MM - SPEED_MM_S * 0.2
        assert distance <= STOP_DISTANCE_MM
        initialize_existing()
        assert db.state()[1] == pytest.approx(0, abs=0.01, rel=0)
        box = field.obstacles[0]
        max_error = 0
        for x, y, _ in trace:
            dx = max(box.xmin - x, 0, x - box.xmax)
            dy = max(box.ymin - y, 0, y - box.ymax)
            assert math.hypot(dx, dy) > 65
            max_error = max(max_error, field.distance_to_line(x, y))
        assert max_error < 20
