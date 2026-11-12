from pathlib import Path
import runpy

from render import video
from sim.clock import clock


def _check_example_entrypoints(filename, entrypoint, monkeypatch, capsys):
    saved = []

    def save(path, field, frames):
        saved.append((path, frames))
    monkeypatch.setattr(video, 'save_video', save)
    import sys
    monkeypatch.setattr(sys, 'path', sys.path.copy())
    path = Path(__file__).resolve().parents[1] / 'examples' / filename
    namespace = runpy.run_path(str(path), run_name=entrypoint)
    if filename == 'friction_test.py' and entrypoint == 'example':
        assert clock._initialization is None
    else:
        assert clock._initialization is True
    if filename == 'line_follow/line_follow.py' and entrypoint == '__main__':
        assert saved[0][0].suffix == '.mp4'
        assert len(saved[0][1]) > 1
        assert 'Stopped for obstacle' in capsys.readouterr().out
    if filename == 'friction_test.py' and entrypoint == '__main__':
        assert namespace['robot'].pose()[0] > 0


def test_friction_script(monkeypatch, capsys):
    _check_example_entrypoints(
        'friction_test.py', '__main__', monkeypatch, capsys)


def test_import_friction_example(monkeypatch, capsys):
    _check_example_entrypoints(
        'friction_test.py', 'example', monkeypatch, capsys)


def test_imu_script(monkeypatch, capsys):
    _check_example_entrypoints('imu_test.py', '__main__', monkeypatch, capsys)


def test_import_imu_example(monkeypatch, capsys):
    _check_example_entrypoints('imu_test.py', 'example', monkeypatch, capsys)


def test_motor_script(monkeypatch, capsys):
    _check_example_entrypoints(
        'motor_test.py', '__main__', monkeypatch, capsys)


def test_import_motor_example(monkeypatch, capsys):
    _check_example_entrypoints('motor_test.py', 'example', monkeypatch, capsys)


def test_line_follow_script(monkeypatch, capsys):
    _check_example_entrypoints(
        'line_follow/line_follow.py', '__main__', monkeypatch, capsys)


def test_import_line_follow_example(monkeypatch, capsys):
    _check_example_entrypoints(
        'line_follow/line_follow.py', 'example', monkeypatch, capsys)


def test_line_follow_main_without_frames(monkeypatch, capsys):
    from examples.line_follow import line_follow
    monkeypatch.setattr(line_follow, 'run',
                        lambda recording: (None, None, [], 0))
    line_follow.main()
    assert 'No recorded frames' in capsys.readouterr().err


def test_save_map_wrapper(tmp_path):
    from examples.line_follow.line_follow import save_map
    from maps.line_follow.world import LineFollowWorld
    path = tmp_path / 'map.svg'
    save_map(path, LineFollowWorld(), [(0, 0, 0)])
    assert path.read_text().endswith('</svg>')
