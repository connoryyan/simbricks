from pathlib import Path
import runpy

import pytest

from render import video
from sim.clock import clock


@pytest.mark.parametrize('filename', ['friction_test.py', 'imu_test.py', 'motor_test.py', 'line_follow/line_follow.py'])
@pytest.mark.parametrize('entrypoint', ['__main__', 'example'])
def test_example_entrypoints(filename, entrypoint, monkeypatch, capsys):
    saved = []

    def save(path, field, frames):
        saved.append((path, frames))

    monkeypatch.setattr(video, 'save_video', save)
    # Each script adjusts sys.path for direct execution.
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


def test_line_follow_main_without_frames(monkeypatch, capsys):
    from examples.line_follow import line_follow

    monkeypatch.setattr(line_follow, 'run', lambda recording: (None, None, [], 0))
    line_follow.main()
    assert 'No recorded frames' in capsys.readouterr().err


def test_save_map_wrapper(tmp_path):
    from examples.line_follow.line_follow import save_map
    from maps.line_follow.world import LineFollowWorld

    path = tmp_path / 'map.svg'
    save_map(path, LineFollowWorld(), [(0, 0, 0)])
    assert path.read_text().endswith('</svg>')
