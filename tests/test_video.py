import io

import pytest

from maps.flat_white.world import FlatWhiteWorld
from maps.line_follow.world import LineFollowWorld
from pybricks.parameters import Port
from pybricks.pupdevices import ForceSensor
from pybricks.tools import wait
from render import video
from render.video import PoseRecorder, save_video
from sim.clock import clock
from world import Line, Obstacle


def test_recorder_argument_validation(robot):
    frames = []
    for args in (
        (None, (), frames, clock),
        (robot, [None], frames, clock),
        (robot, (), (), clock),
        (robot, (), frames, None),
    ):
        with pytest.raises(TypeError):
            PoseRecorder(*args)
    assert frames == []
    recorder = PoseRecorder(robot, (), frames, clock)
    with pytest.raises(ValueError):
        recorder.step(0)
    assert len(frames) == 1


def test_recorder_samples_simulated_time_until_detached(robot):
    frames = []
    sensor = ForceSensor(Port.A)
    robot.model.mount(sensor.model, forward_mm=75, above_mm=12)
    recorder = PoseRecorder(robot, (sensor,), frames, clock)
    clock.add_steppable(recorder)
    robot.drive(100, 0)
    wait(100)
    assert len(frames) == 4
    assert frames[0][0] == 0
    assert frames[-1][0] == pytest.approx(0.1)
    assert frames[-1][1] == robot.pose()
    assert frames[-1][2] == (sensor.pose(),)
    assert recorder.next_frame == 4
    clock.remove_steppable(recorder)
    wait(50)
    assert len(frames) == 4


def test_video_rejects_bad_arguments_before_encoding(monkeypatch):
    import render.video as video

    def start(*args, **kwargs):
        pytest.fail('Invalid input must not start an encoder')
    monkeypatch.setattr(video.subprocess, 'Popen', start)
    field = FlatWhiteWorld()
    valid = [(0, (0, 0, 0), ())]
    for operation in (
        lambda: save_video(None, field, valid),
        lambda: save_video('ignored.mp4', None, valid),
        lambda: save_video('ignored.mp4', field, None),
        lambda: save_video('ignored.mp4', field, valid, fps=True),
        lambda: save_video('ignored.mp4', field, valid, fps=30.0),
        lambda: save_video('ignored.mp4', field, [(0, (0, 0, 0), None)]),
        lambda: save_video('ignored.mp4', field, [('0', (0, 0, 0), ())]),
    ):
        with pytest.raises(TypeError):
            operation()
    for frames in (
        [(0, (0, 0), ())],
        [(0, (0, 0, 0))],
        [(-1, (0, 0, 0), ())],
        [(0, (0, 0, 0), [(0, 0, -1, 0, 0, 0)])],
        [(1, (0, 0, 0), ()), (0, (0, 0, 0), ())],
    ):
        with pytest.raises(ValueError):
            save_video('ignored.mp4', field, frames)


def test_video_rejects_unsupported_geometry():

    class CustomLine(Line):

        '''
        Provide a non-polyline line for video format validation tests.

        Attributes:
            width_mm (float): Total line width in mm.
            reflection (float): Reflection from 0 to 100.
            edge_width_mm (float): Width of the reflection blend at the line
                edge, in mm.
        '''

        def distance_to(self, x, y):
            return 0

        def to_svg(self):
            return ''

    class CustomObstacle(Obstacle):

        '''
        Provide a non-box obstacle for video format validation tests.
        '''

        def raycast(self, origin, direction, max_mm):
            return None

        def to_svg(self):
            return ''
    field = FlatWhiteWorld()
    field.add_line(CustomLine())
    with pytest.raises(TypeError, match='Polyline'):
        save_video('ignored.mp4', field, [(0, (0, 0, 0), ())])
    field.lines.clear()
    field.add_obstacle(CustomObstacle())
    with pytest.raises(TypeError, match='BoxObstacle'):
        save_video('ignored.mp4', field, [(0, (0, 0, 0), ())])


class InputStream:

    '''
    Count frame writes in encoder tests.

    Attributes:
        fail (bool): Whether writes raise a simulated broken-pipe error.
        frames (int): Number of complete video frames written.
        closed (bool): Whether close() has been called.
    '''

    def __init__(self, fail=False):
        self.fail: bool = fail
        self.frames: int = 0
        self.closed: bool = False

    def write(self, data):
        if self.fail:
            raise BrokenPipeError('encoder stopped')
        assert len(data) == 1280 * 480 * 3
        self.frames += 1

    def close(self):
        self.closed = True


class Encoder:

    '''
    Provide a fake FFmpeg process for video export tests.

    Attributes:
        returncode (int): Exit code returned by wait().
        stdin (InputStream): Fake input stream receiving rendered video frames.
        stderr (BytesIO): Byte stream containing a simulated encoder error.
        finished (bool): Whether wait() has marked the fake process as
            finished.
        killed (bool): Whether kill() has been called.
    '''

    def __init__(self, returncode=0, fail=False):
        self.returncode: int = returncode
        self.stdin: InputStream = InputStream(fail)
        self.stderr: io.BytesIO = io.BytesIO(b'encoder error')
        self.finished: bool = False
        self.killed: bool = False

    def wait(self):
        self.finished = True
        return self.returncode

    def poll(self):
        return self.returncode if self.finished else None

    def kill(self):
        self.killed = True


def test_video_draws_frames_and_holds_final_frame(monkeypatch, tmp_path):
    encoder = Encoder()
    commands = []

    def start(command, **kwargs):
        commands.append(command)
        return encoder
    monkeypatch.setattr(video.shutil, 'which', lambda name: 'ffmpeg')
    monkeypatch.setattr(video.subprocess, 'Popen', start)
    frames = [
        (0, (0, 0, 0), ()),
        (0.033, (10, 0, 30), ((50, 0, 10, 0, 90, 0),)),
        (0.067, (20, 0, 0), ((60, 10, 10, 0, 90, 0), (80, 0, 60, 0, 0, 0))),
        (0.1, (30, 0, 0), ((70, 10, 10, 0, 90, 0), (90, 0, 60, 180, 0, 0))),
    ]
    path = tmp_path / 'video.mp4'
    video.save_video(path, LineFollowWorld(), frames)
    assert commands[0][-1] == str(path)
    assert 'libx264' in commands[0]
    assert encoder.stdin.frames == len(frames) + 30
    assert encoder.stdin.closed
    assert encoder.stderr.closed
    assert not encoder.killed


def test_video_rejects_missing_encoder_frames_and_wrong_fps(
    monkeypatch,
    tmp_path,
):
    field = LineFollowWorld()
    path = tmp_path / 'video.mp4'
    monkeypatch.setattr(video.shutil, 'which', lambda name: None)
    with pytest.raises(RuntimeError, match='FFmpeg'):
        video.save_video(path, field, [(0, (0, 0, 0), ())])
    monkeypatch.setattr(video.shutil, 'which', lambda name: 'ffmpeg')
    with pytest.raises(ValueError, match='No recorded'):
        video.save_video(path, field, [])
    with pytest.raises(ValueError, match='30 FPS'):
        video.save_video(path, field, [(0, (0, 0, 0), ())], fps=60)


def test_video_reports_encoder_failure(monkeypatch, tmp_path):
    encoder = Encoder(returncode=1)
    monkeypatch.setattr(video.shutil, 'which', lambda name: 'ffmpeg')
    monkeypatch.setattr(video.subprocess, 'Popen',
                        lambda *args, **kwargs: encoder)
    with pytest.raises(RuntimeError, match='encoder error'):
        video.save_video(tmp_path / 'failed.mp4',
                         LineFollowWorld(), [(0, (0, 0, 0), ())])
    assert encoder.stderr.closed


def test_video_kills_encoder_when_pipe_breaks(monkeypatch, tmp_path):
    encoder = Encoder(fail=True)
    monkeypatch.setattr(video.shutil, 'which', lambda name: 'ffmpeg')
    monkeypatch.setattr(video.subprocess, 'Popen',
                        lambda *args, **kwargs: encoder)
    with pytest.raises(BrokenPipeError):
        video.save_video(tmp_path / 'failed.mp4',
                         LineFollowWorld(), [(0, (0, 0, 0), ())])
    assert encoder.killed
    assert encoder.finished
    assert encoder.stderr.closed
