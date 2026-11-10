import io

import pytest

from maps.line_follow.world import LineFollowWorld
from render import video


class InputStream:
    def __init__(self, fail=False):
        self.fail = fail
        self.frames = 0
        self.closed = False

    def write(self, data):
        if self.fail:
            raise BrokenPipeError('encoder stopped')
        assert len(data) == 1280 * 480 * 3
        self.frames += 1

    def close(self):
        self.closed = True


class Encoder:
    def __init__(self, returncode=0, fail=False):
        self.returncode = returncode
        self.stdin = InputStream(fail)
        self.stderr = io.BytesIO(b'encoder error')
        self.finished = False
        self.killed = False

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


def test_video_rejects_missing_encoder_frames_and_wrong_fps(monkeypatch, tmp_path):
    field = LineFollowWorld()
    path = tmp_path / 'video.mp4'
    monkeypatch.setattr(video.shutil, 'which', lambda name: None)
    with pytest.raises(RuntimeError, match='FFmpeg'):
        video.save_video(path, field, [])
    monkeypatch.setattr(video.shutil, 'which', lambda name: 'ffmpeg')
    with pytest.raises(ValueError, match='No recorded'):
        video.save_video(path, field, [])
    with pytest.raises(ValueError, match='30 FPS'):
        video.save_video(path, field, [(0, (0, 0, 0), ())], fps=60)


def test_video_reports_encoder_failure(monkeypatch, tmp_path):
    encoder = Encoder(returncode=1)
    monkeypatch.setattr(video.shutil, 'which', lambda name: 'ffmpeg')
    monkeypatch.setattr(video.subprocess, 'Popen', lambda *args, **kwargs: encoder)
    with pytest.raises(RuntimeError, match='encoder error'):
        video.save_video(tmp_path / 'failed.mp4', LineFollowWorld(), [(0, (0, 0, 0), ())])
    assert encoder.stderr.closed


def test_video_kills_encoder_when_pipe_breaks(monkeypatch, tmp_path):
    encoder = Encoder(fail=True)
    monkeypatch.setattr(video.shutil, 'which', lambda name: 'ffmpeg')
    monkeypatch.setattr(video.subprocess, 'Popen', lambda *args, **kwargs: encoder)
    with pytest.raises(BrokenPipeError):
        video.save_video(tmp_path / 'failed.mp4', LineFollowWorld(), [(0, (0, 0, 0), ())])
    assert encoder.killed
    assert encoder.finished
    assert encoder.stderr.closed
