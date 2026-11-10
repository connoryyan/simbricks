from pybricks.tools import StopWatch, wait


def test_stopwatch_pause_resume_and_reset(initialized):
    timer = StopWatch()
    wait(10)
    assert timer.time() == 10
    timer.resume()
    timer.pause()
    wait(20)
    timer.pause()
    assert timer.time() == 10
    timer.resume()
    wait(5)
    assert timer.time() == 15
    timer.reset()
    assert timer.time() == 0
    timer.pause()
    wait(10)
    timer.reset()
    wait(3)
    assert timer.time() == 3
