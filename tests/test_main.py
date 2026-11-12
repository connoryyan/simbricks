from pathlib import Path
import runpy

import pytest

import main
import world
from pybricks.tools import wait
from sim.clock import clock


def test_integration_checks_preserve_existing_simulation(
    hub_robot, flat_field, capsys,
):
    wait(20)
    previous_devices = set(clock.steppables)
    previous_time = clock.time_ms
    previous_init = clock._initialization
    main.main()
    assert 'All integration checks passed.' in capsys.readouterr().out
    assert set(clock.steppables) == previous_devices
    assert clock.time_ms == previous_time
    assert clock._initialization == previous_init
    assert world.current_world is flat_field


def test_failed_integration_check_propagates_and_restores_state(monkeypatch):
    def fail(robot, color, ultrasonic):
        wait(10)
        raise AssertionError('Motion integration failed')

    monkeypatch.setattr(main, 'check_motion_and_sensors', fail)
    with pytest.raises(AssertionError, match='Motion integration failed'):
        main.main()
    assert clock.steppables == ()
    assert clock.time_ms == 0
    assert clock._initialization is None
    assert world.current_world is None


def test_main_script_entrypoint(capsys):
    runpy.run_path(str(Path(main.__file__)), run_name='__main__')
    assert 'All integration checks passed.' in capsys.readouterr().out
