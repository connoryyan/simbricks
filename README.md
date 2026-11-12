# simbricks

A drop-in simulator for Pybricks MicroPython, re-implementing the Pybricks API on top of a small physics core, so unmodified robot scripts run against a virtual robot on computer. Because the same script also runs on a real LEGO hub, it supports a simulate-then-deploy workflow, and closing the sim-to-real gap through calibration against logged hardware data is a future goal.

Simulation programs define their own `init` function. Use `sim.run(init, main)`
or `sim.initialize(init)` to enter it before robot operations. Missing initialization
raises `sim.NotInitializedError`. No drivebase, sensor or map is mandatory. See
[docs/initialization.md](docs/initialization.md) for examples.

Run the line-following example from the repository root:

```powershell
python -B examples/line_follow/line_follow.py
```

For a map using force contact, finite wheel friction and gyro feedback, run
`python -B examples/force_contact/force_contact.py`. It calibrates the IMU, drives
on a slippery floor and stops when the front force sensor reaches 3 N.

The map has a winding 20 mm black line and a 150 mm high obstacle at its end.
The robot starts on the line centre. A downward colour sensor sits 10 mm above
the ground and 10 mm left of the robot centre, so it follows the left line edge
with P control. A forward ultrasonic sensor sits 60 mm above the ground. The
robot brakes when the measured obstacle distance reaches 100 mm. Each run saves
a 1280 x 480, 30 FPS H.264 video as `examples/line_follow/line_follow.mp4`.
Video export requires Pillow and FFmpeg on PATH. The video shows the map,
robot, sensor positions, ultrasonic ray and actual path at simulation speed.
A simulation timeout also brakes
the robot if it does not reach the obstacle.

The root `world` package defines `World`, `Line` and `Obstacle` base classes.
Reusable geometry includes `Polyline` and `BoxObstacle`. Concrete maps live in
`maps/<name>/world.py`, beside their `preview.svg`. See [maps/README.md](maps/README.md)
for the map folder layout and a custom map example.

The planar IMU supports gyro heading, stationary bias calibration and zero pitch
and roll. Run `python -B examples/imu_test.py`. See [docs/imu.md](docs/imu.md) for
the simulator binding, coordinate conventions and official implementation references.

Optional finite wheel friction separates encoder odometry from actual body motion.
Run `python -B examples/friction_test.py` for wheelspin and braking slip. See
[docs/friction.md](docs/friction.md) for configuration and model limits.

Motors accept `gears=[12, 36]` or multiple gear trains. Angles and speeds use
the output shaft, including DriveBase odometry. See [docs/gears.md](docs/gears.md).

DriveBase moves default to `Stop.HOLD`, which actively holds both wheel shaft
positions after the move. `Stop.BRAKE` applies passive braking, while `Stop.COAST`
and `Stop.COAST_SMART` release the wheels. New drive commands release the hold.
Call `robot.hold()` to cancel an active move and hold the current wheel positions.

Public APIs reject incorrect argument types and values before changing simulation
state. See [docs/validation.md](docs/validation.md) for the parameter rules.

`World.save_preview(path, trace)` handles file write errors inside the class.
It returns `True` on success or `False` on an `OSError`. Inspect
`field.preview_error` for the cause; a successful retry clears it. Invalid
arguments still raise their normal exceptions.

Install the test dependencies and run the tests from the repository root:

```powershell
python -m pip install -r requirements-dev.txt
python -B -m pytest
```

Run the integration checks separately with `python -B main.py`. They check
geared motors, DriveBase motion, sensor mounting and map readings, shared IMU
calibration and turning, wheel slip, force readings and braking, and SVG save
failure and retry. Failed assertions stop the checks with an error. The entry
point restores the previous simulation state and requires no pytest dependency.

Check Python code against PEP 8 with `python -m flake8`. Lines use a maximum
of 79 characters. Standalone examples may import local packages after adding
the repository root to the import path. Format Python files with
`python -m autopep8 --in-place --recursive sim pybricks world maps render examples tests`.
Docstrings keep triple single quotes.

Tests use pytest assertions and fixtures. Coverage includes `sim`, `pybricks`,
`world`, `maps`, `render`, `examples` and `main`, with a required 100% statement and branch
coverage. Open `htmlcov/index.html` for the detailed report. FFmpeg on PATH enables
the real MP4 export test; encoder failure tests use a substitute process.
