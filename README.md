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
