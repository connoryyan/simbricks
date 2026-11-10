# Program initialization

A simulation program must define its own callable `init` function. Its body and
signature belong to the program. There are no mandatory device lists, mounts,
positions or map arguments. A motor-only program does not need a map or sensors.
A program without any devices is also valid.

```python
import sim
from pybricks.tools import wait


def init():
    # Create or configure only what this program needs.
    pass


def main():
    wait(100)


if __name__ == '__main__':
    sim.run(init, main)
```

`sim.run(init, main)` checks that the initializer is callable and named `init`,
executes it, then runs `main`. Missing or misnamed initialization raises
`sim.NotInitializedError`. If init fails, main is not called and robot operations
remain disabled. The original initialization error is propagated.

`NotInitializedError()` has this default message:

```text
Simulation is not initialized. Define init() and call sim.initialize(init) or sim.run(init, main) before using robot APIs.
```

Pass a message to the exception constructor to replace this text.

For existing examples that manage their own main loop, use
`sim.initialize(init, *args, **kwargs)`. It forwards the arguments and returns the
value from the user-defined initializer. For example:

```python
import world


def init(robot, color, field):
    world.current_world = field
    robot.model.mount(color.model, forward_mm=40, above_mm=-18, pitch_deg=90)
    robot.model.place(50, 0, heading_deg=0)
    return field


field = sim.initialize(init, robot, color, field)
```

The initializer may create no devices, use only motors, configure a map, mount
sensors, place a robot, or wait for IMU calibration. These are choices for each
program. The simulator does not inspect or enforce its device configuration.

Robot-facing Pybricks operations require entering initialization first. They are
also allowed inside init, so calibration and sensor checks can be performed
there. Object construction, settings, stopping and braking remain available
before initialization. Low-level physics and direct clock stepping are available
for engine tests.

New devices or maps do not require declaring a new inventory. Repeating init is
allowed, but the program owns any cleanup or repeated mount handling. Merely
defining init is not enough; enter it through one of the simulation entry points.
