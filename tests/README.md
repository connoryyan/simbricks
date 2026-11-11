# Tests

Run from the repository root:

```powershell
python -B -m pytest
```

Each file tests one area: motors, gears, drive bases, friction, hubs, IMU,
sensors, sensor mounts, worlds, video, examples, clock, initialization, shared
validation or constructors. Invalid arguments live beside the related behavior.
`test_constructors.py` checks constructor rules shared across several classes.

Tests use ordinary functions and `assert`. Different behavior has a named test;
similar invalid inputs use a normal `for` loop. No `parametrize` decorators are
used.

Shared fixtures in `conftest.py`:

- `clean_simulation`: automatically isolates clock, devices and world state.
- `initialized`: calls an empty `sim.initialize(init)` before a test.
- `robot`: creates a drive base with a reversed left motor.
- `hub_robot`: creates a hub and drive base sharing one IMU.
- `flat_field`: selects a white field with default ambient light.
- `white_field`: selects a white field with no ambient light for sensor tests.

Initialization tests omit `initialized` so they can check missing setup errors.
The line-following tests prepare their example devices in a local fixture.
Use pytest's `monkeypatch`, `capsys` and `tmp_path` for temporary replacements,
captured output and generated files.

The suite requires 100% statement and branch coverage of the application code.
Open `htmlcov/index.html` for the report. Encoder unit tests use a small fake
process; the real MP4 test also runs when Pillow and FFmpeg are available.
