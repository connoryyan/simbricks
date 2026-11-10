# simbricks

A drop-in simulator for Pybricks MicroPython, re-implementing the Pybricks API on top of a small physics core, so unmodified robot scripts run against a virtual robot on computer. Because the same script also runs on a real LEGO hub, it supports a simulate-then-deploy workflow, and closing the sim-to-real gap through calibration against logged hardware data is a future goal.

Run the line-following example from the repository root:

```powershell
python -B examples/line_follow/line_follow.py
```

The map has a winding 20 mm black line and a 150 mm high obstacle at its end.
The robot starts on the line centre. A downward colour sensor sits 10 mm above
the ground and 10 mm left of the robot centre, so it follows the left line edge
with P control. A forward ultrasonic sensor sits 60 mm above the ground. The
robot brakes when the measured obstacle distance reaches 100 mm. The SVG shows
the map, actual robot path and final position. Each run saves it as
`examples/line_follow/map_visual.svg`. A simulation timeout also brakes
the robot if it does not reach the obstacle.

The root `world` package defines `World`, `Line` and `Obstacle` base classes.
Reusable geometry includes `Polyline` and `BoxObstacle`. Concrete maps live in
`maps/<name>/world.py`, beside their `preview.svg`. See [maps/README.md](maps/README.md)
for the map folder layout and a custom map example.
