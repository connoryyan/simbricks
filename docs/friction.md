# Wheel friction and sliding

Finite friction is an optional simulator model. Configure it in your own init:

```python
def init():
    field = FlatWhiteWorld()
    field.friction_coefficient = 0.02
    world.current_world = field
    robot.model.configure_friction(
        mass_kg=1.2,
        yaw_inertia_kg_m2=0.003,
        contact_stiffness=30.0,
    )
    robot.model.place(0, 0, 0)
```

The example assumes the motor and drivebase objects have already been created.
Alternatively create them inside init, as in `examples/friction_test.py`.
Mass is kg, yaw inertia is kg m squared, and contact stiffness is N/(m/s).
If inertia is omitted, it is estimated for a square body with side equal to the
axle track. These defaults are illustrative and need calibration to your robot.
Call `configure_friction(enabled=False)` to return to ideal rolling. Existing
programs retain ideal rolling until they enable the finite-friction model.

`World.get_friction_at(x_mm, y_mm)` provides the local coefficient at each wheel.
Override it in a map subclass for slippery patches or different left/right grip:

```python
def get_friction_at(self, x_mm, y_mm):
    return 0.02 if 500 < x_mm < 1000 else 0.8
```

The model compares wheel circumference speed with actual ground contact speed.
Longitudinal force is proportional to the slip speed. Lateral force opposes
sideways motion. The combined force at each wheel is limited to mu * normal
load, so turning and acceleration share the available traction. Normal load is
split equally between the two driven wheels. Dynamics use SI units internally,
with substeps at most 1 ms and smaller substeps for stiff contacts.

| Reading | Source |
| --- | --- |
| `robot.distance()` | Wheel encoder travel |
| `robot.angle()` without gyro | Wheel encoder rotation difference |
| `robot.angle()` with gyro | IMU integrated heading |
| `robot.state()` speed | Wheel encoder speed |
| `robot.state()` turn rate | Wheel encoders, or gyro when enabled |
| `robot.model.pose()` | Actual world position and heading |
| `robot.model.speed_mm_s()` | Actual forward body speed |
| `robot.model.velocity_mm_s()` | Actual world x/y velocity |
| `robot.model.turn_rate_dps()` | Actual clockwise body turn rate |

Sensors follow actual body pose. The IMU receives actual yaw changes and body
acceleration. Lateral sliding prevents stationary calibration, even if forward
speed is zero. Encoder resets do not modify actual motion. Simulator placement
clears body momentum without resetting encoder travel.

This is a compliant planar contact approximation. It does not model a separate
static/kinetic friction transition, wheel deformation, caster load, load transfer,
motor torque reaction or obstacle collision response. MotorModel still prescribes
wheel response independently of contact load. Consequently this model demonstrates
wheelspin and braking/turning slip but is not a calibrated full drivetrain model.
No drag is added: a moving body on zero-friction ground keeps its momentum.

Run `python -B examples/friction_test.py` to see encoder travel differ from true
travel and the body continue sliding after the wheels brake.
