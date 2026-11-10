# Planar IMU

This implementation follows the interface and planar heading path in the
[official IMU source](https://github.com/pybricks/pybricks-micropython/blob/master/lib/pbio/src/imu.c).
See the [official IMU API](https://docs.pybricks.com/en/latest/hubs/primehub.html#using-the-imu)
and [drive base implementation](https://github.com/pybricks/pybricks-micropython/blob/master/lib/pbio/src/drivebase.c).

The official firmware separately integrates calibrated gyro samples for 1D
rotation. It also integrates a quaternion with a gravity-vector correction for
3D attitude. The correction weight decreases during motion. Stationary gyro
samples update the bias estimate using weight `1 / min(calibration_count, 20)`.
Heading is continuous and clockwise-positive, while gyro z and z rotation use
the right hand convention. Heading reset changes an offset, not the integrated
rotation or physical pose.

Our robot stays level. Pitch and roll are constrained to zero, so the simulator
implements the 1D gyro path and stationary bias calibration, rather than a full
quaternion filter. Gravity does not determine horizontal yaw, so accelerometer
data does not correct yaw drift. Wheel encoder heading is not blended into the
gyro estimate. The physical body's yaw increments generate synthetic angular
velocity samples, with optional z bias and seeded noise. These samples are
integrated independently; the reported heading is not copied from the body.

```python
from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Direction, Port
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import wait

hub = PrimeHub()
left = Motor(Port.B, Direction.COUNTERCLOCKWISE)
right = Motor(Port.D)
db = DriveBase(left, right, wheel_diameter=56, axle_track=112)
db.use_gyro(True)
while not hub.imu.ready():
    wait(10)
hub.imu.reset_heading(0)
imu = hub.imu
print(imu.heading())
print(imu.tilt())                     # (pitch, roll) = (0, 0)
print(imu.angular_velocity(Axis.Z))  # right hand rotation, deg/s
print(imu.rotation(-Axis.Z))         # clockwise accumulated rotation
```

`PrimeHub` and `InventorHub` share one simulated physical hub per clock session.
The IMU implementation is internal to `pybricks/_common/imu.py`; user programs
access it through `hub.imu`. Creating a drive base connects the hub automatically.
Creating a hub after one drive base also connects it to that body's existing IMU.
The hub can calibrate before motors or a drive base exist. Multiple independent
active robots require separate simulator sessions.

The public imports in the initialization example are supported. Hub buttons
provide `pressed()` with scripted state in `hub.model.pressed_buttons`.
Hub displays, lights, speakers and communication interfaces are not implemented.
Only default level mounting (`top_side=Axis.Z`, `front_side=Axis.X`) is supported.
Vector readings are tuples rather than Pybricks Matrix objects.
`pitch()` and `roll()` are additional simulator helpers that return zero.

`acceleration()` reports axle-centre forward acceleration, lateral acceleration
and positive z gravity, in mm/s/s. Accelerometer noise, bias and arbitrary hub
mounting orientations are not simulated. `calibrated=False` therefore only
changes gyro and rotation readings. The IMU reference frame is forward x, left y,
up z, at the axle centre.

Use `db.use_gyro(True)` for gyro heading feedback. The constructor takes the
standard motors, wheel diameter and axle track arguments. Distance remains based
on the wheels. `db.state()` reports gyro turn rate when enabled. Stop or brake
before `imu.reset_heading()`. `db.reset(angle=...)` stops the controller and resets
the gyro heading. This does not add missing motor-load or wheel-slip physics.

Model updates run after body odometry and before the drive controller. Reading
the IMU does not advance time or draw more noise samples. Placing the robot with
`db.model.place()` rebases the physical sampling origin, without injecting a
spurious rotation. It does not reset the reported heading.

`ready()` becomes true after a stationary calibration window and expires after
ten minutes without calibration. Calibration uses one-second stationary windows,
gyro sample variation, acceleration limits, and a simulator idle gate for body
motion and wheel speeds. It does not persist settings to disk.

Run the example with `python -B examples/imu_test.py`.

`ForceSensor` is available from `pybricks.pupdevices`. Its `force()`, `distance()`,
`pressed(force=3)` and `touched()` methods use scripted measurements supplied by
`sensor.model.set_measurement(force_n=..., distance_mm=...)`. Mechanical contact
forces are not simulated.
