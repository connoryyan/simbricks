# Motor gears

Pass gear teeth from the motor to the output shaft:

```python
motor = Motor(Port.A, Direction.CLOCKWISE, gears=[12, 36])
motor.run(100)
motor.run_angle(100, 90)
```

This reduction has a ratio of 3. Commands, `angle()`, `speed()` and
`reset_angle()` use output shaft degrees. A 90 degree output movement requires
270 degrees of motor shaft rotation. `positive_direction` sets the sign;
meshing gears do not automatically reverse it.

Use `gears=[[12, 36], [20, 40]]` for two stages with a total ratio of 6.
Each stage uses its last tooth count divided by its first. Intermediate idlers
do not change the ratio. Tooth counts must be positive integers, and gear
lists must not be empty. Omitting `gears` gives a ratio of 1.

`dc()` remains a motor duty percentage. A reduction lowers the maximum output
speed: the default 1500 deg/s motor shaft limit becomes 500 deg/s with a ratio
of 3. Choose DriveBase speeds that both geared wheels can reach. DriveBase
odometry and friction use output shaft rotation and speed.

The simulator model keeps `angle`, `speed` and `max_speed_dps` in motor shaft
units. Its `output_angle` and `output_speed` properties divide by `gear_ratio`.

References:

- [Official Motor API](https://code.pybricks.com/static/docs/pupdevices/motor)
- [Official gear ratio parser](https://github.com/pybricks/pybricks-micropython/blob/master/pybricks/common/pb_type_motor.c)

The simulator uses floating point ratios. The firmware stores ratios in
millidegrees. The simulator also validates every tooth count, including idlers.
