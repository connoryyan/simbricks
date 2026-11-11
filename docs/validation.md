# Parameter validation

Public robot APIs and simulator helpers check arguments before changing state.

- Wrong types raise `TypeError`. Numeric arguments accept `int` and `float`,
  excluding `bool`. Numeric strings are not converted.
- Non-finite numbers, invalid ranges and incorrect vector lengths raise
  `ValueError`. Error messages name the parameter.
- Flags such as `wait`, `use_gyro`, `enabled` and `calibrated` require `bool`.
- Device ports, motor directions and stop modes require their enum types.
- Durations are non-negative. Simulation steps, physical dimensions and motion
  limits are positive. Noise and friction coefficients are non-negative.
- Duty is between -100 and 100. Reflection, ambient light and color saturation
  and value are between 0 and 100. Hue is between 0 and 360.
- Geometry vectors contain finite numbers. Ray directions must be unit vectors.
  IMU projection axes may have any non-zero length.
- Random seeds and wheel direction signs require integers, excluding `bool`.
  Seeds may also be `None`; direction signs must be -1 or 1.
- Vector projection checks both its input vector and its axis. Clamp bounds must
  be ordered. Motion helpers require an active maneuver when stepping a move.

Classes declare instance fields at their first assignment. Properties specify
their return and setter types. Enum value types use `_value_` annotations, while
member types are inferred according to the
[Python enum typing specification](https://typing.python.org/en/latest/spec/enums.html).
Type annotations describe the API; runtime validation still performs the checks.

Multi-argument operations validate all inputs before applying settings, moving
motors or advancing the clock. Video frames are validated before starting FFmpeg.
A failed constructor does not register a device with the clock for invalid
parameter types or ranges.

Existing domain errors remain: `NotInitializedError` for missing initialization,
`NotSteppableError` for objects without `step()`, and the official-style
`ZeroDivisionError` for non-positive gear tooth counts. Unsupported operations
still raise `NotImplementedError`.

`None` remains valid for optional arguments such as `gears`, unspecified settings,
IMU model bodies and automatic yaw inertia. It is rejected for required numbers.
