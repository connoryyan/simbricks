import math
from pathlib import Path
from tempfile import TemporaryDirectory
import xml.etree.ElementTree as ET

import sim
import world
from maps.straight_line.world import LineWorld
from pybricks.hubs import PrimeHub
from pybricks.parameters import Direction, Port, Stop
from pybricks.pupdevices import ColorSensor, ForceSensor, Motor
from pybricks.pupdevices import UltrasonicSensor
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from sim.clock import clock
from world import BoxObstacle


def init() -> tuple:
    '''
    Build a robot and map for integration testing.
    '''
    field = LineWorld(ambient_light=0)
    field.add_obstacle(BoxObstacle(1000, -100, 1100, 100, 150))
    world.current_world = field
    hub = PrimeHub()
    left = Motor(Port.B, Direction.COUNTERCLOCKWISE, gears=[12, 36])
    right = Motor(Port.D, gears=[12, 36])
    robot = DriveBase(left, right, wheel_diameter=56, axle_track=112)
    color = ColorSensor(Port.C)
    ultrasonic = UltrasonicSensor(Port.A)
    force = ForceSensor(Port.E)
    robot.model.mount(color.model, forward_mm=40, above_mm=-18,
                      pitch_deg=90)
    robot.model.mount(ultrasonic.model, forward_mm=60, above_mm=32)
    robot.model.mount(force.model, forward_mm=75, above_mm=12)
    robot.model.place(0, 0, 0)
    return field, hub, robot, color, ultrasonic, force


def check_motion_and_sensors(robot, color, ultrasonic) -> None:
    '''
    Check geared motion, mounted sensors and world geometry together.
    '''
    assert color.reflection() == 10, 'Color sensor did not detect the line'
    robot.model.place(0, 80, 0)
    assert color.reflection() == 85, 'Color sensor did not detect the floor'
    robot.model.place(0, 0, 0)
    before = ultrasonic.distance()
    assert abs(before - 940) <= 1, 'Ultrasonic ray missed the obstacle'
    robot.straight(200, then=Stop.BRAKE)
    wait(500)
    assert abs(robot.pose()[0] - 200) < 3, 'DriveBase travel is incorrect'
    assert abs(robot.distance() - robot.pose()[0]) < 0.01
    assert abs(color.pose()[0] - robot.pose()[0] - 40) < 0.01
    assert abs(before - ultrasonic.distance() - 200) < 3
    for motor in (robot._left, robot._right):
        assert abs(motor.model.angle - motor.model.output_angle * 3) < 0.01
    print('PASS: geared motors, DriveBase, sensor mounts and map readings')


def check_gyro(hub, robot) -> None:
    '''
    Check shared IMU calibration and gyro-controlled turning.
    '''
    wait(1500)
    assert hub.imu.ready(), 'IMU did not calibrate while stationary'
    robot.use_gyro(True)
    assert hub.imu.model is robot.model.imu
    hub.imu.reset_heading(0)
    robot.turn(90, then=Stop.BRAKE)
    wait(500)
    assert abs(hub.imu.heading() - 90) < 1, 'Gyro turn missed its target'
    assert abs(robot.pose()[2] + hub.imu.heading()) < 0.01
    assert hub.imu.tilt() == (0, 0)
    assert hub.imu.roll() == 0
    print('PASS: shared IMU, stationary calibration and gyro turning')


def check_friction_and_force(field, robot, force) -> None:
    '''
    Check wheel slip and braking after a scripted force measurement.
    '''
    robot.use_gyro(False)
    robot.reset()
    robot.model.place(0, 0, 0)
    field.friction_coefficient = 0.04
    robot.model.configure_friction(enabled=True)
    robot.drive(200, 0)
    wait(400)
    assert robot.distance() > robot.pose()[0] + 5, 'Wheel slip was not modeled'
    force.model.set_measurement(force_n=3, distance_mm=3)
    assert force.pressed(), 'Force measurement did not reach the device API'
    robot.brake()
    wait(1000)
    assert math.hypot(*robot.model.velocity_mm_s()) < 0.01
    assert force.distance() == 3
    print('PASS: finite friction, force sensor readings and braking')


def check_preview(field) -> None:
    '''
    Check class-owned file error handling and successful retry.
    '''
    with TemporaryDirectory() as folder:
        path = Path(folder) / 'missing' / 'map.svg'
        assert field.save_preview(path) is False
        assert isinstance(field.preview_error, FileNotFoundError)
        path.parent.mkdir()
        assert field.save_preview(path, [(0, 0, 0)]) is True
        assert field.preview_error is None
        assert ET.parse(path).getroot().tag.endswith('svg')
    print('PASS: World SVG save failure, retry and valid output')


def main() -> None:
    '''
    Run integration checks and restore the previous simulation state.
    '''
    previous_groups = {key: group.copy()
                       for key, group in clock._steppables.items()}
    previous_time = clock.time_ms
    previous_init = clock._initialization
    previous_world = world.current_world
    clock._steppables.clear()
    clock.time_ms = 0
    clock._initialization = None
    world.current_world = None
    try:
        field, hub, robot, color, ultrasonic, force = sim.initialize(init)
        check_motion_and_sensors(robot, color, ultrasonic)
        check_gyro(hub, robot)
        check_friction_and_force(field, robot, force)
        check_preview(field)
        print('All integration checks passed.')
    finally:
        clock._steppables.clear()
        clock._steppables.update(previous_groups)
        clock.time_ms = previous_time
        clock._initialization = previous_init
        world.current_world = previous_world


if __name__ == '__main__':
    main()
