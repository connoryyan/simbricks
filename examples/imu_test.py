import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Direction, Port
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import wait
import sim



hub = PrimeHub()
left = Motor(Port.B, Direction.COUNTERCLOCKWISE)
right = Motor(Port.D)
db = DriveBase(left, right, wheel_diameter=56, axle_track=112)
db.use_gyro(True)


def init(robot):
    '''
    Set up the robot devices, initial position and field.
    '''
    import world
    from maps.flat_white.world import FlatWhiteWorld
    world.current_world = FlatWhiteWorld()
    robot.model.place(0, 0, 0)
sim.initialize(init, db)


def main():
    '''
    Calibrate a level IMU and make a gyro-controlled clockwise turn.
    '''
    imu = hub.imu
    while not imu.ready():
        wait(10)

    print('Ready:', imu.ready(), 'Tilt:', imu.tilt())
    imu.reset_heading(0)
    db.turn(90)
    wait(500)
    print('Clockwise heading:', imu.heading())
    print('Right hand z rotation:', imu.rotation(Axis.Z))
    print('Angular velocity:', imu.angular_velocity())
    print('Acceleration:', imu.acceleration())
    db.reset(angle=0)
    print('Reset heading:', imu.heading())


if __name__ == '__main__':
    main()
