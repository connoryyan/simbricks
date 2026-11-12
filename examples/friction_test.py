import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import sim
import world
from maps.flat_white.world import FlatWhiteWorld
from pybricks.parameters import Direction, Port
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import wait


left = Motor(Port.B, Direction.COUNTERCLOCKWISE)
right = Motor(Port.D)
robot = DriveBase(left, right, 56, 112)


def init():
    '''
    Create a robot on a slippery field.
    '''
    field = FlatWhiteWorld()
    field.friction_coefficient = 0.02
    world.current_world = field
    robot.model.configure_friction(mass_kg=1.2)
    robot.model.place(0, 0, 0)


def main():
    '''
    Compare encoder travel with true motion during wheelspin and braking.
    '''
    left.run(1000)
    right.run(1000)
    wait(1000)
    print(f'Encoder distance: {robot.distance():.1f} mm')
    print(f'True x position: {robot.pose()[0]:.1f} mm')
    robot.brake()
    wait(300)
    print(f'Wheel speed after braking: {robot.state()[1]:.1f} mm/s')
    print(f'True body speed: {robot.model.speed_mm_s():.1f} mm/s')
    before = robot.pose()[0]
    wait(2000)
    print(f'Additional sliding distance: {robot.pose()[0] - before:.1f} mm')
    print(f'Final body speed: {robot.model.speed_mm_s():.3f} mm/s')


if __name__ == '__main__':
    sim.run(init, main)
