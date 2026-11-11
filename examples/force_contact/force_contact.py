import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import sim
import world
from maps.force_contact.world import ForceContactWorld
from pybricks.hubs import PrimeHub
from pybricks.parameters import Direction, Port
from pybricks.pupdevices import ForceSensor, Motor
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, wait
from render.video import PoseRecorder, save_video
from sim.clock import clock
from sim.validation import number
from world import BoxObstacle


APPROACH_SPEED_MM_S = 30.0
CONTACT_FORCE_N = 3.0
HEADING_KP = 3.0
LOOP_MS = 10
CALIBRATION_TIMEOUT_MS = 5000
CONTACT_TIMEOUT_MS = 30_000


class BumperContact:
    '''
    Convert front button compression against the wall into a force reading.

    This local model senses contact with the wall's xmin face.
    It does not apply collision forces to the drive base.

    Attributes:
        step_priority (int): Update after body motion and before pose recording.
        sensor (ForceSensor): Front-facing force sensor receiving scripted input.
        obstacle (BoxObstacle): Wall pressed by the button.
        stiffness_n_mm (float): Button spring stiffness in N/mm.
        max_travel_mm (float): Maximum simulated button travel in mm.
    '''
    step_priority: int = 2

    def __init__(self, sensor: ForceSensor, obstacle: BoxObstacle) -> None:
        '''
        Attach a button spring model to a sensor and wall.

        Arg:
            sensor (ForceSensor): Mounted front force sensor.
            obstacle (BoxObstacle): Wall with its front face at xmin.
        '''
        if not isinstance(sensor, ForceSensor):
            raise TypeError('sensor must be a ForceSensor')
        if not isinstance(obstacle, BoxObstacle):
            raise TypeError('obstacle must be a BoxObstacle')
        self.sensor: ForceSensor = sensor
        self.obstacle: BoxObstacle = obstacle
        self.stiffness_n_mm: float = 1.0
        self.max_travel_mm: float = 8.0

    def step(self, dt_s: float) -> None:
        '''
        Update the force reading from the current button position.

        Arg:
            dt_s (float): Positive simulation step in seconds.
        '''
        number(dt_s, 'dt_s', positive=True)
        x, y, z, _, _, _ = self.sensor.pose()
        wall = self.obstacle
        travel = 0.0
        if wall.ymin <= y <= wall.ymax and 0 <= z <= wall.height_mm:
            travel = min(self.max_travel_mm, max(0.0, x - wall.xmin))
        force = min(10.0, self.stiffness_n_mm * travel)
        self.sensor.model.set_measurement(force_n=force, distance_mm=travel)


def init() -> tuple:
    '''
    Create the field, hub, wheel motors and front force sensor.

    Returns:
        tuple: Field, hub, drive base, force sensor and contact updater.
    '''
    field = ForceContactWorld()
    world.current_world = field
    hub = PrimeHub()
    left = Motor(Port.B, Direction.COUNTERCLOCKWISE)
    right = Motor(Port.D)
    robot = DriveBase(left, right, wheel_diameter=56, axle_track=112)
    force = ForceSensor(Port.A)
    robot.model.mount(force.model, forward_mm=75, above_mm=12)
    robot.model.place(0, 0, heading_deg=0)
    robot.model.configure_friction(enabled=True, mass_kg=1.2)
    robot.settings(straight_speed=200, straight_acceleration=700,
                   turn_rate=60, turn_acceleration=150)
    robot.use_gyro(True)
    hub.imu.model.bias_dps = 0.8
    contact = BumperContact(force, field.obstacles[0])
    clock.add_steppable(contact)
    return field, hub, robot, force, contact


def run(recording: list) -> tuple:
    '''
    Use gyro turns on a slippery floor and stop when the button presses the wall.

    Arg:
        recording (list): Mutable list receiving video pose samples.

    Returns:
        tuple: Field, stopped drive base and pressed force sensor.
    '''
    if not isinstance(recording, list):
        raise TypeError('recording must be a list')
    field, hub, robot, force, contact = sim.initialize(init)
    recorder = PoseRecorder(robot, (force,), recording, clock)
    clock.add_steppable(recorder)
    try:
        timer = StopWatch()
        while not hub.imu.ready():
            if timer.time() >= CALIBRATION_TIMEOUT_MS:
                raise TimeoutError('IMU calibration timed out')
            wait(10)
        wait(10)
        hub.imu.reset_heading(0)
        print(f'IMU ready. Corrected gyro rate: {hub.imu.angular_velocity()[2]:.3f} deg/s')

        robot.straight(700, wait=False)
        wait(400)
        print(f'Acceleration: encoder {robot.distance():.1f} mm, true x {robot.pose()[0]:.1f} mm')
        while not robot.done():
            wait(10)
        wait(500)
        print(f'First leg: encoder {robot.distance():.1f} mm, true x {robot.pose()[0]:.1f} mm')
        robot.turn(-90)
        wait(500)
        robot.straight(400)
        wait(500)
        robot.turn(90)
        wait(500)

        timer.reset()
        while not force.pressed(CONTACT_FORCE_N):
            if timer.time() >= CONTACT_TIMEOUT_MS:
                raise TimeoutError('No force sensor contact before timeout')
            # Correct clockwise heading towards the original world x direction.
            turn_rate = max(-40.0, min(40.0, -HEADING_KP * hub.imu.heading()))
            robot.drive(APPROACH_SPEED_MM_S, turn_rate)
            wait(LOOP_MS)
        print(f'Contact detected: {force.force():.2f} N, travel {force.distance():.2f} mm')
    finally:
        robot.brake()
        wait(1000)
        clock.remove_steppable(recorder)
        clock.remove_steppable(contact)

    print(f'Stopped pose: {robot.pose()}')
    print(f'IMU heading: {hub.imu.heading():.2f} deg')
    print(f'Final force: {force.force():.2f} N')
    print(f'True forward speed: {robot.model.speed_mm_s():.3f} mm/s')
    return field, robot, force


def main() -> None:
    '''
    Run the example and save a video, including after a timeout.
    '''
    recording = []
    try:
        run(recording)
    except TimeoutError as error:
        print(f'Simulation timeout: {error}', file=sys.stderr)
    if recording:
        path = Path(__file__).resolve().with_name('force_contact.mp4')
        save_video(path, world.get_current_world(), recording)
        print(f'Video saved to {path}')


if __name__ == '__main__':
    main()
