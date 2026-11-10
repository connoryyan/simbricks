import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from maps.line_follow.world import LineFollowWorld
import world
from pybricks.tools import StopWatch, wait
from pybricks.robotics import DriveBase
from pybricks.pupdevices import ColorSensor, Motor, UltrasonicSensor
from pybricks.parameters import Direction, Port
from sim.clock import clock
from render.video import PoseRecorder, save_video


SPEED_MM_S = 100.0
KP = 3
MAX_TURN_RATE = 100.0
TARGET_REFLECTION = 50.0
LOOP_MS = 10
STOP_DISTANCE_MM = 100
TIMEOUT_MS = 90_000


def init():
    '''
    Create the map, robot and mounted sensors.

    Returns:
        LineFollowWorld: Map used by the robot.
    '''
    global left, right, db, color, ultrasonic
    field = LineFollowWorld()
    world.current_world = field

    left = Motor(Port.A, Direction.COUNTERCLOCKWISE)
    right = Motor(Port.B)
    db = DriveBase(left, right, wheel_diameter=56, axle_track=112)
    color = ColorSensor(Port.C)
    ultrasonic = UltrasonicSensor(Port.D)

    db.model.mount(color.model, forward_mm=40, left_mm=10,
                   above_mm=-18, pitch_deg=90)
    db.model.mount(ultrasonic.model, forward_mm=60,
                   above_mm=32, pitch_deg=0)
    db.model.place(50, 0, heading_deg=0)
    return field


def run(recording=None):
    '''
    Follow the left line edge with P control and stop before the obstacle.

    Arg:
        recording (list | None): Optional list for video pose samples.

    Returns:
        tuple: Map, robot, recorded poses and final obstacle distance.
    '''
    field = init()
    recorder = None
    if recording is not None:
        recorder = PoseRecorder(db, (color, ultrasonic), recording, clock)
        clock.add_steppable(recorder)
    timer = StopWatch()
    trace = [db.pose()]
    stopped_for_obstacle = False
    try:
        while timer.time() < TIMEOUT_MS:
            distance = ultrasonic.distance()
            if distance <= STOP_DISTANCE_MM:
                stopped_for_obstacle = True
                break

            # White on the left edge means steering clockwise towards the line.
            error = color.reflection() - TARGET_REFLECTION
            turn_rate = max(-MAX_TURN_RATE, min(MAX_TURN_RATE, KP * error))
            db.drive(SPEED_MM_S, turn_rate)
            wait(LOOP_MS)
            trace.append(db.pose())
            wait(50)
    finally:
        db.brake()
        # Advance the simulated time while the wheels finish braking.
        wait(500)
        trace.append(db.pose())
        if recorder is not None:
            clock.remove_steppable(recorder)

    if not stopped_for_obstacle:
        raise TimeoutError(
            "Line following timed out before detecting the obstacle")

    print(f"Stopped for obstacle after {timer.time()/1000:.2f} s")
    print(f"Pose (mm, mm, deg): {db.pose()}")
    print(f"Obstacle distance: {ultrasonic.distance()} mm")
    print(f"Forward speed after braking: {db.state()[1]:.3f} mm/s")
    return field, db, trace, ultrasonic.distance()


def save_map(path, field, trace):
    '''
    Save the line, obstacle and robot path as an SVG map.

    Arg:
        path (Path): Output SVG path.
        field (LineFollowWorld): Simulated map.
        trace (list): Recorded robot poses.
    '''
    field.save_preview(path, trace)


def main():
    '''
    Run the simulation and save recorded frames even after a timeout.
    '''
    recording = []
    try:
        field, db, trace, distance = run(recording)
    except TimeoutError as err:
        print(f"Simulation timeout: {err}", file=sys.stderr)
        field = world.get_current_world()

    if not recording:
        print("No recorded frames to save", file=sys.stderr)
        return
    video_path = Path(__file__).resolve().with_name('line_follow.mp4')
    save_video(video_path, field, recording)
    print(f"Video saved to {video_path}")


if __name__ == '__main__':
    main()
