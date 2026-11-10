import math
import shutil
import subprocess


class PoseRecorder:
    '''
    Record poses at 30 frames per simulated second without sampling sensors.
    '''
    step_priority = 3

    def __init__(self, robot, sensors, frames, clock):
        self.robot = robot
        self.sensors = sensors
        self.frames = frames
        self.clock = clock
        self.start_ms = clock.time_ms
        self.next_frame = 0
        self.capture()

    def capture(self):
        '''
        Store the current robot and sensor poses.
        '''
        self.frames.append((
            (self.clock.time_ms - self.start_ms) / 1000,
            self.robot.pose(), tuple(sensor.pose() for sensor in self.sensors),
        ))
        self.next_frame += 1

    def step(self, dt_s):
        '''
        Record the state at the end of the current simulation step.
        '''
        # The clock publishes its new time after all steppables have run.
        elapsed = self.clock.time_ms - self.start_ms + dt_s * 1000
        if elapsed + 1e-9 >= self.next_frame * 1000 / 30:
            self.frames.append((elapsed / 1000, self.robot.pose(),
                                tuple(sensor.pose() for sensor in self.sensors)))
            self.next_frame += 1


def save_video(path, field, frames, fps=30):
    '''
    Save a top view of the recorded simulation as an H.264 MP4 video.

    Arg:
        path (Path): Output video path.
        field (World): Map with polyline lines and box obstacles.
        frames (list): Recorded times, robot poses and sensor poses.
        fps (int): Video frames per second. Use 30 for PoseRecorder output.
    '''
    from PIL import Image, ImageDraw

    executable = shutil.which('ffmpeg')
    if executable is None:
        raise RuntimeError('Install FFmpeg and add it to PATH to export MP4')
    if not frames:
        raise ValueError('No recorded frames')
    if fps != 30:
        raise ValueError('PoseRecorder records at 30 FPS')

    width, height = 1280, 480
    xmin, ymin, xmax, ymax = field.bounds
    scale = min((width-60)/(xmax-xmin), (height-130)/(ymax-ymin))
    ox = (width - (xmax-xmin)*scale)/2
    oy = 90 + (height-120 - (ymax-ymin)*scale)/2

    def point(x, y):
        return (ox + (x-xmin)*scale, oy + (ymax-y)*scale)

    background = Image.new('RGB', (width, height), '#f6f7f9')
    draw = ImageDraw.Draw(background)
    draw.rectangle([point(xmin, ymax), point(xmax, ymin)], fill='white')
    for line in field.lines:
        points = [point(x, y) for x, y in line.points]
        shade = round(line.reflection*255/100)
        draw.line(points, fill=(shade, shade, shade),
                  width=max(1, round(line.width_mm*scale)), joint='curve')
    for box in field.obstacles:
        draw.rectangle([point(box.xmin, box.ymax), point(box.xmax, box.ymin)],
                       fill='#e57235', outline='#a74715')
        draw.text(point(box.xmin, box.ymax+35), f'{box.height_mm:g} mm high', fill='#a74715')
    draw.text((30, 42), 'Black: 20 mm line   Blue: robot path   Green: colour sensor   Red: ultrasound',
              fill='#394957')

    command = [executable, '-y', '-v', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
               '-s', f'{width}x{height}', '-r', str(fps), '-i', 'pipe:0', '-an',
               '-c:v', 'libx264', '-preset', 'fast', '-crf', '20', '-pix_fmt', 'yuv420p',
               '-movflags', '+faststart', str(path)]
    process = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
    trail = []
    try:
        for index, (time_s, (x, y, heading), sensors) in enumerate(frames):
            frame = background.copy()
            draw = ImageDraw.Draw(frame)
            trail.append(point(x, y))
            if len(trail) > 1:
                draw.line(trail, fill='#168dc1', width=2)
            angle = math.radians(heading)
            c, s = math.cos(angle), math.sin(angle)

            def local(forward, left):
                return point(x+forward*c-left*s, y+forward*s+left*c)

            # Draw a 110 x 112 mm body and two wheels around the axle centre.
            draw.polygon([local(55, 56), local(55, -56), local(-55, -56), local(-55, 56)],
                         fill='#8ec7e8', outline='#155b87')
            for side in (-1, 1):
                draw.polygon([local(-28, side*56-8), local(28, side*56-8),
                              local(28, side*56+8), local(-28, side*56+8)], fill='#354453')
            draw.line([local(0, 0), local(70, 0)], fill='#155b87', width=3)
            if sensors:
                sx, sy, *_ = sensors[0]
                px, py = point(sx, sy)
                draw.ellipse((px-3, py-3, px+3, py+3), fill='#129952')
            if len(sensors) > 1:
                sx, sy, sz, yaw, pitch, _ = sensors[1]
                yaw, pitch = math.radians(yaw), math.radians(pitch)
                dx, dy, dz = math.cos(pitch)*math.cos(yaw), math.cos(pitch)*math.sin(yaw), -math.sin(pitch)
                # Query geometry only. Do not consume sensor noise during rendering.
                hit = field.raycast_surface(sx, sy, sz, dx, dy, dz, 2000)
                distance = 2000 if hit is None else hit.distance_mm
                visible = min(distance, 400)
                draw.line([point(sx, sy), point(sx+dx*visible, sy+dy*visible)], fill='#d94545', width=2)
                draw.text((width-330, 18), f'Ray distance: {distance:.0f} mm', fill='#a72c2c')
            draw.text((30, height-35), f'Simulation time: {time_s:.2f} s   x: {x:.0f} mm   y: {y:.0f} mm',
                      fill='#18212b')
            if index == len(frames)-1:
                draw.text((width-240, height-35), 'STOPPED', fill='#a72c2c')
            process.stdin.write(frame.tobytes())
        # Keep the final stopped state visible for one more second.
        for _ in range(fps):
            process.stdin.write(frame.tobytes())
        process.stdin.close()
        error = process.stderr.read().decode('utf-8', errors='replace')
        if process.wait() != 0:
            raise RuntimeError(f'Video encoding failed: {error}')
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        process.stderr.close()
