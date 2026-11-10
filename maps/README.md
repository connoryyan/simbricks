# Maps

Each map has its own package folder:

```text
maps/
    my_map/
        __init__.py
        world.py
        preview.svg
```

Define a `World` subclass in `world.py`. Use millimetres for coordinates and
degrees for angles. Ground is at z = 0. The world x axis points right, y points
up in the top view, and z points above the ground.

```python
from pathlib import Path
from world import World, Polyline, BoxObstacle


class MyWorld(World):
    def __init__(self):
        super().__init__(ambient_light=5, bounds=(-100, -300, 1600, 300))
        self.add_line(Polyline([(0, 0), (500, 200), (1000, 0)], width_mm=20))
        self.add_obstacle(BoxObstacle(1000, -100, 1150, 100, 150))


if __name__ == '__main__':
    MyWorld().save_preview(Path(__file__).with_name('preview.svg'))
```

Generate the preview from the repository root with
`python -B -m maps.my_map.world`.

Select the map before creating or running a robot:

```python
import world
from maps.my_map.world import MyWorld

world.current_world = MyWorld()
```

`World` combines ground reflection, all lines and the nearest obstacle hit.
Override `get_ambient_at()` for spatial lighting. Create a `Line` subclass to
provide a centre distance and SVG element, or an `Obstacle` subclass to provide
ray intersection and an SVG element. The `scriptable` map intentionally returns
a configured ultrasonic reading for tests.

The map preview contains the map only. The example's `map_visual.svg` also
contains the robot's actual path and final pose.
