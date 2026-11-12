from pathlib import Path
import math
import xml.etree.ElementTree as ET

import pytest

from maps.flat_white.world import FlatWhiteWorld
from maps.force_contact.world import ForceContactWorld
from maps.line_follow.world import LineFollowWorld
from maps.scriptable.world import ScriptableWorld
from maps.straight_line.world import LineWorld
from world import BoxObstacle, Line, Obstacle, Polyline, SurfaceHit, World
import world


class Field(World):

    '''
    Provide configurable world geometry for validation tests.

    Attributes:
        ambient_light (float): Uniform ambient light signal from 0 to 100.
        reflection (float): Ground reflection outside lines, from 0 to 100.
        bounds (tuple): Preview rectangle (xmin, ymin, xmax, ymax) in world mm.
        lines (list): Ground lines contributing to reflection readings.
        obstacles (list): Obstacle shapes tested by surface rays.
        friction_coefficient (float): Uniform non-negative ground friction
            coefficient.
        _friction_coefficient (float): Stored coefficient backing
            friction_coefficient.
        _ambient_light (float): Stored ambient signal backing ambient_light.
    '''

    def __init__(self, **kwargs):
        super().__init__(**kwargs)


def test_map_validation():
    for reflection in (-1, 101, float('nan')):
        with pytest.raises(ValueError, match='reflection'):
            Field(reflection=reflection)
    for bounds in ((0, 0, 1), (0, 0, float('inf'), 1),
                   (0, 0, 0, 1), (0, 1, 1, 0)):
        with pytest.raises(ValueError, match='bounds'):
            Field(bounds=bounds)


def test_line_validation():
    for options in ({'width_mm': 0}, {'edge_width_mm': 30},
                    {'reflection': -1}):
        with pytest.raises(ValueError):
            Polyline([(0, 0), (1, 1)], **options)
    for points in ([], [(0, 0)], [(0, 0), (1,)], [(0, 0), (float('nan'), 1)]):
        with pytest.raises(ValueError):
            Polyline(points)


def test_line_distance_before_and_after_segment():
    line = Polyline([(0, 0), (10, 0)])
    assert line.distance_to(-3, 4) == 5
    assert line.distance_to(13, 4) == 5
    assert line.distance_to(5, 4) == 4
    assert line.get_reflection_at(5, 0, 85) == 10
    assert line.get_reflection_at(5, 10, 85) == 48
    assert line.get_reflection_at(5, 20, 85) == 85


def test_box_rays():
    box = BoxObstacle(0, 0, 10, 10, 10)
    with pytest.raises(ValueError, match='reflection'):
        BoxObstacle(0, 0, 10, 10, 10, reflection=101)
    hit = box.raycast((5, 5, 5), (1, 0, 0), 100)
    assert hit.distance_mm == 5
    assert hit.x_mm == 10
    assert box.raycast((-5, 20, 5), (1, 0, 0), 100) is None
    direction = (1 / math.sqrt(1.01), -0.1 / math.sqrt(1.01), 0)
    assert box.raycast((-5, 20, 5), direction, 100) is None
    assert box.raycast((20, 5, 5), (1, 0, 0), 100) is None
    assert box.raycast((-5, 5, 5), (1, 0, 0), 4) is None


def test_nearest_surface_and_ground():
    field = Field()
    field.add_obstacle(BoxObstacle(0, 0, 10, 10, 10))
    field.add_obstacle(BoxObstacle(0, 0, 10, 10, 20))
    assert field.raycast_surface(5, 5, 30, 0, 0, -1, 100).z_mm == 20
    assert field.raycast_surface(50, 50, 30, 0, 0, -1, 100).z_mm == 0
    assert field.raycast_surface(50, 50, 30, 0, 0, -1, 10) is None


def test_preview_with_and_without_trace(tmp_path):
    field = Field()
    field.add_line(Polyline([(0, 0), (100, 0)]))
    field.add_obstacle(BoxObstacle(150, -10, 200, 10, 100))
    path = tmp_path / 'preview.svg'
    assert field.preview_error is None
    for trace in ((), [(0, 0, 0), (100, 0, 90)]):
        assert field.save_preview(path, trace) is True
        assert field.preview_error is None
        root = ET.parse(path).getroot()
        namespace = '{http://www.w3.org/2000/svg}'
        assert len(root.findall(namespace + 'polyline')) == (2 if trace else 1)
        assert len(root.findall(namespace + 'circle')) == (1 if trace else 0)
        assert len(root.findall(namespace + 'rect')) == 2


def test_preview_handles_missing_directory_and_can_retry(tmp_path):
    field = Field()
    path = tmp_path / 'missing' / 'preview.svg'
    assert field.save_preview(path) is False
    assert isinstance(field.preview_error, FileNotFoundError)
    assert not path.exists()

    path.parent.mkdir()
    assert field.save_preview(path) is True
    assert field.preview_error is None
    assert ET.parse(path).getroot().tag.endswith('svg')


def test_preview_handles_permission_error(monkeypatch, tmp_path):
    field = Field()
    error = PermissionError('Folder is read-only')

    def fail_write(path, text, encoding):
        raise error

    monkeypatch.setattr(Path, 'write_text', fail_write)
    assert field.save_preview(tmp_path / 'preview.svg') is False
    assert field.preview_error is error


def test_abstract_geometry_defaults():
    with pytest.raises(NotImplementedError):
        Line.distance_to(None, 0, 0)
    with pytest.raises(NotImplementedError):
        Line.to_svg(None)
    with pytest.raises(NotImplementedError):
        Obstacle.raycast(None, (0, 0, 0), (1, 0, 0), 100)
    with pytest.raises(NotImplementedError):
        Obstacle.to_svg(None)


def test_map_queries_shapes_and_ranges():
    field = FlatWhiteWorld()
    line = Polyline([(0, 0), (10, 0)])
    box = BoxObstacle(0, 0, 10, 10, 10)
    for operation in (
        lambda: field.get_reflection_at('0', 0),
        lambda: field.get_friction_at(0, None),
        lambda: field.get_ambient_at(0, 0, True),
        lambda: field.raycast_distance(0, 0, '0'),
        lambda: field.raycast_distance(0, 0, 0, pitch_deg='0'),
        lambda: box.raycast(None, (1, 0, 0), 100),
        lambda: Polyline('points'),
        lambda: Polyline([(0, 0), ('1', 0)]),
        lambda: line.distance_to(True, 0),
        lambda: field.save_preview('ignored.svg', trace=None),
    ):
        with pytest.raises(TypeError):
            operation()
    for operation in (
        lambda: field.raycast_surface(0, 0, 0, 0, 0, 0, 100),
        lambda: field.raycast_surface(0, 0, 0, 1, 0, 0, -1),
        lambda: box.raycast((0, 0), (1, 0, 0), 100),
        lambda: line.get_reflection_at(0, 0, 101),
        lambda: field.save_preview('ignored.svg', trace=[(0, 0)]),
        lambda: SurfaceHit(0, 0, 0, -1, 40, 1),
        lambda: SurfaceHit(0, 0, 0, 1, 40, 2),
    ):
        with pytest.raises(ValueError):
            operation()


def test_world_setters_preserve_previous_values():
    field = ScriptableWorld()
    for operation in (
        lambda: setattr(field, 'ambient_light', '5'),
        lambda: setattr(field, 'friction_coefficient', True),
        lambda: field.set_reflection('10'),
        lambda: field.set_distance('100'),
        lambda: field.get_reflection_at(None, 0),
        lambda: field.raycast_distance(0, 0, 0, max_mm='100'),
    ):
        with pytest.raises(TypeError):
            operation()
    for operation in (lambda: field.set_reflection(101),
                      lambda: field.set_distance(-1)):
        with pytest.raises(ValueError):
            operation()
    assert field.ambient_light == 5
    assert field.friction_coefficient == 0.8
    assert field.get_reflection_at(0, 0) == 50
    assert field.raycast_distance(0, 0, 0) == 2000


class CustomWorld(World):

    '''
    Provide two lines and two boxes for reusable world tests.

    Attributes:
        ambient_light (float): Uniform ambient light signal from 0 to 100.
        reflection (float): Ground reflection outside lines, from 0 to 100.
        bounds (tuple): Preview rectangle (xmin, ymin, xmax, ymax) in world mm.
        lines (list): Ground lines contributing to reflection readings.
        obstacles (list): Obstacle shapes tested by surface rays.
        friction_coefficient (float): Uniform non-negative ground friction
            coefficient.
        _friction_coefficient (float): Stored coefficient backing
            friction_coefficient.
        _ambient_light (float): Stored ambient signal backing ambient_light.
    '''

    def __init__(self):
        super().__init__(ambient_light=7)
        self.add_line(Polyline([(0, 0), (500, 0)], width_mm=20))
        self.add_line(Polyline([(0, 100), (500, 100)], width_mm=20))
        self.add_obstacle(BoxObstacle(200, -50, 250, 50, 150))
        self.add_obstacle(BoxObstacle(100, -50, 150, 50, 150))


def test_base_classes_are_abstract():
    for cls in (World, Line, Obstacle):
        with pytest.raises(TypeError):
            cls()


def test_custom_map_only_needs_geometry_configuration():
    field = CustomWorld()
    assert field.get_reflection_at(50, 0) == 10
    assert field.get_reflection_at(50, 100) == 10
    assert field.get_reflection_at(50, 50) == 85
    assert field.get_ambient_at(0, 0, 20) == 7
    assert field.raycast_distance(0, 0, 0, z_mm=60) == 100
    assert field.raycast_distance(0, 0, 0, z_mm=160) == 2000


def test_invalid_geometry_is_rejected():
    field = CustomWorld()
    with pytest.raises(TypeError):
        field.add_line(object())
    with pytest.raises(TypeError):
        field.add_obstacle(object())
    for points in ([], [(0, 0)], [(0, 0), (0, 0)],
                   [(0, 0), (1, float('nan'))]):
        with pytest.raises(ValueError):
            Polyline(points)
    with pytest.raises(ValueError):
        BoxObstacle(10, 0, 0, 100, 150)
    with pytest.raises(ValueError):
        BoxObstacle(0, 0, 100, 100, -1)


def test_default_map_uses_one_active_reference():
    previous = world.current_world
    try:
        world.current_world = None
        assert isinstance(world.get_current_world(), FlatWhiteWorld)
        field = CustomWorld()
        world.current_world = field
        assert world.get_current_world() is field
    finally:
        world.current_world = previous


def test_each_map_has_a_subclass_and_valid_preview():
    root = Path(__file__).resolve().parents[1] / 'maps'
    namespace = '{http://www.w3.org/2000/svg}'
    for name, cls in (
        ('flat_white', FlatWhiteWorld),
        ('straight_line', LineWorld),
        ('scriptable', ScriptableWorld),
        ('line_follow', LineFollowWorld),
        ('force_contact', ForceContactWorld),
    ):
        assert issubclass(cls, World)
        assert (root / name / 'world.py').is_file()
        image = ET.parse(root / name / 'preview.svg').getroot()
        assert image.tag == namespace + 'svg'
        field = cls()
        assert len(image.findall(namespace + 'polyline')) == len(field.lines)
        assert len(image.findall(namespace + 'rect')
                   ) == 1 + len(field.obstacles)
