import xml.etree.ElementTree as ET

import pytest

from world import BoxObstacle, Line, Obstacle, Polyline, World


class Field(World):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)


def test_map_validation():
    for reflection in (-1, 101, float('nan')):
        with pytest.raises(ValueError, match='reflection'):
            Field(reflection=reflection)
    for bounds in ((0, 0, 1), (0, 0, float('inf'), 1), (0, 0, 0, 1), (0, 1, 1, 0)):
        with pytest.raises(ValueError, match='bounds'):
            Field(bounds=bounds)


def test_line_validation():
    for options in ({'width_mm': 0}, {'edge_width_mm': 30}, {'reflection': -1}):
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
    assert box.raycast((-5, 20, 5), (1, -0.1, 0), 100) is None
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
    for trace in ((), [(0, 0, 0), (100, 0, 90)]):
        field.save_preview(path, trace)
        root = ET.parse(path).getroot()
        namespace = '{http://www.w3.org/2000/svg}'
        assert len(root.findall(namespace + 'polyline')) == (2 if trace else 1)
        assert len(root.findall(namespace + 'circle')) == (1 if trace else 0)
        assert len(root.findall(namespace + 'rect')) == 2


def test_abstract_geometry_defaults():
    with pytest.raises(NotImplementedError):
        Line.distance_to(None, 0, 0)
    with pytest.raises(NotImplementedError):
        Line.to_svg(None)
    with pytest.raises(NotImplementedError):
        Obstacle.raycast(None, (0, 0, 0), (1, 0, 0), 100)
    with pytest.raises(NotImplementedError):
        Obstacle.to_svg(None)
