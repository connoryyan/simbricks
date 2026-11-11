import pytest

from sim.validation import boolean, integer, number, sequence, unit_vector, vector


def test_integer_and_length_validation():
    assert integer(0, 'count', minimum=0) == 0
    assert integer(10 ** 400, 'seed') == 10 ** 400
    for value in (None, True, 1.0, '1'):
        with pytest.raises(TypeError):
            integer(value, 'count')
        with pytest.raises(TypeError):
            vector((1, 2, 3), 'vector', value)
    with pytest.raises(ValueError):
        sequence([], 'items', length=-1)
    assert sequence([], 'items', length=0) == []


def test_numeric_validation():
    assert number(0, 'value', minimum=0, maximum=1) == 0
    assert number(1, 'value', minimum=0, maximum=1) == 1
    for value in (None, '1', True, [], object()):
        with pytest.raises(TypeError, match='value'):
            number(value, 'value')
    for value in (float('nan'), float('inf'), -float('inf'), 10 ** 400):
        with pytest.raises(ValueError, match='finite'):
            number(value, 'value')
    with pytest.raises(ValueError, match='positive'):
        number(0, 'value', positive=True)
    with pytest.raises(ValueError, match='at least'):
        number(-1, 'value', minimum=0)
    with pytest.raises(ValueError, match='at most'):
        number(2, 'value', maximum=1)


def test_boolean_and_vector_validation():
    assert boolean(False, 'flag') is False
    with pytest.raises(TypeError, match='flag'):
        boolean(1, 'flag')
    assert sequence([], 'items') == []
    with pytest.raises(TypeError, match='items'):
        sequence('abc', 'items')
    with pytest.raises(ValueError, match='3 values'):
        vector((1, 2), 'pose', 3)
    assert unit_vector([1, 0, 0], 'direction') == (1, 0, 0)
    for direction in ((0, 0, 0), (2, 0, 0)):
        with pytest.raises(ValueError, match='unit vector'):
            unit_vector(direction, 'direction')
