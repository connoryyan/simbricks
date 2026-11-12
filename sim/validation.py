import math


def number(
    value: object,
    name: str,
    *,
    minimum: float | None = None,
    maximum: float | None = None,
    positive: bool = False,
) -> float:
    '''
    Check a finite int or float without accepting bool or numeric strings.
    '''
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f'{name} must be an int or float')
    try:
        finite = math.isfinite(value)
    except OverflowError:
        finite = False
    if not finite:
        raise ValueError(f'{name} must be finite')
    if positive and value <= 0:
        raise ValueError(f'{name} must be positive')
    if minimum is not None and value < minimum:
        raise ValueError(f'{name} must be at least {minimum}')
    if maximum is not None and value > maximum:
        raise ValueError(f'{name} must be at most {maximum}')
    return value


def boolean(value: object, name: str) -> bool:
    '''
    Check a boolean flag.
    '''
    if not isinstance(value, bool):
        raise TypeError(f'{name} must be a bool')
    return value


def integer(value: object, name: str, *, minimum: int | None = None) -> int:
    '''
    Check an integer without accepting bool.
    '''
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f'{name} must be an int')
    if minimum is not None and value < minimum:
        raise ValueError(f'{name} must be at least {minimum}')
    return value


def sequence(
    value: object,
    name: str,
    length: int | None = None,
) -> list[object] | tuple[object, ...]:
    '''
    Check a list or tuple and its optional length.
    '''
    if length is not None:
        integer(length, 'length', minimum=0)
    if not isinstance(value, (list, tuple)):
        raise TypeError(f'{name} must be a list or tuple')
    if length is not None and len(value) != length:
        raise ValueError(f'{name} must contain {length} values')
    return value


def vector(value: object, name: str, length: int) -> tuple[float, ...]:
    '''
    Check a vector of finite numbers.
    '''
    integer(length, 'length', minimum=0)
    sequence(value, name, length)
    for component in value:
        number(component, name)
    return tuple(value)


def unit_vector(value: object, name: str) -> tuple[float, float, float]:
    '''
    Check a unit direction in three dimensions.
    '''
    value = vector(value, name, 3)
    if not math.isclose(math.hypot(*value), 1.0, rel_tol=1e-9, abs_tol=1e-9):
        raise ValueError(f'{name} must be a unit vector')
    return value
