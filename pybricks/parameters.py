from enum import Enum


class Port(Enum):
    A = "A"
    B = "B"
    C = "C"
    D = "D"
    E = "E"
    F = "F"


class Direction(Enum):
    CLOCKWISE = 1
    COUNTERCLOCKWISE = -1


class Axis(Enum):
    X = (1.0, 0.0, 0.0)
    Y = (0.0, 1.0, 0.0)
    Z = (0.0, 0.0, 1.0)

    def __neg__(self):
        return tuple(-v for v in self.value)


class Stop(Enum):
    COAST = 0
    BRAKE = 1
    HOLD = 2
    COAST_SMART = 3


class Button(Enum):
    LEFT = 'LEFT'
    RIGHT = 'RIGHT'
    CENTER = 'CENTER'
    BLUETOOTH = 'BLUETOOTH'
    LEFT_PLUS = 'LEFT_PLUS'
    LEFT_MINUS = 'LEFT_MINUS'
    RIGHT_PLUS = 'RIGHT_PLUS'
    RIGHT_MINUS = 'RIGHT_MINUS'


class Side(Enum):
    TOP = 8
    BOTTOM = 2
    FRONT = 0
    BACK = 5
    LEFT = 4
    RIGHT = 6


class Color:

    def __init__(self, h: float = 0, s: float = 0, v: float = 0):
        self.h, self.s, self.v = h, s, v

    def __repr__(self) -> str:
        return f"Color(h={self.h}, s={self.s}, v={self.v})"


Color.BLACK = Color(0, 0, 0)
Color.WHITE = Color(0, 0, 100)
Color.RED = Color(0, 100, 100)
Color.GREEN = Color(120, 100, 100)
Color.BLUE = Color(240, 100, 100)
Color.NONE = None
