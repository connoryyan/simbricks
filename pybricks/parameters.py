from __future__ import annotations

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


class Stop(Enum):
    COAST = 0
    BRAKE = 1
    HOLD = 2
    COAST_SMART = 3


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