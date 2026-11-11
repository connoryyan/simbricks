from enum import Enum
from sim.validation import number


class Port(Enum):
    '''
    List supported device ports.

    Attributes:
        _value_ (str): Stored port letter from A through F.
        A (Port): Device port A.
        B (Port): Device port B.
        C (Port): Device port C.
        D (Port): Device port D.
        E (Port): Device port E.
        F (Port): Device port F.
        name (str): Name of the selected enum member.
        value (str): Port letter from A through F.
    '''
    _value_: str
    A = 'A'
    B = 'B'
    C = 'C'
    D = 'D'
    E = 'E'
    F = 'F'


class Direction(Enum):
    '''
    Select the positive motor rotation direction.

    Attributes:
        _value_ (int): Stored motor polarity sign, +1 or -1.
        CLOCKWISE (Direction): Clockwise rotation with polarity +1.
        COUNTERCLOCKWISE (Direction): Counter-clockwise rotation with polarity -1.
        name (str): Name of the selected enum member.
        value (int): Polarity sign used by signed motor commands.
    '''
    _value_: int
    CLOCKWISE = 1
    COUNTERCLOCKWISE = -1


class Axis(Enum):
    '''
    Select a three-dimensional reference axis.

    Attributes:
        _value_ (tuple): Stored three-component unit direction.
        X (Axis): Unit vector along the positive x axis.
        Y (Axis): Unit vector along the positive y axis.
        Z (Axis): Unit vector along the positive z axis.
        name (str): Name of the selected enum member.
        value (tuple): Selected unit direction as (x, y, z).
    '''
    _value_: tuple[float, float, float]
    X = (1.0, 0.0, 0.0)
    Y = (0.0, 1.0, 0.0)
    Z = (0.0, 0.0, 1.0)

    def __neg__(self) -> tuple[float, float, float]:
        return tuple(-v for v in self.value)


class Stop(Enum):
    '''
    Select the stop mode after a move.

    Attributes:
        _value_ (int): Stored numeric stop-mode identifier.
        COAST (Stop): Release motor power and allow the motor to slow down.
        BRAKE (Stop): Apply passive electrical braking.
        HOLD (Stop): Actively hold the motor or wheel position.
        COAST_SMART (Stop): Smart-coast mode; currently simulated like COAST.
        name (str): Name of the selected enum member.
        value (int): Numeric identifier for the selected stop mode.
    '''
    _value_: int
    COAST = 0
    BRAKE = 1
    HOLD = 2
    COAST_SMART = 3


class Button(Enum):
    '''
    Identify hub buttons used by scripted input.

    Attributes:
        _value_ (str): Stored hub button identifier.
        LEFT (Button): Left hub button.
        RIGHT (Button): Right hub button.
        CENTER (Button): Center hub button.
        BLUETOOTH (Button): Bluetooth hub button.
        LEFT_PLUS (Button): Plus button on the left side.
        LEFT_MINUS (Button): Minus button on the left side.
        RIGHT_PLUS (Button): Plus button on the right side.
        RIGHT_MINUS (Button): Minus button on the right side.
        name (str): Name of the selected enum member.
        value (str): Identifier used by scripted button input.
    '''
    _value_: str
    LEFT = 'LEFT'
    RIGHT = 'RIGHT'
    CENTER = 'CENTER'
    BLUETOOTH = 'BLUETOOTH'
    LEFT_PLUS = 'LEFT_PLUS'
    LEFT_MINUS = 'LEFT_MINUS'
    RIGHT_PLUS = 'RIGHT_PLUS'
    RIGHT_MINUS = 'RIGHT_MINUS'


class Side(Enum):
    '''
    Identify a face of the hub.

    Attributes:
        _value_ (int): Stored numeric hub face identifier.
        TOP (Side): Top hub face.
        BOTTOM (Side): Bottom hub face.
        FRONT (Side): Front hub face.
        BACK (Side): Back hub face.
        LEFT (Side): Left hub face.
        RIGHT (Side): Right hub face.
        name (str): Name of the selected enum member.
        value (int): Numeric identifier for the selected hub face.
    '''
    _value_: int
    TOP = 8
    BOTTOM = 2
    FRONT = 0
    BACK = 5
    LEFT = 4
    RIGHT = 6


class Color:
    '''
    Store hue, saturation and brightness.

    Attributes:
        BLACK (Color): Predefined black color.
        WHITE (Color): Predefined white color.
        RED (Color): Predefined red color.
        GREEN (Color): Predefined green color.
        BLUE (Color): Predefined blue color.
        NONE (None): Marker for no detected color.
        h (float): Hue angle from 0 to 360 degrees.
        s (float): Saturation from 0 to 100 percent.
        v (float): Brightness from 0 to 100 percent.
    '''
    BLACK: 'Color'
    WHITE: 'Color'
    RED: 'Color'
    GREEN: 'Color'
    BLUE: 'Color'
    NONE: None

    def __init__(self, h: float = 0, s: float = 0, v: float = 0) -> None:
        number(h, 'h', minimum=0, maximum=360)
        number(s, 's', minimum=0, maximum=100)
        number(v, 'v', minimum=0, maximum=100)
        self.h: float = h
        self.s: float = s
        self.v: float = v

    def __repr__(self) -> str:
        return f"Color(h={self.h}, s={self.s}, v={self.v})"


Color.BLACK = Color(0, 0, 0)
Color.WHITE = Color(0, 0, 100)
Color.RED = Color(0, 100, 100)
Color.GREEN = Color(120, 100, 100)
Color.BLUE = Color(240, 100, 100)
Color.NONE = None
